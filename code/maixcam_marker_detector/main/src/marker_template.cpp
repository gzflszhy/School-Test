#include "marker_template.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace maixcam_marker {
namespace {

float clamp01(float value) { return std::clamp(value, 0.0F, 1.0F); }

int percentile8u(const cv::Mat& image, float fraction) {
    std::array<int, 256> histogram{};
    for (int y = 0; y < image.rows; ++y) {
        const auto* row = image.ptr<std::uint8_t>(y);
        for (int x = 0; x < image.cols; ++x) ++histogram[row[x]];
    }
    const int target = static_cast<int>(fraction * static_cast<float>(image.total()));
    int cumulative = 0;
    for (int value = 0; value < 256; ++value) {
        cumulative += histogram[static_cast<std::size_t>(value)];
        if (cumulative >= target) return value;
    }
    return 255;
}

}  // namespace

MarkerTemplate::MarkerTemplate(const DetectorConfig& config, MarkerGeometry geometry)
    : config_(config), geometry_(std::move(geometry)) {
    config_.normalize();
    const cv::Size canvas(config_.canonical_size, config_.canonical_size);
    cv::Mat all_led_mask = cv::Mat::zeros(canvas, CV_8UC1);
    large_l_mask_ = cv::Mat::zeros(canvas, CV_8UC1);
    background_mask_ = cv::Mat::zeros(canvas, CV_8UC1);
    const auto board_corners = geometry_.boardCornersCanonical(config_.canonical_size);
    std::vector<cv::Point> board_polygon;
    board_polygon.reserve(board_corners.size());
    for (const auto& point : board_corners) {
        board_polygon.emplace_back(cvRound(point.x), cvRound(point.y));
    }
    cv::fillConvexPoly(background_mask_, board_polygon, cv::Scalar(255), cv::LINE_8);

    const auto& components = geometry_.components();
    for (std::size_t i = 0; i < components.size(); ++i) {
        component_masks_[i] = cv::Mat::zeros(canvas, CV_8UC1);
        const std::vector<cv::Point> polygon =
            geometry_.polygonToCanonical(components[i], config_.canonical_size);
        cv::fillConvexPoly(component_masks_[i], polygon, cv::Scalar(255), cv::LINE_8);
        // L polygons are concave: fillPoly preserves their re-entrant corner.
        component_masks_[i].setTo(0);
        const std::vector<std::vector<cv::Point>> polygons{polygon};
        cv::fillPoly(component_masks_[i], polygons, cv::Scalar(255), cv::LINE_8);
        cv::bitwise_or(all_led_mask, component_masks_[i], all_led_mask);
    }
    const auto cad_large_centers = geometry_.largeLCentersMm();
    // CAD indices: 2 is the right-angle vertex, 1 is its +X neighbour and 0
    // is its +Y neighbour. evaluateTriple relies on this exact semantic order.
    large_l_centers_px_[0] = geometry_.mmToCanonical(cad_large_centers[2],
                                                      config_.canonical_size);
    large_l_centers_px_[1] = geometry_.mmToCanonical(cad_large_centers[1],
                                                      config_.canonical_size);
    large_l_centers_px_[2] = geometry_.mmToCanonical(cad_large_centers[0],
                                                      config_.canonical_size);
    if (config_.template_dilate_px > 0) {
        const int diameter = config_.template_dilate_px * 2 + 1;
        const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                         cv::Size(diameter, diameter));
        cv::dilate(all_led_mask, all_led_mask, kernel);
        for (auto& mask : component_masks_) cv::dilate(mask, mask, kernel);
    }
    for (std::size_t i = 0; i < component_masks_.size(); ++i) {
        if (components[i].kind == ComponentKind::LARGE_L) {
            cv::bitwise_or(large_l_mask_, component_masks_[i], large_l_mask_);
        }
    }
    // Contrast is measured against the non-LED region inside the CAD outline.
    // This is not a black-board detector and does not constrain acceptance.
    background_mask_.setTo(0, all_led_mask);
    for (std::size_t i = 0; i < component_masks_.size(); ++i) {
        component_pixel_counts_[i] = std::max(1, cv::countNonZero(component_masks_[i]));
    }
    const auto& cad_large_l = components[0].polygon_mm;
    float min_x = cad_large_l.front().x;
    float max_x = cad_large_l.front().x;
    float min_y = cad_large_l.front().y;
    float max_y = cad_large_l.front().y;
    for (const auto& point : cad_large_l) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }
    constexpr float kShapeMargin = 8.0F;
    constexpr float kShapeSpan = 48.0F;
    large_l_shape_contour_.reserve(cad_large_l.size());
    for (const auto& point : cad_large_l) {
        large_l_shape_contour_.emplace_back(
            cvRound(kShapeMargin + kShapeSpan * (point.x - min_x) / (max_x - min_x)),
            cvRound(kShapeMargin + kShapeSpan * (point.y - min_y) / (max_y - min_y)));
    }
}

TemplateScore MarkerTemplate::score(const cv::Mat& canonical_gray,
                                     cv::Mat& scratch_binary) const {
    TemplateScore result;
    if (canonical_gray.empty() || canonical_gray.type() != CV_8UC1 ||
        canonical_gray.size() != large_l_mask_.size()) {
        return result;
    }

    const double otsu = cv::threshold(canonical_gray, otsu_scratch_, 0, 255,
                                      cv::THRESH_BINARY | cv::THRESH_OTSU);
    // Subtract one because cv::threshold uses a strict greater-than test; a
    // flat, unsaturated LED plateau can otherwise disappear at its percentile.
    const int high = std::max(0, percentile8u(canonical_gray,
                                              config_.bright_percentile) - 1);
    const cv::Scalar background_mean = cv::mean(canonical_gray, background_mask_);
    const int contrast_threshold = cvRound(background_mean[0] +
                                           config_.local_contrast_threshold);
    const int threshold = std::clamp(std::max({static_cast<int>(otsu), high,
                                                contrast_threshold}),
                                     config_.min_led_threshold,
                                     config_.max_led_threshold);
    cv::threshold(canonical_gray, scratch_binary, threshold, 255, cv::THRESH_BINARY);

    float large_coverage = 0.0F;
    float optional_coverage = 0.0F;
    int large_count = 0;
    int optional_count = 0;
    for (std::size_t i = 0; i < component_masks_.size(); ++i) {
        cv::bitwise_and(scratch_binary, component_masks_[i], overlap_scratch_);
        const int expected = component_pixel_counts_[i];
        const int observed_pixels = cv::countNonZero(overlap_scratch_);
        const float coverage = static_cast<float>(observed_pixels) /
                               static_cast<float>(expected);
        const bool is_large = geometry_.components()[i].kind == ComponentKind::LARGE_L;
        const float required = is_large ? config_.min_large_component_coverage
                                        : config_.min_small_component_coverage;
        if (coverage >= required) {
            if (!is_large) ++result.matched_optional_components;
            result.component_detected[i] = true;
            // Large-L centres already come from the source contours. Moments
            // are only needed for the three optional refinement features.
            if (!is_large) {
                const cv::Moments moments = cv::moments(overlap_scratch_, true);
                if (moments.m00 > 0.0) {
                    result.component_centers_canonical[i] = {
                        static_cast<float>(moments.m10 / moments.m00),
                        static_cast<float>(moments.m01 / moments.m00)};
                }
            }
        }
        const float normalized_coverage = clamp01(coverage / std::max(0.01F, required));
        if (is_large) {
            large_coverage += normalized_coverage;
            ++large_count;
        } else {
            optional_coverage += normalized_coverage;
            ++optional_count;
        }
    }
    result.large_l_template_score = large_count > 0
        ? large_coverage / static_cast<float>(large_count) : 0.0F;
    result.optional_template_score = optional_count > 0
        ? optional_coverage / static_cast<float>(optional_count) : 0.0F;

    // Primary contrast must depend only on the three required large Ls. The
    // optional code marks may be off, occluded or outside their ideal masks.
    const double led_mean = cv::mean(canonical_gray, large_l_mask_)[0];
    const double background_intensity = background_mean[0];
    result.contrast_score = clamp01(static_cast<float>(led_mean - background_intensity) /
                                    std::max(1.0F, config_.contrast_score_full_scale *
                                                       config_.min_led_board_contrast));

    cv::compare(canonical_gray, config_.saturation_threshold, saturated_scratch_, cv::CMP_GE);
    result.saturation_fraction = static_cast<float>(cv::countNonZero(saturated_scratch_)) /
                                 static_cast<float>(canonical_gray.total());
    return result;
}

}  // namespace maixcam_marker
