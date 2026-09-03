#include "marker_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace maixcam_marker {
namespace {

constexpr float kPi = 3.14159265358979323846F;

float clamp01(float value) { return std::clamp(value, 0.0F, 1.0F); }

DetectorConfig normalizedConfig(DetectorConfig config) {
    config.normalize();
    return config;
}

float pointDistance(const cv::Point2f& a, const cv::Point2f& b) {
    return cv::norm(a - b);
}

cv::Rect clampRect(const cv::Rect2f& rectangle, const cv::Size& bounds) {
    const int x0 = std::clamp(static_cast<int>(std::floor(rectangle.x)), 0, bounds.width);
    const int y0 = std::clamp(static_cast<int>(std::floor(rectangle.y)), 0, bounds.height);
    const int x1 = std::clamp(static_cast<int>(std::ceil(rectangle.x + rectangle.width)),
                              0, bounds.width);
    const int y1 = std::clamp(static_cast<int>(std::ceil(rectangle.y + rectangle.height)),
                              0, bounds.height);
    return {x0, y0, std::max(0, x1 - x0), std::max(0, y1 - y0)};
}

cv::Rect2f boundingRect2f(const std::array<cv::Point2f, 4>& points) {
    float min_x = points[0].x;
    float min_y = points[0].y;
    float max_x = points[0].x;
    float max_y = points[0].y;
    for (const auto& point : points) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
    }
    return {min_x, min_y, max_x - min_x, max_y - min_y};
}

cv::Point2f transformPoint(const cv::Matx23f& transform, const cv::Point2f& point) {
    return {transform(0, 0) * point.x + transform(0, 1) * point.y + transform(0, 2),
            transform(1, 0) * point.x + transform(1, 1) * point.y + transform(1, 2)};
}

cv::Matx23f asMatx23f(const cv::Mat& matrix) {
    cv::Mat converted;
    matrix.convertTo(converted, CV_32F);
    return {converted.at<float>(0, 0), converted.at<float>(0, 1), converted.at<float>(0, 2),
            converted.at<float>(1, 0), converted.at<float>(1, 1), converted.at<float>(1, 2)};
}

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

MarkerOrientation quantizeOrientation(const cv::Point2f& x_axis) {
    float degrees = std::atan2(x_axis.y, x_axis.x) * 180.0F / kPi;
    if (degrees < 0.0F) degrees += 360.0F;
    const int quadrant = static_cast<int>(std::lround(degrees / 90.0F)) & 3;
    switch (quadrant) {
        case 1: return MarkerOrientation::DEG_90;
        case 2: return MarkerOrientation::DEG_180;
        case 3: return MarkerOrientation::DEG_270;
        default: return MarkerOrientation::DEG_0;
    }
}

}  // namespace

MarkerDetector::MarkerDetector(DetectorConfig config)
    : config_(normalizedConfig(std::move(config))),
      marker_template_(config_),
      benchmark_(config_.benchmark_window) {
    morphology_kernel_ = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(config_.morphology_kernel, config_.morphology_kernel));
    canonical_gray_.create(config_.canonical_size, config_.canonical_size, CV_8UC1);
    canonical_binary_.create(config_.canonical_size, config_.canonical_size, CV_8UC1);
    candidate_mask_.create(config_.frame_height, config_.frame_width, CV_8UC1);
    board_corner_buffer_.resize(4);
}

void MarkerDetector::reset() {
    state_.reset();
    benchmark_.reset();
    previous_output_ = SteadyTimePoint{};
}

void MarkerDetector::recordDroppedFrame() {
    if (config_.benchmark_enabled) benchmark_.add(0.0, 0.0, 0.0, false, true);
}

void MarkerDetector::makeGray(const cv::Mat& frame) {
    if (frame.empty()) {
        gray_.release();
        return;
    }
    if (frame.type() == CV_8UC1) {
        gray_ = frame;
    } else if (frame.channels() == 1) {
        frame.convertTo(gray_, CV_8U);
    } else if (frame.channels() == 3) {
        cv::cvtColor(frame, gray_, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray_, cv::COLOR_BGRA2GRAY);
    } else {
        gray_.release();
    }
}

cv::Rect MarkerDetector::trackingSearchRoi(SteadyTimePoint capture_timestamp) const {
    if (state_.mode != DetectorState::TRACK_ROI || !state_.has_observation) {
        return {0, 0, gray_.cols, gray_.rows};
    }
    double dt = std::chrono::duration<double>(capture_timestamp - state_.last_capture).count();
    dt = std::clamp(dt, 0.0, static_cast<double>(config_.prediction_horizon_seconds));
    const cv::Point2f predicted = state_.last_center +
        state_.velocity_px_per_second * static_cast<float>(dt);
    const float scale = state_.lost_count >= config_.expand_after_lost_frames
                            ? config_.tracking_roi_lost_scale
                            : config_.tracking_roi_scale;
    const float width = std::max(static_cast<float>(config_.min_marker_width_px),
                                 state_.last_bbox.width * scale);
    const float height = std::max(static_cast<float>(config_.min_marker_width_px),
                                  state_.last_bbox.height * scale);
    return clampRect({predicted.x - width * 0.5F, predicted.y - height * 0.5F,
                      width, height}, gray_.size());
}

int MarkerDetector::makeLedMask(const cv::Rect& roi) {
    const cv::Mat source = gray_(roi);
    const double otsu = cv::threshold(source, roi_binary_, 0, 255,
                                      cv::THRESH_BINARY | cv::THRESH_OTSU);
    const int high = std::max(0, percentile8u(source, config_.bright_percentile) - 1);
    const int mean_based = cvRound(cv::mean(source)[0] + config_.local_contrast_threshold);
    const int threshold = std::clamp(std::max({static_cast<int>(otsu), high, mean_based}),
                                     config_.min_led_threshold, config_.max_led_threshold);
    cv::threshold(source, roi_binary_, threshold, 255, cv::THRESH_BINARY);
    if (config_.morphology_kernel > 1) {
        // A single close heals narrow rolling-shutter/PWM dark stripes without
        // the expansion caused by a large-kernel pipeline.
        cv::morphologyEx(roi_binary_, roi_binary_, cv::MORPH_CLOSE, morphology_kernel_);
    }
    return threshold;
}

std::vector<MarkerDetector::LedCandidate> MarkerDetector::findLedCandidates(
    const cv::Rect& roi) {
    const int led_threshold = makeLedMask(roi);
    if (debug_enabled_) {
        debug_snapshot_.search_roi = roi;
        debug_snapshot_.led_threshold = led_threshold;
        roi_binary_.copyTo(debug_snapshot_.led_mask);
    }
    contour_buffer_.clear();
    cv::findContours(roi_binary_, contour_buffer_, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE, roi.tl());
    if (debug_enabled_) {
        debug_snapshot_.raw_contour_count = static_cast<int>(contour_buffer_.size());
    }

    struct PreCandidate {
        std::size_t contour_index;
        cv::RotatedRect box;
        float maximum_side;
        float fill;
        float cheap_score;
    };
    std::vector<PreCandidate> pre_candidates;
    const std::size_t pre_candidate_limit =
        static_cast<std::size_t>(config_.max_pre_shape_candidates);
    pre_candidates.reserve(pre_candidate_limit);
    for (std::size_t contour_index = 0; contour_index < contour_buffer_.size();
         ++contour_index) {
        const auto& contour = contour_buffer_[contour_index];
        const float area = static_cast<float>(std::abs(cv::contourArea(contour)));
        if (area < config_.min_component_area_px || area > config_.max_component_area_px ||
            contour.size() < 4) continue;
        const cv::RotatedRect box = cv::minAreaRect(contour);
        const float width = std::max(1.0F, box.size.width);
        const float height = std::max(1.0F, box.size.height);
        const float minimum_side = std::min(width, height);
        const float maximum_side = std::max(width, height);
        if (minimum_side < config_.min_component_size_px ||
            maximum_side > config_.max_component_size_px) continue;
        const float aspect = width / height;
        const float fill = area / (width * height);
        if (aspect < config_.component_aspect_min || aspect > config_.component_aspect_max ||
            fill < config_.component_fill_min || fill > config_.component_fill_max) continue;

        // Only cheap area/box features are evaluated before this bounded
        // shortlist. Convex hull, Hu matching and intensity masks are never
        // run for more than max_pre_shape_candidates contours.
        const float square_score = minimum_side / maximum_side;
        const float fill_score = clamp01(1.0F -
            std::abs(fill - config_.component_fill_target) /
            std::max(0.01F, config_.component_fill_score_tolerance));
        const PreCandidate candidate{contour_index, box, maximum_side, fill,
            config_.pre_shape_square_weight * square_score +
            config_.pre_shape_fill_weight * fill_score};
        if (pre_candidates.size() < pre_candidate_limit) {
            pre_candidates.push_back(candidate);
        } else {
            const auto weakest = std::min_element(
                pre_candidates.begin(), pre_candidates.end(),
                [](const PreCandidate& lhs, const PreCandidate& rhs) {
                    return lhs.cheap_score < rhs.cheap_score;
                });
            if (candidate.cheap_score > weakest->cheap_score) *weakest = candidate;
        }
    }
    std::sort(pre_candidates.begin(), pre_candidates.end(),
              [](const PreCandidate& lhs, const PreCandidate& rhs) {
                  return lhs.cheap_score > rhs.cheap_score;
              });

    const auto& reference = marker_template_.largeLShapeContour();
    std::vector<LedCandidate> result;
    result.reserve(pre_candidates.size());
    for (const auto& pre_candidate : pre_candidates) {
        const auto& contour = contour_buffer_[pre_candidate.contour_index];

        hull_buffer_.clear();
        cv::convexHull(contour, hull_buffer_);
        const float contour_area = static_cast<float>(std::abs(cv::contourArea(contour)));
        const float hull_area = static_cast<float>(std::abs(cv::contourArea(hull_buffer_)));
        const float solidity = hull_area > 0.0F ? contour_area / hull_area : 1.0F;
        if (solidity < config_.component_solidity_min ||
            solidity > config_.component_solidity_max) continue;
        const float shape_distance = static_cast<float>(
            cv::matchShapes(contour, reference, cv::CONTOURS_MATCH_I1, 0.0));
        const float shape_score = clamp01(1.0F - shape_distance /
                                                   std::max(0.01F, config_.max_shape_distance));
        const float fill_target_score = clamp01(1.0F -
            std::abs(pre_candidate.fill - config_.component_fill_target) /
            std::max(0.01F, config_.component_fill_score_tolerance));
        const float concavity_score = clamp01((1.0F - solidity) /
            std::max(0.01F, config_.component_concavity_score_span));
        const float score = config_.candidate_shape_score_weight * shape_score +
                            config_.candidate_fill_score_weight * fill_target_score +
                            config_.candidate_concavity_score_weight * concavity_score;
        if (score < config_.min_component_score) continue;
        result.push_back({contour, pre_candidate.box, pre_candidate.box.center,
                          pre_candidate.maximum_side, 0.0F, score});
    }
    std::sort(result.begin(), result.end(), [](const LedCandidate& lhs, const LedCandidate& rhs) {
        return lhs.score > rhs.score;
    });
    const std::size_t final_candidate_limit = static_cast<std::size_t>(
        std::min(config_.max_led_candidates, config_.max_topology_candidates));
    if (result.size() > final_candidate_limit) {
        result.resize(final_candidate_limit);
    }
    if (debug_enabled_) {
        debug_snapshot_.candidates.clear();
        debug_snapshot_.candidates.reserve(result.size());
        for (const auto& candidate : result) {
            debug_snapshot_.candidates.push_back(candidate.box);
        }
    }
    // Build an intensity mask only for the final bounded candidate set. Both
    // Mat storage and point-vector capacity are reused across candidates.
    for (auto& candidate : result) {
        const cv::Rect bounds = clampRect(cv::boundingRect(candidate.contour), gray_.size());
        if (bounds.area() <= 0) continue;
        if (candidate_mask_.cols < bounds.width || candidate_mask_.rows < bounds.height) {
            candidate_mask_.create(std::max(candidate_mask_.rows, bounds.height),
                                   std::max(candidate_mask_.cols, bounds.width), CV_8UC1);
        }
        cv::Mat mask_roi = candidate_mask_(cv::Rect(0, 0, bounds.width, bounds.height));
        mask_roi.setTo(0);
        local_contour_buffer_.clear();
        local_contour_buffer_.reserve(candidate.contour.size());
        for (const auto& point : candidate.contour) {
            local_contour_buffer_.push_back(point - bounds.tl());
        }
        const cv::Point* points = local_contour_buffer_.data();
        const int point_count = static_cast<int>(local_contour_buffer_.size());
        cv::fillPoly(mask_roi, &points, &point_count, 1, cv::Scalar(255));
        candidate.mean_intensity = static_cast<float>(cv::mean(gray_(bounds),
                                                               mask_roi)[0]);
    }
    return result;
}

MarkerDetector::Hypothesis MarkerDetector::evaluateTransform(
    const cv::Matx23f& image_to_canonical, float topology_score,
    MarkerOrientation orientation, HypothesisPath path) {
    Hypothesis hypothesis;
    cv::warpAffine(gray_, canonical_gray_, cv::Mat(image_to_canonical),
                   cv::Size(config_.canonical_size, config_.canonical_size),
                   cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(127));
    hypothesis.detail = marker_template_.score(canonical_gray_, canonical_binary_);
    const int additional_components = hypothesis.detail.matched_components -
                                      hypothesis.detail.matched_large_components;
    const bool pattern_valid = path == HypothesisPath::LED_TRIPLE
        ? hypothesis.detail.matched_large_components == 3 &&
              hypothesis.detail.contrast_score >= config_.min_detection_contrast_score &&
              (additional_components >= config_.min_additional_led_components ||
               hypothesis.detail.black_board_score >=
                   config_.min_black_board_validation_score)
        : hypothesis.detail.matched_large_components >= config_.fallback_min_large_components &&
              hypothesis.detail.matched_components >= config_.fallback_min_total_components &&
              hypothesis.detail.template_score >= config_.fallback_min_template_score &&
              hypothesis.detail.contrast_score >= config_.fallback_min_contrast_score &&
              hypothesis.detail.black_board_score >= config_.fallback_min_black_board_score;
    if (!pattern_valid) return hypothesis;

    cv::invertAffineTransform(cv::Mat(image_to_canonical), affine_inverse_);
    hypothesis.image_to_canonical = image_to_canonical;
    hypothesis.canonical_to_image = asMatx23f(affine_inverse_);
    const float last = static_cast<float>(config_.canonical_size - 1);
    const auto canonical_corners = marker_template_.geometry().boardCornersCanonical(
        config_.canonical_size);
    for (std::size_t i = 0; i < board_corner_buffer_.size(); ++i) {
        board_corner_buffer_[i] = transformPoint(hypothesis.canonical_to_image,
                                                 canonical_corners[i]);
    }
    hypothesis.rotated_board = cv::minAreaRect(board_corner_buffer_);
    const std::array<cv::Point2f, 4> image_corners{{board_corner_buffer_[0],
                                                    board_corner_buffer_[1],
                                                    board_corner_buffer_[2],
                                                    board_corner_buffer_[3]}};
    hypothesis.bbox = boundingRect2f(image_corners);
    hypothesis.center = transformPoint(hypothesis.canonical_to_image,
                                       {last * 0.5F, last * 0.5F});
    hypothesis.topology_score = clamp01(topology_score);
    hypothesis.total_score =
        config_.topology_weight * hypothesis.topology_score +
        config_.template_weight * hypothesis.detail.template_score +
        config_.contrast_weight * hypothesis.detail.contrast_score +
        config_.black_board_weight * hypothesis.detail.black_board_score;
    hypothesis.orientation = orientation;
    hypothesis.path = path;
    hypothesis.valid = hypothesis.bbox.width >= config_.min_marker_width_px &&
                       hypothesis.bbox.width <= config_.max_marker_width_px *
                                                    config_.hypothesis_max_width_factor &&
                       hypothesis.detail.extra_bright_fraction <=
                           config_.max_extra_bright_fraction *
                               config_.extra_bright_reject_factor;
    return hypothesis;
}

bool MarkerDetector::makeTripleProposal(const LedCandidate& a, const LedCandidate& b,
                                        const LedCandidate& c,
                                        TripleProposal& proposal) const {
    const std::array<const LedCandidate*, 3> candidates{{&a, &b, &c}};
    int right_index = 0;
    float best_abs_cosine = std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i) {
        const cv::Point2f u = candidates[(i + 1) % 3]->center - candidates[i]->center;
        const cv::Point2f v = candidates[(i + 2) % 3]->center - candidates[i]->center;
        const float denominator = static_cast<float>(std::max(1e-3, cv::norm(u) * cv::norm(v)));
        const float abs_cosine = std::abs(u.dot(v) / denominator);
        if (abs_cosine < best_abs_cosine) {
            best_abs_cosine = abs_cosine;
            right_index = i;
        }
    }
    if (best_abs_cosine > config_.right_angle_cosine_max) return false;
    const int p_index = (right_index + 1) % 3;
    const int q_index = (right_index + 2) % 3;
    const cv::Point2f r = candidates[right_index]->center;
    const cv::Point2f p = candidates[p_index]->center;
    const cv::Point2f q = candidates[q_index]->center;
    const float leg_a = pointDistance(r, p);
    const float leg_b = pointDistance(r, q);
    const float diagonal = pointDistance(p, q);
    const float mean_size = (a.outer_size + b.outer_size + c.outer_size) / 3.0F;
    const float leg_scale = 0.5F * (leg_a + leg_b) / std::max(1.0F, mean_size);
    if (std::abs(leg_scale - config_.expected_leg_scale) > config_.leg_scale_tolerance)
        return false;
    const float leg_similarity = std::abs(leg_a - leg_b) / std::max(leg_a, leg_b);
    if (leg_similarity > config_.leg_similarity_tolerance) return false;
    const float pyth_error = std::abs(diagonal * diagonal - leg_a * leg_a - leg_b * leg_b) /
                             std::max(1.0F, leg_a * leg_a + leg_b * leg_b);
    if (pyth_error > config_.pythagorean_relative_error_max) return false;
    const float min_size = std::min({a.outer_size, b.outer_size, c.outer_size});
    const float max_size = std::max({a.outer_size, b.outer_size, c.outer_size});
    if (max_size / std::max(1.0F, min_size) > config_.component_scale_ratio_max) return false;
    const float min_intensity = std::min({a.mean_intensity, b.mean_intensity, c.mean_intensity});
    const float max_intensity = std::max({a.mean_intensity, b.mean_intensity, c.mean_intensity});
    if (max_intensity - min_intensity > config_.component_intensity_delta_max) return false;

    const float topology_score = clamp01(
        config_.topology_shape_subweight * (a.score + b.score + c.score) / 3.0F +
        config_.topology_angle_subweight *
            (1.0F - best_abs_cosine / config_.right_angle_cosine_max) +
        config_.topology_leg_similarity_subweight *
            (1.0F - leg_similarity / config_.leg_similarity_tolerance) +
        config_.topology_pythagorean_subweight *
            (1.0F - pyth_error / config_.pythagorean_relative_error_max) +
        config_.topology_scale_subweight *
            (1.0F - std::abs(leg_scale - config_.expected_leg_scale) /
                        config_.leg_scale_tolerance));

    proposal.image_points = {{r, p, q}};
    proposal.topology_score = topology_score;
    return true;
}

MarkerDetector::Hypothesis MarkerDetector::evaluateTriple(
    const TripleProposal& proposal, int& transform_budget) {
    const auto& canonical = marker_template_.largeLCentersPx();
    Hypothesis best;
    for (int swap = 0; swap < 2 && transform_budget > 0; ++swap) {
        const cv::Point2f& r = proposal.image_points[0];
        const cv::Point2f& p = proposal.image_points[1];
        const cv::Point2f& q = proposal.image_points[2];
        const std::array<cv::Point2f, 3> image_points = swap == 0
            ? std::array<cv::Point2f, 3>{{r, p, q}}
            : std::array<cv::Point2f, 3>{{r, q, p}};
        const cv::Mat affine = cv::getAffineTransform(image_points.data(), canonical.data());
        --transform_budget;
        Hypothesis candidate = evaluateTransform(
            asMatx23f(affine), proposal.topology_score,
            quantizeOrientation(image_points[1] - image_points[0]),
            HypothesisPath::LED_TRIPLE);
        if (candidate.valid && (!best.valid || candidate.total_score > best.total_score)) {
            best = std::move(candidate);
        }
    }
    return best;
}

MarkerDetector::Hypothesis MarkerDetector::findBestLedHypothesis(
    const std::vector<LedCandidate>& candidates, const cv::Rect& /*roi*/) {
    Hypothesis best;
    const std::size_t count = std::min(candidates.size(),
        static_cast<std::size_t>(config_.max_topology_candidates));
    const auto pair_can_belong = [this](const LedCandidate& lhs,
                                        const LedCandidate& rhs) {
        const float min_size = std::min(lhs.outer_size, rhs.outer_size);
        const float max_size = std::max(lhs.outer_size, rhs.outer_size);
        if (max_size / std::max(1.0F, min_size) > config_.component_scale_ratio_max)
            return false;
        if (std::abs(lhs.mean_intensity - rhs.mean_intensity) >
            config_.component_intensity_delta_max) return false;
        const float distance_scale = pointDistance(lhs.center, rhs.center) /
            std::max(1.0F, 0.5F * (lhs.outer_size + rhs.outer_size));
        const float leg_min = config_.expected_leg_scale - config_.leg_scale_tolerance;
        const float leg_max = config_.expected_leg_scale + config_.leg_scale_tolerance;
        const float diagonal_min = std::sqrt(2.0F) * leg_min;
        const float diagonal_max = std::sqrt(2.0F) * leg_max;
        return (distance_scale >= leg_min && distance_scale <= leg_max) ||
               (distance_scale >= diagonal_min && distance_scale <= diagonal_max);
    };

    // All C(n,3) work below is scalar geometry. Only topology-valid proposals
    // reach warpAffine, in descending cheap-score order.
    triple_proposals_.clear();
    const std::size_t maximum_proposals = count >= 3
        ? count * (count - 1) * (count - 2) / 6 : 0;
    triple_proposals_.reserve(maximum_proposals);
    for (std::size_t i = 0; i + 2 < count; ++i) {
        for (std::size_t j = i + 1; j + 1 < count; ++j) {
            for (std::size_t k = j + 1; k < count; ++k) {
                if (!pair_can_belong(candidates[i], candidates[j]) ||
                    !pair_can_belong(candidates[i], candidates[k]) ||
                    !pair_can_belong(candidates[j], candidates[k])) continue;
                TripleProposal proposal;
                if (makeTripleProposal(candidates[i], candidates[j], candidates[k], proposal))
                    triple_proposals_.push_back(proposal);
            }
        }
    }
    std::sort(triple_proposals_.begin(), triple_proposals_.end(),
              [](const TripleProposal& lhs, const TripleProposal& rhs) {
                  return lhs.topology_score > rhs.topology_score;
              });
    int transform_budget = config_.max_led_transform_evaluations;
    for (const auto& proposal : triple_proposals_) {
        if (transform_budget <= 0) break;
        Hypothesis candidate = evaluateTriple(proposal, transform_budget);
        if (candidate.valid && (!best.valid || candidate.total_score > best.total_score))
            best = std::move(candidate);
    }
    return best;
}

MarkerDetector::Hypothesis MarkerDetector::findDarkBoardFallback(const cv::Rect& roi) {
    const cv::Mat source = gray_(roi);
    const double otsu = cv::threshold(source, roi_binary_, 0, 255,
                                      cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    const int threshold = std::min(static_cast<int>(otsu), config_.dark_board_max_threshold);
    cv::threshold(source, roi_binary_, threshold, 255, cv::THRESH_BINARY_INV);
    if (config_.morphology_kernel > 1) {
        cv::morphologyEx(roi_binary_, roi_binary_, cv::MORPH_CLOSE, morphology_kernel_);
    }
    contour_buffer_.clear();
    cv::findContours(roi_binary_, contour_buffer_, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE, roi.tl());

    struct BoardCandidate { cv::RotatedRect box; float score; };
    std::vector<BoardCandidate> boards;
    for (const auto& contour : contour_buffer_) {
        const float area = static_cast<float>(std::abs(cv::contourArea(contour)));
        if (area <= 0.0F) continue;
        const cv::RotatedRect box = cv::minAreaRect(contour);
        const float width = std::max(box.size.width, box.size.height);
        const float height = std::max(1.0F, std::min(box.size.width, box.size.height));
        if (width < config_.min_marker_width_px || width > config_.max_marker_width_px ||
            height < config_.min_marker_width_px *
                         config_.dark_board_min_height_width_fraction) continue;
        const float aspect = width / height;
        if (aspect < config_.dark_board_aspect_min || aspect > config_.dark_board_aspect_max)
            continue;
        const float rectangularity = area / std::max(1.0F, width * height);
        if (rectangularity < config_.dark_board_rectangularity_min) continue;
        const cv::Rect bounds = clampRect(cv::boundingRect(contour), gray_.size());
        if (bounds.area() <= 0) continue;
        const double mean_intensity = cv::mean(gray_(bounds))[0];
        double max_intensity = 0.0;
        cv::minMaxLoc(gray_(bounds), nullptr, &max_intensity);
        const float bright_delta = static_cast<float>(max_intensity - mean_intensity);
        if (bright_delta < config_.fallback_min_candidate_bright_delta) continue;
        const float bright_range_score = clamp01(
            bright_delta / std::max(1.0F, config_.contrast_score_full_scale *
                config_.fallback_min_candidate_bright_delta));
        const float board_score =
            config_.fallback_board_rectangularity_weight * clamp01(rectangularity) +
            config_.fallback_board_bright_range_weight * bright_range_score;
        boards.push_back({box, board_score});
    }
    std::sort(boards.begin(), boards.end(), [](const BoardCandidate& lhs,
                                                const BoardCandidate& rhs) {
        return lhs.score > rhs.score;
    });
    if (boards.size() > static_cast<std::size_t>(config_.max_board_candidates))
        boards.resize(static_cast<std::size_t>(config_.max_board_candidates));

    Hypothesis best;
    int transform_budget = config_.max_board_transform_evaluations;
    const auto canonical = marker_template_.geometry().boardCornersCanonical(
        config_.canonical_size);
    for (const auto& board : boards) {
        if (transform_budget <= 0) break;
        std::array<cv::Point2f, 4> points{};
        board.box.points(points.data());
        const cv::Point2f centre = board.box.center;
        std::sort(points.begin(), points.end(), [centre](const cv::Point2f& lhs,
                                                        const cv::Point2f& rhs) {
            return std::atan2(lhs.y - centre.y, lhs.x - centre.x) <
                   std::atan2(rhs.y - centre.y, rhs.x - centre.x);
        });
        for (int offset = 0; offset < 4 && transform_budget > 0; ++offset) {
            const std::array<cv::Point2f, 3> image_points{{
                points[static_cast<std::size_t>(offset)],
                points[static_cast<std::size_t>((offset + 1) & 3)],
                points[static_cast<std::size_t>((offset + 3) & 3)]}};
            const std::array<cv::Point2f, 3> canonical_points{{canonical[0], canonical[1],
                                                               canonical[3]}};
            const cv::Mat affine = cv::getAffineTransform(image_points.data(),
                                                           canonical_points.data());
            --transform_budget;
            Hypothesis candidate = evaluateTransform(
                asMatx23f(affine), config_.fallback_topology_score * board.score,
                quantizeOrientation(image_points[1] - image_points[0]),
                HypothesisPath::DARK_BOARD);
            if (candidate.valid && (!best.valid || candidate.total_score > best.total_score)) {
                best = std::move(candidate);
            }
        }
    }
    return best;
}

float MarkerDetector::temporalScore(const Hypothesis& hypothesis,
                                    SteadyTimePoint capture_timestamp) const {
    if (!state_.has_observation) return config_.no_history_temporal_score;
    double dt = std::chrono::duration<double>(capture_timestamp - state_.last_capture).count();
    dt = std::clamp(dt, 0.0, static_cast<double>(config_.prediction_horizon_seconds));
    const cv::Point2f predicted = state_.last_center +
        state_.velocity_px_per_second * static_cast<float>(dt);
    const float normalizer = std::max(config_.temporal_min_normalizer_px,
                                      state_.last_bbox.width *
                                          config_.max_tracking_displacement_bbox);
    return clamp01(1.0F - pointDistance(hypothesis.center, predicted) / normalizer);
}

DetectionResult MarkerDetector::makeResult(const Hypothesis& hypothesis, float confidence,
                                           SteadyTimePoint capture_timestamp) {
    DetectionResult result;
    result.capture_timestamp = capture_timestamp;
    result.confidence = clamp01(confidence);
    result.state = state_.mode;
    float threshold = state_.mode == DetectorState::TRACK_ROI
                                ? config_.track_exit_threshold
                                : config_.trackable_threshold;
    if (hypothesis.path == HypothesisPath::DARK_BOARD) {
        threshold = std::max(threshold, config_.fallback_trackable_threshold);
    }
    const int additional_components = hypothesis.detail.matched_components -
                                      hypothesis.detail.matched_large_components;
    const bool primary_additional_validation =
        additional_components >= config_.min_additional_led_components ||
        hypothesis.detail.black_board_score >= config_.min_black_board_validation_score;
    const bool primary_pattern_valid =
        hypothesis.path == HypothesisPath::LED_TRIPLE &&
        hypothesis.detail.matched_large_components == 3 && primary_additional_validation;
    const bool fallback_pattern_valid =
        hypothesis.path == HypothesisPath::DARK_BOARD &&
        hypothesis.detail.matched_large_components >= config_.fallback_min_large_components &&
        hypothesis.detail.matched_components >= config_.fallback_min_total_components &&
        hypothesis.detail.template_score >= config_.fallback_min_template_score &&
        hypothesis.detail.contrast_score >= config_.fallback_min_contrast_score &&
        hypothesis.detail.black_board_score >= config_.fallback_min_black_board_score;
    result.found = hypothesis.valid && (primary_pattern_valid || fallback_pattern_valid) &&
                   hypothesis.detail.contrast_score > config_.min_detection_contrast_score &&
                   result.confidence >= threshold;
    if (result.found) {
        result.bbox = hypothesis.bbox & cv::Rect2f(0.0F, 0.0F,
                                                    static_cast<float>(gray_.cols),
                                                    static_cast<float>(gray_.rows));
        result.center = hypothesis.center;
        result.error_x_px = result.center.x - config_.optical_center_x;
        result.error_x_norm = result.error_x_px /
                              std::max(1.0F, 0.5F * static_cast<float>(config_.frame_width));
        result.matched_components = hypothesis.detail.matched_components;
        result.saturation_fraction = hypothesis.detail.saturation_fraction;
        const int matched_small = hypothesis.detail.matched_components -
                                  hypothesis.detail.matched_large_components;
        const int required_full_id_components = hypothesis.path == HypothesisPath::DARK_BOARD
            ? config_.fallback_full_id_min_components : config_.full_id_min_components;
        if (result.confidence >= config_.full_id_threshold &&
            result.matched_components >= required_full_id_components &&
            matched_small >= 2) {
            result.quality = DetectionQuality::FULL_ID;
            result.orientation = hypothesis.orientation;
        } else {
            result.quality = DetectionQuality::TRACKABLE;
            result.orientation = MarkerOrientation::UNKNOWN;
        }
    }
    return result;
}

void MarkerDetector::updateState(const DetectionResult& result,
                                 SteadyTimePoint capture_timestamp) {
    if (result.found) {
        bool consistent = true;
        if (state_.has_observation) {
            const float allowance = std::max(result.bbox.width, state_.last_bbox.width) *
                                    config_.max_tracking_displacement_bbox;
            consistent = pointDistance(result.center, state_.last_center) <= allowance;
            const double dt = std::chrono::duration<double>(capture_timestamp -
                                                             state_.last_capture).count();
            if (dt > 1e-4 && dt < config_.velocity_update_max_dt_seconds) {
                const cv::Point2f measured_velocity =
                    (result.center - state_.last_center) * static_cast<float>(1.0 / dt);
                state_.velocity_px_per_second =
                    state_.velocity_px_per_second * config_.previous_velocity_weight +
                    measured_velocity * (1.0F - config_.previous_velocity_weight);
            }
        }
        if (state_.mode == DetectorState::SEARCH_FULL) {
            if (result.confidence >= config_.search_strong_threshold) {
                state_.confirmation_count = consistent ? state_.confirmation_count + 1 : 1;
            } else {
                state_.confirmation_count = 0;
            }
            if (state_.confirmation_count >= config_.search_confirm_frames) {
                state_.mode = DetectorState::TRACK_ROI;
                state_.lost_count = 0;
            }
        } else {
            // A verified result above the exit threshold is still a real
            // observation, but repeated weak results below the hold threshold
            // deliberately return the state machine to full search.
            state_.lost_count = result.confidence >= config_.track_hold_threshold
                                    ? 0 : state_.lost_count + 1;
        }
        state_.has_observation = true;
        state_.last_center = result.center;
        state_.last_bbox = result.bbox;
        state_.last_capture = capture_timestamp;
        if (state_.mode == DetectorState::TRACK_ROI &&
            state_.lost_count >= config_.return_to_search_lost_frames) {
            state_.mode = DetectorState::SEARCH_FULL;
            state_.confirmation_count = 0;
            state_.lost_count = 0;
            state_.has_observation = false;
            state_.velocity_px_per_second = {};
        }
    } else if (state_.mode == DetectorState::TRACK_ROI) {
        ++state_.lost_count;
        if (state_.lost_count >= config_.return_to_search_lost_frames) {
            state_.mode = DetectorState::SEARCH_FULL;
            state_.confirmation_count = 0;
            state_.lost_count = 0;
            state_.has_observation = false;
            state_.velocity_px_per_second = {};
        }
    } else {
        state_.confirmation_count = 0;
        state_.has_observation = false;
        state_.velocity_px_per_second = cv::Point2f{};
    }
}

DetectionResult MarkerDetector::process(const cv::Mat& frame,
                                        SteadyTimePoint capture_timestamp) {
    const SteadyTimePoint processing_start = std::chrono::steady_clock::now();
    if (debug_enabled_) {
        debug_snapshot_.search_roi = {};
        debug_snapshot_.led_threshold = 0;
        debug_snapshot_.raw_contour_count = 0;
        debug_snapshot_.led_mask.release();
        debug_snapshot_.candidates.clear();
    }
    makeGray(frame);
    Hypothesis best;
    float confidence = 0.0F;
    if (!gray_.empty()) {
        const cv::Rect roi = trackingSearchRoi(capture_timestamp);
        if (roi.width >= config_.min_marker_width_px &&
            roi.height >= config_.min_marker_width_px) {
            const auto candidates = findLedCandidates(roi);
            best = findBestLedHypothesis(candidates, roi);
            if (!best.valid || best.total_score < config_.trackable_threshold) {
                Hypothesis fallback = findDarkBoardFallback(roi);
                if (fallback.valid && (!best.valid || fallback.total_score > best.total_score)) {
                    best = std::move(fallback);
                }
            }
            if (best.valid) {
                confidence = best.total_score +
                    config_.temporal_weight * temporalScore(best, capture_timestamp);
            }
        }
    }
    DetectionResult result = makeResult(best, confidence, capture_timestamp);
    updateState(result, capture_timestamp);
    result.state = state_.mode;
    // Refresh output time after state bookkeeping so latency covers the complete
    // detector call, not just image processing.
    result.output_timestamp = std::chrono::steady_clock::now();
    result.processing_ms = std::chrono::duration<double, std::milli>(
        result.output_timestamp - processing_start).count();
    result.capture_to_output_latency_ms = std::max(0.0,
        std::chrono::duration<double, std::milli>(result.output_timestamp - capture_timestamp).count());
    if (previous_output_ != SteadyTimePoint{}) {
        const double interval = std::chrono::duration<double>(result.output_timestamp -
                                                              previous_output_).count();
        if (interval > 0.0) result.effective_detection_fps = 1.0 / interval;
    }
    previous_output_ = result.output_timestamp;
    if (config_.benchmark_enabled) {
        benchmark_.add(result.processing_ms, result.capture_to_output_latency_ms,
                       result.effective_detection_fps, result.found, frame.empty());
    }
    return result;
}

}  // namespace maixcam_marker
