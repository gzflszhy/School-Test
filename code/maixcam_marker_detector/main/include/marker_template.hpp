#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include "detector_config.hpp"
#include "marker_geometry.hpp"

namespace maixcam_marker {

struct TemplateScore {
    float template_score = 0.0F;
    float large_l_template_score = 0.0F;
    float optional_template_score = 0.0F;
    float contrast_score = 0.0F;
    float black_board_score = 0.0F;
    float extra_bright_fraction = 1.0F;
    float saturation_fraction = 0.0F;
    int matched_components = 0;
    int matched_large_components = 0;
    std::array<float, MarkerGeometry::kComponentCount> coverage{};
};

class MarkerTemplate {
public:
    explicit MarkerTemplate(const DetectorConfig& config,
                            MarkerGeometry geometry = MarkerGeometry{});

    const cv::Mat& ledMask() const noexcept { return led_mask_; }
    const cv::Mat& boardMask() const noexcept { return board_mask_; }
    const cv::Mat& largeLShapeMask() const noexcept { return large_l_shape_mask_; }
    const std::vector<cv::Point>& largeLShapeContour() const noexcept {
        return large_l_shape_contour_;
    }
    const MarkerGeometry& geometry() const noexcept { return geometry_; }
    // Topology order is right-angle vertex, +X leg, +Y leg. It intentionally
    // differs from the CAD component order returned by MarkerGeometry.
    const std::array<cv::Point2f, 3>& largeLCentersPx() const noexcept {
        return large_l_centers_px_;
    }

    TemplateScore score(const cv::Mat& canonical_gray, cv::Mat& scratch_binary) const;

private:
    DetectorConfig config_;
    MarkerGeometry geometry_;
    cv::Mat led_mask_;
    cv::Mat large_l_mask_;
    cv::Mat board_mask_;
    cv::Mat large_l_shape_mask_;
    std::vector<cv::Point> large_l_shape_contour_;
    std::array<cv::Mat, MarkerGeometry::kComponentCount> component_masks_{};
    std::array<int, MarkerGeometry::kComponentCount> component_pixel_counts_{};
    int board_pixel_count_ = 1;
    std::array<cv::Point2f, 3> large_l_centers_px_{};
    mutable cv::Mat otsu_scratch_;
    mutable cv::Mat overlap_scratch_;
    mutable cv::Mat dark_scratch_;
    mutable cv::Mat outside_scratch_;
    mutable cv::Mat saturated_scratch_;
};

}  // namespace maixcam_marker
