#pragma once

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include "detector_config.hpp"
#include "marker_geometry.hpp"

namespace maixcam_marker {

struct TemplateScore {
    float large_l_template_score = 0.0F;
    float optional_template_score = 0.0F;
    float contrast_score = 0.0F;
    float saturation_fraction = 0.0F;
    int matched_optional_components = 0;
    std::array<bool, MarkerGeometry::kComponentCount> component_detected{};
    std::array<cv::Point2f, MarkerGeometry::kComponentCount> component_centers_canonical{};
};

class MarkerTemplate {
public:
    explicit MarkerTemplate(const DetectorConfig& config,
                            MarkerGeometry geometry = MarkerGeometry{});

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
    cv::Mat large_l_mask_;
    cv::Mat background_mask_;
    std::vector<cv::Point> large_l_shape_contour_;
    std::array<cv::Mat, MarkerGeometry::kComponentCount> component_masks_{};
    std::array<int, MarkerGeometry::kComponentCount> component_pixel_counts_{};
    std::array<cv::Point2f, 3> large_l_centers_px_{};
    mutable cv::Mat otsu_scratch_;
    mutable cv::Mat overlap_scratch_;
    mutable cv::Mat saturated_scratch_;
};

}  // namespace maixcam_marker
