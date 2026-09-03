#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace maixcam_marker {

enum class ComponentKind { LARGE_L, SMALL_L, SQUARE };

struct MarkerComponent {
    ComponentKind kind = ComponentKind::SQUARE;
    std::vector<cv::Point2f> polygon_mm;
    cv::Point2f center_mm{};
    float area_mm2 = 0.0F;
};

class MarkerGeometry {
public:
    static constexpr float kBoardWidthMm = 115.0F;
    static constexpr float kBoardHeightMm = 120.0F;
    static constexpr float kPatternWidthMm = 80.0F;
    static constexpr float kPatternHeightMm = 80.0F;
    // marker.3mf places the LED object at Z=+2.5 mm relative to the board.
    // Geometry coordinates retain the LED-local values, so the board centre
    // is at Y=-2.5 mm in this 2-D projection.
    static constexpr float kBoardCenterXMm = 0.0F;
    static constexpr float kBoardCenterYMm = -2.5F;
    static constexpr std::size_t kComponentCount = 6;

    MarkerGeometry();

    const std::array<MarkerComponent, kComponentCount>& components() const noexcept {
        return components_;
    }
    std::array<cv::Point2f, 3> largeLCentersMm() const noexcept;
    cv::Point2f boardCenterMm() const noexcept {
        return {kBoardCenterXMm, kBoardCenterYMm};
    }
    std::array<cv::Point2f, 4> boardCornersCanonical(int canvas_size) const noexcept;
    cv::Point2f mmToCanonical(const cv::Point2f& point_mm, int canvas_size) const noexcept;
    std::vector<cv::Point> polygonToCanonical(const MarkerComponent& component,
                                               int canvas_size) const;

private:
    std::array<MarkerComponent, kComponentCount> components_{};
};

}  // namespace maixcam_marker
