#include "marker_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace maixcam_marker {
namespace {

MarkerComponent makeComponent(ComponentKind kind, cv::Point2f bounding_box_center,
                              std::vector<cv::Point2f> polygon) {
    MarkerComponent result;
    result.kind = kind;
    result.polygon_mm = std::move(polygon);
    const cv::Moments moments = cv::moments(result.polygon_mm);
    result.area_mm2 = static_cast<float>(std::abs(moments.m00));
    // Topology is defined by the component bounding-box centres from CAD, not
    // polygon centroids (an L's centroid is displaced toward its corner).
    result.center_mm = bounding_box_center;
    return result;
}

}  // namespace

MarkerGeometry::MarkerGeometry() {
    // Exact XY projection of object 1 in marker.3mf (the model uses X/Z for
    // this face). Coordinates are millimetres about the LED-pattern centre.
    // Preserve component order from the CAD mesh: three large Ls, small L,
    // upper code square, right code square.
    components_[0] = makeComponent(ComponentKind::LARGE_L, {-25.0F, 25.0F},
        {{-40.0F, 10.0F}, {-32.0F, 10.0F}, {-32.0F, 32.0F},
         {-10.0F, 32.0F}, {-10.0F, 40.0F}, {-40.0F, 40.0F}});
    components_[1] = makeComponent(ComponentKind::LARGE_L, {25.0F, -25.0F},
        {{10.0F, -40.0F}, {40.0F, -40.0F}, {40.0F, -10.0F},
         {32.0F, -10.0F}, {32.0F, -32.0F}, {10.0F, -32.0F}});
    components_[2] = makeComponent(ComponentKind::LARGE_L, {-25.0F, -25.0F},
        {{-40.0F, -40.0F}, {-10.0F, -40.0F}, {-10.0F, -32.0F},
         {-32.0F, -32.0F}, {-32.0F, -10.0F}, {-40.0F, -10.0F}});
    components_[3] = makeComponent(ComponentKind::SMALL_L, {33.0F, 33.0F},
        {{26.0F, 32.0F}, {26.0F, 40.0F}, {40.0F, 40.0F},
         {40.0F, 26.0F}, {32.0F, 26.0F}, {32.0F, 32.0F}});
    components_[4] = makeComponent(ComponentKind::SQUARE, {14.0F, 36.0F},
        {{10.0F, 32.0F}, {18.0F, 32.0F}, {18.0F, 40.0F}, {10.0F, 40.0F}});
    components_[5] = makeComponent(ComponentKind::SQUARE, {36.0F, 14.0F},
        {{32.0F, 10.0F}, {40.0F, 10.0F}, {40.0F, 18.0F}, {32.0F, 18.0F}});
}

std::array<cv::Point2f, 3> MarkerGeometry::largeLCentersMm() const noexcept {
    return {components_[0].center_mm, components_[1].center_mm, components_[2].center_mm};
}

std::array<cv::Point2f, 4> MarkerGeometry::boardCornersCanonical(
    int canvas_size) const noexcept {
    const float half_width = 0.5F * kBoardWidthMm;
    const float half_height = 0.5F * kBoardHeightMm;
    const cv::Point2f board_center = boardCenterMm();
    return {mmToCanonical(board_center + cv::Point2f{-half_width, -half_height},
                          canvas_size),
            mmToCanonical(board_center + cv::Point2f{half_width, -half_height},
                          canvas_size),
            mmToCanonical(board_center + cv::Point2f{half_width, half_height},
                          canvas_size),
            mmToCanonical(board_center + cv::Point2f{-half_width, half_height},
                          canvas_size)};
}

cv::Point2f MarkerGeometry::mmToCanonical(const cv::Point2f& point_mm,
                                           int canvas_size) const noexcept {
    const float scale = static_cast<float>(canvas_size) /
                        std::max(kBoardWidthMm, kBoardHeightMm);
    const cv::Point2f centre{0.5F * static_cast<float>(canvas_size - 1),
                            0.5F * static_cast<float>(canvas_size - 1)};
    return centre + (point_mm - boardCenterMm()) * scale;
}

std::vector<cv::Point> MarkerGeometry::polygonToCanonical(
    const MarkerComponent& component, int canvas_size) const {
    std::vector<cv::Point> result;
    result.reserve(component.polygon_mm.size());
    for (const auto& point : component.polygon_mm) {
        const cv::Point2f mapped = mmToCanonical(point, canvas_size);
        result.emplace_back(cvRound(mapped.x), cvRound(mapped.y));
    }
    return result;
}

}  // namespace maixcam_marker
