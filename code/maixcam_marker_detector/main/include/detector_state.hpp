#pragma once

#include <array>
#include <chrono>

#include <opencv2/core.hpp>

#include "detection_result.hpp"

namespace maixcam_marker {

struct DetectorStateData {
    DetectorState mode = DetectorState::SEARCH_FULL;
    int confirmation_count = 0;
    int lost_count = 0;
    bool has_observation = false;
    cv::Point2f last_center{};
    cv::Point2f velocity_px_per_second{};
    cv::Rect2f last_bbox{};
    bool has_large_l_identity = false;
    std::array<cv::Point2f, 3> last_large_l_centers{};
    int canonical_x_label = -1;
    SteadyTimePoint last_capture{};

    void reset() noexcept { *this = DetectorStateData{}; }
};

}  // namespace maixcam_marker
