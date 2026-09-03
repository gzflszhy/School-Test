#pragma once

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
    SteadyTimePoint last_capture{};

    void reset() noexcept { *this = DetectorStateData{}; }
};

}  // namespace maixcam_marker
