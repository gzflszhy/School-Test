#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <string_view>

#include <opencv2/core.hpp>

namespace maixcam_marker {

enum class DetectionQuality : std::uint8_t { NOT_FOUND = 0, TRACKABLE = 1, FULL_ID = 2 };
enum class MarkerOrientation : std::int16_t { UNKNOWN = -1, DEG_0 = 0, DEG_90 = 90,
                                              DEG_180 = 180, DEG_270 = 270 };
enum class DetectorState : std::uint8_t { SEARCH_FULL = 0, TRACK_ROI = 1 };

using SteadyTimePoint = std::chrono::steady_clock::time_point;

struct DetectionResult {
    bool found = false;
    DetectionQuality quality = DetectionQuality::NOT_FOUND;
    float confidence = 0.0F;
    cv::Rect2f bbox{};
    cv::Point2f center{-1.0F, -1.0F};
    float error_x_px = 0.0F;
    float error_x_norm = 0.0F;
    MarkerOrientation orientation = MarkerOrientation::UNKNOWN;
    SteadyTimePoint capture_timestamp{};
    SteadyTimePoint output_timestamp{};
    double capture_to_output_latency_ms = 0.0;
    double processing_ms = 0.0;
    double effective_detection_fps = 0.0;
    DetectorState state = DetectorState::SEARCH_FULL;
    float saturation_fraction = 0.0F;
    int matched_components = 0;
    int matched_large_components = 0;
    int matched_optional_components = 0;
    std::array<bool, 3> optional_component_detected{};
    std::array<cv::Point2f, 3> optional_component_centers{
        cv::Point2f{-1.0F, -1.0F}, cv::Point2f{-1.0F, -1.0F},
        cv::Point2f{-1.0F, -1.0F}};
};

constexpr std::string_view toString(DetectionQuality value) noexcept {
    switch (value) {
        case DetectionQuality::TRACKABLE: return "TRACKABLE";
        case DetectionQuality::FULL_ID: return "FULL_ID";
        default: return "NOT_FOUND";
    }
}

constexpr std::string_view toString(DetectorState value) noexcept {
    return value == DetectorState::TRACK_ROI ? "TRACK_ROI" : "SEARCH_FULL";
}

constexpr std::string_view toString(MarkerOrientation value) noexcept {
    switch (value) {
        case MarkerOrientation::DEG_0: return "0";
        case MarkerOrientation::DEG_90: return "90";
        case MarkerOrientation::DEG_180: return "180";
        case MarkerOrientation::DEG_270: return "270";
        default: return "unknown";
    }
}

}  // namespace maixcam_marker
