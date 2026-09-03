#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "maix_basic.hpp"
#include "maix_display.hpp"
#include "maix_image.hpp"

#include <opencv2/imgproc.hpp>

#include "detector_config.hpp"
#include "maix_camera_source.hpp"
#include "marker_detector.hpp"

namespace {

using namespace maixcam_marker;

struct RuntimeOptions {
    bool debug = false;
    bool debug_display = false;
};

RuntimeOptions parseOptions(int argc, char** argv) {
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view argument(argv[i]);
        if (argument == "--debug") {
            options.debug = true;
        } else if (argument == "--debug-display") {
            options.debug = true;
            options.debug_display = true;
        } else if (argument == "--help") {
            std::cout << "Usage: maixcam_marker_detector [--debug|--debug-display]\n"
                         "  --debug          stream annotated frames to MaixVision\n"
                         "  --debug-display  show on the device display and MaixVision\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + std::string(argument));
        }
    }
    return options;
}

std::int64_t steadyMicros(SteadyTimePoint timestamp) {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               timestamp.time_since_epoch())
        .count();
}

void writeJsonl(const DetectionResult& result, std::uint64_t frame_sequence) {
    // Values are written directly because every string enum is produced by a
    // closed local enum-to-string conversion; no untrusted string is encoded.
    std::cout << "{\"frame_sequence\":" << frame_sequence
              << ",\"found\":" << (result.found ? "true" : "false")
              << ",\"quality\":\"" << toString(result.quality)
              << "\",\"confidence\":" << result.confidence
              << ",\"bbox_x\":" << result.bbox.x
              << ",\"bbox_y\":" << result.bbox.y
              << ",\"bbox_w\":" << result.bbox.width
              << ",\"bbox_h\":" << result.bbox.height
              << ",\"center_x\":" << result.center.x
              << ",\"center_y\":" << result.center.y
              << ",\"error_x_px\":" << result.error_x_px
              << ",\"error_x_norm\":" << result.error_x_norm
              << ",\"orientation\":\"" << toString(result.orientation)
              << "\",\"capture_timestamp_us\":" << steadyMicros(result.capture_timestamp)
              << ",\"output_timestamp_us\":" << steadyMicros(result.output_timestamp)
              << ",\"capture_to_output_latency_ms\":"
              << result.capture_to_output_latency_ms
              << ",\"processing_ms\":" << result.processing_ms
              << ",\"effective_detection_fps\":" << result.effective_detection_fps
              << ",\"state\":\"" << toString(result.state)
              << "\",\"saturation_fraction\":" << result.saturation_fraction
              << ",\"matched_components\":" << result.matched_components
              << ",\"matched_large_components\":" << result.matched_large_components
              << ",\"matched_optional_components\":"
              << result.matched_optional_components
              << ",\"optional_features\":[";
    constexpr std::array<std::string_view, 3> optional_names{
        "small_l", "square_0", "square_1"};
    for (std::size_t i = 0; i < optional_names.size(); ++i) {
        if (i != 0) std::cout << ',';
        std::cout << "{\"name\":\"" << optional_names[i]
                  << "\",\"found\":"
                  << (result.optional_component_detected[i] ? "true" : "false")
                  << ",\"center_x\":" << result.optional_component_centers[i].x
                  << ",\"center_y\":" << result.optional_component_centers[i].y
                  << '}';
    }
    std::cout << ']'
              << "}\n" << std::flush;
}

void showDebugFrame(const cv::Mat& gray, const DetectionResult& result,
                    const DetectorDebugSnapshot& debug,
                    maix::display::Display* display) {
    cv::Mat annotated;
    cv::cvtColor(gray, annotated, cv::COLOR_GRAY2RGB);

    if (debug.search_roi.area() > 0) {
        cv::rectangle(annotated, debug.search_roi, cv::Scalar(255, 255, 0), 1);
    }
    for (const auto& candidate : debug.candidates) {
        cv::Point2f corners[4];
        candidate.points(corners);
        for (int i = 0; i < 4; ++i) {
            const cv::Point start(cvRound(corners[i].x), cvRound(corners[i].y));
            const cv::Point end(cvRound(corners[(i + 1) % 4].x),
                                cvRound(corners[(i + 1) % 4].y));
            cv::line(annotated, start, end,
                     cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }
        if (display != nullptr) {
            const cv::Point candidate_center(cvRound(candidate.center.x),
                                             cvRound(candidate.center.y));
            cv::circle(annotated, candidate_center, 2, cv::Scalar(255, 0, 255),
                       cv::FILLED, cv::LINE_AA);
        }
    }
    if (display != nullptr && debug.selected_triplet_valid) {
        std::array<cv::Point, 3> centers{};
        for (std::size_t i = 0; i < centers.size(); ++i) {
            centers[i] = cv::Point(
                cvRound(debug.selected_large_l_centers[i].x),
                cvRound(debug.selected_large_l_centers[i].y));
        }
        cv::line(annotated, centers[0], centers[1], cv::Scalar(0, 255, 0), 2,
                 cv::LINE_AA);
        cv::line(annotated, centers[0], centers[2], cv::Scalar(0, 255, 0), 2,
                 cv::LINE_AA);
        cv::line(annotated, centers[1], centers[2], cv::Scalar(0, 255, 0), 2,
                 cv::LINE_AA);
        constexpr std::array<const char*, 3> labels{{"R", "A", "B"}};
        for (std::size_t i = 0; i < centers.size(); ++i) {
            cv::circle(annotated, centers[i], 5, cv::Scalar(255, 64, 64), 2,
                       cv::LINE_AA);
            cv::putText(annotated, labels[i], centers[i] + cv::Point(6, -6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 64, 64), 2,
                        cv::LINE_AA);
        }
    }
    if (display != nullptr && result.found) {
        constexpr std::array<const char*, 3> optional_labels{{"S", "Q0", "Q1"}};
        for (std::size_t i = 0; i < optional_labels.size(); ++i) {
            if (!result.optional_component_detected[i]) continue;
            const cv::Point center(cvRound(result.optional_component_centers[i].x),
                                   cvRound(result.optional_component_centers[i].y));
            cv::drawMarker(annotated, center, cv::Scalar(64, 128, 255),
                           cv::MARKER_TILTED_CROSS, 10, 2);
            cv::putText(annotated, optional_labels[i], center + cv::Point(5, 12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(64, 128, 255), 1,
                        cv::LINE_AA);
        }
    }
    if (result.found) {
        const cv::Rect detected_box(cvRound(result.bbox.x), cvRound(result.bbox.y),
                                    cvRound(result.bbox.width), cvRound(result.bbox.height));
        const cv::Point detected_center(cvRound(result.center.x), cvRound(result.center.y));
        cv::rectangle(annotated, detected_box, cv::Scalar(0, 255, 0), 2);
        cv::drawMarker(annotated, detected_center, cv::Scalar(0, 255, 0),
                       cv::MARKER_CROSS, 14, 2);
    }

    std::ostringstream status;
    status << (result.found ? "FOUND " : "LOST ") << toString(result.quality)
           << " conf=" << static_cast<int>(result.confidence * 100.0F)
           << "% cand=" << debug.candidates.size()
           << "/" << debug.raw_contour_count
           << " L=" << result.matched_large_components
           << " opt=" << result.matched_optional_components
           << " thr=" << debug.led_threshold
           << " fps=" << static_cast<int>(result.effective_detection_fps + 0.5);
    cv::rectangle(annotated, cv::Rect(0, 0, annotated.cols, 24),
                  cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(annotated, status.str(), cv::Point(5, 17),
                cv::FONT_HERSHEY_SIMPLEX, 0.43, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);

    cv::Mat full_mask = cv::Mat::zeros(gray.size(), CV_8UC1);
    if (!debug.led_mask.empty() && debug.search_roi.area() > 0 &&
        debug.led_mask.size() == debug.search_roi.size()) {
        debug.led_mask.copyTo(full_mask(debug.search_roi));
    }
    cv::Mat mask_rgb;
    cv::cvtColor(full_mask, mask_rgb, cv::COLOR_GRAY2RGB);
    cv::putText(mask_rgb, "LED threshold mask", cv::Point(5, 17),
                cv::FONT_HERSHEY_SIMPLEX, 0.43, cv::Scalar(255, 255, 0), 1,
                cv::LINE_AA);

    cv::Mat canvas;
    cv::hconcat(annotated, mask_rgb, canvas);
    maix::image::Image image(canvas.cols, canvas.rows,
                             maix::image::Format::FMT_RGB888, canvas.data,
                             static_cast<int>(canvas.total() * canvas.elemSize()), false);
    if (display != nullptr) {
        display->show(image, maix::image::Fit::FIT_CONTAIN);
    } else {
        maix::display::send_to_maixvision(image);
    }
}

int run(const RuntimeOptions& options) {
    DetectorConfig detector_config;
    detector_config.normalize();

    CameraConfig camera_config;
    camera_config.width = detector_config.frame_width;
    camera_config.height = detector_config.frame_height;
    camera_config.fps = static_cast<double>(detector_config.requested_fps);
    camera_config.buffer_count = detector_config.camera_buffer_count;
    camera_config.warmup_frames = detector_config.camera_warmup_frames;
    camera_config.use_auto_exposure = detector_config.use_auto_exposure;
    camera_config.manual_exposure_us = detector_config.exposure_us;
    camera_config.prefer_latest_frame = true;

    MaixCameraSource camera(camera_config);
    MarkerDetector detector(detector_config);
    detector.setDebugEnabled(options.debug);
    detector.setDebugCenterOverlayEnabled(options.debug_display);
    std::unique_ptr<maix::display::Display> display;
    if (options.debug_display) {
        display = std::make_unique<maix::display::Display>();
    }
    CameraFrame frame;
    std::uint64_t consecutive_empty_reads = 0;

    std::cerr << "maixcam_marker_detector: camera " << camera_config.width << "x"
              << camera_config.height << " @ " << camera_config.fps
              << " FPS, grayscale, buff_num=" << camera_config.buffer_count << '\n';

    while (!maix::app::need_exit()) {
        if (!camera.read(frame)) {
            ++consecutive_empty_reads;
            detector.recordDroppedFrame();
            // Do not contaminate stdout: it is an exclusively JSONL data API.
            // Rate-limit the exceptional path to avoid log-induced latency.
            if (consecutive_empty_reads == 1 || consecutive_empty_reads % 120 == 0) {
                std::cerr << "camera read failed: " << camera.last_error() << '\n';
            }
            maix::time::sleep_ms(10);
            continue;
        }
        consecutive_empty_reads = 0;

        // frame owns the MaixCDK Image backing frame.gray().  process must
        // complete before the next read replaces (and releases) that image.
        const DetectionResult result = detector.process(frame.gray(), frame.capture_time());
        if (options.debug) {
            showDebugFrame(frame.gray(), result, detector.debugSnapshot(), display.get());
        }
        writeJsonl(result, frame.sequence());
    }

    std::cerr << detector.benchmark().summary() << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    maix::sys::register_default_signal_handle();
    try {
        return run(parseOptions(argc, argv));
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}
