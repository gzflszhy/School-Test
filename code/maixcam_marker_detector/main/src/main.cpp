#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <iomanip>
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
#include "avc1_uart_sender.hpp"
#include "lateral_controller.hpp"
#include "maix_camera_source.hpp"
#include "marker_detector.hpp"

namespace {

using namespace maixcam_marker;

struct RuntimeOptions {
    bool debug = false;
    bool debug_display = false;
    std::string detector_config_path;
    std::string control_config_path;
    std::string chassis_uart_path;
    float chassis_vy_limit_mps = 0.10F;
};

RuntimeOptions parseOptions(int argc, char** argv) {
    RuntimeOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view argument(argv[i]);
        if (argument == "--debug") {
            options.debug = true;
        } else if (argument == "--debug-display" || argument == "--display") {
            options.debug = true;
            options.debug_display = true;
        } else if (argument == "--detector-config") {
            if (++i >= argc) throw std::runtime_error("--detector-config requires a path");
            options.detector_config_path = argv[i];
        } else if (argument == "--control-config") {
            if (++i >= argc) throw std::runtime_error("--control-config requires a path");
            options.control_config_path = argv[i];
        } else if (argument == "--chassis-uart") {
            if (++i >= argc) throw std::runtime_error("--chassis-uart requires a path");
            options.chassis_uart_path = argv[i];
        } else if (argument == "--chassis-vy-limit") {
            if (++i >= argc) {
                throw std::runtime_error("--chassis-vy-limit requires m/s");
            }
            std::size_t consumed = 0;
            options.chassis_vy_limit_mps = std::stof(argv[i], &consumed);
            if (consumed != std::string(argv[i]).size() ||
                !std::isfinite(options.chassis_vy_limit_mps) ||
                options.chassis_vy_limit_mps <= 0.0F ||
                options.chassis_vy_limit_mps > 0.8F) {
                throw std::runtime_error(
                    "--chassis-vy-limit must be in (0, 0.8]");
            }
        } else if (argument == "--help") {
            std::cout << "Usage: maixcam_marker_detector [--debug|--debug-display|--display] "
                         "[--detector-config PATH] [--control-config PATH] [--chassis-uart PATH] "
                         "[--chassis-vy-limit MPS]\n"
                         "  --debug          stream annotated frames to MaixVision\n"
                         "  --debug-display  show on the device display and MaixVision\n"
                         "  --display        alias for --debug-display\n"
                         "  --detector-config load exposure/bright extraction overrides\n"
                         "  --control-config load lateral tracking key=value overrides\n"
                         "  --chassis-uart   enable AVC1 output on this UART (e.g. /dev/ttyS1)\n"
                         "  --chassis-vy-limit independent transmit limit, default 0.10 m/s\n";
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

void writeJsonl(const DetectionResult& result, const LateralControlOutput& control,
                const Avc1TransmitResult& chassis_tx,
                const DetectorDebugSnapshot& debug,
                const DetectorConfig& detector_config,
                const CameraTelemetry& camera_telemetry,
                std::uint64_t frame_sequence) {
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
              << ",\"control_valid\":" << (control.valid ? "true" : "false")
              << ",\"control_source\":\"" << toString(control.source)
              << "\",\"vy_mps\":" << control.vy_mps
              << ",\"vy_unconstrained_mps\":" << control.unconstrained_vy_mps
              << ",\"marker_scale_px\":" << control.marker_scale_px
              << ",\"distance_m\":" << control.distance_m
              << ",\"lateral_error_raw_m\":" << control.raw_lateral_error_m
              << ",\"lateral_error_filtered_m\":"
              << control.filtered_lateral_error_m
              << ",\"relative_lateral_velocity_mps\":"
              << control.relative_lateral_velocity_mps
              << ",\"vy_position_mps\":" << control.vy_position_mps
              << ",\"vy_integral_mps\":" << control.vy_integral_mps
              << ",\"vy_velocity_mps\":" << control.vy_velocity_mps
              << ",\"command_saturated\":"
              << (control.command_saturated ? "true" : "false")
              << ",\"acceleration_limited\":"
              << (control.acceleration_limited ? "true" : "false")
              << ",\"search_elapsed_s\":" << control.search_elapsed_s
              << ",\"search_leg\":" << control.search_leg
              << ",\"chassis_uart_enabled\":"
              << (chassis_tx.enabled ? "true" : "false")
              << ",\"chassis_tx_attempted\":"
              << (chassis_tx.attempted ? "true" : "false")
              << ",\"chassis_tx_success\":"
              << (chassis_tx.success ? "true" : "false")
              << ",\"chassis_tx_seq\":" << chassis_tx.sequence
              << ",\"chassis_tx_valid\":"
              << (chassis_tx.valid ? "true" : "false")
              << ",\"chassis_tx_vx_mps\":" << chassis_tx.vx_mps
              << ",\"chassis_tx_vy_mps\":" << chassis_tx.vy_mps
              << ",\"image_diagnostics_valid\":"
              << (debug.diagnostics_valid ? "true" : "false")
              << ",\"threshold_mode\":\""
              << (debug.fixed_threshold ? "FIXED" : "ADAPTIVE")
              << "\",\"led_threshold\":" << debug.led_threshold
              << ",\"adaptive_unclamped_threshold\":"
              << debug.adaptive_unclamped_threshold
              << ",\"otsu_threshold\":" << debug.otsu_threshold
              << ",\"percentile_threshold\":" << debug.percentile_threshold
              << ",\"mean_based_threshold\":" << debug.mean_based_threshold
              << ",\"configured_bright_percentile\":"
              << detector_config.bright_percentile
              << ",\"configured_min_led_threshold\":"
              << detector_config.min_led_threshold
              << ",\"configured_max_led_threshold\":"
              << detector_config.max_led_threshold
              << ",\"configured_local_contrast_threshold\":"
              << detector_config.local_contrast_threshold
              << ",\"configured_saturation_threshold\":"
              << detector_config.saturation_threshold
              << ",\"configured_morphology_kernel\":"
              << detector_config.morphology_kernel
              << ",\"configured_fixed_led_threshold\":"
              << detector_config.fixed_led_threshold
              << ",\"roi_x\":" << debug.search_roi.x
              << ",\"roi_y\":" << debug.search_roi.y
              << ",\"roi_w\":" << debug.search_roi.width
              << ",\"roi_h\":" << debug.search_roi.height
              << ",\"roi_gray_min\":" << debug.gray_min
              << ",\"roi_gray_max\":" << debug.gray_max
              << ",\"roi_gray_mean\":" << debug.gray_mean
              << ",\"roi_gray_stddev\":" << debug.gray_stddev
              << ",\"roi_gray_p50\":" << debug.gray_p50
              << ",\"roi_gray_p90\":" << debug.gray_p90
              << ",\"roi_gray_p95\":" << debug.gray_p95
              << ",\"roi_gray_p99\":" << debug.gray_p99
              << ",\"roi_bright_fraction\":" << debug.bright_fraction
              << ",\"roi_saturation_fraction\":"
              << debug.roi_saturation_fraction
              << ",\"raw_contour_count\":" << debug.raw_contour_count
              << ",\"led_candidate_count\":" << debug.candidates.size()
              << ",\"camera_telemetry_available\":"
              << (camera_telemetry.available ? "true" : "false")
              << ",\"camera_auto_exposure\":"
              << (camera_telemetry.auto_exposure ? "true" : "false")
              << ",\"camera_exposure_us\":" << camera_telemetry.exposure_us
              << ",\"camera_gain\":" << camera_telemetry.gain
              << ",\"geometric_center_x_px\":" << control.geometric_center_x_px
              << ",\"refined_center_x_px\":" << control.refined_center_x_px
              << ",\"optional_refinement_used\":"
              << (control.optional_refinement_used ? "true" : "false")
              << ",\"optional_refinement_components\":"
              << control.optional_refinement_components
              << ",\"optional_correction_x_px\":"
              << control.optional_correction_x_px
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
                    const LateralControlOutput& control,
                    const LateralControlConfig& control_config,
                    const DetectorDebugSnapshot& debug,
                    const DetectorConfig& detector_config,
                    const CameraTelemetry& camera_telemetry,
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
        constexpr std::array<const char*, 3> labels{{"L0", "L1", "L2"}};
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
        if (display != nullptr && control.optional_refinement_used) {
            const cv::Point refined_center(
                cvRound(control.refined_center_x_px), cvRound(result.center.y));
            cv::drawMarker(annotated, refined_center, cv::Scalar(0, 255, 255),
                           cv::MARKER_DIAMOND, 12, 2);
        }
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

    if (display != nullptr) {
        const int baseline_y = annotated.rows - 24;
        const int origin_x = std::clamp(cvRound(control_config.principal_x_px),
                                        0, annotated.cols - 1);
        const int maximum_arrow_px = std::min(120, annotated.cols / 3);
        const float command_fraction = std::clamp(
            control.vy_mps / std::max(0.01F, control_config.max_vy_mps), -1.0F, 1.0F);
        // Positive AVC1 vy means vehicle-left, hence decreasing image X.
        const int endpoint_x = origin_x - cvRound(command_fraction * maximum_arrow_px);
        const cv::Scalar command_color = control.valid
            ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 64, 64);
        cv::line(annotated, cv::Point(origin_x - maximum_arrow_px, baseline_y),
                 cv::Point(origin_x + maximum_arrow_px, baseline_y),
                 cv::Scalar(96, 96, 96), 1, cv::LINE_AA);
        cv::line(annotated, cv::Point(origin_x, 24),
                 cv::Point(origin_x, annotated.rows - 50),
                 cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
        if (control.valid && control.distance_m > 0.0F) {
            const int deadband_px = std::max(1, cvRound(
                control_config.position_deadband_m * control_config.focal_x_px /
                control.distance_m));
            cv::line(annotated, cv::Point(origin_x - deadband_px, 24),
                     cv::Point(origin_x - deadband_px, annotated.rows - 50),
                     cv::Scalar(96, 160, 160), 1, cv::LINE_AA);
            cv::line(annotated, cv::Point(origin_x + deadband_px, 24),
                     cv::Point(origin_x + deadband_px, annotated.rows - 50),
                     cv::Scalar(96, 160, 160), 1, cv::LINE_AA);
        }
        cv::drawMarker(annotated, cv::Point(origin_x, baseline_y),
                       cv::Scalar(255, 255, 255), cv::MARKER_CROSS, 8, 1);
        if (control.valid && endpoint_x != origin_x) {
            cv::arrowedLine(annotated, cv::Point(origin_x, baseline_y),
                            cv::Point(endpoint_x, baseline_y), command_color, 4,
                            cv::LINE_AA, 0, 0.22);
        } else {
            cv::circle(annotated, cv::Point(origin_x, baseline_y), 6,
                       command_color, 2, cv::LINE_AA);
        }
        const char* direction = !control.valid ? "STOP" :
            (control.vy_mps > 0.001F ? "LEFT" :
             (control.vy_mps < -0.001F ? "RIGHT" : "HOLD"));
        std::ostringstream control_status;
        control_status << std::fixed << std::setprecision(3)
                       << toString(control.source) << " vy=" << control.vy_mps
                       << "m/s " << direction
                       << "  x=" << control.filtered_lateral_error_m * 1000.0F
                       << "mm  z=" << control.distance_m << "m";
        cv::putText(annotated, control_status.str(), cv::Point(5, baseline_y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, command_color, 1, cv::LINE_AA);
        std::ostringstream terms;
        terms << std::fixed << std::setprecision(3);
        if (control.source == LateralControlSource::SEARCHING) {
            terms << "SEARCH leg=" << control.search_leg
                  << " t=" << control.search_elapsed_s << "s";
        } else {
            terms << "P=" << control.vy_position_mps
                  << " I=" << control.vy_integral_mps
                  << " V=" << control.vy_velocity_mps
                  << " vrel=" << control.relative_lateral_velocity_mps;
        }
        if (control.command_saturated) terms << " SAT";
        if (control.acceleration_limited) terms << " SLEW";
        cv::putText(annotated, terms.str(), cv::Point(5, baseline_y - 28),
                    cv::FONT_HERSHEY_SIMPLEX, 0.38, command_color, 1, cv::LINE_AA);
    }

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
    if (debug.diagnostics_valid) {
        std::ostringstream threshold_status;
        threshold_status << (debug.fixed_threshold ? "FIX" : "AUTO")
                         << " T=" << debug.led_threshold
                         << " raw=" << debug.adaptive_unclamped_threshold
                         << " O=" << debug.otsu_threshold
                         << " P" << std::fixed << std::setprecision(3)
                         << detector_config.bright_percentile << "="
                         << debug.percentile_threshold
                         << " M=" << debug.mean_based_threshold;
        cv::putText(mask_rgb, threshold_status.str(), cv::Point(5, 36),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(255, 255, 255), 1,
                    cv::LINE_AA);
        std::ostringstream gray_status;
        gray_status << std::fixed << std::setprecision(1)
                    << "gray " << debug.gray_mean << "+/-" << debug.gray_stddev
                    << " p50=" << debug.gray_p50
                    << " p95=" << debug.gray_p95
                    << " p99=" << debug.gray_p99;
        cv::putText(mask_rgb, gray_status.str(), cv::Point(5, 55),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(255, 255, 255), 1,
                    cv::LINE_AA);
        std::ostringstream camera_status;
        camera_status << "AE=" << (camera_telemetry.auto_exposure ? 1 : 0)
                      << " exp=" << camera_telemetry.exposure_us
                      << "us gain=" << camera_telemetry.gain
                      << std::fixed << std::setprecision(3)
                      << " bright=" << debug.bright_fraction
                      << " sat=" << debug.roi_saturation_fraction;
        cv::putText(mask_rgb, camera_status.str(), cv::Point(5, 74),
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(255, 255, 255), 1,
                    cv::LINE_AA);
    }

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
    if (!options.detector_config_path.empty()) {
        std::string error;
        if (!loadDetectorConfig(options.detector_config_path, detector_config, error)) {
            throw std::runtime_error(error);
        }
    } else {
        detector_config.normalize();
    }
    LateralControlConfig control_config;
    if (!options.control_config_path.empty()) {
        std::string error;
        if (!loadLateralControlConfig(options.control_config_path, control_config, error)) {
            throw std::runtime_error(error);
        }
    }

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
    LateralController lateral_controller(control_config);
    detector.setDebugEnabled(options.debug);
    detector.setDebugCenterOverlayEnabled(options.debug_display);
    std::unique_ptr<maix::display::Display> display;
    if (options.debug_display) {
        display = std::make_unique<maix::display::Display>();
    }
    CameraFrame frame;
    std::uint64_t consecutive_empty_reads = 0;
    std::unique_ptr<Avc1UartSender> chassis_sender;
    if (!options.chassis_uart_path.empty()) {
        chassis_sender = std::make_unique<Avc1UartSender>(
            options.chassis_uart_path);
        std::cerr << "AVC1 chassis output enabled on "
                  << chassis_sender->port() << " at 115200 8N1, vx=0, |vy|<="
                  << options.chassis_vy_limit_mps << " m/s\n";
    }

    std::cerr << "maixcam_marker_detector: camera " << camera_config.width << "x"
              << camera_config.height << " @ " << camera_config.fps
              << " FPS, grayscale, buff_num=" << camera_config.buffer_count << '\n';

    while (!maix::app::need_exit()) {
        if (!camera.read(frame)) {
            ++consecutive_empty_reads;
            detector.recordDroppedFrame();
            if (chassis_sender && consecutive_empty_reads == 1) {
                const Avc1TransmitResult stopped =
                    chassis_sender->transmit(0.0F, 0.0F, false);
                if (!stopped.success) {
                    throw std::runtime_error("AVC1 stop write failed: " +
                                             chassis_sender->lastError());
                }
            }
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
        const LateralControlOutput control = lateral_controller.update(result);
        CameraTelemetry camera_telemetry;
        camera_telemetry.auto_exposure = camera_config.use_auto_exposure;
        if (options.debug) camera_telemetry = camera.telemetry();
        Avc1TransmitResult chassis_tx;
        if (chassis_sender) {
            const float transmitted_vy = control.valid
                ? std::clamp(control.vy_mps, -options.chassis_vy_limit_mps,
                             options.chassis_vy_limit_mps)
                : 0.0F;
            chassis_tx = chassis_sender->transmit(
                0.0F, transmitted_vy, control.valid);
            if (!chassis_tx.success) {
                throw std::runtime_error("AVC1 command write failed: " +
                                         chassis_sender->lastError());
            }
        }
        if (options.debug) {
            showDebugFrame(frame.gray(), result, control, lateral_controller.config(),
                           detector.debugSnapshot(), detector_config,
                           camera_telemetry, display.get());
        }
        writeJsonl(result, control, chassis_tx, detector.debugSnapshot(),
                   detector_config, camera_telemetry, frame.sequence());
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
