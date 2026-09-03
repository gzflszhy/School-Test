#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <string_view>

#include "maix_basic.hpp"

#include "detector_config.hpp"
#include "maix_camera_source.hpp"
#include "marker_detector.hpp"

namespace {

using namespace maixcam_marker;

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
              << "}\n" << std::flush;
}

int run() {
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
        writeJsonl(result, frame.sequence());
    }

    std::cerr << detector.benchmark().summary() << '\n';
    return 0;
}

}  // namespace

int main(int /*argc*/, char** /*argv*/) {
    maix::sys::register_default_signal_handle();
    try {
        return run();
    } catch (const std::exception& error) {
        std::cerr << "fatal: " << error.what() << '\n';
        return 1;
    }
}
