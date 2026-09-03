#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <opencv2/core.hpp>

namespace maixcam_marker {

/**
 * Camera settings deliberately live outside DetectorConfig: the detector is
 * usable with recorded OpenCV frames and knows nothing about MaixCDK.
 */
struct CameraConfig {
    int width = 480;
    int height = 270;
    double fps = 60.0;
    int buffer_count = 1;
    int warmup_frames = 30;
    bool use_auto_exposure = true;
    int manual_exposure_us = 1500;

    // Drop driver-queued frames immediately before each blocking read.  This
    // favours a fresh observation over processing every captured frame.
    bool prefer_latest_frame = true;
};

/**
 * A move-only view of a MaixCDK image.  pixels remains valid until this object
 * is destroyed or moved-from; callers must finish cv::Mat processing first.
 */
class CameraFrame {
public:
    CameraFrame();
    ~CameraFrame();
    CameraFrame(CameraFrame&&) noexcept;
    CameraFrame& operator=(CameraFrame&&) noexcept;
    CameraFrame(const CameraFrame&) = delete;
    CameraFrame& operator=(const CameraFrame&) = delete;

    [[nodiscard]] bool valid() const noexcept;
    [[nodiscard]] const cv::Mat& gray() const noexcept;
    [[nodiscard]] std::chrono::steady_clock::time_point capture_time() const noexcept;
    [[nodiscard]] std::uint64_t sequence() const noexcept;

private:
    class Holder;
    std::unique_ptr<Holder> holder_;
    cv::Mat gray_;
    std::chrono::steady_clock::time_point capture_time_{};
    std::uint64_t sequence_ = 0;

    void reset() noexcept;
    friend class MaixCameraSource;
};

/**
 * MaixCAM-Pro capture adapter.  This is the only production header/source pair
 * that includes or depends on MaixCDK camera types.
 */
class MaixCameraSource {
public:
    explicit MaixCameraSource(CameraConfig config = {});
    ~MaixCameraSource();
    MaixCameraSource(MaixCameraSource&&) noexcept;
    MaixCameraSource& operator=(MaixCameraSource&&) noexcept;
    MaixCameraSource(const MaixCameraSource&) = delete;
    MaixCameraSource& operator=(const MaixCameraSource&) = delete;

    /** Blocks for one fresh grayscale frame. Returns false for a recoverable
     * empty read; construction/configuration failures throw std::runtime_error. */
    bool read(CameraFrame& frame);

    [[nodiscard]] const CameraConfig& config() const noexcept;
    [[nodiscard]] const std::string& last_error() const noexcept;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    CameraConfig config_;
    std::string last_error_;
    std::uint64_t next_sequence_ = 0;
};

}  // namespace maixcam_marker
