#include "maix_camera_source.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

#include "maix_camera.hpp"
#include "maix_image.hpp"

namespace maixcam_marker {

class CameraFrame::Holder {
public:
    explicit Holder(maix::image::Image* image) : image(image) {}
    std::unique_ptr<maix::image::Image> image;
};

class MaixCameraSource::Impl {
public:
    explicit Impl(const CameraConfig& config)
        : camera(std::make_unique<maix::camera::Camera>(
              config.width,
              config.height,
              maix::image::Format::FMT_GRAYSCALE,
              nullptr,
              config.fps,
              config.buffer_count,
              true,
              false)) {}

    std::unique_ptr<maix::camera::Camera> camera;
};

CameraFrame::CameraFrame() = default;
CameraFrame::~CameraFrame() = default;
CameraFrame::CameraFrame(CameraFrame&&) noexcept = default;
CameraFrame& CameraFrame::operator=(CameraFrame&&) noexcept = default;

bool CameraFrame::valid() const noexcept {
    return holder_ != nullptr && !gray_.empty();
}

const cv::Mat& CameraFrame::gray() const noexcept {
    return gray_;
}

std::chrono::steady_clock::time_point CameraFrame::capture_time() const noexcept {
    return capture_time_;
}

std::uint64_t CameraFrame::sequence() const noexcept {
    return sequence_;
}

void CameraFrame::reset() noexcept {
    gray_.release();
    holder_.reset();
    capture_time_ = {};
    sequence_ = 0;
}

MaixCameraSource::MaixCameraSource(CameraConfig config) : config_(config) {
    if (config_.width <= 0 || config_.height <= 0 || config_.fps <= 0.0 ||
        config_.buffer_count <= 0 || config_.warmup_frames < 0 ||
        (!config_.use_auto_exposure && config_.manual_exposure_us <= 0)) {
        throw std::runtime_error("invalid Maix camera configuration");
    }

    impl_ = std::make_unique<Impl>(config_);
    if (!impl_->camera->is_opened()) {
        throw std::runtime_error("MaixCDK could not open the requested camera");
    }

    // MaixCDK 4.10.3 uses the legacy integer exposure-mode API:
    // 0 = auto exposure, 1 = manual exposure.
    const int requested_ae_mode = config_.use_auto_exposure ? 0 : 1;
    if (impl_->camera->exp_mode(requested_ae_mode) != requested_ae_mode) {
        throw std::runtime_error("MaixCDK could not set the requested exposure mode");
    }
    if (!config_.use_auto_exposure) {
        // Camera::exposure uses microseconds. Do not map the detector's
        // floating gain fields to Camera::gain(int) without target calibration.
        impl_->camera->exposure(config_.manual_exposure_us);
    }

    // Discard initial frames after applying AE/manual exposure so the ISP and
    // capture pipeline settle before detector latency is measured.
    if (config_.warmup_frames > 0) {
        impl_->camera->skip_frames(config_.warmup_frames);
    }
}

MaixCameraSource::~MaixCameraSource() = default;
MaixCameraSource::MaixCameraSource(MaixCameraSource&&) noexcept = default;
MaixCameraSource& MaixCameraSource::operator=(MaixCameraSource&&) noexcept = default;

bool MaixCameraSource::read(CameraFrame& frame) {
    frame.reset();
    last_error_.clear();

    if (!impl_ || !impl_->camera || !impl_->camera->is_opened()) {
        last_error_ = "camera is not open";
        return false;
    }

    // MaixCDK 4.10.3 reports clear_buff() as unsupported for the default
    // single-buffer GC4653 channel. With one buffer there is no queued backlog
    // to drain, so only request a flush when multiple buffers are configured.
    if (config_.prefer_latest_frame && config_.buffer_count > 1) {
        impl_->camera->clear_buff();
    }

    // Use the documented blocking default rather than a very short timeout:
    // MaixCDK cautions that a low block_ms may duplicate MaixCam frames.
    maix::image::Image* image = impl_->camera->read(nullptr, 0, true, -1);
    const auto captured_at = std::chrono::steady_clock::now();
    if (image == nullptr) {
        last_error_ = "MaixCDK Camera::read returned no image";
        return false;
    }

    if (image->format() != maix::image::Format::FMT_GRAYSCALE || image->data() == nullptr ||
        image->width() != config_.width || image->height() != config_.height ||
        image->data_size() < config_.width * config_.height) {
        delete image;
        last_error_ = "camera did not return the requested contiguous grayscale frame";
        return false;
    }

    // This cv::Mat is a zero-copy header over Image::data().  Holder owns the
    // Image and ensures its buffer is released only after detector processing.
    frame.holder_ = std::make_unique<CameraFrame::Holder>(image);
    frame.gray_ = cv::Mat(config_.height, config_.width, CV_8UC1, image->data());
    frame.capture_time_ = captured_at;
    frame.sequence_ = ++next_sequence_;
    return true;
}

const CameraConfig& MaixCameraSource::config() const noexcept {
    return config_;
}

const std::string& MaixCameraSource::last_error() const noexcept {
    return last_error_;
}

CameraTelemetry MaixCameraSource::telemetry() const noexcept {
    CameraTelemetry result;
    result.auto_exposure = config_.use_auto_exposure;
    if (!impl_ || !impl_->camera || !impl_->camera->is_opened()) return result;
    try {
        result.exposure_us = impl_->camera->exposure();
        result.gain = impl_->camera->gain();
        result.available = result.exposure_us >= 0 && result.gain >= 0;
    } catch (...) {
        result.available = false;
    }
    return result;
}

}  // namespace maixcam_marker
