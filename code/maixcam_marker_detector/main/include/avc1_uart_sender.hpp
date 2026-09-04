#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace maixcam_marker {

struct Avc1TransmitResult {
    bool enabled = false;
    bool attempted = false;
    bool success = false;
    std::uint32_t sequence = 0;
    float vx_mps = 0.0F;
    float vy_mps = 0.0F;
    bool valid = false;
};

class Avc1UartSender {
public:
    explicit Avc1UartSender(const std::string& port);
    ~Avc1UartSender();

    Avc1UartSender(const Avc1UartSender&) = delete;
    Avc1UartSender& operator=(const Avc1UartSender&) = delete;

    Avc1TransmitResult transmit(float vx_mps, float vy_mps, bool valid);
    const std::string& port() const noexcept { return port_; }
    const std::string& lastError() const noexcept { return last_error_; }

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    std::string port_;
    std::string last_error_;
    std::uint32_t next_sequence_ = 0;
};

}  // namespace maixcam_marker
