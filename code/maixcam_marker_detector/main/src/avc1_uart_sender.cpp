#include "avc1_uart_sender.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include <string>

#include "maix_pinmap.hpp"
#include "maix_uart.hpp"

#include "avc1_protocol.hpp"

namespace maixcam_marker {

class Avc1UartSender::Impl {
public:
    explicit Impl(const std::string& port)
        : uart(port, 115200, maix::peripheral::uart::BITS_8,
               maix::peripheral::uart::PARITY_NONE,
               maix::peripheral::uart::STOP_1,
               maix::peripheral::uart::FLOW_CTRL_NONE) {}

    maix::peripheral::uart::UART uart;
};

Avc1UartSender::Avc1UartSender(const std::string& port)
    : port_(port) {
    if (port_ == "/dev/ttyS1") {
        const maix::err::Err result =
            maix::peripheral::pinmap::set_pin_function("A19", "UART1_TX");
        if (result != maix::err::ERR_NONE) {
            throw std::runtime_error(
                "cannot set A19 pinmux to UART1_TX for /dev/ttyS1");
        }
    }
    // Configure the physical output pin before opening the UART controller so
    // the first AVC1 stop frame is observable on A19.
    impl_ = std::make_unique<Impl>(port_);
    const Avc1TransmitResult stopped = transmit(0.0F, 0.0F, false);
    if (!stopped.success) {
        throw std::runtime_error("cannot send initial AVC1 stop frame on " + port_ +
                                 ": " + last_error_);
    }
}

Avc1UartSender::~Avc1UartSender() {
    if (!impl_) return;
    try {
        (void)transmit(0.0F, 0.0F, false);
    } catch (...) {
        // Destructors must not mask the original shutdown reason. The C board
        // independently stops after 200 ms without a legal frame.
    }
    (void)impl_->uart.close();
}

Avc1TransmitResult Avc1UartSender::transmit(float vx_mps, float vy_mps,
                                            bool valid) {
    Avc1TransmitResult result;
    result.enabled = impl_ != nullptr;
    result.attempted = result.enabled;
    result.sequence = next_sequence_++;
    if (!std::isfinite(vx_mps) || !std::isfinite(vy_mps)) valid = false;
    result.valid = valid;
    result.vx_mps = valid ? std::clamp(vx_mps, -kAvc1MaximumAxisSpeedMps,
                                       kAvc1MaximumAxisSpeedMps) : 0.0F;
    result.vy_mps = valid ? std::clamp(vy_mps, -kAvc1MaximumAxisSpeedMps,
                                       kAvc1MaximumAxisSpeedMps) : 0.0F;
    if (!impl_) {
        last_error_ = "UART is not initialized";
        return result;
    }

    const Avc1Frame frame = packAvc1Frame(
        result.sequence, result.vx_mps, result.vy_mps, result.valid);
    std::size_t written = 0;
    try {
        while (written < frame.size()) {
            const int count = impl_->uart.write(
                frame.data() + written,
                static_cast<int>(frame.size() - written));
            if (count <= 0) {
                last_error_ = "UART write returned " + std::to_string(count) +
                              " after " + std::to_string(written) + " bytes";
                return result;
            }
            if (static_cast<std::size_t>(count) > frame.size() - written) {
                last_error_ = "UART write returned an impossible byte count";
                return result;
            }
            written += static_cast<std::size_t>(count);
        }
    } catch (const std::exception& error) {
        last_error_ = error.what();
        return result;
    } catch (...) {
        last_error_ = "unknown UART write failure";
        return result;
    }
    last_error_.clear();
    result.success = true;
    return result;
}

}  // namespace maixcam_marker
