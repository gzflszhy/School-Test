// Standalone MaixCAM Pro UART AVC1 sender test.
// TX: A16 (UART0_TX), device: /dev/ttyS0, 115200 8N1, 20 Hz.
// Sends vx = 0.0 m/s, vy = +0.2 m/s, valid = 1 continuously.

#include "maix_uart.hpp"
#include "maix_pinmap.hpp"
#include "maix_err.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace {
constexpr std::uint32_t kMagic = 0x31435641U;        // "AVC1"
constexpr std::uint32_t kChecksumSeed = 0xA5A51234U;
constexpr int kFrameSize = 24;
constexpr float kVx = 0.0F;
constexpr float kVy = 0.2F;
constexpr useconds_t kSendPeriodUs = 50000;           // 20 Hz

void store_le32(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v & 0xFFU);
    p[1] = static_cast<std::uint8_t>((v >> 8U) & 0xFFU);
    p[2] = static_cast<std::uint8_t>((v >> 16U) & 0xFFU);
    p[3] = static_cast<std::uint8_t>((v >> 24U) & 0xFFU);
}

std::uint32_t avc1_checksum(const std::uint8_t* data, std::size_t n) {
    std::uint32_t checksum = kChecksumSeed;
    for (std::size_t i = 0; i < n; ++i) {
        checksum = (checksum << 5U) ^ (checksum >> 2U) ^ data[i];
    }
    return checksum;
}

bool write_all(maix::peripheral::uart::UART& uart,
               const std::uint8_t* data,
               int size) {
    int written = 0;
    while (written < size) {
        const int n = uart.write(data + written, size - written);
        if (n <= 0) {
            std::printf("UART write failed: %d\n", n);
            return false;
        }
        written += n;
    }
    return true;
}
}  // namespace

int main() {
    if (maix::peripheral::pinmap::set_pin_function("A16", "UART0_TX") !=
        maix::err::ERR_NONE) {
        std::printf("pinmap A16 -> UART0_TX FAILED\n");
        return 1;
    }

    maix::peripheral::uart::UART uart(
        "/dev/ttyS0",
        115200,
        maix::peripheral::uart::BITS_8,
        maix::peripheral::uart::PARITY_NONE,
        maix::peripheral::uart::STOP_1,
        maix::peripheral::uart::FLOW_CTRL_NONE);

    std::uint8_t frame[kFrameSize] = {};
    std::uint32_t seq = 0;

    std::printf(
        "AVC1 constant sender: A16 UART0_TX, /dev/ttyS0, 115200 8N1, "
        "20 Hz, vx=%.2f, vy=%.2f, valid=1\n",
        kVx,
        kVy);

    while (true) {
        store_le32(frame + 0, kMagic);
        store_le32(frame + 4, seq);
        std::memcpy(frame + 8, &kVx, sizeof(kVx));
        std::memcpy(frame + 12, &kVy, sizeof(kVy));
        frame[16] = 1;  // valid
        frame[17] = 0;
        frame[18] = 0;
        frame[19] = 0;
        store_le32(frame + 20, avc1_checksum(frame, 20));

        if (write_all(uart, frame, kFrameSize)) {
            std::printf(
                "sent seq=%u vx=%.2f vy=%.2f valid=1 (%d bytes)\n",
                seq,
                kVx,
                kVy,
                kFrameSize);
        }

        ++seq;
        usleep(kSendPeriodUs);
    }

    return 0;
}
