// Standalone MaixCAM Pro UART1 AVC1 sender test.
// TX: A16 (UART0_TX), device: /dev/ttyS0, 115200 8N1, 20 Hz.

#include "maix_uart.hpp"
#include "maix_pinmap.hpp"
#include "maix_err.hpp"
#include "maix_time.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace {
constexpr std::uint32_t kMagic = 0x31435641U;
constexpr std::uint32_t kChecksumSeed = 0xA5A51234U;
constexpr int kFrameSize = 24;

void store_le32(std::uint8_t* p, std::uint32_t v) {
    p[0] = static_cast<std::uint8_t>(v & 0xFFU);
    p[1] = static_cast<std::uint8_t>((v >> 8U) & 0xFFU);
    p[2] = static_cast<std::uint8_t>((v >> 16U) & 0xFFU);
    p[3] = static_cast<std::uint8_t>((v >> 24U) & 0xFFU);
}

std::uint32_t avc1_checksum(const std::uint8_t* d, std::size_t n) {
    std::uint32_t c = kChecksumSeed;
    for (std::size_t i = 0; i < n; ++i) {
        c = (c << 5U) ^ (c >> 2U) ^ d[i];
    }
    return c;
}
}  // namespace

int main() {
    if (maix::peripheral::pinmap::set_pin_function("A16", "UART0_TX") != maix::err::ERR_NONE) {
        std::printf("pinmap A16->UART0_TX FAILED\n");
        return 1;
    }

    maix::peripheral::uart::UART uart(
        "/dev/ttyS0", 115200,
        maix::peripheral::uart::BITS_8,
        maix::peripheral::uart::PARITY_NONE,
        maix::peripheral::uart::STOP_1,
        maix::peripheral::uart::FLOW_CTRL_NONE);

    std::uint8_t frame[kFrameSize] = {};
    std::uint32_t seq = 0;
    const std::uint64_t t0 = maix::time::ticks_ms() / 1000U;

    std::printf("AVC1 SEND: A16 UART0_TX, /dev/ttyS0, 115200 8N1, 20 Hz\n");

    while (true) {
        const std::uint64_t phase = (maix::time::ticks_ms() / 1000U - t0) % 15U;
        const float vx = (phase >= 5U && phase < 10U) ? 0.05F : 0.0F;
        const float vy = 0.0F;

        store_le32(frame + 0, kMagic);
        store_le32(frame + 4, seq++);
        std::memcpy(frame + 8, &vx, sizeof(vx));
        std::memcpy(frame + 12, &vy, sizeof(vy));
        frame[16] = 1;
        frame[17] = frame[18] = frame[19] = 0;
        store_le32(frame + 20, avc1_checksum(frame, 20));

        int written = 0;
        while (written < kFrameSize) {
            const int n = uart.write(frame + written, kFrameSize - written);
            if (n <= 0) {
                std::printf("write fail %d\n", n);
                break;
            }
            written += n;
        }

        std::printf("sent seq=%u vx=%.2f vy=%.2f valid=1 (%d bytes)\n",
                    seq - 1U, vx, vy, written);
        usleep(50000);
    }

    return 0;
}
