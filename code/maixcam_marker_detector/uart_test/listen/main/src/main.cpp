// Standalone MaixCAM Pro UART1 AVC1 listener test.
// RX: A18 (UART1_RX), device: /dev/ttyS1, 115200 8N1.

#include "maix_uart.hpp"
#include "maix_pinmap.hpp"
#include "maix_err.hpp"
#include "maix_time.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {
constexpr std::uint32_t kMagic = 0x31435641U;
constexpr std::uint32_t kChecksumSeed = 0xA5A51234U;
constexpr std::size_t kFrameSize = 24;

std::uint32_t load_le32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8U) |
           (static_cast<std::uint32_t>(p[2]) << 16U) |
           (static_cast<std::uint32_t>(p[3]) << 24U);
}

std::uint32_t avc1_checksum(const std::uint8_t* d, std::size_t n) {
    std::uint32_t c = kChecksumSeed;
    for (std::size_t i = 0; i < n; ++i) {
        c = (c << 5U) ^ (c >> 2U) ^ d[i];
    }
    return c;
}

void decode_and_print(const std::uint8_t* frame) {
    const std::uint32_t seq = load_le32(frame + 4);
    float vx = 0.0F;
    float vy = 0.0F;
    std::memcpy(&vx, frame + 8, sizeof(vx));
    std::memcpy(&vy, frame + 12, sizeof(vy));
    const unsigned valid = frame[16];
    const std::uint32_t received_checksum = load_le32(frame + 20);
    const std::uint32_t calculated_checksum = avc1_checksum(frame, 20);

    if (received_checksum != calculated_checksum) {
        std::printf("BAD checksum seq=%u recv=0x%08X calc=0x%08X\n",
                    seq, received_checksum, calculated_checksum);
        return;
    }

    std::printf("recv seq=%u vx=%.3f vy=%.3f valid=%u checksum=OK\n",
                seq, vx, vy, valid);
}
}  // namespace

int main() {
    if (maix::peripheral::pinmap::set_pin_function("A18", "UART1_RX") != maix::err::ERR_NONE) {
        std::printf("pinmap A18->UART1_RX FAILED\n");
        return 1;
    }

    maix::peripheral::uart::UART uart(
        "/dev/ttyS1", 115200,
        maix::peripheral::uart::BITS_8,
        maix::peripheral::uart::PARITY_NONE,
        maix::peripheral::uart::STOP_1,
        maix::peripheral::uart::FLOW_CTRL_NONE);

    std::printf("AVC1 LISTEN: A18 UART1_RX, /dev/ttyS1, 115200 8N1\n");
    std::printf("Waiting for 24-byte AVC1 frames...\n");

    std::vector<std::uint8_t> stream;
    stream.reserve(256);
    std::uint8_t rx[128];

    while (true) {
        const int n = uart.read(rx, static_cast<int>(sizeof(rx)), -1, 100);
        if (n < 0) {
            std::printf("read fail %d\n", n);
            maix::time::sleep_ms(10);
            continue;
        }
        if (n == 0) {
            maix::time::sleep_ms(1);
            continue;
        }

        stream.insert(stream.end(), rx, rx + n);

        while (stream.size() >= 4U) {
            std::size_t start = 0;
            while (start + 4U <= stream.size() && load_le32(stream.data() + start) != kMagic) {
                ++start;
            }

            if (start > 0U) {
                stream.erase(stream.begin(), stream.begin() + static_cast<std::ptrdiff_t>(start));
            }

            if (stream.size() < kFrameSize) {
                break;
            }

            if (load_le32(stream.data()) != kMagic) {
                break;
            }

            decode_and_print(stream.data());
            stream.erase(stream.begin(), stream.begin() + static_cast<std::ptrdiff_t>(kFrameSize));
        }

        if (stream.size() > 1024U) {
            stream.erase(stream.begin(), stream.end() - 3);
        }
    }

    return 0;
}
