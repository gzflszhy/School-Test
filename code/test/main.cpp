#include "maix_uart.hpp"
#include "maix_pinmap.hpp"
#include "maix_err.hpp"
#include "maix_time.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace {

constexpr std::uint32_t kMagic = 0x31435641U;
constexpr std::uint32_t kChecksumSeed = 0xA5A51234U;

void store_le32(std::uint8_t* p, std::uint32_t v)
{
    p[0] = static_cast<std::uint8_t>(v & 0xFFU);
    p[1] = static_cast<std::uint8_t>((v >> 8U) & 0xFFU);
    p[2] = static_cast<std::uint8_t>((v >> 16U) & 0xFFU);
    p[3] = static_cast<std::uint8_t>((v >> 24U) & 0xFFU);
}

std::uint32_t avc1_checksum(const std::uint8_t* d, std::size_t n)
{
    std::uint32_t c = kChecksumSeed;

    for (std::size_t i = 0; i < n; ++i)
        c = (c << 5U) ^ (c >> 2U) ^ d[i];

    return c;
}

}

int main()
{
    if (maix::peripheral::pinmap::set_pin_function(
            "A16", "UART0_TX") != maix::err::ERR_NONE)
    {
        printf("pinmap A16->UART0_TX FAILED\n");
        return 1;
    }

    maix::peripheral::uart::UART uart(
        "/dev/ttyS0",
        115200,
        maix::peripheral::uart::BITS_8,
        maix::peripheral::uart::PARITY_NONE,
        maix::peripheral::uart::STOP_1,
        maix::peripheral::uart::FLOW_CTRL_NONE
    );

    std::uint8_t frame[24] = {};

    std::uint32_t seq = 0;

    const std::uint64_t t0 =
        maix::time::ticks_ms() / 1000;

    while (true)
    {
        const std::uint64_t phase =
            (maix::time::ticks_ms() / 1000 - t0) % 15;

        const float vx =
            (phase >= 5 && phase < 10)
                ? 0.05F
                : 0.0F;

        const float vy = 0.0F;

        store_le32(frame + 0, kMagic);
        store_le32(frame + 4, seq++);

        std::memcpy(frame + 8, &vx, 4);
        std::memcpy(frame + 12, &vy, 4);

        frame[16] = 1;

        frame[17] = 0;
        frame[18] = 0;
        frame[19] = 0;

        store_le32(
            frame + 20,
            avc1_checksum(frame, 20)
        );

        int written = 0;

        while (written < 24)
        {
            const int n =
                uart.write(
                    frame + written,
                    24 - written
                );

            if (n <= 0)
            {
                printf("write fail %d\n", n);
                break;
            }

            written += n;
        }

        printf(
            "sent seq=%u vx=%.2f (%d bytes)\n",
            seq - 1,
            vx,
            written
        );

        usleep(50000);
    }

    return 0;
}