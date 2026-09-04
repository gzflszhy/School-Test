#include "avc1_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace maixcam_marker {
namespace {

void storeLe32(Avc1Frame& frame, std::size_t offset,
               std::uint32_t value) noexcept {
    frame[offset] = static_cast<std::uint8_t>(value & 0xFFU);
    frame[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    frame[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    frame[offset + 3U] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

void storeFloatLe(Avc1Frame& frame, std::size_t offset, float value) noexcept {
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "AVC1 requires 32-bit float");
    static_assert(std::numeric_limits<float>::is_iec559,
                  "AVC1 requires IEEE-754 float");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    storeLe32(frame, offset, bits);
}

}  // namespace

std::uint32_t avc1Checksum(const std::uint8_t* data,
                           std::size_t size) noexcept {
    std::uint32_t value = 0xA5A51234U;
    for (std::size_t i = 0; i < size; ++i) {
        value = (value << 5U) ^ (value >> 2U) ^ data[i];
    }
    return value;
}

Avc1Frame packAvc1Frame(std::uint32_t sequence, float vx_mps, float vy_mps,
                        bool valid) noexcept {
    Avc1Frame frame{};
    if (!std::isfinite(vx_mps) || !std::isfinite(vy_mps)) valid = false;
    if (!valid) {
        vx_mps = 0.0F;
        vy_mps = 0.0F;
    } else {
        vx_mps = std::clamp(vx_mps, -kAvc1MaximumAxisSpeedMps,
                            kAvc1MaximumAxisSpeedMps);
        vy_mps = std::clamp(vy_mps, -kAvc1MaximumAxisSpeedMps,
                            kAvc1MaximumAxisSpeedMps);
    }

    storeLe32(frame, 0U, kAvc1Magic);
    storeLe32(frame, 4U, sequence);
    storeFloatLe(frame, 8U, vx_mps);
    storeFloatLe(frame, 12U, vy_mps);
    frame[16U] = valid ? 1U : 0U;
    // Bytes 17..19 remain zero from value initialization.
    storeLe32(frame, 20U, avc1Checksum(frame.data(), kAvc1PayloadSize));
    return frame;
}

}  // namespace maixcam_marker
