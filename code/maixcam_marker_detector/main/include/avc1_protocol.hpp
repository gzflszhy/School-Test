#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace maixcam_marker {

constexpr std::uint32_t kAvc1Magic = 0x31435641U;
constexpr std::size_t kAvc1PayloadSize = 20U;
constexpr std::size_t kAvc1FrameSize = 24U;
constexpr float kAvc1MaximumAxisSpeedMps = 0.8F;
using Avc1Frame = std::array<std::uint8_t, kAvc1FrameSize>;

std::uint32_t avc1Checksum(const std::uint8_t* data, std::size_t size) noexcept;
Avc1Frame packAvc1Frame(std::uint32_t sequence, float vx_mps, float vy_mps,
                        bool valid) noexcept;

}  // namespace maixcam_marker
