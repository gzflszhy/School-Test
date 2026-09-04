#include "lateral_controller.hpp"

#include <cassert>
#include <chrono>

using namespace maixcam_marker;

namespace {

DetectionResult markerAt(float center_x, SteadyTimePoint timestamp) {
    DetectionResult result;
    result.found = true;
    result.capture_timestamp = timestamp;
    result.right_angle_label = 1;
    result.canonical_x_label = 0;
    result.large_l_centers = {{{center_x - 25.0F, 85.0F},
                               {center_x - 25.0F, 135.0F},
                               {center_x + 25.0F, 135.0F}}};
    result.marker_scale_px = 50.0F;
    result.center = {center_x, 110.0F};
    return result;
}

}  // namespace

int main() {
    LateralControlConfig config;
    config.optional_refinement_enabled = false;
    config.max_command_acceleration_mps2 = 100.0F;
    LateralController controller(config);
    const auto start = std::chrono::steady_clock::now();

    const auto centered = controller.update(markerAt(240.0F, start));
    assert(centered.valid);
    assert(centered.source == LateralControlSource::MEASURED);
    assert(centered.vy_mps == 0.0F);

    controller.reset();
    const auto target_right = controller.update(markerAt(270.0F, start));
    assert(target_right.valid);
    assert(target_right.raw_lateral_error_m < 0.0F);
    assert(target_right.vy_mps < 0.0F);

    controller.reset();
    const auto target_left = controller.update(markerAt(210.0F, start));
    assert(target_left.valid);
    assert(target_left.raw_lateral_error_m > 0.0F);
    assert(target_left.vy_mps > 0.0F);

    DetectionResult missing;
    missing.capture_timestamp = start + std::chrono::milliseconds(50);
    const auto predicted = controller.update(missing);
    assert(predicted.valid);
    assert(predicted.source == LateralControlSource::PREDICTED);

    missing.capture_timestamp = start + std::chrono::milliseconds(160);
    const auto stopped = controller.update(missing);
    assert(!stopped.valid);
    assert(stopped.vy_mps == 0.0F);
    return 0;
}
