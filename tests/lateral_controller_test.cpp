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

    DetectionResult initially_missing;
    initially_missing.capture_timestamp = start;
    LateralController never_locked_controller(config);
    const auto never_locked = never_locked_controller.update(initially_missing);
    assert(!never_locked.valid);
    assert(never_locked.source == LateralControlSource::INVALID);

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

    LateralControlConfig integral_config = config;
    integral_config.position_deadband_m = 0.0F;
    integral_config.relative_velocity_gain = 0.0F;
    integral_config.filter_alpha = 1.0F;
    integral_config.filter_beta = 0.0F;
    LateralController integral_controller(integral_config);
    LateralControlOutput integrating;
    for (int frame = 0; frame < 25; ++frame) {
        integrating = integral_controller.update(markerAt(
            210.0F, start + std::chrono::milliseconds(40 * frame)));
    }
    assert(integrating.vy_integral_mps > 0.0F);
    const auto centered_after_motion = integral_controller.update(markerAt(
        240.0F, start + std::chrono::milliseconds(1000)));
    assert(centered_after_motion.vy_position_mps == 0.0F);
    assert(centered_after_motion.vy_integral_mps > 0.0F);
    assert(centered_after_motion.vy_mps > 0.0F);
    integral_controller.reset();
    const auto centered_after_reset = integral_controller.update(markerAt(
        240.0F, start + std::chrono::milliseconds(1040)));
    assert(centered_after_reset.vy_integral_mps == 0.0F);
    assert(centered_after_reset.vy_mps == 0.0F);

    DetectionResult missing;
    missing.capture_timestamp = start + std::chrono::milliseconds(50);
    const auto predicted = controller.update(missing);
    assert(predicted.valid);
    assert(predicted.source == LateralControlSource::PREDICTED);

    missing.capture_timestamp = start + std::chrono::milliseconds(160);
    const auto searching = controller.update(missing);
    assert(searching.valid);
    assert(searching.source == LateralControlSource::SEARCHING);
    assert(searching.vy_mps > 0.0F);

    const auto reacquired = controller.update(markerAt(
        240.0F, start + std::chrono::milliseconds(200)));
    assert(reacquired.valid);
    assert(reacquired.source == LateralControlSource::MEASURED);

    missing.capture_timestamp = start + std::chrono::milliseconds(400);
    const auto searching_again = controller.update(missing);
    assert(searching_again.source == LateralControlSource::SEARCHING);
    missing.capture_timestamp = start + std::chrono::milliseconds(8500);
    const auto search_timeout = controller.update(missing);
    assert(!search_timeout.valid);
    assert(search_timeout.vy_mps == 0.0F);
    return 0;
}
