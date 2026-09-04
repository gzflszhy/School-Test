#pragma once

#include <array>
#include <chrono>
#include <string>
#include <string_view>

#include "detection_result.hpp"
#include "marker_geometry.hpp"

namespace maixcam_marker {

struct LateralControlConfig {
    // Nominal GC4653 3.05 mm lens at 480x270. Replace focal_x_px with the
    // calibrated value without changing the controller implementation.
    float focal_x_px = 281.5F;
    float principal_x_px = 240.0F;
    float marker_leg_m = 0.050F;
    float min_marker_scale_px = 12.0F;
    float min_distance_m = 0.08F;
    float max_distance_m = 3.00F;

    bool optional_refinement_enabled = true;
    int optional_refinement_min_components = 2;
    float optional_residual_gate_scale = 0.10F;
    float optional_consensus_gate_scale = 0.05F;
    float optional_max_correction_scale = 0.04F;
    float optional_correction_weight = 0.35F;

    // Alpha-beta constant-velocity filter. beta is divided by dt when updating
    // velocity, so keep it substantially smaller than alpha.
    float filter_alpha = 0.35F;
    float filter_beta = 0.06F;
    float filter_min_dt_s = 0.010F;
    float filter_max_dt_s = 0.120F;
    float filter_reset_gap_s = 0.250F;
    float max_prediction_age_s = 0.100F;
    float innovation_gate_base_m = 0.030F;
    float innovation_gate_distance_scale = 0.15F;
    float max_relative_velocity_mps = 1.50F;

    // vy is positive to vehicle-left. These are deliberately conservative
    // defaults for log-only validation before the first chassis test.
    float position_deadband_m = 0.005F;
    float lateral_kp_per_s = 2.50F;
    float relative_velocity_gain = 0.85F;
    float max_vy_mps = 0.60F;
    float max_command_acceleration_mps2 = 2.00F;
    float nominal_update_period_s = 0.040F;

    void normalize();
};

enum class LateralControlSource { INVALID, MEASURED, PREDICTED };

constexpr std::string_view toString(LateralControlSource source) noexcept {
    switch (source) {
        case LateralControlSource::MEASURED: return "MEASURED";
        case LateralControlSource::PREDICTED: return "PREDICTED";
        default: return "INVALID";
    }
}

struct LateralControlOutput {
    bool valid = false;
    LateralControlSource source = LateralControlSource::INVALID;
    float marker_scale_px = 0.0F;
    float distance_m = -1.0F;
    float geometric_center_x_px = -1.0F;
    float refined_center_x_px = -1.0F;
    bool optional_refinement_used = false;
    int optional_refinement_components = 0;
    float optional_correction_x_px = 0.0F;
    float raw_lateral_error_m = 0.0F;
    float filtered_lateral_error_m = 0.0F;
    float relative_lateral_velocity_mps = 0.0F;
    float unconstrained_vy_mps = 0.0F;
    float vy_mps = 0.0F;
};

class LateralController {
public:
    explicit LateralController(LateralControlConfig config = {});

    LateralControlOutput update(const DetectionResult& detection);
    void reset() noexcept;
    const LateralControlConfig& config() const noexcept { return config_; }

private:
    bool makeMeasurement(const DetectionResult& detection,
                         LateralControlOutput& output) const;
    float limitedCommand(float requested, float dt_s);

    LateralControlConfig config_;
    MarkerGeometry geometry_;
    std::array<cv::Point2f, 3> optional_centroids_mm_{};
    bool initialized_ = false;
    float filtered_position_m_ = 0.0F;
    float filtered_velocity_mps_ = 0.0F;
    float last_command_mps_ = 0.0F;
    SteadyTimePoint last_update_{};
    SteadyTimePoint last_measurement_{};
};

// Optional runtime override file: one `key=value` per line, '#' comments.
bool loadLateralControlConfig(const std::string& path, LateralControlConfig& config,
                              std::string& error);

}  // namespace maixcam_marker
