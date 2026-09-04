#include "lateral_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace maixcam_marker {
namespace {

float distance(const cv::Point2f& lhs, const cv::Point2f& rhs) {
    return cv::norm(lhs - rhs);
}

std::string trim(std::string value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

float median(std::vector<float> values) {
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if ((values.size() & 1U) != 0U) return values[middle];
    return 0.5F * (values[middle - 1] + values[middle]);
}

bool parseFloat(const std::string& text, float& value) {
    try {
        std::size_t consumed = 0;
        value = std::stof(text, &consumed);
        return consumed == text.size() && std::isfinite(value);
    } catch (...) {
        return false;
    }
}

}  // namespace

void LateralControlConfig::normalize() {
    focal_x_px = std::max(1.0F, focal_x_px);
    principal_x_px = std::max(0.0F, principal_x_px);
    marker_leg_m = std::max(0.001F, marker_leg_m);
    min_marker_scale_px = std::max(1.0F, min_marker_scale_px);
    min_distance_m = std::max(0.01F, min_distance_m);
    max_distance_m = std::max(min_distance_m, max_distance_m);
    optional_refinement_min_components = std::clamp(
        optional_refinement_min_components, 1, 3);
    optional_residual_gate_scale = std::max(0.001F, optional_residual_gate_scale);
    optional_consensus_gate_scale = std::max(0.001F, optional_consensus_gate_scale);
    optional_max_correction_scale = std::max(0.0F, optional_max_correction_scale);
    optional_correction_weight = std::clamp(optional_correction_weight, 0.0F, 1.0F);
    filter_alpha = std::clamp(filter_alpha, 0.0F, 1.0F);
    filter_beta = std::clamp(filter_beta, 0.0F, 1.0F);
    filter_min_dt_s = std::max(0.001F, filter_min_dt_s);
    filter_max_dt_s = std::max(filter_min_dt_s, filter_max_dt_s);
    filter_reset_gap_s = std::max(filter_max_dt_s, filter_reset_gap_s);
    max_prediction_age_s = std::clamp(max_prediction_age_s, 0.0F,
                                      filter_reset_gap_s);
    innovation_gate_base_m = std::max(0.001F, innovation_gate_base_m);
    innovation_gate_distance_scale = std::max(0.0F, innovation_gate_distance_scale);
    max_relative_velocity_mps = std::max(0.01F, max_relative_velocity_mps);
    position_deadband_m = std::max(0.0F, position_deadband_m);
    lateral_kp_per_s = std::max(0.0F, lateral_kp_per_s);
    lateral_ki_per_s2 = std::max(0.0F, lateral_ki_per_s2);
    relative_velocity_gain = std::max(0.0F, relative_velocity_gain);
    max_vy_mps = std::clamp(max_vy_mps, 0.01F, 0.8F);
    max_integral_vy_mps = std::clamp(max_integral_vy_mps, 0.0F, max_vy_mps);
    max_command_acceleration_mps2 = std::max(0.01F,
                                             max_command_acceleration_mps2);
    nominal_update_period_s = std::clamp(nominal_update_period_s,
                                         filter_min_dt_s, filter_max_dt_s);
    search_speed_mps = std::clamp(search_speed_mps, 0.0F, max_vy_mps);
    search_first_leg_s = std::max(nominal_update_period_s, search_first_leg_s);
    search_max_duration_s = std::max(search_first_leg_s, search_max_duration_s);
    search_velocity_direction_threshold_mps = std::max(
        0.0F, search_velocity_direction_threshold_mps);
}

LateralController::LateralController(LateralControlConfig config)
    : config_(std::move(config)) {
    config_.normalize();
    const auto& components = geometry_.components();
    for (std::size_t optional = 0; optional < optional_centroids_mm_.size(); ++optional) {
        const cv::Moments moments = cv::moments(components[optional + 3].polygon_mm);
        if (std::abs(moments.m00) > std::numeric_limits<double>::epsilon()) {
            optional_centroids_mm_[optional] = {
                static_cast<float>(moments.m10 / moments.m00),
                static_cast<float>(moments.m01 / moments.m00)};
        } else {
            optional_centroids_mm_[optional] = components[optional + 3].center_mm;
        }
    }
}

void LateralController::clearTrackingState(bool clear_command) noexcept {
    initialized_ = false;
    filtered_position_m_ = 0.0F;
    filtered_velocity_mps_ = 0.0F;
    integral_command_mps_ = 0.0F;
    if (clear_command) last_command_mps_ = 0.0F;
    last_measurement_ = {};
}

void LateralController::reset() noexcept {
    clearTrackingState(true);
    last_update_ = {};
    has_target_history_ = false;
    search_active_ = false;
    search_initial_direction_ = 1.0F;
    last_observed_position_m_ = 0.0F;
    last_observed_velocity_mps_ = 0.0F;
    search_started_ = {};
}

bool LateralController::makeMeasurement(const DetectionResult& detection,
                                        LateralControlOutput& output) const {
    if (!detection.found || detection.right_angle_label < 0 ||
        detection.right_angle_label >= 3 || detection.canonical_x_label < 0 ||
        detection.canonical_x_label >= 3 ||
        detection.right_angle_label == detection.canonical_x_label ||
        detection.marker_scale_px < config_.min_marker_scale_px) {
        return false;
    }

    const int r_label = detection.right_angle_label;
    const int x_label = detection.canonical_x_label;
    int y_label = -1;
    for (int label = 0; label < 3; ++label) {
        if (label != r_label && label != x_label) y_label = label;
    }
    if (y_label < 0) return false;

    const cv::Point2f& r = detection.large_l_centers[static_cast<std::size_t>(r_label)];
    const cv::Point2f& x = detection.large_l_centers[static_cast<std::size_t>(x_label)];
    const cv::Point2f& y = detection.large_l_centers[static_cast<std::size_t>(y_label)];
    const float leg_x_px = distance(r, x);
    const float leg_y_px = distance(r, y);
    const float marker_scale_px = 0.5F * (leg_x_px + leg_y_px);
    if (!std::isfinite(marker_scale_px) ||
        marker_scale_px < config_.min_marker_scale_px) return false;

    cv::Point2f refined_center = 0.5F * (x + y);
    output.geometric_center_x_px = refined_center.x;
    output.marker_scale_px = marker_scale_px;

    if (config_.optional_refinement_enabled) {
        const cv::Point2f pixels_per_mm_x = (x - r) / 50.0F;
        const cv::Point2f pixels_per_mm_y = (y - r) / 50.0F;
        std::vector<cv::Point2f> residuals;
        residuals.reserve(3);
        const float residual_gate_px = std::max(
            1.0F, config_.optional_residual_gate_scale * marker_scale_px);
        for (std::size_t optional = 0;
             optional < detection.optional_component_detected.size(); ++optional) {
            if (!detection.optional_component_detected[optional]) continue;
            const cv::Point2f& marker_point = optional_centroids_mm_[optional];
            const cv::Point2f expected = r +
                pixels_per_mm_x * (marker_point.x + 25.0F) +
                pixels_per_mm_y * (marker_point.y + 25.0F);
            const cv::Point2f residual =
                detection.optional_component_centers[optional] - expected;
            if (cv::norm(residual) <= residual_gate_px) residuals.push_back(residual);
        }

        if (residuals.size() >= static_cast<std::size_t>(
                                  config_.optional_refinement_min_components)) {
            std::vector<float> residual_x;
            std::vector<float> residual_y;
            residual_x.reserve(residuals.size());
            residual_y.reserve(residuals.size());
            for (const auto& residual : residuals) {
                residual_x.push_back(residual.x);
                residual_y.push_back(residual.y);
            }
            const cv::Point2f consensus{median(residual_x), median(residual_y)};
            const float consensus_gate_px = std::max(
                0.5F, config_.optional_consensus_gate_scale * marker_scale_px);
            cv::Point2f correction{};
            int inliers = 0;
            for (const auto& residual : residuals) {
                if (cv::norm(residual - consensus) <= consensus_gate_px) {
                    correction += residual;
                    ++inliers;
                }
            }
            if (inliers >= config_.optional_refinement_min_components) {
                correction *= 1.0F / static_cast<float>(inliers);
                const float max_correction_px =
                    config_.optional_max_correction_scale * marker_scale_px;
                const float correction_norm = cv::norm(correction);
                if (correction_norm > max_correction_px && correction_norm > 0.0F) {
                    correction *= max_correction_px / correction_norm;
                }
                correction *= config_.optional_correction_weight;
                refined_center += correction;
                output.optional_refinement_used = true;
                output.optional_refinement_components = inliers;
                output.optional_correction_x_px = correction.x;
            }
        }
    }

    const float distance_m = config_.focal_x_px * config_.marker_leg_m /
                             marker_scale_px;
    if (!std::isfinite(distance_m) || distance_m < config_.min_distance_m ||
        distance_m > config_.max_distance_m) return false;
    output.distance_m = distance_m;
    output.refined_center_x_px = refined_center.x;
    // Camera image X is right-positive; AVC1 vy is vehicle-left-positive.
    output.raw_lateral_error_m =
        -(refined_center.x - config_.principal_x_px) * distance_m /
        config_.focal_x_px;
    return std::isfinite(output.raw_lateral_error_m);
}

float LateralController::limitedCommand(float requested, float dt_s,
                                        bool& saturated,
                                        bool& acceleration_limited) {
    const float speed_limited =
        std::clamp(requested, -config_.max_vy_mps, config_.max_vy_mps);
    saturated = speed_limited != requested;
    const float maximum_change = config_.max_command_acceleration_mps2 * dt_s;
    const float requested_change = speed_limited - last_command_mps_;
    const float limited_change = std::clamp(requested_change,
                                            -maximum_change, maximum_change);
    acceleration_limited = limited_change != requested_change;
    last_command_mps_ += limited_change;
    return last_command_mps_;
}

LateralControlOutput LateralController::searchCommand(
    LateralControlOutput output, SteadyTimePoint now, float dt_s) {
    if (!search_active_) {
        float direction_source = last_observed_position_m_;
        if (std::abs(last_observed_velocity_mps_) >=
            config_.search_velocity_direction_threshold_mps) {
            direction_source = last_observed_velocity_mps_;
        } else if (std::abs(direction_source) <= config_.position_deadband_m &&
                   std::abs(last_command_mps_) > 0.001F) {
            direction_source = last_command_mps_;
        }
        search_initial_direction_ = direction_source < 0.0F ? -1.0F : 1.0F;
        search_started_ = now;
        search_active_ = true;
        clearTrackingState(false);
    }

    const float elapsed_s = std::max(
        0.0F, static_cast<float>(
                  std::chrono::duration<double>(now - search_started_).count()));
    if (elapsed_s > config_.search_max_duration_s ||
        config_.search_speed_mps <= 0.0F) {
        reset();
        return output;
    }

    int leg = 0;
    float direction = search_initial_direction_;
    if (elapsed_s >= config_.search_first_leg_s) {
        leg = 1 + static_cast<int>(std::floor(
            (elapsed_s - config_.search_first_leg_s) /
            (2.0F * config_.search_first_leg_s)));
        direction = (leg & 1) != 0
            ? -search_initial_direction_ : search_initial_direction_;
    }

    output.valid = true;
    output.source = LateralControlSource::SEARCHING;
    output.search_elapsed_s = elapsed_s;
    output.search_leg = leg;
    output.unconstrained_vy_mps = direction * config_.search_speed_mps;
    output.vy_mps = limitedCommand(output.unconstrained_vy_mps, dt_s,
                                   output.command_saturated,
                                   output.acceleration_limited);
    return output;
}

LateralControlOutput LateralController::update(const DetectionResult& detection) {
    LateralControlOutput output;
    const bool measurement_valid = makeMeasurement(detection, output);
    const SteadyTimePoint now = detection.capture_timestamp;
    float dt_s = config_.nominal_update_period_s;
    if (last_update_ != SteadyTimePoint{}) {
        dt_s = static_cast<float>(std::chrono::duration<double>(now - last_update_).count());
    }

    if (measurement_valid && search_active_) {
        search_active_ = false;
        search_started_ = {};
        clearTrackingState(false);
    }

    if (measurement_valid &&
        (!initialized_ || !std::isfinite(dt_s) || dt_s <= 0.0F ||
         dt_s > config_.filter_reset_gap_s)) {
        if (initialized_) {
            last_command_mps_ = 0.0F;
            integral_command_mps_ = 0.0F;
        }
        initialized_ = true;
        filtered_position_m_ = output.raw_lateral_error_m;
        filtered_velocity_mps_ = 0.0F;
        last_measurement_ = now;
        dt_s = config_.nominal_update_period_s;
        output.source = LateralControlSource::MEASURED;
    } else if (initialized_) {
        dt_s = std::clamp(dt_s, config_.filter_min_dt_s, config_.filter_max_dt_s);
        filtered_position_m_ += filtered_velocity_mps_ * dt_s;
        if (measurement_valid) {
            const float innovation = output.raw_lateral_error_m - filtered_position_m_;
            const float gate = config_.innovation_gate_base_m +
                config_.innovation_gate_distance_scale * std::max(0.0F, output.distance_m);
            if (std::abs(innovation) <= gate) {
                filtered_position_m_ += config_.filter_alpha * innovation;
                filtered_velocity_mps_ += config_.filter_beta * innovation / dt_s;
                filtered_velocity_mps_ = std::clamp(
                    filtered_velocity_mps_, -config_.max_relative_velocity_mps,
                    config_.max_relative_velocity_mps);
                last_measurement_ = now;
                output.source = LateralControlSource::MEASURED;
            }
        }
        if (output.source == LateralControlSource::INVALID &&
            last_measurement_ != SteadyTimePoint{}) {
            const double prediction_age =
                std::chrono::duration<double>(now - last_measurement_).count();
            if (prediction_age >= 0.0 && prediction_age <= config_.max_prediction_age_s) {
                output.source = LateralControlSource::PREDICTED;
            }
        }
    }

    last_update_ = now;
    output.valid = initialized_ && output.source != LateralControlSource::INVALID;
    if (!output.valid) {
        if (config_.search_enabled && (search_active_ || has_target_history_)) {
            return searchCommand(std::move(output), now, dt_s);
        }
        reset();
        return output;
    }

    output.filtered_lateral_error_m = filtered_position_m_;
    output.relative_lateral_velocity_mps = filtered_velocity_mps_;
    if (output.source == LateralControlSource::MEASURED) {
        has_target_history_ = true;
        last_observed_position_m_ = filtered_position_m_;
        last_observed_velocity_mps_ = filtered_velocity_mps_;
    }
    float controlled_position = filtered_position_m_;
    if (std::abs(controlled_position) <= config_.position_deadband_m) {
        controlled_position = 0.0F;
    } else {
        controlled_position = std::copysign(
            std::abs(controlled_position) - config_.position_deadband_m,
            controlled_position);
    }
    output.vy_position_mps = config_.lateral_kp_per_s * controlled_position;
    output.vy_velocity_mps =
        config_.relative_velocity_gain * filtered_velocity_mps_;

    // The integral term retains the chassis velocity needed to keep a moving
    // target centered after its image-relative velocity has fallen to zero.
    // Only real measurements update it; short prediction gaps hold the last
    // learned value. Conditional integration prevents wind-up at the speed
    // limit while still allowing an opposing error to unwind the integral.
    if (output.source == LateralControlSource::MEASURED &&
        config_.lateral_ki_per_s2 > 0.0F && controlled_position != 0.0F) {
        const float delta =
            config_.lateral_ki_per_s2 * controlled_position * dt_s;
        const float candidate = std::clamp(
            integral_command_mps_ + delta, -config_.max_integral_vy_mps,
            config_.max_integral_vy_mps);
        const float candidate_command =
            output.vy_position_mps + candidate + output.vy_velocity_mps;
        const bool pushes_positive_saturation =
            candidate_command > config_.max_vy_mps && delta > 0.0F;
        const bool pushes_negative_saturation =
            candidate_command < -config_.max_vy_mps && delta < 0.0F;
        if (!pushes_positive_saturation && !pushes_negative_saturation) {
            integral_command_mps_ = candidate;
        }
    }
    output.vy_integral_mps = integral_command_mps_;
    output.unconstrained_vy_mps = output.vy_position_mps +
        output.vy_integral_mps + output.vy_velocity_mps;
    output.vy_mps = limitedCommand(output.unconstrained_vy_mps, dt_s,
                                   output.command_saturated,
                                   output.acceleration_limited);
    return output;
}

bool loadLateralControlConfig(const std::string& path, LateralControlConfig& config,
                              std::string& error) {
    std::ifstream input(path);
    if (!input) {
        error = "cannot open control config: " + path;
        return false;
    }
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find('#');
        if (comment != std::string::npos) line.erase(comment);
        line = trim(line);
        if (line.empty()) continue;
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            error = "control config line " + std::to_string(line_number) +
                    " has no '='";
            return false;
        }
        const std::string key = trim(line.substr(0, equals));
        const std::string text = trim(line.substr(equals + 1));
        float value = 0.0F;
        if (!parseFloat(text, value)) {
            error = "invalid value for '" + key + "' on line " +
                    std::to_string(line_number);
            return false;
        }

#define SET_FLOAT(name) if (key == #name) { config.name = value; continue; }
        SET_FLOAT(focal_x_px)
        SET_FLOAT(principal_x_px)
        SET_FLOAT(marker_leg_m)
        SET_FLOAT(min_marker_scale_px)
        SET_FLOAT(min_distance_m)
        SET_FLOAT(max_distance_m)
        SET_FLOAT(optional_residual_gate_scale)
        SET_FLOAT(optional_consensus_gate_scale)
        SET_FLOAT(optional_max_correction_scale)
        SET_FLOAT(optional_correction_weight)
        SET_FLOAT(filter_alpha)
        SET_FLOAT(filter_beta)
        SET_FLOAT(filter_min_dt_s)
        SET_FLOAT(filter_max_dt_s)
        SET_FLOAT(filter_reset_gap_s)
        SET_FLOAT(max_prediction_age_s)
        SET_FLOAT(innovation_gate_base_m)
        SET_FLOAT(innovation_gate_distance_scale)
        SET_FLOAT(max_relative_velocity_mps)
        SET_FLOAT(position_deadband_m)
        SET_FLOAT(lateral_kp_per_s)
        SET_FLOAT(lateral_ki_per_s2)
        SET_FLOAT(max_integral_vy_mps)
        SET_FLOAT(relative_velocity_gain)
        SET_FLOAT(max_vy_mps)
        SET_FLOAT(max_command_acceleration_mps2)
        SET_FLOAT(nominal_update_period_s)
        SET_FLOAT(search_speed_mps)
        SET_FLOAT(search_first_leg_s)
        SET_FLOAT(search_max_duration_s)
        SET_FLOAT(search_velocity_direction_threshold_mps)
#undef SET_FLOAT
        if (key == "optional_refinement_enabled") {
            if (value != 0.0F && value != 1.0F) {
                error = "optional_refinement_enabled must be 0 or 1";
                return false;
            }
            config.optional_refinement_enabled = value != 0.0F;
            continue;
        }
        if (key == "search_enabled") {
            if (value != 0.0F && value != 1.0F) {
                error = "search_enabled must be 0 or 1";
                return false;
            }
            config.search_enabled = value != 0.0F;
            continue;
        }
        if (key == "optional_refinement_min_components") {
            config.optional_refinement_min_components = static_cast<int>(value);
            continue;
        }
        error = "unknown control config key '" + key + "' on line " +
                std::to_string(line_number);
        return false;
    }
    config.normalize();
    return true;
}

}  // namespace maixcam_marker
