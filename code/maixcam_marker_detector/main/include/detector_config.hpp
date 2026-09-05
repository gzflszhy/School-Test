#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

namespace maixcam_marker {

// All tunable values live here so field calibration never requires changing
// the detector implementation. Pixel limits refer to the 480x270 input image.
struct DetectorConfig {
    // GC4653 is sampled at 720p/60 by the ISP and scaled to this lower-cost
    // detector stream. The target is expected to remain reasonably close.
    int frame_width = 480;
    int frame_height = 270;
    int requested_fps = 60;
    int camera_buffer_count = 1;
    int camera_warmup_frames = 30;
    bool use_auto_exposure = true;
    int exposure_us = 1500;

    int min_marker_width_px = 28;
    int max_marker_width_px = 210;
    int min_component_size_px = 2;
    int max_component_size_px = 68;
    int min_component_area_px = 7;
    int max_component_area_px = 2813;
    int max_led_candidates = 14;
    int max_topology_candidates = 9;
    // Only this many contours reach convexHull/matchShapes.
    int max_pre_shape_candidates = 18;

    // LED segmentation. The final threshold is derived from Otsu, a high
    // percentile and the local mean, then clamped to this interval.
    float bright_percentile = 0.92F;
    bool use_fixed_led_threshold = false;
    int fixed_led_threshold = 180;
    int min_led_threshold = 105;
    int max_led_threshold = 250;
    int local_contrast_threshold = 24;
    int saturation_threshold = 250;
    int morphology_kernel = 3;

    // Large-L contour filtering and scoring.
    float component_aspect_min = 0.45F;
    float component_aspect_max = 2.20F;
    float component_fill_min = 0.12F;
    float component_fill_max = 0.94F;
    float component_solidity_min = 0.35F;
    float component_solidity_max = 0.99F;
    float max_shape_distance = 1.60F;
    float min_component_score = 0.16F;
    float component_fill_target = 0.46F;
    float component_fill_score_tolerance = 0.36F;
    float component_concavity_score_span = 0.30F;
    float candidate_shape_score_weight = 0.50F;
    float candidate_fill_score_weight = 0.25F;
    float candidate_concavity_score_weight = 0.25F;
    float pre_shape_square_weight = 0.55F;
    float pre_shape_fill_weight = 0.45F;

    // Large-L center topology, expressed in units of mean L outer size.
    // CAD: adjacent large-L bounding-box centres are 50 mm apart while each
    // L has a 30 mm outer size, giving 50/30 = 1.6667.
    float expected_leg_scale = 1.6667F;
    float leg_scale_tolerance = 0.60F;
    // Perspective and motion blur need margin, while these bounds still
    // reject visibly non-isosceles/non-right triples.
    float leg_similarity_tolerance = 0.35F;
    float right_angle_cosine_max = 0.35F;
    float pythagorean_relative_error_max = 0.30F;
    float component_scale_ratio_max = 2.20F;
    float component_intensity_delta_max = 120.0F;
    float topology_shape_subweight = 0.22F;
    float topology_angle_subweight = 0.25F;
    float topology_leg_similarity_subweight = 0.18F;
    float topology_pythagorean_subweight = 0.18F;
    float topology_scale_subweight = 0.17F;

    int canonical_size = 128;
    int template_dilate_px = 2;
    float min_large_component_coverage = 0.42F;
    float min_small_component_coverage = 0.30F;
    float min_led_board_contrast = 18.0F;
    float contrast_score_full_scale = 3.0F;
    float hypothesis_max_width_factor = 1.35F;

    // A valid three-L geometry is accepted. Template, contrast, optional marks
    // and temporal continuity only describe confidence/ranking.
    float topology_weight = 0.70F;
    float template_weight = 0.20F;
    float contrast_weight = 0.10F;
    float optional_evidence_weight = 0.05F;
    float temporal_weight = 0.10F;
    float no_history_temporal_score = 0.50F;
    float temporal_min_normalizer_px = 8.0F;
    float full_id_threshold = 0.72F;
    int full_id_min_components = 5;

    float tracking_roi_scale = 2.10F;
    float tracking_roi_lost_scale = 3.00F;
    float max_tracking_displacement_bbox = 1.50F;
    float prediction_horizon_seconds = 0.25F;
    float velocity_update_max_dt_seconds = 0.50F;
    float previous_velocity_weight = 0.55F;
    int search_confirm_frames = 2;
    int expand_after_lost_frames = 1;
    int return_to_search_lost_frames = 6;

    float optical_center_x = 240.0F;
    bool benchmark_enabled = true;
    std::size_t benchmark_window = 600;

    void normalize() {
        frame_width = std::max(1, frame_width);
        frame_height = std::max(1, frame_height);
        requested_fps = std::max(1, requested_fps);
        camera_buffer_count = std::max(1, camera_buffer_count);
        morphology_kernel = std::max(1, morphology_kernel | 1);
        canonical_size = std::max(64, canonical_size);
        min_marker_width_px = std::max(1, min_marker_width_px);
        max_marker_width_px = std::max(min_marker_width_px, max_marker_width_px);
        min_component_size_px = std::max(1, min_component_size_px);
        max_component_size_px = std::max(min_component_size_px, max_component_size_px);
        min_component_area_px = std::max(1, min_component_area_px);
        max_component_area_px = std::max(min_component_area_px, max_component_area_px);
        max_led_candidates = std::max(3, max_led_candidates);
        max_topology_candidates = std::clamp(max_topology_candidates, 3,
                                              max_led_candidates);
        max_pre_shape_candidates = std::max(max_led_candidates,
                                             max_pre_shape_candidates);
        search_confirm_frames = std::max(1, search_confirm_frames);
        expand_after_lost_frames = std::max(1, expand_after_lost_frames);
        return_to_search_lost_frames = std::max(expand_after_lost_frames,
                                                return_to_search_lost_frames);
        bright_percentile = std::clamp(bright_percentile, 0.50F, 0.999F);
        fixed_led_threshold = std::clamp(fixed_led_threshold, 0, 255);
        min_led_threshold = std::clamp(min_led_threshold, 0, 255);
        max_led_threshold = std::clamp(max_led_threshold, min_led_threshold, 255);
        saturation_threshold = std::clamp(saturation_threshold, 0, 255);
        max_shape_distance = std::max(0.01F, max_shape_distance);
        component_fill_score_tolerance = std::max(0.01F,
                                                   component_fill_score_tolerance);
        component_concavity_score_span = std::max(0.01F,
                                                   component_concavity_score_span);
        leg_scale_tolerance = std::max(0.01F, leg_scale_tolerance);
        leg_similarity_tolerance = std::max(0.01F, leg_similarity_tolerance);
        right_angle_cosine_max = std::max(0.01F, right_angle_cosine_max);
        pythagorean_relative_error_max = std::max(0.01F,
                                                   pythagorean_relative_error_max);
        min_large_component_coverage = std::max(0.01F, min_large_component_coverage);
        min_small_component_coverage = std::max(0.01F, min_small_component_coverage);
        prediction_horizon_seconds = std::max(0.0F, prediction_horizon_seconds);
        velocity_update_max_dt_seconds = std::max(0.01F,
                                                   velocity_update_max_dt_seconds);
        full_id_threshold = std::clamp(full_id_threshold, 0.0F, 1.0F);
        optical_center_x = std::clamp(optical_center_x, 0.0F,
                                      static_cast<float>(frame_width));
        benchmark_window = std::max<std::size_t>(16, benchmark_window);
        previous_velocity_weight = std::clamp(previous_velocity_weight, 0.0F, 1.0F);
    }
};

// Runtime field-tuning overrides: one `key=value` per line, '#' comments.
// Only camera/exposure and bright-region extraction parameters are accepted.
bool loadDetectorConfig(const std::string& path, DetectorConfig& config,
                        std::string& error);

}  // namespace maixcam_marker
