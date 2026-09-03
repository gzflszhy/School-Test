#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace maixcam_marker {

// All tunable values live here so field calibration never requires changing
// the detector implementation. Pixel limits refer to the 640x360 input image.
struct DetectorConfig {
    static constexpr int kMaxTotalTransformEvaluations = 10;

    int frame_width = 640;
    int frame_height = 360;
    int requested_fps = 60;
    int camera_buffer_count = 1;
    int camera_warmup_frames = 30;
    bool use_auto_exposure = true;
    int exposure_us = 1500;
    float analogue_gain = 1.0F;
    float digital_gain = 1.0F;

    int min_marker_width_px = 38;
    int max_marker_width_px = 280;
    int min_component_size_px = 3;
    int max_component_size_px = 90;
    int min_component_area_px = 12;
    int max_component_area_px = 5000;
    int max_led_candidates = 14;
    int max_topology_candidates = 9;
    // Only this many contours reach convexHull/matchShapes.
    int max_pre_shape_candidates = 18;
    // Full 160x160 warp/template evaluations. normalize() enforces the
    // non-overridable total cap kMaxTotalTransformEvaluations; the default
    // split is 6 LED + 4 board.
    int max_led_transform_evaluations = 6;

    // LED segmentation. The final threshold is derived from Otsu, a high
    // percentile and the local mean, then clamped to this interval.
    float bright_percentile = 0.92F;
    int min_led_threshold = 105;
    int max_led_threshold = 250;
    int local_contrast_threshold = 24;
    int saturation_threshold = 250;
    float max_saturation_fraction = 0.32F;
    int morphology_kernel = 3;

    // Large-L contour filtering and scoring.
    float component_aspect_min = 0.55F;
    float component_aspect_max = 1.80F;
    float component_fill_min = 0.20F;
    float component_fill_max = 0.82F;
    float component_solidity_min = 0.45F;
    float component_solidity_max = 0.96F;
    float max_shape_distance = 1.20F;
    float min_component_score = 0.24F;
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
    float leg_scale_tolerance = 0.45F;
    float leg_similarity_tolerance = 0.35F;
    float right_angle_cosine_max = 0.32F;
    float pythagorean_relative_error_max = 0.28F;
    float component_scale_ratio_max = 1.85F;
    float component_intensity_delta_max = 80.0F;
    float topology_shape_subweight = 0.22F;
    float topology_angle_subweight = 0.25F;
    float topology_leg_similarity_subweight = 0.18F;
    float topology_pythagorean_subweight = 0.18F;
    float topology_scale_subweight = 0.17F;

    int canonical_size = 160;
    int template_dilate_px = 2;
    float min_large_component_coverage = 0.42F;
    float min_small_component_coverage = 0.30F;
    float max_extra_bright_fraction = 0.38F;
    float min_led_board_contrast = 18.0F;
    float min_black_board_fraction = 0.30F;
    float min_black_board_validation_score = 0.30F;
    int min_additional_led_components = 1;
    float contrast_score_full_scale = 3.0F;
    float large_template_coverage_weight = 2.0F;
    float min_detection_contrast_score = 0.12F;
    float hypothesis_max_width_factor = 1.35F;
    float extra_bright_reject_factor = 1.80F;

    float topology_weight = 0.40F;
    float template_weight = 0.25F;
    float contrast_weight = 0.15F;
    float black_board_weight = 0.10F;
    float temporal_weight = 0.10F;
    float no_history_temporal_score = 0.50F;
    float temporal_min_normalizer_px = 8.0F;
    float trackable_threshold = 0.54F;
    float full_id_threshold = 0.72F;
    int full_id_min_components = 5;

    float search_strong_threshold = 0.66F;
    float track_hold_threshold = 0.50F;
    float track_exit_threshold = 0.40F;
    float tracking_roi_scale = 1.75F;
    float tracking_roi_lost_scale = 2.35F;
    float max_tracking_displacement_bbox = 1.50F;
    float prediction_horizon_seconds = 0.25F;
    float velocity_update_max_dt_seconds = 0.50F;
    float previous_velocity_weight = 0.55F;
    int search_confirm_frames = 2;
    int expand_after_lost_frames = 2;
    int return_to_search_lost_frames = 3;

    // Dark-board fallback.
    int dark_board_max_threshold = 100;
    float dark_board_aspect_min = 0.65F;
    float dark_board_aspect_max = 1.55F;
    float dark_board_rectangularity_min = 0.58F;
    float dark_board_min_height_width_fraction = 0.65F;
    float fallback_topology_score = 0.45F;
    int max_board_candidates = 6;
    int max_board_transform_evaluations = 4;
    int fallback_min_large_components = 2;
    int fallback_min_total_components = 4;
    int fallback_full_id_min_components = 6;
    float fallback_min_template_score = 0.52F;
    float fallback_min_contrast_score = 0.24F;
    float fallback_min_black_board_score = 0.45F;
    float fallback_trackable_threshold = 0.60F;
    float fallback_min_candidate_bright_delta = 20.0F;
    float fallback_board_rectangularity_weight = 0.60F;
    float fallback_board_bright_range_weight = 0.40F;

    float optical_center_x = 320.0F;
    float optical_center_y = 180.0F;
    bool benchmark_enabled = true;
    bool debug_enabled = false;
    bool failure_recorder_enabled = false;
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
        max_led_transform_evaluations = std::clamp(
            max_led_transform_evaluations, 1, kMaxTotalTransformEvaluations - 1);
        max_board_transform_evaluations = std::clamp(
            max_board_transform_evaluations, 1,
            kMaxTotalTransformEvaluations - max_led_transform_evaluations);
        max_board_candidates = std::max(1, max_board_candidates);
        search_confirm_frames = std::max(1, search_confirm_frames);
        expand_after_lost_frames = std::max(1, expand_after_lost_frames);
        return_to_search_lost_frames = std::max(expand_after_lost_frames,
                                                return_to_search_lost_frames);
        bright_percentile = std::clamp(bright_percentile, 0.50F, 0.999F);
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
        min_black_board_fraction = std::max(0.01F, min_black_board_fraction);
        prediction_horizon_seconds = std::max(0.0F, prediction_horizon_seconds);
        velocity_update_max_dt_seconds = std::max(0.01F,
                                                   velocity_update_max_dt_seconds);
        track_exit_threshold = std::clamp(track_exit_threshold, 0.0F, 1.0F);
        track_hold_threshold = std::clamp(track_hold_threshold,
                                          track_exit_threshold, 1.0F);
        trackable_threshold = std::clamp(trackable_threshold,
                                         track_hold_threshold, 1.0F);
        search_strong_threshold = std::clamp(search_strong_threshold,
                                             trackable_threshold, 1.0F);
        full_id_threshold = std::clamp(full_id_threshold,
                                       trackable_threshold, 1.0F);
        fallback_min_large_components = std::clamp(fallback_min_large_components, 1, 3);
        fallback_min_total_components = std::clamp(fallback_min_total_components,
                                                    fallback_min_large_components, 6);
        fallback_full_id_min_components = std::clamp(fallback_full_id_min_components,
                                                      fallback_min_total_components, 6);
        fallback_trackable_threshold = std::clamp(fallback_trackable_threshold,
                                                   trackable_threshold, 1.0F);
        fallback_min_template_score = std::clamp(fallback_min_template_score,
                                                  0.0F, 1.0F);
        fallback_min_contrast_score = std::clamp(fallback_min_contrast_score,
                                                  0.0F, 1.0F);
        fallback_min_black_board_score = std::clamp(fallback_min_black_board_score,
                                                     0.0F, 1.0F);
        fallback_min_candidate_bright_delta = std::max(1.0F,
                                                        fallback_min_candidate_bright_delta);
        optical_center_x = std::clamp(optical_center_x, 0.0F,
                                      static_cast<float>(frame_width));
        optical_center_y = std::clamp(optical_center_y, 0.0F,
                                      static_cast<float>(frame_height));
        benchmark_window = std::max<std::size_t>(16, benchmark_window);
        min_additional_led_components = std::max(1, min_additional_led_components);
        previous_velocity_weight = std::clamp(previous_velocity_weight, 0.0F, 1.0F);
    }
};

}  // namespace maixcam_marker
