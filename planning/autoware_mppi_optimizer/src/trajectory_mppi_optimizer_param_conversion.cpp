// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/mppi_optimizer/trajectory_mppi_optimizer_param_conversion.hpp"

#include <stdexcept>

namespace autoware::mppi_optimizer
{

FirstOrderDubinsMppiCostParams costParamsFromTrajectoryMppiOptimizerParams(
  const trajectory_mppi_optimizer::Params & params)
{
  if (params.obstacle_safe_margin < params.obstacle_collision_margin) {
    throw std::invalid_argument(
      "mppi_optimizer.obstacle_safe_margin must be greater than or equal to "
      "mppi_optimizer.obstacle_collision_margin");
  }
  if (params.road_border_safe_margin < params.road_border_collision_margin) {
    throw std::invalid_argument(
      "mppi_optimizer.road_border_safe_margin must be greater than or equal to "
      "mppi_optimizer.road_border_collision_margin");
  }

  FirstOrderDubinsMppiCostParams output;
  output.lambda = static_cast<float>(params.lambda);
  output.lambda_min = static_cast<float>(params.lambda_min);
  output.lambda_max = static_cast<float>(params.lambda_max);
  output.target_ess_ratio = static_cast<float>(params.target_ess_ratio);
  output.lambda_adaptation_gain = static_cast<float>(params.lambda_adaptation_gain);
  output.unsafe_rollout_fraction_threshold =
    static_cast<float>(params.unsafe_rollout_fraction_threshold);
  output.cost_normalization_percentile = static_cast<float>(params.cost_normalization_percentile);
  output.max_iter = static_cast<int>(params.max_iter);
  output.spatial_overspeed_coeff = static_cast<float>(params.spatial_overspeed_coeff);
  output.track_coeff = static_cast<float>(params.track_coeff);
  output.track_terminal_scale = static_cast<float>(params.track_terminal_scale);
  output.terminal_error_coeff = static_cast<float>(params.terminal_error_coeff);
  output.terminal_heading_coeff = static_cast<float>(params.terminal_heading_coeff);
  output.heading_coeff = static_cast<float>(params.heading_coeff);
  output.lateral_distance_coeff = static_cast<float>(params.lateral_distance_coeff);
  output.lateral_yaw_error_coeff = static_cast<float>(params.lateral_yaw_error_coeff);
  output.remaining_distance_coeff = static_cast<float>(params.remaining_distance_coeff);
  output.path_overshoot_coeff = static_cast<float>(params.path_overshoot_coeff);
  output.preferred_lane_center_coeff = static_cast<float>(params.preferred_lane_center_coeff);
  output.track_center_coeff = static_cast<float>(params.track_center_coeff);
  output.corner_buffer_coeff = static_cast<float>(params.corner_buffer_coeff);
  output.corner_safe_margin = static_cast<float>(params.corner_safe_margin);
  output.boundary_threshold = static_cast<float>(params.boundary_threshold);
  output.lateral_boundary_soft_margin = static_cast<float>(params.lateral_boundary_soft_margin);
  output.accel_cmd_coeff = static_cast<float>(params.accel_cmd_coeff);
  output.steer_cmd_coeff = static_cast<float>(params.steer_cmd_coeff);
  output.steer_rate_coeff = static_cast<float>(params.steer_rate_coeff);
  output.initial_steer_rate_coeff = static_cast<float>(params.initial_steer_rate_coeff);
  output.accel_cmd_rate_coeff = static_cast<float>(params.accel_cmd_rate_coeff);
  output.steer_cmd_rate_coeff = static_cast<float>(params.steer_cmd_rate_coeff);
  output.overlimit_coeff = static_cast<float>(params.overlimit_coeff);
  output.accel_cmd_std_dev = static_cast<float>(params.accel_cmd_std_dev);
  output.steer_cmd_std_dev = static_cast<float>(params.steer_cmd_std_dev);
  output.std_dev_decay = static_cast<float>(params.std_dev_decay);
  output.accel_cmd_noise_exponent = static_cast<float>(params.accel_cmd_noise_exponent);
  output.steer_cmd_noise_exponent = static_cast<float>(params.steer_cmd_noise_exponent);
  output.nominal_curvature_min_chord_length_m =
    static_cast<float>(params.nominal_curvature_min_chord_length_m);
  output.nominal_curvature_fit_window_m = static_cast<float>(params.nominal_curvature_fit_window_m);
  output.lateral_acceleration_coeff = static_cast<float>(params.lateral_acceleration_coeff);
  output.lateral_jerk_coeff = static_cast<float>(params.lateral_jerk_coeff);
  output.longitudinal_jerk_coeff = static_cast<float>(params.longitudinal_jerk_coeff);
  output.obstacle_collision_margin = static_cast<float>(params.obstacle_collision_margin);
  output.road_border_collision_margin = static_cast<float>(params.road_border_collision_margin);
  output.obstacle_safe_margin = static_cast<float>(params.obstacle_safe_margin);
  output.road_border_safe_margin = static_cast<float>(params.road_border_safe_margin);
  output.drivable_area_safe_margin = static_cast<float>(params.drivable_area_safe_margin);
  output.drivable_area_barrier_weight = static_cast<float>(params.drivable_area_barrier_weight);
  output.crash_contact_penalty = static_cast<float>(params.crash_contact_penalty);
  return output;
}

FirstOrderDubinsMppiRuntimeOptions runtimeOptionsFromTrajectoryMppiOptimizerParams(
  const trajectory_mppi_optimizer::Params & params)
{
  FirstOrderDubinsMppiRuntimeOptions output;
  output.enable_debug_trajectory_log = params.enable_debug_trajectory_log;
  output.debug_trajectory_log_directory = params.debug_trajectory_log_directory;
  output.enable_distance_map_texture_debug = params.enable_distance_map_texture_debug;
  output.enable_iteration_rollout_debug = params.enable_iteration_rollout_debug;
  output.ignore_obstacles = params.ignore_obstacles;
  output.dynamic_obstacle_horizon_s = static_cast<float>(params.dynamic_obstacle_horizon_s);
  output.ignore_road_borders = params.ignore_road_borders;
  output.ignore_drivable_area = params.ignore_drivable_area;
  output.force_cold_start_each_step = params.force_cold_start_each_step;
  output.skip_if_invalid = params.skip_if_invalid;
  output.min_optimization_length = static_cast<float>(params.min_optimization_length);
  output.steering_hold_reference_length_threshold_m =
    static_cast<float>(params.steering_hold_reference_length_threshold_m);
  output.min_trajectory_progress_m = static_cast<float>(params.min_trajectory_progress_m);
  output.use_last_control_as_nominal = params.use_last_control_as_nominal;
  output.nominal_initial_steering_max_deviation_rad =
    static_cast<float>(params.nominal_initial_steering_max_deviation_rad);
  output.last_control_warm_start_max_age_s =
    static_cast<float>(params.last_control_warm_start_max_age_s);
  output.last_control_warm_start_max_position_error_m =
    static_cast<float>(params.last_control_warm_start_max_position_error_m);
  output.last_control_warm_start_max_yaw_error_rad =
    static_cast<float>(params.last_control_warm_start_max_yaw_error_rad);
  output.last_control_warm_start_max_velocity_error_mps =
    static_cast<float>(params.last_control_warm_start_max_velocity_error_mps);
  output.last_control_warm_start_max_reference_position_error_m =
    static_cast<float>(params.last_control_warm_start_max_reference_position_error_m);
  output.last_control_warm_start_max_reference_yaw_error_rad =
    static_cast<float>(params.last_control_warm_start_max_reference_yaw_error_rad);
  output.last_control_warm_start_max_reference_velocity_error_mps =
    static_cast<float>(params.last_control_warm_start_max_reference_velocity_error_mps);
  output.last_control_warm_start_stop_enter_velocity_mps =
    static_cast<float>(params.last_control_warm_start_stop_enter_velocity_mps);
  output.last_control_warm_start_stop_exit_velocity_mps =
    static_cast<float>(params.last_control_warm_start_stop_exit_velocity_mps);
  output.use_temporal_mpt_as_nominal = params.use_temporal_mpt_as_nominal;
  output.prevent_reverse_velocity = params.prevent_reverse_velocity;
  output.enable_input_delay_compensation = params.enable_input_delay_compensation;
  return output;
}

CurvatureAdaptiveSteeringFilterParams curvatureAdaptiveSteeringFilterParamsFromTrajectoryMppiOptimizerParams(
  const trajectory_mppi_optimizer::Params & params)
{
  return {
    static_cast<float>(params.steering_filter_alpha_straight),
    static_cast<float>(params.steering_filter_alpha_turn),
    static_cast<float>(params.steering_filter_turn_angle_rad)};
}

}  // namespace autoware::mppi_optimizer
