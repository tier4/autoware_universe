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
  output.speed_coeff = static_cast<float>(params.speed_coeff);
  output.track_coeff = static_cast<float>(params.track_coeff);
  output.track_terminal_scale = static_cast<float>(params.track_terminal_scale);
  output.heading_coeff = static_cast<float>(params.heading_coeff);
  output.lateral_distance_coeff = static_cast<float>(params.lateral_distance_coeff);
  output.lateral_yaw_error_coeff = static_cast<float>(params.lateral_yaw_error_coeff);
  output.remaining_distance_coeff = static_cast<float>(params.remaining_distance_coeff);
  output.path_overshoot_coeff = static_cast<float>(params.path_overshoot_coeff);
  output.track_center_coeff = static_cast<float>(params.track_center_coeff);
  output.corner_buffer_coeff = static_cast<float>(params.corner_buffer_coeff);
  output.corner_safe_margin = static_cast<float>(params.corner_safe_margin);
  output.boundary_threshold = static_cast<float>(params.boundary_threshold);
  output.lateral_boundary_soft_margin = static_cast<float>(params.lateral_boundary_soft_margin);
  output.accel_cmd_coeff = static_cast<float>(params.accel_cmd_coeff);
  output.steer_cmd_coeff = static_cast<float>(params.steer_cmd_coeff);
  output.steer_rate_coeff = static_cast<float>(params.steer_rate_coeff);
  output.accel_cmd_std_dev = static_cast<float>(params.accel_cmd_std_dev);
  output.steer_cmd_std_dev = static_cast<float>(params.steer_cmd_std_dev);
  output.accel_cmd_noise_exponent = static_cast<float>(params.accel_cmd_noise_exponent);
  output.steer_cmd_noise_exponent = static_cast<float>(params.steer_cmd_noise_exponent);
  output.nominal_curvature_min_chord_length_m =
    static_cast<float>(params.nominal_curvature_min_chord_length_m);
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
  output.ignore_obstacles = params.ignore_obstacles;
  output.ignore_road_borders = params.ignore_road_borders;
  output.ignore_drivable_area = params.ignore_drivable_area;
  output.force_cold_start_each_step = params.force_cold_start_each_step;
  output.skip_if_invalid = params.skip_if_invalid;
  output.min_optimization_length = static_cast<float>(params.min_optimization_length);
  output.use_last_control_as_nominal = params.use_last_control_as_nominal;
  output.use_temporal_mpt_as_nominal = params.use_temporal_mpt_as_nominal;
  output.prevent_reverse_velocity = params.prevent_reverse_velocity;
  output.enable_input_delay_compensation = params.enable_input_delay_compensation;
  return output;
}

}  // namespace autoware::mppi_optimizer
