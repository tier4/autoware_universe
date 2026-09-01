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

#ifndef AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_HPP_

namespace autoware::mppi_optimizer
{

/** Host-side MPPI tunables; defaults match mppi_optimizer.param.yaml. */
struct FirstOrderDubinsMppiCostParams
{
  /** Softmax temperature for trajectory weighting (higher = softer weighting). */
  float lambda{1500.0F};
  /** Number of optimization iterations performed for each MPPI update. */
  int max_iter{20};
  float speed_coeff{500.0F};
  float track_coeff{1000.0F};
  float track_terminal_scale{10.0F};
  /** Terminal-only squared XY error to the final reference point. */
  float terminal_error_coeff{0.0F};
  /** Terminal-only squared shortest-angle error to the final reference heading. */
  float terminal_heading_coeff{0.0F};
  float heading_coeff{500.0F};
  float lateral_distance_coeff{0.0F};
  float lateral_yaw_error_coeff{0.0F};
  /** Soft cost on remaining corridor chord length [m]; 0 disables. */
  float remaining_distance_coeff{0.0F};
  /** Soft cost on along-track overshoot past the corridor tip [m]; 0 disables. */
  float path_overshoot_coeff{0.0F};
  float track_center_coeff{0.0F};
  float corner_buffer_coeff{0.0F};
  float corner_safe_margin{0.3F};
  float boundary_threshold{1.5F};
  /** Distance inside boundary_threshold at which the gradual lateral barrier activates. */
  float lateral_boundary_soft_margin{0.2F};
  float lateral_boundary_barrier_weight{};
  float accel_cmd_coeff{0.0F};
  float steer_cmd_coeff{0.0F};
  float steer_rate_coeff{0.0F};
  /** Shared weight for optional VelocityLimit interval violations. */
  float overlimit_coeff{10000.0F};
  /** Gaussian sampling std-dev on acceleration command [m/s^2] around u_nom. */
  float accel_cmd_std_dev{0.35F};
  /** Gaussian sampling std-dev on steer command [rad] around u_nom. */
  float steer_cmd_std_dev{0.024F};
  /** Per-iteration multiplier applied to the sampling standard deviation. */
  float std_dev_decay{0.84F};
  /** Power-law PSD exponent for acceleration-command sampling noise. */
  float accel_cmd_noise_exponent{1.0F};
  /** Power-law PSD exponent for steering-command sampling noise. */
  float steer_cmd_noise_exponent{1.0F};
  /** Spatial window used only when deriving a cold-start nominal steer from the reference. */
  float nominal_curvature_min_chord_length_m{1.5F};
  float lateral_acceleration_coeff{300.0F};
  float lateral_jerk_coeff{300.0F};
  float longitudinal_jerk_coeff{10.0F};
  float obstacle_collision_margin{0.5F};
  float road_border_collision_margin{0.3F};
  float obstacle_safe_margin{0.5F};
  float obstacle_barrier_weight{};
  float road_border_safe_margin{0.3F};
  float road_border_barrier_weight{};
  float drivable_area_safe_margin{0.0F};
  float drivable_area_barrier_weight{2000.0F};
  float crash_contact_penalty{100000.0F};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__FIRST_ORDER_DUBINS_MPPI_COST_PARAMS_HPP_
