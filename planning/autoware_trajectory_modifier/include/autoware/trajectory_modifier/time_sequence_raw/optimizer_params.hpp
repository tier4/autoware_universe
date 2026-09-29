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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TIME_SEQUENCE_RAW__OPTIMIZER_PARAMS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TIME_SEQUENCE_RAW__OPTIMIZER_PARAMS_HPP_

namespace autoware::trajectory_modifier::time_sequence_raw
{

/// @brief Runtime parameters for the pose-only acados trajectory optimization.
struct TrajectoryOptimizationParams
{
  double weight_longitudinal{0.5};
  double weight_lateral{0.5};
  double weight_yaw{0.05};
  double weight_jerk{0.1};
  double weight_steering_rate{10.0};
  double terminal_weight_scale{2.5};

  /// Extra terminal penalty latched once the predicted endpoint is near the route goal.
  /// Weights are added on the terminal cost only and are not scaled by terminal_weight_scale.
  struct GoalParams
  {
    double weight_longitudinal{5.0};
    double weight_lateral{5.0};
    double weight_yaw{0.5};
    double weight_velocity{0.1};
    double snap_distance_m{1.0};
    /// Drop the latch when ego-to-goal exceeds this time horizon times speed.
    /// <= 0 disables the far-away unlatch.
    double unlatch_horizon_s{8.0};
    /// Floor on speed used by the far-away range: range = horizon * max(|v|, this).
    double unlatch_min_speed_mps{3.0};
  } goal;

  /**
   * @brief Weakly track the previous cycle's solved plan (same idea as autoware_ml_planner).
   *
   * Default weights are ~0.01 of the ml_planner yaml values so this pose-only OCP does not
   * overpower tracking. The term is skipped while goal snap is latched or the reference was
   * shifted by road-border avoidance, because those cases are not "clean" network output.
   */
  struct TemporalConsistencyParams
  {
    bool enable{false};
    double weight_longitudinal{0.004};
    double weight_lateral{0.2};
    double weight_yaw{0.002};
    double weight_velocity{0.004};
    double decay_time_constant_s{1.0};
    double far_weight_ratio{0.5};
  } temporal_consistency;

  double min_velocity_mps{0.0};
  double max_velocity_mps{30.0};

  double min_acceleration_mps2{-4.0};
  double max_acceleration_mps2{3.0};
  double min_jerk_mps3{-5.0};
  double max_jerk_mps3{5.0};
  double max_steering_rate_rps{1.0};

  double max_lateral_acceleration_mps2{3.0};

  int max_sqp_iterations{50};
};

}  // namespace autoware::trajectory_modifier::time_sequence_raw

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TIME_SEQUENCE_RAW__OPTIMIZER_PARAMS_HPP_
