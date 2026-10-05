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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TIME_SEQUENCE_RAW__TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TIME_SEQUENCE_RAW__TRAJECTORY_OPTIMIZER_HPP_

#include "autoware/trajectory_modifier/time_sequence_raw/acados_solver_wrapper.hpp"
#include "autoware/trajectory_modifier/time_sequence_raw/optimizer_params.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::trajectory_modifier::time_sequence_raw
{
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

struct OptimizationResult
{
  Trajectory trajectory;
  bool optimized{false};
  int solver_status{0};
  double solve_time_ms{0.0};
  /// Chord-speed seed for acados x0 (first three poses), not ego twist.
  double initial_speed_mps{0.0};
  double initial_accel_mps2{0.0};
  bool temporal_applied{false};
  bool goal_snap_active{false};
  /// Literal: none | disabled | border_shift | no_warm_start | stamp_rewind |
  ///          warm_start_stale | goal_flag_mismatch | beyond_previous_path
  const char * temporal_skip_reason{"disabled"};
  /// Stages 0..N-1 whose station is still on the previous path (consistency loss applied).
  size_t temporal_valid_stages{0};
  /// Age of the stored previous solution, or -1 if none existed this cycle.
  double warm_start_age_s{-1.0};
};

/// Tracks a pose-only time-indexed trajectory with a kinematic bicycle OCP.
/// Initial pose/steering come from ego odometry + measured steering; initial speed is the
/// average chord speed of the first three trajectory points (not ego twist). Initial
/// acceleration comes from measured ego longitudinal acceleration.
class TrajectoryOptimizer
{
public:
  TrajectoryOptimizer(
    const TrajectoryOptimizationParams & params,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, size_t batch_size);

  /**
   * @param goal_pose Route goal in the same frame as the trajectory. Once the predicted
   *                  endpoint is within goal.snap_distance_m, the terminal pose is snapped
   *                  to this goal and the extra terminal weights stay latched until the
   *                  goal position changes, or until ego is farther from the goal than
   *                  unlatch_horizon_s * max(|v|, unlatch_min_speed_mps).
   * @param reference_was_shifted True when road-border avoidance moved the input. Temporal
   *                             consistency is then skipped so a geometric correction is not
   *                             blended with the previous (unshifted) plan.
   */
  OptimizationResult optimize(
    const Trajectory & raw_trajectory, const Odometry & ego_odometry,
    const std::optional<double> & current_steering_angle_rad,
    double current_longitudinal_accel_mps2, size_t batch_index,
    const std::optional<geometry_msgs::msg::Pose> & goal_pose = std::nullopt,
    bool reference_was_shifted = false);

  void clear_warm_start(size_t batch_index);

private:
  void reset_goal_snap_state();
  TrajectoryOptimizationParams params_;
  double wheelbase_m_;
  double max_steering_angle_rad_;
  std::unique_ptr<AcadosSolverWrapper> solver_;
  std::optional<geometry_msgs::msg::Pose> observed_goal_pose_;
  std::optional<geometry_msgs::msg::Pose> latched_goal_pose_;

  struct PreviousSolution
  {
    SolverSolution solution;
    rclcpp::Time stamp;
    bool goal_active{false};
  };
  std::vector<std::optional<PreviousSolution>> previous_solutions_;
};

}  // namespace autoware::trajectory_modifier::time_sequence_raw

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TIME_SEQUENCE_RAW__TRAJECTORY_OPTIMIZER_HPP_
