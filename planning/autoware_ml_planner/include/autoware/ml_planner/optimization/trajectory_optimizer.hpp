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

#ifndef AUTOWARE__ML_PLANNER__OPTIMIZATION__TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__ML_PLANNER__OPTIMIZATION__TRAJECTORY_OPTIMIZER_HPP_

#include "autoware/ml_planner/optimization/acados_solver_wrapper.hpp"
#include "autoware/ml_planner/optimization/optimizer_params.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::ml_planner::optimization
{
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

/// A converged solution in map frame, kept to warm start the next cycle.
struct StoredSolution
{
  SolverSolution solution;
  rclcpp::Time stamp;
  bool goal_active{false};
};

struct OptimizationResult
{
  Trajectory trajectory;
  bool optimized{false};
  int solver_status{0};
  double solve_time_ms{0.0};
  bool goal_snap_active{false};
  bool temporal_applied{false};
  /// Set when optimized; handed back through accept() to warm start the next cycle.
  std::optional<StoredSolution> solution;
};

/**
 * @brief Optimizes the raw ML planner trajectory with an acados OCP.
 *
 * The raw model output is a noisy, pose-only 80-point sequence (t = 0.1..8.0 s) that
 * does not start at base_link. This class solves a kinematic bicycle OCP (inputs:
 * acceleration and steering rate) tracking that sequence, with the initial state fixed to
 * the current ego state. The result is an 80-point trajectory (t = 0.1..8.0 s, same timing
 * convention as the raw output) that is dynamically consistent with the current ego state
 * and carries velocity, acceleration and steering profiles.
 *
 * optimize() has no side effects, so one candidate can be solved several times per cycle
 * with different references (e.g. by the road border re-check). The state carried across
 * cycles is updated only explicitly: set_goal() and latch_goal_if_reached() once per cycle
 * before solving, and accept() with the result that is actually used.
 */
class TrajectoryOptimizer
{
public:
  TrajectoryOptimizer(
    const TrajectoryOptimizationParams & params,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, size_t batch_size);

  /**
   * @brief Track the route goal. Call once per cycle before optimize().
   *
   * A goal that moved releases the latch and drops all warm starts, which were planned
   * toward the old goal. While ego is farther from the goal than
   * unlatch_horizon_s * max(|v|, unlatch_min_speed_mps) the latch is released and cannot
   * latch again; warm starts are kept so temporal consistency survives a drive-away.
   * Without a goal the previous state is kept.
   */
  void set_goal(
    const std::optional<geometry_msgs::msg::Pose> & goal_pose, const Odometry & ego_odometry);

  /// Latch the goal as the terminal reference once a candidate reference ends within
  /// goal.snap_distance_m of it. Stays latched until the goal moves or ego gets too far.
  void latch_goal_if_reached(const Trajectory & reference);

  /**
   * @brief Optimize one candidate trajectory. Does not modify the optimizer state.
   *
   * @param reference Reference trajectory (>= 80 points, map frame).
   * @param ego_odometry Current ego kinematic state (base_link in map frame).
   * @param current_steering_angle_rad Measured steering angle.
   * @param batch_index Candidate index; warm starts are kept per candidate.
   * @return Optimized trajectory, or the reference when the solver fails. The terminal pose
   *         is snapped to the goal while it is latched (see set_goal()).
   */
  [[nodiscard]] OptimizationResult optimize(
    const Trajectory & reference, const Odometry & ego_odometry, double current_steering_angle_rad,
    size_t batch_index) const;

  /// Keep the result used for this cycle as the next cycle's warm start (a failed result
  /// clears it).
  void accept(size_t batch_index, const OptimizationResult & result);

  /// Forget the previous solutions: the next cycle has no warm start and no temporal
  /// consistency term (used when the planning start jumped, e.g. a virtual pose reset).
  void drop_previous_solutions();

private:
  TrajectoryOptimizationParams params_;
  double wheelbase_m_;
  double max_steering_angle_rad_;
  std::unique_ptr<AcadosSolverWrapper> solver_;
  std::optional<geometry_msgs::msg::Pose> observed_goal_pose_;
  std::optional<geometry_msgs::msg::Pose> latched_goal_pose_;
  // Set by set_goal() while ego is too far from the goal to latch the snap.
  bool ego_too_far_from_goal_{false};

  // Accepted solutions in map frame, per candidate, used as warm starts.
  std::vector<std::optional<StoredSolution>> previous_solutions_;
};

}  // namespace autoware::ml_planner::optimization

#endif  // AUTOWARE__ML_PLANNER__OPTIMIZATION__TRAJECTORY_OPTIMIZER_HPP_
