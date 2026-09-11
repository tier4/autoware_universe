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

#ifndef AUTOWARE__DIFFUSION_PLANNER__OPTIMIZATION__TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__OPTIMIZATION__TRAJECTORY_OPTIMIZER_HPP_

#include "autoware/diffusion_planner/optimization/acados_solver_wrapper.hpp"
#include "autoware/diffusion_planner/optimization/optimizer_params.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <optional>
#include <vector>

namespace autoware::diffusion_planner::optimization
{
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

struct OptimizationResult
{
  Trajectory trajectory;
  bool optimized{false};
  int solver_status{0};
  double solve_time_ms{0.0};
};

/**
 * @brief Optimizes the raw diffusion planner trajectory with an acados OCP.
 *
 * The raw model output is a noisy, pose-only 80-point sequence (t = 0.1..8.0 s) that
 * does not start at base_link. This class solves a kinematic bicycle OCP (inputs:
 * acceleration and steering rate) tracking that sequence, with the initial state fixed to
 * the current ego state. The result is an 80-point trajectory (t = 0.1..8.0 s, same timing
 * convention as the raw output) that is dynamically consistent with the current ego state
 * and carries velocity, acceleration and steering profiles.
 */
class TrajectoryOptimizer
{
public:
  TrajectoryOptimizer(
    const TrajectoryOptimizationParams & params,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, size_t batch_size);

  /**
   * @brief Optimize one candidate trajectory.
   *
   * @param raw_trajectory Raw trajectory from the model (>= 80 points, map frame).
   * @param ego_odometry Current ego kinematic state (base_link in map frame).
   * @param current_steering_angle_rad Measured steering angle; estimated from yaw rate
   *                                   when not available.
   * @param batch_index Candidate index; warm starts are kept per candidate.
   * @return Optimized trajectory, or the raw trajectory when the solver fails.
   */
  OptimizationResult optimize(
    const Trajectory & raw_trajectory, const Odometry & ego_odometry,
    const std::optional<double> & current_steering_angle_rad, size_t batch_index);

private:
  TrajectoryOptimizationParams params_;
  double wheelbase_m_;
  double max_steering_angle_rad_;
  std::unique_ptr<AcadosSolverWrapper> solver_;

  // Previous solutions in map frame, per candidate, used as warm starts.
  struct PreviousSolution
  {
    SolverSolution solution;
    rclcpp::Time stamp;
  };
  std::vector<std::optional<PreviousSolution>> previous_solutions_;
};

}  // namespace autoware::diffusion_planner::optimization

#endif  // AUTOWARE__DIFFUSION_PLANNER__OPTIMIZATION__TRAJECTORY_OPTIMIZER_HPP_
