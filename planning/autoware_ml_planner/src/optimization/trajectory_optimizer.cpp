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

#include "autoware/ml_planner/optimization/trajectory_optimizer.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>

namespace autoware::ml_planner::optimization
{
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace
{
// Warm starts older than this are discarded (stale after a planner hiccup).
constexpr double max_warm_start_age_s = 0.5;
constexpr double goal_position_change_threshold_m = 1.0e-3;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

TrajectoryOptimizer::TrajectoryOptimizer(
  const TrajectoryOptimizationParams & params,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t batch_size)
: params_(params),
  wheelbase_m_(vehicle_info.wheel_base_m),
  max_steering_angle_rad_(vehicle_info.max_steer_angle_rad),
  solver_(
    std::make_unique<AcadosSolverWrapper>(params, wheelbase_m_, vehicle_info.max_steer_angle_rad)),
  previous_solutions_(batch_size)
{
}

OptimizationResult TrajectoryOptimizer::optimize(
  const Trajectory & raw_trajectory, const Odometry & ego_odometry,
  const double current_steering_angle_rad, const size_t batch_index,
  const std::optional<geometry_msgs::msg::Pose> & goal_pose)
{
  OptimizationResult result;
  result.trajectory = raw_trajectory;

  if (raw_trajectory.points.size() < opt_horizon || batch_index >= previous_solutions_.size()) {
    return result;
  }

  // Initial state at base_link. Positions are solved in a local frame centered on the ego
  // position for numerical conditioning.
  const auto & ego_pose = ego_odometry.pose.pose;
  const double base_x = ego_pose.position.x;
  const double base_y = ego_pose.position.y;
  const double yaw0 = yaw_from_quaternion(ego_pose.orientation);
  const double v0 = std::clamp(
    static_cast<double>(ego_odometry.twist.twist.linear.x), params_.min_velocity_mps,
    params_.max_velocity_mps);

  const double delta0 =
    std::clamp(current_steering_angle_rad, -max_steering_angle_rad_, max_steering_angle_rad_);

  const std::array<double, opt_nx> initial_state{0.0, 0.0, yaw0, v0, delta0};

  // References for stages 1..N from the raw 80-point sequence (t = k * 0.1 s).
  // Yaw is unwrapped so the reference stays continuous across the +-pi boundary.
  std::array<StageReference, opt_horizon> references;
  double previous_yaw = yaw0;
  for (size_t k = 0; k < opt_horizon; ++k) {
    const auto & point = raw_trajectory.points[k];
    StageReference & ref = references[k];
    ref.x = point.pose.position.x - base_x;
    ref.y = point.pose.position.y - base_y;
    const double raw_yaw = yaw_from_quaternion(point.pose.orientation);
    ref.yaw = previous_yaw + autoware_utils::normalize_radian(raw_yaw - previous_yaw);
    previous_yaw = ref.yaw;
  }

  if (goal_pose) {
    const bool goal_position_changed =
      !observed_goal_pose_ ||
      std::hypot(
        goal_pose->position.x - observed_goal_pose_->position.x,
        goal_pose->position.y - observed_goal_pose_->position.y) > goal_position_change_threshold_m;
    if (goal_position_changed) {
      latched_goal_pose_.reset();
      for (auto & previous : previous_solutions_) {
        previous.reset();
      }
    }
    observed_goal_pose_ = goal_pose;

    const auto & terminal = raw_trajectory.points[opt_horizon - 1].pose.position;
    const double distance =
      std::hypot(terminal.x - goal_pose->position.x, terminal.y - goal_pose->position.y);
    if (!latched_goal_pose_ && distance <= params_.goal.snap_distance_m) {
      latched_goal_pose_ = goal_pose;
    } else if (latched_goal_pose_) {
      latched_goal_pose_ = goal_pose;
    }
  }

  std::optional<GoalTerminalReference> goal_terminal_reference;
  if (latched_goal_pose_) {
    const auto & goal = *latched_goal_pose_;
    const double raw_goal_yaw = yaw_from_quaternion(goal.orientation);
    references.back().x = goal.position.x - base_x;
    references.back().y = goal.position.y - base_y;
    references.back().yaw =
      previous_yaw + autoware_utils::normalize_radian(raw_goal_yaw - previous_yaw);
    goal_terminal_reference =
      GoalTerminalReference{references.back().x, references.back().y, references.back().yaw, 0.0};
  }

  // Warm start from the previous solution of this candidate, re-centered on the current
  // ego position. Discarded when stale.
  const rclcpp::Time stamp(raw_trajectory.header.stamp);
  const bool goal_active = goal_terminal_reference.has_value();
  SolverSolution warm_start;
  const SolverSolution * warm_start_ptr = nullptr;
  auto & previous = previous_solutions_[batch_index];
  if (previous.has_value()) {
    const double age_s = (stamp - previous->stamp).seconds();
    if (age_s >= 0.0 && age_s <= max_warm_start_age_s && previous->goal_active == goal_active) {
      warm_start = previous->solution;
      for (auto & state : warm_start.states) {
        state[0] -= base_x;
        state[1] -= base_y;
      }
      warm_start_ptr = &warm_start;
    } else {
      previous.reset();
    }
  }

  // Temporal consistency reference: the previous plan of this candidate, resampled onto the
  // absolute times of this cycle's stages. Current stage k sits at t_now + k * dt, which in
  // the previous plan is index k + (t_now - t_prev) / dt, so the whole plan is shifted by
  // the elapsed interval and interpolated. It is available under exactly the conditions that
  // make the previous solution usable as a warm start, and `warm_start` is already
  // re-centered on the current ego position, so it shares the solver's frame.
  std::array<StageTemporalReference, opt_horizon> temporal_references;
  const std::array<StageTemporalReference, opt_horizon> * temporal_references_ptr = nullptr;
  if (params_.temporal_consistency.enable && warm_start_ptr != nullptr) {
    const double stage_shift = std::max(0.0, (stamp - previous->stamp).seconds() / opt_dt_s);
    for (size_t k = 0; k < opt_horizon; ++k) {
      // Beyond the end of the previous horizon there is nothing left to be consistent with,
      // so the last few stages fall back to its terminal state.
      const double index =
        std::min(static_cast<double>(k + 1) + stage_shift, static_cast<double>(opt_horizon));
      const auto lower = static_cast<size_t>(std::floor(index));
      const size_t upper = std::min(lower + 1, opt_horizon);
      const double ratio = index - static_cast<double>(lower);
      const auto & from = warm_start.states[lower];
      const auto & to = warm_start.states[upper];
      const auto interpolate = [ratio](const double a, const double b) {
        return a + ratio * (b - a);
      };
      StageTemporalReference & ref = temporal_references[k];
      ref.x = interpolate(from[0], to[0]);
      ref.y = interpolate(from[1], to[1]);
      // The previous solution's yaw is continuous within itself but lives on its own branch;
      // put it on the branch of this stage's tracking reference so the two can be blended.
      const double previous_yaw = interpolate(from[2], to[2]);
      ref.yaw =
        references[k].yaw + autoware_utils::normalize_radian(previous_yaw - references[k].yaw);
      ref.velocity = interpolate(from[3], to[3]);
    }
    temporal_references_ptr = &temporal_references;
  }

  SolverSolution solution = solver_->solve(
    initial_state, references, goal_terminal_reference, temporal_references_ptr, warm_start_ptr);
  result.solver_status = solution.status;
  result.solve_time_ms = solution.solve_time_s * 1e3;

  if (!solution.success()) {
    previous.reset();
    return result;
  }

  // Build the output trajectory from stages 1..N (t = 0.1..8.0 s, same timing convention
  // as the raw model output). Stage 0 is the base_link initial state and is not published,
  // but the whole solution is dynamically consistent with it.
  Trajectory optimized;
  optimized.header = raw_trajectory.header;
  optimized.points.reserve(opt_horizon);
  for (size_t i = 1; i <= opt_horizon; ++i) {
    const auto & state = solution.states[i];
    TrajectoryPoint point;
    const double time_s = opt_dt_s * static_cast<double>(i);
    point.time_from_start.sec = static_cast<int32_t>(time_s);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((time_s - point.time_from_start.sec) * 1e9);
    point.pose.position.x = state[0] + base_x;
    point.pose.position.y = state[1] + base_y;
    point.pose.position.z = raw_trajectory.points[i - 1].pose.position.z;
    point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(autoware_utils::normalize_radian(state[2]));
    point.longitudinal_velocity_mps = static_cast<float>(state[3]);
    point.front_wheel_angle_rad = static_cast<float>(state[4]);
    point.acceleration_mps2 = (i < opt_horizon) ? static_cast<float>(solution.inputs[i][0]) : 0.0F;
    point.heading_rate_rps = static_cast<float>(state[3] * std::tan(state[4]) / wheelbase_m_);
    optimized.points.push_back(point);
  }
  result.trajectory = std::move(optimized);
  result.optimized = true;

  // Store the solution in map frame for the next cycle's warm start.
  for (auto & state : solution.states) {
    state[0] += base_x;
    state[1] += base_y;
  }
  previous = PreviousSolution{solution, stamp, goal_active};

  return result;
}

}  // namespace autoware::ml_planner::optimization
