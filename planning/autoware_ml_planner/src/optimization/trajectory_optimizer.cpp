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

#include "autoware/ml_planner/optimization/lateral_acceleration_guard.hpp"

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

std::array<StageReference, opt_horizon> stage_references(
  const Trajectory & trajectory, const double base_x, const double base_y, const double yaw0)
{
  std::array<StageReference, opt_horizon> references;
  double previous_yaw = yaw0;
  for (size_t k = 0; k < opt_horizon; ++k) {
    const auto & point = trajectory.points[k];
    StageReference & ref = references[k];
    ref.x = point.pose.position.x - base_x;
    ref.y = point.pose.position.y - base_y;
    const double raw_yaw = yaw_from_quaternion(point.pose.orientation);
    ref.yaw = previous_yaw + autoware_utils::normalize_radian(raw_yaw - previous_yaw);
    previous_yaw = ref.yaw;
  }
  return references;
}

/// Restores the configured ``a_lat`` bound when the geometry pass returns.
class RestoreLateralBound
{
public:
  RestoreLateralBound(AcadosSolverWrapper & solver, const double restore_mps2)
  : solver_(solver), restore_mps2_(restore_mps2)
  {
  }
  ~RestoreLateralBound() { solver_.set_max_lateral_acceleration(restore_mps2_); }

private:
  AcadosSolverWrapper & solver_;
  double restore_mps2_;
};

/// Large enough that |v^2 κ| never hits it. Used only to turn the bound off.
constexpr double unrestricted_lateral_acceleration_mps2 = 1.0e6;
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
  const double current_steering_angle_rad, const double current_acceleration_mps2,
  const size_t batch_index, const std::optional<geometry_msgs::msg::Pose> & goal_pose)
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
  const double a0 = std::clamp(
    current_acceleration_mps2, params_.min_acceleration_mps2, params_.max_acceleration_mps2);

  const std::array<double, opt_nx> initial_state{0.0, 0.0, yaw0, v0, delta0, a0};

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

    const double ego_to_goal_m = std::hypot(
      goal_pose->position.x - ego_pose.position.x, goal_pose->position.y - ego_pose.position.y);
    const bool ego_too_far_from_goal =
      params_.goal.unlatch_horizon_s > 0.0 &&
      ego_to_goal_m >
        params_.goal.unlatch_horizon_s *
          std::max(std::abs(ego_odometry.twist.twist.linear.x), params_.goal.unlatch_min_speed_mps);
    if (ego_too_far_from_goal) {
      latched_goal_pose_.reset();
    } else {
      const auto & terminal = raw_trajectory.points[opt_horizon - 1].pose.position;
      const double distance =
        std::hypot(terminal.x - goal_pose->position.x, terminal.y - goal_pose->position.y);
      if (!latched_goal_pose_ && distance <= params_.goal.snap_distance_m) {
        latched_goal_pose_ = goal_pose;
      } else if (latched_goal_pose_) {
        latched_goal_pose_ = goal_pose;
      }
    }
  }

  Trajectory tracking_trajectory = raw_trajectory;
  double geometry_solve_time_ms = 0.0;
  if (params_.lateral_acceleration_guard) {
    if (
      auto prepared = build_laterally_guarded_reference(
        raw_trajectory, initial_state, base_x, base_y, batch_index, geometry_solve_time_ms)) {
      tracking_trajectory = std::move(prepared->trajectory);
    }
  }

  // References for stages 1..N (t = k * 0.1 s). Yaw is unwrapped from the ego heading.
  std::array<StageReference, opt_horizon> references =
    stage_references(tracking_trajectory, base_x, base_y, yaw0);
  const double unwrapped_terminal_yaw = references.back().yaw;

  std::optional<GoalTerminalReference> goal_terminal_reference;
  if (latched_goal_pose_) {
    const auto & goal = *latched_goal_pose_;
    const double raw_goal_yaw = yaw_from_quaternion(goal.orientation);
    references.back().x = goal.position.x - base_x;
    references.back().y = goal.position.y - base_y;
    references.back().yaw = unwrapped_terminal_yaw +
                            autoware_utils::normalize_radian(raw_goal_yaw - unwrapped_terminal_yaw);
    goal_terminal_reference =
      GoalTerminalReference{references.back().x, references.back().y, references.back().yaw, 0.0};
  }

  // Warm start from the previous solution of this candidate, re-centered on the current
  // ego position. Discarded when stale. A goal-snap latch mismatch must not wipe this
  // memory: clearing it also drops temporal consistency for the following cycle.
  const rclcpp::Time stamp(raw_trajectory.header.stamp);
  const bool goal_active = goal_terminal_reference.has_value();
  result.goal_snap_active = goal_active;
  auto & previous = previous_solutions_[batch_index];
  std::optional<SolverSolution> warm_start =
    extract_previous_solution(batch_index, stamp, base_x, base_y);
  if (!warm_start && previous.has_value()) {
    previous.reset();
  }
  const SolverSolution * warm_start_ptr = warm_start ? &(*warm_start) : nullptr;
  const bool temporal_goal_compatible =
    !(warm_start && previous.has_value() && previous->goal_active != goal_active);

  // Temporal consistency reference: the previous plan of this candidate, resampled onto the
  // absolute times of this cycle's stages. Current stage k sits at t_now + k * dt, which in
  // the previous plan is index k + (t_now - t_prev) / dt, so the whole plan is shifted by
  // the elapsed interval and interpolated. `warm_start` is already re-centered on the
  // current ego position, so it shares the solver's frame. Skipped on a goal-snap latch
  // mismatch so a latched terminal does not keep pulling after unlatch.
  std::array<StageTemporalReference, opt_horizon> temporal_references;
  const std::array<StageTemporalReference, opt_horizon> * temporal_references_ptr = nullptr;
  result.temporal_applied = false;
  if (
    params_.temporal_consistency.enable && warm_start_ptr != nullptr && temporal_goal_compatible) {
    const double stage_shift = std::max(0.0, (stamp - previous->stamp).seconds() / opt_dt_s);
    for (size_t k = 0; k < opt_horizon; ++k) {
      // Beyond the end of the previous horizon there is nothing left to be consistent with,
      // so the last few stages fall back to its terminal state.
      const double index =
        std::min(static_cast<double>(k + 1) + stage_shift, static_cast<double>(opt_horizon));
      const auto lower = static_cast<size_t>(std::floor(index));
      const size_t upper = std::min(lower + 1, opt_horizon);
      const double ratio = index - static_cast<double>(lower);
      const auto & from = warm_start->states[lower];
      const auto & to = warm_start->states[upper];
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
    result.temporal_applied = true;
  }

  SolverSolution solution = solver_->solve(
    initial_state, references, goal_terminal_reference, temporal_references_ptr, warm_start_ptr);
  result.solver_status = solution.status;
  result.solve_time_ms = solution.solve_time_s * 1e3 + geometry_solve_time_ms;

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
    point.pose.position.z = tracking_trajectory.points[i - 1].pose.position.z;
    point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(autoware_utils::normalize_radian(state[2]));
    point.longitudinal_velocity_mps = static_cast<float>(state[kV]);
    point.front_wheel_angle_rad = static_cast<float>(state[kDelta]);
    point.acceleration_mps2 = static_cast<float>(state[kA]);
    point.heading_rate_rps = static_cast<float>(state[kV] * std::tan(state[kDelta]) / wheelbase_m_);
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

std::optional<SolverSolution> TrajectoryOptimizer::extract_previous_solution(
  const size_t batch_index, const rclcpp::Time & stamp, const double base_x, const double base_y)
  const
{
  const auto & previous = previous_solutions_[batch_index];
  if (!previous.has_value()) {
    return std::nullopt;
  }
  const double age_s = (stamp - previous->stamp).seconds();
  if (age_s < 0.0 || age_s > max_warm_start_age_s) {
    return std::nullopt;
  }
  SolverSolution local = previous->solution;
  for (auto & state : local.states) {
    state[0] -= base_x;
    state[1] -= base_y;
  }
  return local;
}

std::optional<SolverSolution> TrajectoryOptimizer::solve_geometry_pass(
  const Trajectory & raw_trajectory, const std::array<double, opt_nx> & initial_state,
  const double base_x, const double base_y, const size_t batch_index)
{
  solver_->set_max_lateral_acceleration(unrestricted_lateral_acceleration_mps2);
  RestoreLateralBound restore_bound(*solver_, params_.max_lateral_acceleration_mps2);

  const std::optional<SolverSolution> previous = extract_previous_solution(
    batch_index, rclcpp::Time(raw_trajectory.header.stamp), base_x, base_y);
  const auto references = stage_references(raw_trajectory, base_x, base_y, initial_state[kPsi]);
  // No goal, no temporal term, no a_lat bound: this pass only has to follow the pose shape.
  const SolverSolution geometry = solver_->solve(
    initial_state, references, std::nullopt, nullptr, previous ? &(*previous) : nullptr);
  if (!geometry.success()) {
    return std::nullopt;
  }
  return geometry;
}

Trajectory TrajectoryOptimizer::resample_under_lateral_acceleration(
  const Trajectory & raw_trajectory, const SolverSolution & geometry,
  const std::array<double, opt_nx> & initial_state, const double base_x, const double base_y) const
{
  std::array<PoseSample, opt_horizon + 1> poses{};
  for (size_t index = 0; index <= opt_horizon; ++index) {
    poses[index].x = geometry.states[index][kX] + base_x;
    poses[index].y = geometry.states[index][kY] + base_y;
    poses[index].yaw = geometry.states[index][kPsi];
    poses[index].steering_rad = geometry.states[index][kDelta];
    // Station 0 uses the first model speed as well. Copying the measured ego speed
    // there makes a stopped ego target 0 at distance 0, so the integrator never
    // advances and never sees the acceleration the model asked for further along.
    const size_t point_index = index == 0 ? 0 : index - 1;
    poses[index].speed_mps = raw_trajectory.points[point_index].longitudinal_velocity_mps;
  }
  const ResampledSequence resampled = resample_pose_sequence(
    poses, initial_state[kV], params_.max_lateral_acceleration_mps2, wheelbase_m_,
    params_.min_acceleration_mps2, params_.max_acceleration_mps2);

  Trajectory trajectory;
  trajectory.header = raw_trajectory.header;
  trajectory.points.reserve(opt_horizon);
  for (size_t index = 1; index <= opt_horizon; ++index) {
    const PoseSample & pose = resampled.poses[index];
    TrajectoryPoint point = raw_trajectory.points[index - 1];
    point.pose.position.x = pose.x;
    point.pose.position.y = pose.y;
    point.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(autoware_utils::normalize_radian(pose.yaw));
    trajectory.points.push_back(point);
  }
  return trajectory;
}

std::optional<TrajectoryOptimizer::GuardedReference>
TrajectoryOptimizer::build_laterally_guarded_reference(
  const Trajectory & raw_trajectory, const std::array<double, opt_nx> & initial_state,
  const double base_x, const double base_y, const size_t batch_index,
  double & geometry_solve_time_ms)
{
  geometry_solve_time_ms = 0.0;
  const auto geometry =
    solve_geometry_pass(raw_trajectory, initial_state, base_x, base_y, batch_index);
  if (!geometry) {
    return std::nullopt;
  }
  geometry_solve_time_ms = geometry->solve_time_s * 1e3;
  GuardedReference prepared;
  prepared.geometry_solve_time_ms = geometry_solve_time_ms;
  prepared.trajectory = resample_under_lateral_acceleration(
    raw_trajectory, *geometry, initial_state, base_x, base_y);
  return prepared;
}

}  // namespace autoware::ml_planner::optimization
