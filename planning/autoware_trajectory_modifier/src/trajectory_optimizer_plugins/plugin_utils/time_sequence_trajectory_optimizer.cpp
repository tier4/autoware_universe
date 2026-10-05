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

#include "autoware/trajectory_modifier/time_sequence_raw/trajectory_optimizer.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>

namespace autoware::trajectory_modifier::time_sequence_raw
{
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace
{
constexpr double max_warm_start_age_s = 0.5;
constexpr double min_speed_for_steering_estimation_mps = 0.5;
constexpr double goal_position_change_threshold_m = 1.0e-3;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void lerp_state(
  const std::array<double, opt_nx> & from, const std::array<double, opt_nx> & to, const double ratio,
  std::array<double, opt_nx> & out)
{
  for (size_t i = 0; i < opt_nx; ++i) {
    const double delta =
      (i == kPsi) ? autoware_utils_math::normalize_radian(to[i] - from[i]) : (to[i] - from[i]);
    out[i] = from[i] + ratio * delta;
  }
}

void lerp_input(
  const std::array<double, opt_nu> & from, const std::array<double, opt_nu> & to, const double ratio,
  std::array<double, opt_nu> & out)
{
  for (size_t i = 0; i < opt_nu; ++i) {
    out[i] = from[i] + ratio * (to[i] - from[i]);
  }
}

/// Sample the previous OCP at `index` on its own stage grid (0..N states, 0..N-1 inputs).
void sample_previous(
  const SolverSolution & previous, const double index, std::array<double, opt_nx> & state,
  std::array<double, opt_nu> * input)
{
  const double state_index =
    std::clamp(index, 0.0, static_cast<double>(opt_horizon));
  const auto state_lower = static_cast<size_t>(std::floor(state_index));
  const size_t state_upper = std::min(state_lower + 1, opt_horizon);
  lerp_state(
    previous.states[state_lower], previous.states[state_upper], state_index - state_lower, state);

  if (input == nullptr) {
    return;
  }
  const double input_index =
    std::clamp(index, 0.0, static_cast<double>(opt_horizon - 1));
  const auto input_lower = static_cast<size_t>(std::floor(input_index));
  const size_t input_upper = std::min(input_lower + 1, opt_horizon - 1);
  lerp_input(
    previous.inputs[input_lower], previous.inputs[input_upper], input_index - input_lower, *input);
}

void fill_arc_length(
  const SolverSolution & solution, std::array<double, opt_horizon + 1> & arc)
{
  arc[0] = 0.0;
  for (size_t i = 1; i <= opt_horizon; ++i) {
    const auto & a = solution.states[i - 1];
    const auto & b = solution.states[i];
    arc[i] = arc[i - 1] + std::hypot(b[kX] - a[kX], b[kY] - a[kY]);
  }
}

/// Interpolate the previous plan at path station `query_s`. False if `query_s` is past the
/// previous polyline (no clamp to the last point).
bool sample_previous_at_s(
  const SolverSolution & previous, const std::array<double, opt_horizon + 1> & arc,
  const double query_s, std::array<double, opt_nx> & state)
{
  constexpr double slack_m = 1.0e-6;
  if (query_s > arc.back() + slack_m) {
    return false;
  }
  if (query_s <= arc.front()) {
    state = previous.states[0];
    return true;
  }
  for (size_t i = 0; i < opt_horizon; ++i) {
    if (query_s <= arc[i + 1] + slack_m) {
      const double ds = arc[i + 1] - arc[i];
      const double ratio = ds > slack_m ? (query_s - arc[i]) / ds : 0.0;
      lerp_state(
        previous.states[i], previous.states[i + 1], std::clamp(ratio, 0.0, 1.0), state);
      return true;
    }
  }
  return false;
}

/// Mean chord speed of the first three time-indexed points: (|p1-p0| + |p2-p1|) / (2 dt).
double average_speed_from_first_three_points(const Trajectory & trajectory)
{
  constexpr size_t n_points = 3;
  if (trajectory.points.size() < 2) {
    return 0.0;
  }
  const size_t last = std::min(n_points, trajectory.points.size()) - 1;
  double path_length_m = 0.0;
  for (size_t i = 0; i < last; ++i) {
    const auto & a = trajectory.points[i].pose.position;
    const auto & b = trajectory.points[i + 1].pose.position;
    path_length_m += std::hypot(b.x - a.x, b.y - a.y);
  }
  return path_length_m / (opt_dt_s * static_cast<double>(last));
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
  const std::optional<double> & current_steering_angle_rad,
  const double current_longitudinal_accel_mps2, const size_t batch_index,
  const std::optional<geometry_msgs::msg::Pose> & goal_pose, const bool reference_was_shifted)
{
  OptimizationResult result;
  result.trajectory = raw_trajectory;

  if (raw_trajectory.points.size() < opt_horizon || batch_index >= previous_solutions_.size()) {
    return result;
  }

  const auto & ego_pose = ego_odometry.pose.pose;
  const double base_x = ego_pose.position.x;
  const double base_y = ego_pose.position.y;
  const double yaw0 = yaw_from_quaternion(ego_pose.orientation);
  const double v0 = std::clamp(
    average_speed_from_first_three_points(raw_trajectory), params_.min_velocity_mps,
    params_.max_velocity_mps);

  double delta0 = 0.0;
  if (current_steering_angle_rad.has_value()) {
    delta0 = *current_steering_angle_rad;
  } else {
    const double yaw_rate = ego_odometry.twist.twist.angular.z;
    const double speed = std::max(std::abs(v0), min_speed_for_steering_estimation_mps);
    delta0 = std::atan(wheelbase_m_ * yaw_rate / speed);
  }
  delta0 = std::clamp(delta0, -max_steering_angle_rad_, max_steering_angle_rad_);

  const double a0 = std::clamp(
    current_longitudinal_accel_mps2, params_.min_acceleration_mps2, params_.max_acceleration_mps2);

  result.initial_speed_mps = v0;
  result.initial_accel_mps2 = a0;

  const std::array<double, opt_nx> initial_state{0.0, 0.0, yaw0, v0, delta0, a0};

  std::array<StageReference, opt_horizon> references;
  double previous_yaw = yaw0;
  for (size_t k = 0; k < opt_horizon; ++k) {
    const auto & point = raw_trajectory.points[k];
    StageReference & ref = references[k];
    ref.x = point.pose.position.x - base_x;
    ref.y = point.pose.position.y - base_y;
    const double raw_yaw = yaw_from_quaternion(point.pose.orientation);
    ref.yaw = previous_yaw + autoware_utils_math::normalize_radian(raw_yaw - previous_yaw);
    previous_yaw = ref.yaw;
  }

  if (goal_pose) {
    const bool goal_position_changed =
      !observed_goal_pose_ ||
      std::hypot(
        goal_pose->position.x - observed_goal_pose_->position.x,
        goal_pose->position.y - observed_goal_pose_->position.y) > goal_position_change_threshold_m;
    if (goal_position_changed) {
      reset_goal_snap_state();
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
      reset_goal_snap_state();
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

  std::optional<GoalTerminalReference> goal_terminal_reference;
  if (latched_goal_pose_) {
    const auto & goal = *latched_goal_pose_;
    const double raw_goal_yaw = yaw_from_quaternion(goal.orientation);
    references.back().x = goal.position.x - base_x;
    references.back().y = goal.position.y - base_y;
    references.back().yaw =
      previous_yaw + autoware_utils_math::normalize_radian(raw_goal_yaw - previous_yaw);
    goal_terminal_reference =
      GoalTerminalReference{references.back().x, references.back().y, references.back().yaw, 0.0};
  }

  const rclcpp::Time stamp(raw_trajectory.header.stamp, RCL_ROS_TIME);
  const bool goal_active = goal_terminal_reference.has_value();
  result.goal_snap_active = goal_active;
  SolverSolution warm_start;
  std::unique_ptr<SolverSolution> previous_local;
  const SolverSolution * warm_start_ptr = nullptr;
  const char * warm_start_skip = "no_warm_start";
  bool temporal_goal_compatible = true;
  auto & previous = previous_solutions_[batch_index];
  if (previous.has_value()) {
    const double age_s = (stamp - previous->stamp).seconds();
    result.warm_start_age_s = age_s;
    if (age_s >= 0.0 && age_s <= max_warm_start_age_s) {
      const double stage_shift = age_s / opt_dt_s;
      previous_local = std::make_unique<SolverSolution>(previous->solution);
      for (auto & state : previous_local->states) {
        state[0] -= base_x;
        state[1] -= base_y;
      }
      // SQP guess stays time-aligned: current stage s is previous index s + age/dt.
      for (size_t stage = 0; stage <= opt_horizon; ++stage) {
        std::array<double, opt_nu> input{};
        sample_previous(
          *previous_local, static_cast<double>(stage) + stage_shift, warm_start.states[stage],
          stage < opt_horizon ? &input : nullptr);
        if (stage < opt_horizon) {
          warm_start.inputs[stage] = input;
        }
      }
      warm_start_ptr = &warm_start;
      if (previous->goal_active != goal_active) {
        temporal_goal_compatible = false;
        warm_start_skip = "goal_flag_mismatch";
      } else {
        warm_start_skip = "none";
      }
    } else {
      warm_start_skip = age_s < 0.0 ? "stamp_rewind" : "warm_start_stale";
      previous.reset();
    }
  }

  std::array<StageTemporalReference, opt_horizon> temporal_references;
  const std::array<StageTemporalReference, opt_horizon> * temporal_references_ptr = nullptr;
  const bool allow_temporal = params_.temporal_consistency.enable && !reference_was_shifted &&
                              warm_start_ptr != nullptr && temporal_goal_compatible;
  if (!params_.temporal_consistency.enable) {
    result.temporal_skip_reason = "disabled";
  } else if (reference_was_shifted) {
    result.temporal_skip_reason = "border_shift";
  } else {
    result.temporal_skip_reason = warm_start_skip;
  }
  result.temporal_applied = false;
  if (allow_temporal && previous_local) {
    std::array<double, opt_horizon + 1> previous_s{};
    fill_arc_length(*previous_local, previous_s);
    double acc_s = 0.0;
    double prev_x = 0.0;
    double prev_y = 0.0;
    bool past_old_path = false;
    size_t valid_stages = 0;
    for (size_t k = 0; k < opt_horizon; ++k) {
      acc_s += std::hypot(references[k].x - prev_x, references[k].y - prev_y);
      prev_x = references[k].x;
      prev_y = references[k].y;
      if (past_old_path) {
        continue;
      }
      std::array<double, opt_nx> sampled{};
      if (!sample_previous_at_s(*previous_local, previous_s, acc_s, sampled)) {
        past_old_path = true;
        continue;
      }
      StageTemporalReference & ref = temporal_references[k];
      ref.x = sampled[kX];
      ref.y = sampled[kY];
      ref.yaw =
        references[k].yaw + autoware_utils_math::normalize_radian(sampled[kPsi] - references[k].yaw);
      ref.velocity = sampled[kV];
      ref.valid = true;
      ++valid_stages;
    }
    result.temporal_valid_stages = valid_stages;
    if (valid_stages > 0) {
      temporal_references_ptr = &temporal_references;
      result.temporal_applied = true;
    } else {
      result.temporal_skip_reason = "beyond_previous_path";
    }
  }

  SolverSolution solution = solver_->solve(
    initial_state, references, goal_terminal_reference, temporal_references_ptr, warm_start_ptr);
  result.solver_status = solution.status;
  result.solve_time_ms = solution.solve_time_s * 1e3;

  if (!solution.success()) {
    previous.reset();
    return result;
  }

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
    point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
      autoware_utils_math::normalize_radian(state[2]));
    point.longitudinal_velocity_mps = static_cast<float>(state[kV]);
    point.front_wheel_angle_rad = static_cast<float>(state[kDelta]);
    point.acceleration_mps2 = static_cast<float>(state[kA]);
    point.heading_rate_rps = static_cast<float>(state[kV] * std::tan(state[kDelta]) / wheelbase_m_);
    optimized.points.push_back(point);
  }
  result.trajectory = std::move(optimized);
  result.optimized = true;

  for (auto & state : solution.states) {
    state[0] += base_x;
    state[1] += base_y;
  }
  previous = PreviousSolution{solution, stamp, goal_active};

  return result;
}

void TrajectoryOptimizer::reset_goal_snap_state()
{
  latched_goal_pose_.reset();
}

void TrajectoryOptimizer::clear_warm_start(const size_t batch_index)
{
  if (batch_index < previous_solutions_.size()) {
    previous_solutions_[batch_index].reset();
  }
}

}  // namespace autoware::trajectory_modifier::time_sequence_raw
