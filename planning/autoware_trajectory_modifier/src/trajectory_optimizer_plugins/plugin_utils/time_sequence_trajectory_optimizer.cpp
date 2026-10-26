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
#include <utility>

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

double time_from_start_s(const TrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

bool has_increasing_time(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 2) {
    return false;
  }
  return time_from_start_s(trajectory.points.back()) >
         time_from_start_s(trajectory.points.front()) + 1e-3;
}

/// ResWorld prepends the ego pose at the header stamp. Diffusion candidates start at dt.
bool starts_at_stamp(const Trajectory & trajectory)
{
  return !trajectory.points.empty() &&
         std::abs(time_from_start_s(trajectory.points.front())) <= 1e-3;
}

struct SampledPose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

SampledPose sample_indexed(const Trajectory & trajectory, const size_t index)
{
  const auto & point = trajectory.points[std::min(index, trajectory.points.size() - 1)];
  SampledPose pose;
  pose.x = point.pose.position.x;
  pose.y = point.pose.position.y;
  pose.z = point.pose.position.z;
  pose.yaw = yaw_from_quaternion(point.pose.orientation);
  return pose;
}

/// Pose at absolute `t` along `time_from_start`. `t` must lie inside the message, aside from
/// the half-step clamp applied by the caller.
SampledPose sample_at_time(const Trajectory & trajectory, const double t)
{
  const auto & points = trajectory.points;
  if (t <= time_from_start_s(points.front())) {
    return sample_indexed(trajectory, 0);
  }
  if (t >= time_from_start_s(points.back())) {
    return sample_indexed(trajectory, points.size() - 1);
  }
  size_t hi = 1;
  while (hi + 1 < points.size() && time_from_start_s(points[hi]) < t) {
    ++hi;
  }
  const size_t lo = hi - 1;
  const double t_lo = time_from_start_s(points[lo]);
  const double t_hi = time_from_start_s(points[hi]);
  const double ratio = (t_hi > t_lo) ? (t - t_lo) / (t_hi - t_lo) : 0.0;
  const SampledPose a = sample_indexed(trajectory, lo);
  const SampledPose b = sample_indexed(trajectory, hi);
  SampledPose pose;
  pose.x = a.x + ratio * (b.x - a.x);
  pose.y = a.y + ratio * (b.y - a.y);
  pose.z = a.z + ratio * (b.z - a.z);
  pose.yaw = a.yaw + ratio * autoware_utils_math::normalize_radian(b.yaw - a.yaw);
  return pose;
}

void lerp_state(
  const std::array<double, opt_nx> & from, const std::array<double, opt_nx> & to,
  const double ratio, std::array<double, opt_nx> & out)
{
  for (size_t i = 0; i < opt_nx; ++i) {
    const double delta =
      (i == kPsi) ? autoware_utils_math::normalize_radian(to[i] - from[i]) : (to[i] - from[i]);
    out[i] = from[i] + ratio * delta;
  }
}

void lerp_input(
  const std::array<double, opt_nu> & from, const std::array<double, opt_nu> & to,
  const double ratio, std::array<double, opt_nu> & out)
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
  const double state_index = std::clamp(index, 0.0, static_cast<double>(opt_horizon));
  const auto state_lower = static_cast<size_t>(std::floor(state_index));
  const size_t state_upper = std::min(state_lower + 1, opt_horizon);
  lerp_state(
    previous.states[state_lower], previous.states[state_upper], state_index - state_lower, state);

  if (input == nullptr) {
    return;
  }
  const double input_index = std::clamp(index, 0.0, static_cast<double>(opt_horizon - 1));
  const auto input_lower = static_cast<size_t>(std::floor(input_index));
  const size_t input_upper = std::min(input_lower + 1, opt_horizon - 1);
  lerp_input(
    previous.inputs[input_lower], previous.inputs[input_upper], input_index - input_lower, *input);
}

void fill_time_aligned_temporal(
  const SolverSolution & previous_local, const std::array<StageReference, opt_horizon> & references,
  const double stage_shift, std::array<StageTemporalReference, opt_horizon> & out)
{
  for (size_t k = 0; k < opt_horizon; ++k) {
    // Same index as ml_planner: published stage k is previous state (k+1) + age/dt.
    const double index =
      std::min(static_cast<double>(k + 1) + stage_shift, static_cast<double>(opt_horizon));
    std::array<double, opt_nx> sampled{};
    sample_previous(previous_local, index, sampled, nullptr);
    StageTemporalReference & ref = out[k];
    ref.x = sampled[kX];
    ref.y = sampled[kY];
    ref.yaw =
      references[k].yaw + autoware_utils_math::normalize_radian(sampled[kPsi] - references[k].yaw);
    ref.velocity = sampled[kV];
    ref.valid = references[k].track_pose;
  }
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

  // The generated solver is fixed at opt_horizon steps. Fewer input points are sampled onto
  // the stages they cover. An empty message never calls acados. Status 0 means a successful
  // solve, so this return stays distinct from success.
  if (raw_trajectory.points.size() < 2 || batch_index >= previous_solutions_.size()) {
    result.solver_status = -3;
    result.temporal_skip_reason = "empty_or_bad_index";
    return result;
  }

  const auto & ego_pose = ego_odometry.pose.pose;
  const double base_x = ego_pose.position.x;
  const double base_y = ego_pose.position.y;
  const double yaw0 = yaw_from_quaternion(ego_pose.orientation);
  const double v0 = std::clamp(
    static_cast<double>(ego_odometry.twist.twist.linear.x), params_.min_velocity_mps,
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

  // references[k] is the pose target for solver stage k+1. Stage time is (k+1)*dt after
  // the header stamp, which is where the initial state sits. Sampling from the first
  // point's stamp would shift a diffusion candidate that starts at 0.1 s.
  // Messages without stamps keep the index convention used by the unit tests: point k
  // is the target for stage k+1.
  const bool timed = has_increasing_time(raw_trajectory);
  std::array<SampledPose, opt_horizon> sampled{};
  size_t tracked_stages = 0;
  if (timed) {
    const double t_last = time_from_start_s(raw_trajectory.points.back());
    for (size_t k = 0; k < opt_horizon; ++k) {
      const double t_stage = static_cast<double>(k + 1) * opt_dt_s;
      if (t_stage > t_last + 0.5 * opt_dt_s) {
        break;
      }
      sampled[k] = sample_at_time(raw_trajectory, std::min(t_stage, t_last));
      tracked_stages = k + 1;
    }
  } else {
    tracked_stages = std::min(raw_trajectory.points.size(), opt_horizon);
    for (size_t k = 0; k < tracked_stages; ++k) {
      sampled[k] = sample_indexed(raw_trajectory, k);
    }
  }
  if (tracked_stages == 0) {
    result.solver_status = -3;
    result.temporal_skip_reason = "empty_or_bad_index";
    return result;
  }

  std::array<StageReference, opt_horizon> references;
  double previous_yaw = yaw0;
  for (size_t k = 0; k < opt_horizon; ++k) {
    StageReference & ref = references[k];
    if (k >= tracked_stages) {
      continue;
    }
    ref.track_pose = true;
    ref.x = sampled[k].x - base_x;
    ref.y = sampled[k].y - base_y;
    ref.yaw = previous_yaw + autoware_utils_math::normalize_radian(sampled[k].yaw - previous_yaw);
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
      // A new route goal must not reuse the old snapped plan as a temporal reference.
      for (auto & previous_plan : previous_solutions_) {
        previous_plan.reset();
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
      reset_goal_snap_state();
    } else {
      // Predicted endpoint of this message. A 6 s input has no point at the compiled 8 s stage.
      const auto & terminal = raw_trajectory.points.back().pose.position;
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
    // The published endpoint is solver state `tracked_stages`. On a full 8 s input that is the
    // compiled terminal, so the extra goal weights apply. On a shorter input it is the last
    // tracked stage; the compiled 8 s terminal stays past the message and keeps zero pose cost.
    StageReference & published_terminal = references[tracked_stages - 1];
    published_terminal.track_pose = true;
    published_terminal.x = goal.position.x - base_x;
    published_terminal.y = goal.position.y - base_y;
    published_terminal.yaw =
      previous_yaw + autoware_utils_math::normalize_radian(raw_goal_yaw - previous_yaw);
    goal_terminal_reference = GoalTerminalReference{
      published_terminal.x, published_terminal.y, published_terminal.yaw, 0.0};
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
  const bool allow_temporal = params_.temporal_consistency.enable && warm_start_ptr != nullptr &&
                              temporal_goal_compatible && !reference_was_shifted;
  if (!params_.temporal_consistency.enable) {
    result.temporal_skip_reason = "disabled";
  } else if (reference_was_shifted) {
    result.temporal_skip_reason = "border_shift";
  } else {
    result.temporal_skip_reason = warm_start_skip;
  }
  result.temporal_applied = false;
  if (allow_temporal && previous_local) {
    const double stage_shift = result.warm_start_age_s / opt_dt_s;
    fill_time_aligned_temporal(*previous_local, references, stage_shift, temporal_references);
    result.temporal_valid_stages = static_cast<size_t>(std::count_if(
      temporal_references.begin(), temporal_references.end(),
      [](const StageTemporalReference & ref) { return ref.valid; }));
    // Stages past a short message keep jerk and steering-rate cost only. Temporal pose cost
    // stays on the tracked prefix, and on the terminal when goal snap has latched it.
    if (result.temporal_valid_stages > 0) {
      temporal_references_ptr = &temporal_references;
      result.temporal_applied = true;
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

  // A timed input that already contains the ego point at the header stamp keeps that
  // anchor. Temporal followers clamp "now" up to the first timestamp, so dropping it
  // makes a fresh plan use the 0.1 s state as the current reference. Diffusion inputs
  // that start at dt, and untimed inputs, keep the stage-1 .. stage-N output.
  const bool keep_stamp_anchor = timed && starts_at_stamp(raw_trajectory);

  Trajectory optimized;
  optimized.header = raw_trajectory.header;
  optimized.points.reserve(tracked_stages + (keep_stamp_anchor ? 1U : 0U));

  const auto append_state =
    [&](const std::array<double, opt_nx> & state, const double time_s, const double z) {
      TrajectoryPoint point;
      point.time_from_start.sec = static_cast<int32_t>(time_s);
      point.time_from_start.nanosec =
        static_cast<uint32_t>((time_s - point.time_from_start.sec) * 1e9);
      point.pose.position.x = state[0] + base_x;
      point.pose.position.y = state[1] + base_y;
      point.pose.position.z = z;
      point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
        autoware_utils_math::normalize_radian(state[2]));
      point.longitudinal_velocity_mps = static_cast<float>(state[kV]);
      point.front_wheel_angle_rad = static_cast<float>(state[kDelta]);
      point.acceleration_mps2 = static_cast<float>(state[kA]);
      point.heading_rate_rps =
        static_cast<float>(state[kV] * std::tan(state[kDelta]) / wheelbase_m_);
      optimized.points.push_back(point);
    };
  if (keep_stamp_anchor) {
    append_state(solution.states[0], 0.0, raw_trajectory.points.front().pose.position.z);
  }
  for (size_t i = 1; i <= tracked_stages; ++i) {
    append_state(solution.states[i], opt_dt_s * static_cast<double>(i), sampled[i - 1].z);
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
