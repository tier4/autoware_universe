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

#include "mpt_optimizer.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <magic_enum.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{
//! [m/s] a point at or below this speed is a stop point, not a slow one.
constexpr double STOP_VELOCITY_THRESHOLD = 1.0e-3;
//! [m] how far the optimised height may sit outside the height range of the input path.
constexpr double HEIGHT_TOLERANCE = 1.0;

/**
 * @brief Is every optimised height plausible for this input path?
 *
 * The optimiser does not model z - it carries the input path's height through - so the output must
 * stay inside the input's height range. This is checked rather than trusted because the height is
 * not inert: the post-resample step derives the pitch of every published pose from consecutive z
 * values, so a height that is off by metres produces poses that point out of the road plane, and
 * the controller follows them. That failure is silent in every other signal.
 */
bool heights_are_plausible(
  const std::vector<mpt::OptimizedPoint> & points, const TrajectoryPoints & input)
{
  auto [min_it, max_it] = std::minmax_element(
    input.begin(), input.end(),
    [](const auto & a, const auto & b) { return a.pose.position.z < b.pose.position.z; });
  const double lower = min_it->pose.position.z - HEIGHT_TOLERANCE;
  const double upper = max_it->pose.position.z + HEIGHT_TOLERANCE;
  return std::all_of(points.begin(), points.end(), [lower, upper](const auto & point) {
    return std::isfinite(point.z) && point.z >= lower && point.z <= upper;
  });
}

std::string join(const std::vector<std::string> & values)
{
  return std::accumulate(
    values.begin(), values.end(), std::string{}, [](const std::string & acc, const auto & value) {
      return acc.empty() ? value : acc + ", " + value;
    });
}
}  // namespace

mpt::OptimizerParams make_mpt_optimizer_params(const Params & params, const VehicleInfo & vehicle)
{
  const auto & source = params.acados_mpt;
  mpt::OptimizerParams result;

  result.vehicle.wheel_base = vehicle.wheel_base_m;
  result.vehicle.max_steer_angle = vehicle.max_steer_angle_rad;
  result.vehicle.front_overhang = vehicle.front_overhang_m;
  result.vehicle.rear_overhang = vehicle.rear_overhang_m;
  result.vehicle.width = vehicle.vehicle_width_m;

  result.limits.max_velocity = source.limits.max_velocity;
  result.limits.max_acceleration = source.limits.max_acceleration;
  result.limits.min_acceleration = source.limits.min_acceleration;
  result.limits.max_jerk = source.limits.max_jerk;
  result.limits.max_lateral_acceleration = source.limits.max_lateral_acceleration;
  result.limits.max_steer_rate = source.limits.max_steer_rate;
  result.limits.max_curvature = source.limits.max_curvature;
  result.limits.max_curvature_rate = source.limits.max_curvature_rate;
  result.limits.max_acceleration_rate = source.limits.max_acceleration_rate;

  result.weights.position = source.weights.position;
  result.weights.yaw = source.weights.yaw;
  result.weights.curvature = source.weights.curvature;
  result.weights.curvature_rate = source.weights.curvature_rate;
  result.weights.squared_speed = source.weights.squared_speed;
  result.weights.acceleration = source.weights.acceleration;
  result.weights.acceleration_rate = source.weights.acceleration_rate;
  result.weights.stretch = source.weights.stretch;
  result.weights.terminal_position = source.weights.terminal_position;
  result.weights.terminal_yaw = source.weights.terminal_yaw;
  result.weights.terminal_curvature = source.weights.terminal_curvature;
  result.weights.terminal_squared_speed = source.weights.terminal_squared_speed;
  result.weights.soft_limit = source.weights.soft_limit;

  result.stretch.min_stretch = source.stretch.min_stretch;
  result.stretch.max_stretch = source.stretch.max_stretch;
  result.stretch.max_station_error = source.stretch.max_station_error;

  result.solver.max_iterations = static_cast<int>(source.solver.max_iterations);
  result.solver.tolerance = source.solver.tolerance;
  result.solver.warm_start = source.solver.warm_start;
  result.solver.terminal_position_tolerance = source.solver.terminal_position_tolerance;
  result.solver.terminal_yaw_tolerance = source.solver.terminal_yaw_tolerance;
  result.solver.max_arc_length = source.solver.max_arc_length;
  result.solver.arc_length_per_stage_warn = source.solver.arc_length_per_stage_warn;
  result.solver.verification_tolerance = source.solver.verification_tolerance;
  result.solver.max_dynamics_residual = source.solver.max_dynamics_residual;

  // The corridor stays disabled: this first step optimises the path shape and speed alone.
  return result;
}

mpt::OptimizationInput make_optimization_input(
  const TrajectoryPoints & traj_points, const Odometry & odometry, const double acceleration,
  const std::optional<double> & ego_curvature)
{
  mpt::OptimizationInput input;
  input.path.reserve(traj_points.size());
  input.velocity_limits.reserve(traj_points.size());

  double arc_length = 0.0;
  for (size_t i = 0; i < traj_points.size(); ++i) {
    const auto & point = traj_points[i];
    mpt::Pose2d pose;
    pose.x = point.pose.position.x;
    pose.y = point.pose.position.y;
    pose.z = point.pose.position.z;
    pose.yaw = tf2::getYaw(point.pose.orientation);
    input.path.push_back(pose);

    const double velocity = point.longitudinal_velocity_mps;
    input.velocity_limits.push_back(velocity);

    if (i > 0) {
      arc_length += autoware_utils::calc_distance2d(
        traj_points[i - 1].pose.position, traj_points[i].pose.position);
    }
    // The planner has already decided where to stop (modifier / map-based stop points); the first
    // zero is that station. Everything past it is zero too, which the per-point cap carries.
    if (!input.stop_arc_length && velocity <= STOP_VELOCITY_THRESHOLD) {
      input.stop_arc_length = arc_length;
    }
  }

  input.ego.pose.x = odometry.pose.pose.position.x;
  input.ego.pose.y = odometry.pose.pose.position.y;
  input.ego.pose.z = odometry.pose.pose.position.z;
  input.ego.pose.yaw = tf2::getYaw(odometry.pose.pose.orientation);
  input.ego.velocity = odometry.twist.twist.linear.x;
  input.ego.acceleration = acceleration;
  input.ego.curvature = ego_curvature;

  // Mirrors the velocity smoother, which forces the last point to zero.
  input.stop_at_end = true;
  return input;
}

TrajectoryPoints to_trajectory_points(const std::vector<mpt::OptimizedPoint> & points)
{
  TrajectoryPoints traj_points;
  traj_points.reserve(points.size());
  for (const auto & point : points) {
    TrajectoryPoint traj_point;
    traj_point.pose.position.x = point.x;
    traj_point.pose.position.y = point.y;
    traj_point.pose.position.z = point.z;
    traj_point.pose.orientation = autoware_utils::create_quaternion_from_yaw(point.yaw);
    traj_point.longitudinal_velocity_mps = static_cast<float>(point.velocity);
    traj_point.acceleration_mps2 = static_cast<float>(point.acceleration);
    // The solver knows the arc length and the speed at every stage, so this is the exact time of
    // the trajectory it just produced. It freezes at the first stop point, where the elapsed time
    // stops being meaningful - the same convention the velocity smoother follows.
    traj_point.time_from_start =
      rclcpp::Duration::from_seconds(std::max(0.0, point.time_from_start));
    traj_points.push_back(traj_point);
  }
  return traj_points;
}

MptOptimizer::MptOptimizer(
  const mpt::OptimizerParams & params, const EngageParams & engage, const rclcpp::Logger & logger,
  rclcpp::Clock::SharedPtr clock)
: optimizer_(params, logger, clock), engage_(engage), logger_(logger), clock_(std::move(clock))
{
}

void MptOptimizer::apply_engage_speed(
  TrajectoryPoints & traj_points, const TrajectoryPoints & input, const Odometry & odometry) const
{
  if (!engage_.enable || traj_points.empty() || input.size() < 2) return;
  if (odometry.twist.twist.linear.x >= engage_.speed) return;

  // Where to stop is decided on the *input*: the optimised profile begins at zero simply because
  // the vehicle is standing still, and reading that as a stop point would mean never departing.
  double stop_distance = std::numeric_limits<double>::infinity();
  const auto closest = autoware::motion_utils::findNearestIndex(input, odometry.pose.pose.position);
  if (
    const auto stop_index =
      autoware::motion_utils::searchZeroVelocityIndex(input, closest, input.size())) {
    stop_distance = autoware::motion_utils::calcSignedArcLength(input, closest, *stop_index);
    // Something wants the vehicle stopped right here; departing would drive through it.
    if (stop_distance <= engage_.stop_dist_to_prohibit_engage) return;
  }

  // Only up to the stop point. The velocity smoother raises every point instead, which erases the
  // stop points the modifiers wrote and is the known cause of creeping past a stopped vehicle.
  double travelled = 0.0;
  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (i > 0) {
      travelled += autoware_utils::calc_distance2d(
        traj_points[i - 1].pose.position, traj_points[i].pose.position);
    }
    if (travelled >= stop_distance) break;
    traj_points[i].longitudinal_velocity_mps =
      std::max(traj_points[i].longitudinal_velocity_mps, static_cast<float>(engage_.speed));
    traj_points[i].acceleration_mps2 =
      std::max(traj_points[i].acceleration_mps2, static_cast<float>(engage_.acceleration));
  }
}

void MptOptimizer::update_params(const mpt::OptimizerParams & params)
{
  optimizer_.update_params(params);
}

std::optional<TrajectoryPoints> MptOptimizer::optimize(
  const TrajectoryPoints & traj_points, const Odometry & odometry, const double acceleration,
  const std::optional<double> & ego_curvature)
{
  if (traj_points.size() < 2) {
    return std::nullopt;
  }
  // b = v^2 cannot express a direction, so a reverse trajectory would come back as a forward one.
  const bool has_reverse = std::any_of(traj_points.begin(), traj_points.end(), [](const auto & p) {
    return p.longitudinal_velocity_mps < 0.0F;
  });
  if (has_reverse) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000, "[mpt] reverse trajectory is not supported; falling back.");
    return std::nullopt;
  }

  const auto input = make_optimization_input(traj_points, odometry, acceleration, ego_curvature);
  const auto result = optimizer_.optimize(input);

  if (!result.usable()) {
    const std::string violations =
      result.report.violations.empty() ? "" : " violations: " + join(result.report.violations);
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "[mpt] unusable result (%s): %s%s. Falling back.",
      std::string(magic_enum::enum_name(result.status)).c_str(), result.message.c_str(),
      violations.c_str());
    return std::nullopt;
  }
  if (!heights_are_plausible(result.points, traj_points)) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "[mpt] the optimised height left the input path's range (first point %.2f m); the pose pitch "
      "downstream would be meaningless. Falling back.",
      result.points.empty() ? 0.0 : result.points.front().z);
    return std::nullopt;
  }
  if (result.status == mpt::ResultStatus::NOT_CONVERGED) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "[mpt] feasible but not converged after %d iterations (kkt %.3e); the horizon was hard.",
      result.sqp_iterations, result.kkt_norm);
  }
  RCLCPP_DEBUG(
    logger_, "[mpt] solved in %.1f ms, %d SQP iterations, %.1f m output.",
    result.solve_time_s * 1000.0, result.sqp_iterations, result.output_arc_length);

  auto optimized = to_trajectory_points(result.points);
  apply_engage_speed(optimized, traj_points, odometry);
  return optimized;
}

}  // namespace autoware::minimum_rule_based_planner
