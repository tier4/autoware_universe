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

#include "autoware/mppi_optimizer/planning_origin_estimator.hpp"

#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace autoware::mppi_optimizer
{
namespace
{

constexpr double kNanosecondsToSeconds = 1.0E-9;
constexpr double kMinimumInterpolationInterval = 1.0E-9;

double normalize_angle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double time_from_start_seconds(const builtin_interfaces::msg::Duration & time_from_start)
{
  return static_cast<double>(time_from_start.sec) +
         static_cast<double>(time_from_start.nanosec) * kNanosecondsToSeconds;
}

PlanningOriginState state_from_point(const autoware_planning_msgs::msg::TrajectoryPoint & point)
{
  PlanningOriginState state;
  state.x = point.pose.position.x;
  state.y = point.pose.position.y;
  state.yaw = tf2::getYaw(point.pose.orientation);
  state.velocity = point.longitudinal_velocity_mps;
  return state;
}

autoware_planning_msgs::msg::TrajectoryPoint interpolate_trajectory_point(
  const autoware_planning_msgs::msg::TrajectoryPoint & lower,
  const autoware_planning_msgs::msg::TrajectoryPoint & upper, const double ratio)
{
  auto point = lower;
  point.pose.position.x =
    lower.pose.position.x + ratio * (upper.pose.position.x - lower.pose.position.x);
  point.pose.position.y =
    lower.pose.position.y + ratio * (upper.pose.position.y - lower.pose.position.y);
  point.pose.position.z =
    lower.pose.position.z + ratio * (upper.pose.position.z - lower.pose.position.z);

  const double lower_yaw = tf2::getYaw(lower.pose.orientation);
  const double upper_yaw = tf2::getYaw(upper.pose.orientation);
  const double yaw = normalize_angle(lower_yaw + ratio * normalize_angle(upper_yaw - lower_yaw));
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw);
  point.pose.orientation = tf2::toMsg(orientation);

  const auto interpolate_float = [ratio](const float lower_value, const float upper_value) {
    return static_cast<float>(
      static_cast<double>(lower_value) + ratio * static_cast<double>(upper_value - lower_value));
  };
  point.longitudinal_velocity_mps =
    interpolate_float(lower.longitudinal_velocity_mps, upper.longitudinal_velocity_mps);
  point.lateral_velocity_mps =
    interpolate_float(lower.lateral_velocity_mps, upper.lateral_velocity_mps);
  point.acceleration_mps2 = interpolate_float(lower.acceleration_mps2, upper.acceleration_mps2);
  point.heading_rate_rps = interpolate_float(lower.heading_rate_rps, upper.heading_rate_rps);
  point.front_wheel_angle_rad =
    interpolate_float(lower.front_wheel_angle_rad, upper.front_wheel_angle_rad);
  point.rear_wheel_angle_rad =
    interpolate_float(lower.rear_wheel_angle_rad, upper.rear_wheel_angle_rad);
  return point;
}

}  // namespace

PlanningOriginEstimator::PlanningOriginEstimator(const PlanningOriginEstimatorParams & params)
{
  set_params(params);
}

void PlanningOriginEstimator::set_params(const PlanningOriginEstimatorParams & params)
{
  if (
    !std::isfinite(params.max_position_error) || params.max_position_error < 0.0 ||
    !std::isfinite(params.max_yaw_error) || params.max_yaw_error < 0.0) {
    throw std::invalid_argument("Planning-origin divergence thresholds must be finite and >= 0");
  }
  params_ = params;
}

const PlanningOriginEstimatorParams & PlanningOriginEstimator::params() const noexcept
{
  return params_;
}

void PlanningOriginEstimator::store_trajectory(
  const Trajectory & trajectory, const rclcpp::Time & generation_time)
{
  if (trajectory.points.empty()) {
    reset();
    return;
  }

  previous_optimal_trajectory_ = trajectory;
  generation_time_ns_ = generation_time.nanoseconds();
}

PlanningOriginEstimate PlanningOriginEstimator::estimate(
  const PlanningOriginState & raw_ego, const rclcpp::Time & current_time,
  const double lookahead_seconds)
{
  PlanningOriginEstimate result;
  result.state = raw_ego;

  if (!previous_optimal_trajectory_) {
    return result;
  }

  const double lookahead = std::max(lookahead_seconds, 0.0);
  const double elapsed =
    static_cast<double>(current_time.nanoseconds() - generation_time_ns_) * kNanosecondsToSeconds;
  if (!std::isfinite(elapsed) || elapsed < 0.0) {
    reset();
    return result;
  }

  const auto expected_now = interpolate(elapsed);
  const auto expected_at_origin = interpolate(elapsed + lookahead);
  if (!expected_now || !expected_at_origin) {
    reset();
    return result;
  }

  result.position_error = std::hypot(expected_now->x - raw_ego.x, expected_now->y - raw_ego.y);
  result.yaw_error = std::abs(normalize_angle(expected_now->yaw - raw_ego.yaw));

  if (
    result.position_error > params_.max_position_error ||
    result.yaw_error > params_.max_yaw_error) {
    reset();
    return result;
  }

  result.state = *expected_at_origin;
  result.used_stitched_state = true;
  return result;
}

std::size_t PlanningOriginEstimator::copy_prefix(
  Trajectory & trajectory, const rclcpp::Time & current_time, const double prefix_duration) const
{
  if (!previous_optimal_trajectory_ || !std::isfinite(prefix_duration) || prefix_duration <= 0.0) {
    return 0U;
  }

  const double elapsed =
    static_cast<double>(current_time.nanoseconds() - generation_time_ns_) * kNanosecondsToSeconds;
  if (!std::isfinite(elapsed) || elapsed < 0.0) {
    return 0U;
  }

  std::size_t copied_points = 0U;
  for (auto & output_point : trajectory.points) {
    const double output_time = time_from_start_seconds(output_point.time_from_start);
    if (output_time > prefix_duration + kMinimumInterpolationInterval) {
      break;
    }

    const auto aligned_previous = interpolate_point(elapsed + output_time);
    if (!aligned_previous) {
      break;
    }

    const auto output_time_from_start = output_point.time_from_start;
    output_point = *aligned_previous;
    output_point.time_from_start = output_time_from_start;
    ++copied_points;
  }

  return copied_points;
}

void PlanningOriginEstimator::reset() noexcept
{
  previous_optimal_trajectory_.reset();
  generation_time_ns_ = 0;
}

bool PlanningOriginEstimator::has_trajectory() const noexcept
{
  return previous_optimal_trajectory_.has_value();
}

std::optional<PlanningOriginState> PlanningOriginEstimator::interpolate(
  const double relative_time) const
{
  const auto point = interpolate_point(relative_time);
  return point ? std::make_optional(state_from_point(*point)) : std::nullopt;
}

std::optional<autoware_planning_msgs::msg::TrajectoryPoint>
PlanningOriginEstimator::interpolate_point(const double relative_time) const
{
  if (!previous_optimal_trajectory_ || !std::isfinite(relative_time)) {
    return std::nullopt;
  }

  const auto & points = previous_optimal_trajectory_->points;
  if (points.empty()) {
    return std::nullopt;
  }

  const double first_time = time_from_start_seconds(points.front().time_from_start);
  const double last_time = time_from_start_seconds(points.back().time_from_start);
  if (relative_time > last_time) {
    return std::nullopt;
  }
  if (relative_time <= first_time || points.size() == 1U) {
    return points.front();
  }

  for (std::size_t upper_idx = 1U; upper_idx < points.size(); ++upper_idx) {
    const double upper_time = time_from_start_seconds(points[upper_idx].time_from_start);
    if (relative_time > upper_time) {
      continue;
    }

    const std::size_t lower_idx = upper_idx - 1U;
    const double lower_time = time_from_start_seconds(points[lower_idx].time_from_start);
    const double interval = upper_time - lower_time;
    if (interval <= kMinimumInterpolationInterval) {
      return points[upper_idx];
    }

    const double ratio = std::clamp((relative_time - lower_time) / interval, 0.0, 1.0);
    return interpolate_trajectory_point(points[lower_idx], points[upper_idx], ratio);
  }

  return std::nullopt;
}

}  // namespace autoware::mppi_optimizer
