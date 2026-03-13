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

#include "autoware/trajectory_validator/filters/safety/vehicle_constraint_filter.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
/**
 * @brief Convert a Duration message to seconds.
 */
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1e-9;
}

/**
 * @brief Convert TrajectoryPoint to speed (m/s)
 */
double to_speed(const TrajectoryPoint & point)
{
  return std::sqrt(
    point.longitudinal_velocity_mps * point.longitudinal_velocity_mps +
    point.lateral_velocity_mps * point.lateral_velocity_mps);
}

/**
 * @brief Convert two TrajectoryPoints to acceleration (m/s^2)
 */
double to_acceleration(const TrajectoryPoint & prev_point, const TrajectoryPoint & curr_point)
{
  double prev_speed = to_speed(prev_point);
  double curr_speed = to_speed(curr_point);
  double dt = to_seconds(curr_point.time_from_start) - to_seconds(prev_point.time_from_start);
  return dt > 0 ? (curr_speed - prev_speed) / dt : 0.0;
}

/**
 * @brief Convert TrajectoryPoints to steering angle (rad)
 */
double to_steering_angle(
  const TrajectoryPoint & prev_point, const TrajectoryPoint & curr_point,
  const TrajectoryPoint & next_point, const VehicleInfo & vehicle_info)
{
  const auto & prev_p = prev_point.pose.position;
  const auto & curr_p = curr_point.pose.position;
  const auto & next_p = next_point.pose.position;

  try {
    const double curvature = autoware_utils_geometry::calc_curvature(prev_p, curr_p, next_p);
    return std::atan(vehicle_info.wheel_base_m * curvature);
  } catch (...) {
    return 0.0;  // throw exception if three points are too close
  }
}

/**
 * @brief Convert four TrajectoryPoints to steering rate (rad/s)
 */
double to_steering_rate(
  const TrajectoryPoint & prev_prev_point, const TrajectoryPoint & prev_point,
  const TrajectoryPoint & curr_point, const TrajectoryPoint & next_point,
  const VehicleInfo & vehicle_info)
{
  const double prev_steering_angle =
    to_steering_angle(prev_prev_point, prev_point, curr_point, vehicle_info);
  const double curr_steering_angle =
    to_steering_angle(prev_point, curr_point, next_point, vehicle_info);
  const double dt = to_seconds(curr_point.time_from_start) - to_seconds(prev_point.time_from_start);
  return dt > 0 ? std::abs(curr_steering_angle - prev_steering_angle) / dt : 0.0;
}
}  // namespace

VehicleConstraintFilter::VehicleConstraintFilter() : ValidatorInterface("VehicleConstraintFilter")
{
}

void VehicleConstraintFilter::set_parameters(rclcpp::Node & node)
{
  using autoware_utils_rclcpp::get_or_declare_parameter;

  params_.max_speed = get_or_declare_parameter<double>(node, "vehicle_constraint.max_speed");
  params_.max_acceleration =
    get_or_declare_parameter<double>(node, "vehicle_constraint.max_acceleration");
  params_.max_deceleration =
    get_or_declare_parameter<double>(node, "vehicle_constraint.max_deceleration");
  params_.max_steering_angle =
    get_or_declare_parameter<double>(node, "vehicle_constraint.max_steering_angle");
  params_.max_steering_rate =
    get_or_declare_parameter<double>(node, "vehicle_constraint.max_steering_rate");
}

void VehicleConstraintFilter::update_parameters(const std::vector<rclcpp::Parameter> & parameters)
{
  autoware_utils_rclcpp::update_param(
    parameters, "vehicle_constraint.max_speed", params_.max_speed);

  autoware_utils_rclcpp::update_param(
    parameters, "vehicle_constraint.max_acceleration", params_.max_acceleration);
  autoware_utils_rclcpp::update_param(
    parameters, "vehicle_constraint.max_deceleration", params_.max_deceleration);
  autoware_utils_rclcpp::update_param(
    parameters, "vehicle_constraint.max_steering_angle", params_.max_steering_angle);
  autoware_utils_rclcpp::update_param(
    parameters, "vehicle_constraint.max_steering_rate", params_.max_steering_rate);
}

VehicleConstraintFilter::result_t VehicleConstraintFilter::is_feasible(
  const TrajectoryPoints & traj_points, const FilterContext &)
{
  if (!vehicle_info_ptr_) {
    return tl::make_unexpected("Vehicle info not set");
  }

  // NOTE: Feasibility decision logic might be more complex in the future
  std::vector<TrajectoryMetricStatus> metrics;
  bool is_overall_ok = true;
  for (const auto & checker : checkers_) {
    auto [status, is_ok] = (this->*checker)(traj_points);

    // NOTE: Once an error occurred validation level will be ERROR
    is_overall_ok = is_overall_ok && is_ok;

    metrics.push_back(std::move(status));
  }

  return autoware_internal_planning_msgs::build<TrajectoryValidationStatus>()
    .name(get_name())
    .level(is_overall_ok ? TrajectoryValidationStatus::OK : TrajectoryValidationStatus::ERROR)
    .metrics(std::move(metrics));
}

VehicleConstraintFilter::metric_t VehicleConstraintFilter::check_speed(
  const TrajectoryPoints & traj_points) const
{
  const bool is_ok = is_speed_ok(traj_points, params_.max_speed);

  return {
    autoware_internal_planning_msgs::build<TrajectoryMetricStatus>()
      .name("check_speed")
      .level(is_ok ? TrajectoryMetricStatus::OK : TrajectoryMetricStatus::ERROR)
      .score(0.0),  // To be updated
    is_ok};
}

VehicleConstraintFilter::metric_t VehicleConstraintFilter::check_acceleration(
  const TrajectoryPoints & traj_points) const
{
  const bool is_ok = is_acceleration_ok(traj_points, params_.max_acceleration);

  return {
    autoware_internal_planning_msgs::build<TrajectoryMetricStatus>()
      .name("check_acceleration")
      .level(is_ok ? TrajectoryMetricStatus::OK : TrajectoryMetricStatus::ERROR)
      .score(0.0),  // To be updated
    is_ok};
}

VehicleConstraintFilter::metric_t VehicleConstraintFilter::check_deceleration(
  const TrajectoryPoints & traj_points) const
{
  const bool is_ok = is_deceleration_ok(traj_points, params_.max_deceleration);

  return {
    autoware_internal_planning_msgs::build<TrajectoryMetricStatus>()
      .name("check_deceleration")
      .level(is_ok ? TrajectoryMetricStatus::OK : TrajectoryMetricStatus::ERROR)
      .score(0.0),  // To be updated
    is_ok};
}

VehicleConstraintFilter::metric_t VehicleConstraintFilter::check_steering_angle(
  const TrajectoryPoints & traj_points) const
{
  const bool is_ok =
    is_steering_angle_ok(traj_points, *vehicle_info_ptr_, params_.max_steering_angle);

  return {
    autoware_internal_planning_msgs::build<TrajectoryMetricStatus>()
      .name("check_steering_angle")
      .level(is_ok ? TrajectoryMetricStatus::OK : TrajectoryMetricStatus::ERROR)
      .score(0.0),  // To be updated
    is_ok};
}

VehicleConstraintFilter::metric_t VehicleConstraintFilter::check_steering_rate(
  const TrajectoryPoints & traj_points) const
{
  const bool is_ok =
    is_steering_rate_ok(traj_points, *vehicle_info_ptr_, params_.max_steering_rate);

  return {
    autoware_internal_planning_msgs::build<TrajectoryMetricStatus>()
      .name("check_steering_rate")
      .level(is_ok ? TrajectoryMetricStatus::OK : TrajectoryMetricStatus::ERROR)
      .score(0.0),  // To be updated
    is_ok};
}

// --- Helper functions for constraint checks ---

bool is_speed_ok(const TrajectoryPoints & traj_points, double max_speed)
{
  for (const auto & point : traj_points) {
    double speed = to_speed(point);
    if (speed > max_speed) {
      return false;
    }
  }
  return true;
}

bool is_acceleration_ok(const TrajectoryPoints & traj_points, double max_acceleration)
{
  for (size_t i = 1; i < traj_points.size(); ++i) {
    double acc = to_acceleration(traj_points[i - 1], traj_points[i]);
    if (acc > max_acceleration) {
      return false;
    }
  }
  return true;
}

bool is_deceleration_ok(const TrajectoryPoints & traj_points, double max_deceleration)
{
  for (size_t i = 1; i < traj_points.size(); ++i) {
    double dec = to_acceleration(traj_points[i - 1], traj_points[i]);
    if (dec < 0 && std::abs(dec) > max_deceleration) {
      return false;
    }
  }
  return true;
}

bool is_steering_angle_ok(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, double max_steering_angle)
{
  for (size_t i = 1; i < traj_points.size() - 1; ++i) {
    const auto & prev_point = traj_points[i - 1];
    const auto & curr_point = traj_points[i];
    const auto & next_point = traj_points[i + 1];
    double steering_angle = to_steering_angle(prev_point, curr_point, next_point, vehicle_info);
    if (std::abs(steering_angle) > max_steering_angle) {
      return false;
    }
  }
  return true;
}

bool is_steering_rate_ok(
  const TrajectoryPoints & traj_points, const VehicleInfo & vehicle_info, double max_steering_rate)
{
  for (size_t i = 2; i + 1 < traj_points.size(); ++i) {
    double steering_rate = to_steering_rate(
      traj_points[i - 2], traj_points[i - 1], traj_points[i], traj_points[i + 1], vehicle_info);
    if (steering_rate > max_steering_rate) {
      return false;
    }
  }
  return true;
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::VehicleConstraintFilter, autoware::trajectory_validator::plugin::ValidatorInterface)
