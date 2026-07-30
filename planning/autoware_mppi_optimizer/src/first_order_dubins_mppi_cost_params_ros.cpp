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

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params_ros.hpp"

#include <string>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{

std::string param_name(const std::string & prefix, const std::string & name)
{
  return prefix.empty() ? name : prefix + name;
}

}  // namespace

void declare_first_order_dubins_mppi_cost_params(rclcpp::Node & node, const std::string & prefix)
{
  const FirstOrderDubinsMppiCostParams defaults;
  node.declare_parameter(param_name(prefix, "lambda"), defaults.lambda);
  node.declare_parameter(param_name(prefix, "desired_speed"), defaults.desired_speed);
  node.declare_parameter(param_name(prefix, "speed_coeff"), defaults.speed_coeff);
  node.declare_parameter(param_name(prefix, "track_coeff"), defaults.track_coeff);
  node.declare_parameter(param_name(prefix, "track_terminal_scale"), defaults.track_terminal_scale);
  node.declare_parameter(param_name(prefix, "heading_coeff"), defaults.heading_coeff);
  node.declare_parameter(
    param_name(prefix, "lateral_distance_coeff"), defaults.lateral_distance_coeff);
  node.declare_parameter(
    param_name(prefix, "lateral_yaw_error_coeff"), defaults.lateral_yaw_error_coeff);
  node.declare_parameter(param_name(prefix, "crash_coeff"), defaults.crash_coeff);
  node.declare_parameter(param_name(prefix, "boundary_threshold"), defaults.boundary_threshold);
  node.declare_parameter(
    param_name(prefix, "boundary_threshold_left"), defaults.boundary_threshold_left);
  node.declare_parameter(
    param_name(prefix, "boundary_threshold_right"), defaults.boundary_threshold_right);
  node.declare_parameter(param_name(prefix, "accel_cmd_coeff"), defaults.accel_cmd_coeff);
  node.declare_parameter(param_name(prefix, "steer_cmd_coeff"), defaults.steer_cmd_coeff);
  node.declare_parameter(param_name(prefix, "max_velocity"), defaults.max_velocity);
  node.declare_parameter(param_name(prefix, "max_lon_accel"), defaults.max_lon_accel);
  node.declare_parameter(param_name(prefix, "min_lon_accel"), defaults.min_lon_accel);
  node.declare_parameter(param_name(prefix, "max_lon_jerk"), defaults.max_lon_jerk);
  node.declare_parameter(param_name(prefix, "max_lat_accel"), defaults.max_lat_accel);
  node.declare_parameter(param_name(prefix, "max_lat_jerk"), defaults.max_lat_jerk);
  node.declare_parameter(param_name(prefix, "overspeed_coeff"), defaults.overspeed_coeff);
  node.declare_parameter(
    param_name(prefix, "longitudinal_acceleration_coeff"),
    defaults.longitudinal_acceleration_coeff);
  node.declare_parameter(param_name(prefix, "steer_rate_coeff"), defaults.steer_rate_coeff);
  node.declare_parameter(
    param_name(prefix, "lateral_acceleration_coeff"), defaults.lateral_acceleration_coeff);
  node.declare_parameter(param_name(prefix, "lateral_jerk_coeff"), defaults.lateral_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "longitudinal_jerk_coeff"), defaults.longitudinal_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "longitudinal_recovery_coeff"), defaults.longitudinal_recovery_coeff);
  node.declare_parameter(
    param_name(prefix, "longitudinal_recovery_time_constant"),
    defaults.longitudinal_recovery_time_constant);
  node.declare_parameter(
    param_name(prefix, "obstacle_collision_margin"), defaults.obstacle_collision_margin);
  node.declare_parameter(
    param_name(prefix, "road_border_collision_margin"), defaults.road_border_collision_margin);
  node.declare_parameter(
    param_name(prefix, "drivable_area_crossing_coeff"), defaults.drivable_area_crossing_coeff);
}

FirstOrderDubinsMppiCostParams get_first_order_dubins_mppi_cost_params(
  const rclcpp::Node & node, const std::string & prefix)
{
  FirstOrderDubinsMppiCostParams params;
  params.lambda = static_cast<float>(node.get_parameter(param_name(prefix, "lambda")).as_double());
  params.desired_speed =
    static_cast<float>(node.get_parameter(param_name(prefix, "desired_speed")).as_double());
  params.speed_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "speed_coeff")).as_double());
  params.track_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "track_coeff")).as_double());
  params.track_terminal_scale =
    static_cast<float>(node.get_parameter(param_name(prefix, "track_terminal_scale")).as_double());
  params.heading_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "heading_coeff")).as_double());
  params.lateral_distance_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "lateral_distance_coeff")).as_double());
  params.lateral_yaw_error_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "lateral_yaw_error_coeff")).as_double());
  params.crash_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "crash_coeff")).as_double());
  params.boundary_threshold =
    static_cast<float>(node.get_parameter(param_name(prefix, "boundary_threshold")).as_double());
  params.boundary_threshold_left = static_cast<float>(
    node.get_parameter(param_name(prefix, "boundary_threshold_left")).as_double());
  params.boundary_threshold_right = static_cast<float>(
    node.get_parameter(param_name(prefix, "boundary_threshold_right")).as_double());
  params.accel_cmd_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "accel_cmd_coeff")).as_double());
  params.steer_cmd_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "steer_cmd_coeff")).as_double());
  params.max_velocity =
    static_cast<float>(node.get_parameter(param_name(prefix, "max_velocity")).as_double());
  params.max_lon_accel =
    static_cast<float>(node.get_parameter(param_name(prefix, "max_lon_accel")).as_double());
  params.min_lon_accel =
    static_cast<float>(node.get_parameter(param_name(prefix, "min_lon_accel")).as_double());
  params.max_lon_jerk =
    static_cast<float>(node.get_parameter(param_name(prefix, "max_lon_jerk")).as_double());
  params.max_lat_accel =
    static_cast<float>(node.get_parameter(param_name(prefix, "max_lat_accel")).as_double());
  params.max_lat_jerk =
    static_cast<float>(node.get_parameter(param_name(prefix, "max_lat_jerk")).as_double());
  params.overspeed_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "overspeed_coeff")).as_double());
  params.longitudinal_acceleration_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "longitudinal_acceleration_coeff")).as_double());
  params.steer_rate_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "steer_rate_coeff")).as_double());
  params.lateral_acceleration_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "lateral_acceleration_coeff")).as_double());
  params.lateral_jerk_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "lateral_jerk_coeff")).as_double());
  params.longitudinal_jerk_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "longitudinal_jerk_coeff")).as_double());
  params.longitudinal_recovery_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "longitudinal_recovery_coeff")).as_double());
  params.longitudinal_recovery_time_constant = static_cast<float>(
    node.get_parameter(param_name(prefix, "longitudinal_recovery_time_constant")).as_double());
  params.obstacle_collision_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "obstacle_collision_margin")).as_double());
  params.road_border_collision_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "road_border_collision_margin")).as_double());
  params.drivable_area_crossing_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "drivable_area_crossing_coeff")).as_double());
  return params;
}

bool update_first_order_dubins_mppi_kinematic_cost_params(
  FirstOrderDubinsMppiCostParams & params, const std::vector<rclcpp::Parameter> & updates,
  const std::string & prefix)
{
  bool changed = false;
  for (const auto & update : updates) {
    const auto set_float = [&](const char * name, float & value) {
      if (update.get_name() == param_name(prefix, name)) {
        value = static_cast<float>(update.as_double());
        changed = true;
      }
    };
    set_float("max_velocity", params.max_velocity);
    set_float("max_lon_accel", params.max_lon_accel);
    set_float("min_lon_accel", params.min_lon_accel);
    set_float("max_lon_jerk", params.max_lon_jerk);
    set_float("max_lat_accel", params.max_lat_accel);
    set_float("max_lat_jerk", params.max_lat_jerk);
    set_float("overspeed_coeff", params.overspeed_coeff);
    set_float("longitudinal_acceleration_coeff", params.longitudinal_acceleration_coeff);
    set_float("longitudinal_jerk_coeff", params.longitudinal_jerk_coeff);
    set_float("lateral_acceleration_coeff", params.lateral_acceleration_coeff);
    set_float("lateral_jerk_coeff", params.lateral_jerk_coeff);
    set_float("longitudinal_recovery_coeff", params.longitudinal_recovery_coeff);
    set_float("longitudinal_recovery_time_constant", params.longitudinal_recovery_time_constant);
  }
  return changed;
}

}  // namespace autoware::mppi_optimizer
