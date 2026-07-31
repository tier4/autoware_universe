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
  node.declare_parameter(param_name(prefix, "steer_rate_coeff"), defaults.steer_rate_coeff);
  node.declare_parameter(
    param_name(prefix, "lateral_acceleration_coeff"), defaults.lateral_acceleration_coeff);
  node.declare_parameter(param_name(prefix, "lateral_jerk_coeff"), defaults.lateral_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "longitudinal_jerk_coeff"), defaults.longitudinal_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "obstacle_collision_margin"), defaults.obstacle_collision_margin);
  node.declare_parameter(
    param_name(prefix, "road_border_collision_margin"), defaults.road_border_collision_margin);
  node.declare_parameter(
    param_name(prefix, "drivable_area_crossing_coeff"), defaults.drivable_area_crossing_coeff);
  node.declare_parameter(param_name(prefix, "track_center_coeff"), defaults.track_center_coeff);
  node.declare_parameter(param_name(prefix, "corner_safe_margin"), defaults.corner_safe_margin);
  node.declare_parameter(param_name(prefix, "corner_buffer_coeff"), defaults.corner_buffer_coeff);
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
  params.steer_rate_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "steer_rate_coeff")).as_double());
  params.lateral_acceleration_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "lateral_acceleration_coeff")).as_double());
  params.lateral_jerk_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "lateral_jerk_coeff")).as_double());
  params.longitudinal_jerk_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "longitudinal_jerk_coeff")).as_double());
  params.obstacle_collision_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "obstacle_collision_margin")).as_double());
  params.road_border_collision_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "road_border_collision_margin")).as_double());
  params.drivable_area_crossing_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "drivable_area_crossing_coeff")).as_double());
  params.track_center_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "track_center_coeff")).as_double());
  params.corner_safe_margin =
    static_cast<float>(node.get_parameter(param_name(prefix, "corner_safe_margin")).as_double());
  params.corner_buffer_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "corner_buffer_coeff")).as_double());
  return params;
}

bool update_first_order_dubins_mppi_cost_params(
  const std::vector<rclcpp::Parameter> & parameters, FirstOrderDubinsMppiCostParams & params,
  const std::string & prefix)
{
  bool updated = false;
  const auto update = [&](const std::string & name, float & value) {
    const auto full_name = param_name(prefix, name);
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == full_name) {
        value = static_cast<float>(parameter.as_double());
        updated = true;
        return;
      }
    }
  };

  update("lambda", params.lambda);
  update("desired_speed", params.desired_speed);
  update("speed_coeff", params.speed_coeff);
  update("track_coeff", params.track_coeff);
  update("track_terminal_scale", params.track_terminal_scale);
  update("heading_coeff", params.heading_coeff);
  update("lateral_distance_coeff", params.lateral_distance_coeff);
  update("lateral_yaw_error_coeff", params.lateral_yaw_error_coeff);
  update("crash_coeff", params.crash_coeff);
  update("boundary_threshold", params.boundary_threshold);
  update("boundary_threshold_left", params.boundary_threshold_left);
  update("boundary_threshold_right", params.boundary_threshold_right);
  update("accel_cmd_coeff", params.accel_cmd_coeff);
  update("steer_cmd_coeff", params.steer_cmd_coeff);
  update("steer_rate_coeff", params.steer_rate_coeff);
  update("lateral_acceleration_coeff", params.lateral_acceleration_coeff);
  update("lateral_jerk_coeff", params.lateral_jerk_coeff);
  update("longitudinal_jerk_coeff", params.longitudinal_jerk_coeff);
  update("obstacle_collision_margin", params.obstacle_collision_margin);
  update("road_border_collision_margin", params.road_border_collision_margin);
  update("drivable_area_crossing_coeff", params.drivable_area_crossing_coeff);
  update("track_center_coeff", params.track_center_coeff);
  update("corner_safe_margin", params.corner_safe_margin);
  update("corner_buffer_coeff", params.corner_buffer_coeff);
  return updated;
}

}  // namespace autoware::mppi_optimizer
