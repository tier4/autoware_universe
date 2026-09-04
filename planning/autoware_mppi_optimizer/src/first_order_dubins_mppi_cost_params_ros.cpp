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
  node.declare_parameter(param_name(prefix, "speed_coeff"), defaults.speed_coeff);
  node.declare_parameter(param_name(prefix, "track_coeff"), defaults.track_coeff);
  node.declare_parameter(param_name(prefix, "track_terminal_scale"), defaults.track_terminal_scale);
  node.declare_parameter(param_name(prefix, "heading_coeff"), defaults.heading_coeff);
  node.declare_parameter(
    param_name(prefix, "lateral_distance_coeff"), defaults.lateral_distance_coeff);
  node.declare_parameter(
    param_name(prefix, "lateral_yaw_error_coeff"), defaults.lateral_yaw_error_coeff);
  node.declare_parameter(
    param_name(prefix, "remaining_distance_coeff"), defaults.remaining_distance_coeff);
  node.declare_parameter(param_name(prefix, "path_overshoot_coeff"), defaults.path_overshoot_coeff);
  node.declare_parameter(param_name(prefix, "track_center_coeff"), defaults.track_center_coeff);
  node.declare_parameter(param_name(prefix, "corner_buffer_coeff"), defaults.corner_buffer_coeff);
  node.declare_parameter(param_name(prefix, "corner_safe_margin"), defaults.corner_safe_margin);
  node.declare_parameter(param_name(prefix, "boundary_threshold"), defaults.boundary_threshold);
  node.declare_parameter(
    param_name(prefix, "lateral_boundary_soft_margin"), defaults.lateral_boundary_soft_margin);
  node.declare_parameter(param_name(prefix, "accel_cmd_coeff"), defaults.accel_cmd_coeff);
  node.declare_parameter(param_name(prefix, "steer_cmd_coeff"), defaults.steer_cmd_coeff);
  node.declare_parameter(param_name(prefix, "steer_rate_coeff"), defaults.steer_rate_coeff);
  node.declare_parameter(param_name(prefix, "overlimit_coeff"), defaults.overlimit_coeff);
  node.declare_parameter(param_name(prefix, "accel_cmd_std_dev"), defaults.accel_cmd_std_dev);
  node.declare_parameter(param_name(prefix, "steer_cmd_std_dev"), defaults.steer_cmd_std_dev);
  node.declare_parameter(
    param_name(prefix, "accel_cmd_noise_exponent"), defaults.accel_cmd_noise_exponent);
  node.declare_parameter(
    param_name(prefix, "steer_cmd_noise_exponent"), defaults.steer_cmd_noise_exponent);
  node.declare_parameter(
    param_name(prefix, "nominal_curvature_min_chord_length_m"),
    defaults.nominal_curvature_min_chord_length_m);
  node.declare_parameter(
    param_name(prefix, "lateral_acceleration_coeff"), defaults.lateral_acceleration_coeff);
  node.declare_parameter(param_name(prefix, "lateral_jerk_coeff"), defaults.lateral_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "longitudinal_jerk_coeff"), defaults.longitudinal_jerk_coeff);
  node.declare_parameter(
    param_name(prefix, "obstacle_collision_margin"), defaults.obstacle_collision_margin);
  node.declare_parameter(
    param_name(prefix, "road_border_collision_margin"), defaults.road_border_collision_margin);
  node.declare_parameter(param_name(prefix, "obstacle_safe_margin"), defaults.obstacle_safe_margin);
  node.declare_parameter(
    param_name(prefix, "road_border_safe_margin"), defaults.road_border_safe_margin);
  node.declare_parameter(
    param_name(prefix, "drivable_area_safe_margin"), defaults.drivable_area_safe_margin);
  node.declare_parameter(
    param_name(prefix, "drivable_area_barrier_weight"), defaults.drivable_area_barrier_weight);
  node.declare_parameter(
    param_name(prefix, "crash_contact_penalty"), defaults.crash_contact_penalty);
}

FirstOrderDubinsMppiCostParams get_first_order_dubins_mppi_cost_params(
  const rclcpp::Node & node, const std::string & prefix)
{
  FirstOrderDubinsMppiCostParams params;
  params.lambda = static_cast<float>(node.get_parameter(param_name(prefix, "lambda")).as_double());
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
  params.remaining_distance_coeff = static_cast<float>(
    node.get_parameter(param_name(prefix, "remaining_distance_coeff")).as_double());
  params.path_overshoot_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "path_overshoot_coeff")).as_double());
  params.track_center_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "track_center_coeff")).as_double());
  params.corner_buffer_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "corner_buffer_coeff")).as_double());
  params.corner_safe_margin =
    static_cast<float>(node.get_parameter(param_name(prefix, "corner_safe_margin")).as_double());
  params.boundary_threshold =
    static_cast<float>(node.get_parameter(param_name(prefix, "boundary_threshold")).as_double());
  params.lateral_boundary_soft_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "lateral_boundary_soft_margin")).as_double());
  params.accel_cmd_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "accel_cmd_coeff")).as_double());
  params.steer_cmd_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "steer_cmd_coeff")).as_double());
  params.steer_rate_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "steer_rate_coeff")).as_double());
  params.overlimit_coeff =
    static_cast<float>(node.get_parameter(param_name(prefix, "overlimit_coeff")).as_double());
  params.accel_cmd_std_dev =
    static_cast<float>(node.get_parameter(param_name(prefix, "accel_cmd_std_dev")).as_double());
  params.steer_cmd_std_dev =
    static_cast<float>(node.get_parameter(param_name(prefix, "steer_cmd_std_dev")).as_double());
  params.accel_cmd_noise_exponent = static_cast<float>(
    node.get_parameter(param_name(prefix, "accel_cmd_noise_exponent")).as_double());
  params.steer_cmd_noise_exponent = static_cast<float>(
    node.get_parameter(param_name(prefix, "steer_cmd_noise_exponent")).as_double());
  params.nominal_curvature_min_chord_length_m = static_cast<float>(
    node.get_parameter(param_name(prefix, "nominal_curvature_min_chord_length_m")).as_double());
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
  params.obstacle_safe_margin =
    static_cast<float>(node.get_parameter(param_name(prefix, "obstacle_safe_margin")).as_double());
  params.road_border_safe_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "road_border_safe_margin")).as_double());
  params.drivable_area_safe_margin = static_cast<float>(
    node.get_parameter(param_name(prefix, "drivable_area_safe_margin")).as_double());
  params.drivable_area_barrier_weight = static_cast<float>(
    node.get_parameter(param_name(prefix, "drivable_area_barrier_weight")).as_double());
  params.crash_contact_penalty =
    static_cast<float>(node.get_parameter(param_name(prefix, "crash_contact_penalty")).as_double());
  return params;
}

}  // namespace autoware::mppi_optimizer
