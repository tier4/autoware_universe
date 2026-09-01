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

#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options_ros.hpp"

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

void declare_first_order_dubins_mppi_runtime_options(
  rclcpp::Node & node, const std::string & prefix)
{
  const FirstOrderDubinsMppiRuntimeOptions defaults;
  node.declare_parameter(
    param_name(prefix, "enable_debug_trajectory_log"), defaults.enable_debug_trajectory_log);
  node.declare_parameter(
    param_name(prefix, "debug_trajectory_log_directory"), defaults.debug_trajectory_log_directory);
  node.declare_parameter(
    param_name(prefix, "enable_distance_map_texture_debug"),
    defaults.enable_distance_map_texture_debug);
  node.declare_parameter(
    param_name(prefix, "enable_iteration_rollout_debug"), defaults.enable_iteration_rollout_debug);
  node.declare_parameter(param_name(prefix, "ignore_obstacles"), defaults.ignore_obstacles);
  node.declare_parameter(param_name(prefix, "ignore_road_borders"), defaults.ignore_road_borders);
  node.declare_parameter(param_name(prefix, "ignore_drivable_area"), defaults.ignore_drivable_area);
  node.declare_parameter(
    param_name(prefix, "force_cold_start_each_step"), defaults.force_cold_start_each_step);
  node.declare_parameter(param_name(prefix, "skip_if_invalid"), defaults.skip_if_invalid);
  node.declare_parameter(
    param_name(prefix, "min_optimization_length"), defaults.min_optimization_length);
  node.declare_parameter(
    param_name(prefix, "use_last_control_as_nominal"), defaults.use_last_control_as_nominal);
  node.declare_parameter(
    param_name(prefix, "use_temporal_mpt_as_nominal"), defaults.use_temporal_mpt_as_nominal);
  node.declare_parameter(
    param_name(prefix, "prevent_reverse_velocity"), defaults.prevent_reverse_velocity);
  node.declare_parameter(
    param_name(prefix, "enable_curvature_adaptive_steering_filter"),
    defaults.enable_curvature_adaptive_steering_filter);
  node.declare_parameter(
    param_name(prefix, "steering_filter_alpha_straight"), defaults.steering_filter_alpha_straight);
  node.declare_parameter(
    param_name(prefix, "steering_filter_alpha_turn"), defaults.steering_filter_alpha_turn);
  node.declare_parameter(
    param_name(prefix, "steering_filter_turn_angle_rad"), defaults.steering_filter_turn_angle_rad);
  node.declare_parameter(
    param_name(prefix, "enable_input_delay_compensation"),
    defaults.enable_input_delay_compensation);
}

FirstOrderDubinsMppiRuntimeOptions get_first_order_dubins_mppi_runtime_options(
  const rclcpp::Node & node, const std::string & prefix)
{
  FirstOrderDubinsMppiRuntimeOptions options;
  options.enable_debug_trajectory_log =
    node.get_parameter(param_name(prefix, "enable_debug_trajectory_log")).as_bool();
  options.debug_trajectory_log_directory =
    node.get_parameter(param_name(prefix, "debug_trajectory_log_directory")).as_string();
  options.enable_distance_map_texture_debug =
    node.get_parameter(param_name(prefix, "enable_distance_map_texture_debug")).as_bool();
  options.enable_iteration_rollout_debug =
    node.get_parameter(param_name(prefix, "enable_iteration_rollout_debug")).as_bool();
  options.ignore_obstacles = node.get_parameter(param_name(prefix, "ignore_obstacles")).as_bool();
  options.ignore_road_borders =
    node.get_parameter(param_name(prefix, "ignore_road_borders")).as_bool();
  options.ignore_drivable_area =
    node.get_parameter(param_name(prefix, "ignore_drivable_area")).as_bool();
  options.force_cold_start_each_step =
    node.get_parameter(param_name(prefix, "force_cold_start_each_step")).as_bool();
  options.skip_if_invalid = node.get_parameter(param_name(prefix, "skip_if_invalid")).as_bool();
  options.min_optimization_length = static_cast<float>(
    node.get_parameter(param_name(prefix, "min_optimization_length")).as_double());
  options.use_last_control_as_nominal =
    node.get_parameter(param_name(prefix, "use_last_control_as_nominal")).as_bool();
  options.use_temporal_mpt_as_nominal =
    node.get_parameter(param_name(prefix, "use_temporal_mpt_as_nominal")).as_bool();
  options.prevent_reverse_velocity =
    node.get_parameter(param_name(prefix, "prevent_reverse_velocity")).as_bool();
  options.enable_curvature_adaptive_steering_filter =
    node.get_parameter(param_name(prefix, "enable_curvature_adaptive_steering_filter")).as_bool();
  options.steering_filter_alpha_straight = static_cast<float>(
    node.get_parameter(param_name(prefix, "steering_filter_alpha_straight")).as_double());
  options.steering_filter_alpha_turn = static_cast<float>(
    node.get_parameter(param_name(prefix, "steering_filter_alpha_turn")).as_double());
  options.steering_filter_turn_angle_rad = static_cast<float>(
    node.get_parameter(param_name(prefix, "steering_filter_turn_angle_rad")).as_double());
  options.enable_input_delay_compensation =
    node.get_parameter(param_name(prefix, "enable_input_delay_compensation")).as_bool();
  return options;
}

}  // namespace autoware::mppi_optimizer
