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
  node.declare_parameter(param_name(prefix, "ignore_obstacles"), defaults.ignore_obstacles);
  node.declare_parameter(param_name(prefix, "ignore_drivable_area"), defaults.ignore_drivable_area);
  node.declare_parameter(
    param_name(prefix, "force_cold_start_each_step"), defaults.force_cold_start_each_step);
  node.declare_parameter(param_name(prefix, "skip_if_invalid"), defaults.skip_if_invalid);
  node.declare_parameter(
    param_name(prefix, "use_last_control_as_nominal"), defaults.use_last_control_as_nominal);
}

FirstOrderDubinsMppiRuntimeOptions get_first_order_dubins_mppi_runtime_options(
  const rclcpp::Node & node, const std::string & prefix)
{
  FirstOrderDubinsMppiRuntimeOptions options;
  options.enable_debug_trajectory_log =
    node.get_parameter(param_name(prefix, "enable_debug_trajectory_log")).as_bool();
  options.debug_trajectory_log_directory =
    node.get_parameter(param_name(prefix, "debug_trajectory_log_directory")).as_string();
  options.ignore_obstacles = node.get_parameter(param_name(prefix, "ignore_obstacles")).as_bool();
  options.ignore_drivable_area =
    node.get_parameter(param_name(prefix, "ignore_drivable_area")).as_bool();
  options.force_cold_start_each_step =
    node.get_parameter(param_name(prefix, "force_cold_start_each_step")).as_bool();
  options.skip_if_invalid = node.get_parameter(param_name(prefix, "skip_if_invalid")).as_bool();
  options.use_last_control_as_nominal =
    node.get_parameter(param_name(prefix, "use_last_control_as_nominal")).as_bool();
  return options;
}

}  // namespace autoware::mppi_optimizer
