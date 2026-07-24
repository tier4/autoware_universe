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

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <string>

namespace autoware::mppi_optimizer
{

void configure_first_order_dubins_mppi_runtime_options(
  rclcpp::Node & node, FirstOrderDubinsMppiInterface & optimizer)
{
  const bool enable_debug_log =
    node.declare_parameter<bool>("enable_debug_trajectory_log", false);
  // Empty -> $XDG_CACHE_HOME/autoware/mppi_debug_log or $HOME/.cache/autoware/mppi_debug_log
  const std::string debug_log_directory =
    node.declare_parameter<std::string>("debug_trajectory_log_directory", "");
  optimizer.setDebugTrajectoryLogging(enable_debug_log, debug_log_directory);

  optimizer.setAblationOptions(
    node.declare_parameter<bool>("ignore_obstacles", false),
    node.declare_parameter<bool>("ignore_drivable_area", false),
    node.declare_parameter<bool>("force_cold_start_each_step", false));
}

}  // namespace autoware::mppi_optimizer
