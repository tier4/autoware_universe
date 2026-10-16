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

#pragma once

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include "autoware/mppi_optimizer/trajectory_mppi_optimizer_param_conversion.hpp"
#include <autoware_mppi_optimizer/trajectory_mppi_optimizer_parameters.hpp>

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>
#include <string>

namespace autoware::mppi_optimizer::open_loop_sim
{

/** Load `mppi_optimizer.*` via the same generated ParamListener as the trajectory modifier. */
inline void loadMppiOptimizerParamsFromYaml(
  const std::string & params_yaml, FirstOrderDubinsMppiCostParams & cost,
  FirstOrderDubinsMppiRuntimeOptions & runtime)
{
  if (params_yaml.empty()) {
    throw std::runtime_error("params yaml path is empty");
  }
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params_yaml});
  auto node = rclcpp::Node::make_shared("mppi_open_loop_sim_params", options);
  trajectory_mppi_optimizer::ParamListener listener(node, "mppi_optimizer");
  const auto params = listener.get_params();
  cost = costParamsFromTrajectoryMppiOptimizerParams(params);
  runtime = runtimeOptionsFromTrajectoryMppiOptimizerParams(params);
}

}  // namespace autoware::mppi_optimizer::open_loop_sim
