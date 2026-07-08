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
// See the License for the License governing permissions and
// limitations under the License.

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface_params_ros.hpp"

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

void declare_first_order_dubins_mppi_interface_params(
  rclcpp::Node & node, const std::string & prefix)
{
  const FirstOrderDubinsMppiInterfaceParams defaults;
  node.declare_parameter(
    param_name(prefix, "always_seed_nominal_from_diffusion_actuation"),
    defaults.always_seed_nominal_from_diffusion_actuation);
}

FirstOrderDubinsMppiInterfaceParams get_first_order_dubins_mppi_interface_params(
  const rclcpp::Node & node, const std::string & prefix)
{
  FirstOrderDubinsMppiInterfaceParams params;
  params.always_seed_nominal_from_diffusion_actuation = node.get_parameter(
                                                 param_name(
                                                   prefix,
                                                   "always_seed_nominal_from_diffusion_actuation"))
                                                 .as_bool();
  return params;
}

}  // namespace autoware::mppi_optimizer
