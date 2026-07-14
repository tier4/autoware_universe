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

#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp"

#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_conversion.hpp"

#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>

#include <string>

namespace autoware::mppi_optimizer
{
namespace
{
constexpr const char * k_diffusion_sysid_prefix = "delay_steer_acc_geared_for_diffusion_planner";

float get_float_param(rclcpp::Node & node, const std::string & name, const float fallback)
{
  if (!node.has_parameter(name)) {
    return fallback;
  }
  return static_cast<float>(node.get_parameter(name).as_double());
}

void override_from_diffusion_planner_sysid(rclcpp::Node & node, FirstOrderDubinsMppiVehicleParams & params)
{
  const std::string version_key = std::string(k_diffusion_sysid_prefix) + ".version";
  if (!node.has_parameter(version_key)) {
    return;
  }

  const int version = node.get_parameter(version_key).as_int();
  const std::string ns =
    std::string(k_diffusion_sysid_prefix) + ".v" + std::to_string(version) + ".";

  params.acc_time_constant = get_float_param(node, ns + "acc_time_constant", params.acc_time_constant);
  params.steer_time_constant =
    get_float_param(node, ns + "steer_time_constant", params.steer_time_constant);
  params.acc_time_delay = get_float_param(node, ns + "acc_time_delay", params.acc_time_delay);
  params.steer_time_delay = get_float_param(node, ns + "steer_time_delay", params.steer_time_delay);
  // Note: v1 also has steer_rate_lim, but the simulator constructor uses the top-level
  // steer_rate_lim for this model — keep that value for plant matching.
}
}  // namespace

void declare_first_order_dubins_mppi_vehicle_dynamics_params(rclcpp::Node & node)
{
  const FirstOrderDubinsMppiVehicleParams defaults;
  node.declare_parameter("acc_time_constant", defaults.acc_time_constant);
  node.declare_parameter("steer_time_constant", defaults.steer_time_constant);
  node.declare_parameter("steer_rate_lim", defaults.steer_rate_lim);
  node.declare_parameter("vel_rate_lim", defaults.vel_rate_lim);
  node.declare_parameter("acc_time_delay", defaults.acc_time_delay);
  node.declare_parameter("steer_time_delay", defaults.steer_time_delay);
}

FirstOrderDubinsMppiVehicleParams get_first_order_dubins_mppi_vehicle_params(rclcpp::Node & node)
{
  FirstOrderDubinsMppiVehicleParams params =
    makeVehicleParams(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo());
  params.acc_time_constant =
    static_cast<float>(node.get_parameter("acc_time_constant").as_double());
  params.steer_time_constant =
    static_cast<float>(node.get_parameter("steer_time_constant").as_double());
  params.steer_rate_lim = static_cast<float>(node.get_parameter("steer_rate_lim").as_double());
  params.vel_rate_lim = static_cast<float>(node.get_parameter("vel_rate_lim").as_double());
  params.acc_time_delay = static_cast<float>(node.get_parameter("acc_time_delay").as_double());
  params.steer_time_delay = static_cast<float>(node.get_parameter("steer_time_delay").as_double());

  // Prefer the identified DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER plant when present
  // (loaded via simulator_model.param.yaml into the diffusion_planner node).
  override_from_diffusion_planner_sysid(node, params);
  return params;
}

}  // namespace autoware::mppi_optimizer
