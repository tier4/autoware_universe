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

#include <optional>
#include <string>

namespace autoware::mppi_optimizer
{
namespace
{

constexpr const char * kDiffusionPlannerVehicleModel =
  "DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER";
constexpr const char * kDiffusionPlannerVersionParam =
  "delay_steer_acc_geared_for_diffusion_planner.version";

void declare_if_missing(rclcpp::Node & node, const std::string & name, const double value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, value);
  }
}

void declare_if_missing(rclcpp::Node & node, const std::string & name, const int value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, value);
  }
}

void declare_if_missing(rclcpp::Node & node, const std::string & name, const std::string & value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter(name, value);
  }
}

double read_double(rclcpp::Node & node, const std::string & name, const double default_value)
{
  declare_if_missing(node, name, default_value);
  return node.get_parameter(name).as_double();
}

/**
 * When vehicle_model_type is DELAY_STEER_ACC_GEARED_FOR_DIFFUSION_PLANNER, actuator taus/delays
 * live under delay_steer_acc_geared_for_diffusion_planner.v{version}.* (same as
 * simple_planning_simulator). Flat top-level fields are used for other vehicle_model_type values.
 */
std::optional<std::string> diffusion_planner_actuator_prefix(rclcpp::Node & node)
{
  declare_if_missing(node, "vehicle_model_type", std::string{});
  if (node.get_parameter("vehicle_model_type").as_string() != kDiffusionPlannerVehicleModel) {
    return std::nullopt;
  }

  declare_if_missing(node, kDiffusionPlannerVersionParam, 1);
  const int version = node.get_parameter(kDiffusionPlannerVersionParam).as_int();
  return "delay_steer_acc_geared_for_diffusion_planner.v" + std::to_string(version) + ".";
}

float read_actuator_scalar(
  rclcpp::Node & node, const std::string & flat_name,
  const std::optional<std::string> & nested_prefix, const float default_value)
{
  if (nested_prefix) {
    const std::string nested_name = *nested_prefix + flat_name;
    if (node.has_parameter(nested_name)) {
      return static_cast<float>(node.get_parameter(nested_name).as_double());
    }
  }
  return static_cast<float>(read_double(node, flat_name, static_cast<double>(default_value)));
}

}  // namespace

void declare_first_order_dubins_mppi_vehicle_dynamics_params(rclcpp::Node & node)
{
  const FirstOrderDubinsMppiVehicleParams defaults;
  declare_if_missing(node, "vehicle_model_type", std::string{});
  declare_if_missing(node, kDiffusionPlannerVersionParam, 1);
  declare_if_missing(node, "acc_time_constant", static_cast<double>(defaults.acc_time_constant));
  declare_if_missing(
    node, "steer_time_constant", static_cast<double>(defaults.steer_time_constant));
  declare_if_missing(node, "steer_rate_lim", static_cast<double>(defaults.steer_rate_lim));
  declare_if_missing(node, "vel_rate_lim", static_cast<double>(defaults.vel_rate_lim));
  declare_if_missing(node, "acc_time_delay", static_cast<double>(defaults.acc_time_delay));
  declare_if_missing(node, "steer_time_delay", static_cast<double>(defaults.steer_time_delay));

  if (const auto prefix = diffusion_planner_actuator_prefix(node)) {
    declare_if_missing(node, *prefix + "acc_time_constant", 0.1);
    declare_if_missing(node, *prefix + "steer_time_constant", 0.27);
    declare_if_missing(node, *prefix + "acc_time_delay", 0.1);
    declare_if_missing(node, *prefix + "steer_time_delay", 0.24);
  }
}

FirstOrderDubinsMppiVehicleParams get_first_order_dubins_mppi_vehicle_params(rclcpp::Node & node)
{
  const FirstOrderDubinsMppiVehicleParams defaults;
  FirstOrderDubinsMppiVehicleParams params =
    makeVehicleParams(autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo());

  const auto nested_prefix = diffusion_planner_actuator_prefix(node);
  params.acc_time_constant =
    read_actuator_scalar(node, "acc_time_constant", nested_prefix, defaults.acc_time_constant);
  params.steer_time_constant =
    read_actuator_scalar(node, "steer_time_constant", nested_prefix, defaults.steer_time_constant);
  params.acc_time_delay =
    read_actuator_scalar(node, "acc_time_delay", nested_prefix, defaults.acc_time_delay);
  params.steer_time_delay =
    read_actuator_scalar(node, "steer_time_delay", nested_prefix, defaults.steer_time_delay);

  // Rate limits are shared top-level simulator_model fields for all vehicle models.
  params.steer_rate_lim =
    static_cast<float>(read_double(node, "steer_rate_lim", defaults.steer_rate_lim));
  params.vel_rate_lim =
    static_cast<float>(read_double(node, "vel_rate_lim", defaults.vel_rate_lim));
  return params;
}

}  // namespace autoware::mppi_optimizer
