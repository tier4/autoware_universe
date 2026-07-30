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

#include "autoware/mppi_optimizer/mppi_optimizer.hpp"

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params_ros.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options_ros.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{
FirstOrderDubinsMppiKinematicLimits toKinematicLimits(const VelocityLimit & limit)
{
  FirstOrderDubinsMppiKinematicLimits result;
  if (std::isfinite(limit.max_velocity) && limit.max_velocity >= 0.0F) {
    result.max_velocity.push_back(limit.max_velocity);
  }
  if (!limit.use_constraints) {
    return result;
  }

  if (
    std::isfinite(limit.constraints.max_acceleration) &&
    limit.constraints.max_acceleration >= 0.0F) {
    result.max_lon_accel.push_back(limit.constraints.max_acceleration);
  }
  if (
    std::isfinite(limit.constraints.min_acceleration) &&
    limit.constraints.min_acceleration <= 0.0F) {
    result.min_lon_accel.push_back(limit.constraints.min_acceleration);
  }

  float symmetric_jerk_limit = std::numeric_limits<float>::infinity();
  if (std::isfinite(limit.constraints.max_jerk) && limit.constraints.max_jerk >= 0.0F) {
    symmetric_jerk_limit = std::min(symmetric_jerk_limit, limit.constraints.max_jerk);
  }
  if (std::isfinite(limit.constraints.min_jerk) && limit.constraints.min_jerk <= 0.0F) {
    symmetric_jerk_limit = std::min(symmetric_jerk_limit, -limit.constraints.min_jerk);
  }
  if (std::isfinite(symmetric_jerk_limit)) {
    result.max_lon_jerk.push_back(symmetric_jerk_limit);
  }
  return result;
}
}  // namespace

MppiOptimizer::MppiOptimizer(const rclcpp::NodeOptions & options) : Node("mppi_optimizer", options)
{
  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&MppiOptimizer::on_trajectory, this, std::placeholders::_1));
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  mppi_interface_ = std::make_unique<FirstOrderDubinsMppiInterface>();
  declare_first_order_dubins_mppi_cost_params(*this);
  declare_first_order_dubins_mppi_vehicle_dynamics_params(*this);
  declare_first_order_dubins_mppi_runtime_options(*this);
  mppi_interface_->setCostParams(get_first_order_dubins_mppi_cost_params(*this));
  mppi_interface_->setVehicleParams(get_first_order_dubins_mppi_vehicle_params(*this));
  mppi_interface_->setRuntimeOptions(get_first_order_dubins_mppi_runtime_options(*this));
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&MppiOptimizer::on_parameter, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult MppiOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  auto cost_params = get_first_order_dubins_mppi_cost_params(*this);
  if (!update_first_order_dubins_mppi_kinematic_cost_params(cost_params, parameters)) {
    return result;
  }
  try {
    mppi_interface_->setCostParams(cost_params);
  } catch (const std::invalid_argument & error) {
    result.successful = false;
    result.reason = error.what();
  }
  return result;
}

void MppiOptimizer::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  if (!msg || msg->points.empty()) {
    return;
  }

  const auto odometry = sub_odometry_.take_data();
  if (!odometry) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }

  const auto steering_status = sub_steering_.take_data();
  const std::optional<SteeringReport> ego_steering =
    steering_status ? std::make_optional(*steering_status) : std::nullopt;
  const auto external_velocity_limit = sub_external_velocity_limit_.take_data();
  const FirstOrderDubinsMppiKinematicLimits dynamic_limits =
    external_velocity_limit ? toKinematicLimits(*external_velocity_limit)
                            : FirstOrderDubinsMppiKinematicLimits{};

  try {
    const autoware_perception_msgs::msg::TrackedObjects empty_tracked_objects;
    const std::vector<Segment> empty_segments;
    const auto result = mppi_interface_->optimizeTrajectory(
      *msg, *odometry, std::nullopt, ego_steering, empty_tracked_objects, empty_segments,
      empty_segments, dynamic_limits);
    Trajectory tracked = result.trajectory;
    tracked.header = msg->header;
    trajectory_pub_->publish(tracked);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "MPPI tracking failed: " << e.what());
    trajectory_pub_->publish(*msg);
  }
}

}  // namespace autoware::mppi_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mppi_optimizer::MppiOptimizer)
