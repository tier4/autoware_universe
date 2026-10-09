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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/external_velocity_limit.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <optional>

namespace autoware::trajectory_modifier::plugin
{

namespace detail
{
// A zero constraint would make braking impossible, so it falls back to the default as well.
double get_external_velocity_limit_deceleration(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit,
  const double default_deceleration)
{
  const double deceleration = std::abs(velocity_limit.constraints.min_acceleration);
  return velocity_limit.use_constraints && deceleration > 0.0 ? deceleration
                                                              : std::abs(default_deceleration);
}

double get_external_velocity_limit_min_jerk(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit,
  const double default_jerk)
{
  const double jerk = std::abs(velocity_limit.constraints.min_jerk);
  return velocity_limit.use_constraints && jerk > 0.0 ? jerk : std::abs(default_jerk);
}
}  // namespace detail

void ExternalVelocityLimit::on_initialize(const TrajectoryModifierParams & params)
{
  update_params(params);
  velocity_limit_sub_ =
    std::make_shared<autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit>>(
      get_node_ptr(), "~/input/external_velocity_limit_mps", rclcpp::QoS{1});
}

void ExternalVelocityLimit::update_params(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_external_velocity_limit;
  constraints_.max_acceleration = params.velocity_limits.max_acceleration;
  constraints_.max_deceleration = params.velocity_limits.max_deceleration;
  constraints_.max_jerk = params.velocity_limits.max_jerk;
}

ProcessingResult ExternalVelocityLimit::process(
  TrajectoryPoints & traj_points, TrajectoryModifierData & input)
{
  if (!enabled_ || traj_points.empty() || !input.current_odometry) {
    return ProcessingResult::Unchanged;
  }

  const auto velocity_limit = velocity_limit_sub_->take_data();
  if (
    !velocity_limit || !std::isfinite(velocity_limit->max_velocity) ||
    velocity_limit->max_velocity < 0.0F) {
    return ProcessingResult::Unchanged;
  }

  auto constraints = constraints_;
  constraints.max_deceleration = detail::get_external_velocity_limit_deceleration(
    *velocity_limit, constraints_.max_deceleration);
  constraints.max_jerk =
    detail::get_external_velocity_limit_min_jerk(*velocity_limit, constraints_.max_jerk);
  const double max_velocity = velocity_limit->max_velocity;
  const auto result = detail::apply_velocity_limits(
    traj_points, constraints,
    [max_velocity](const geometry_msgs::msg::Point &) {
      return std::optional<double>{max_velocity};
    },
    detail::make_velocity_limit_options(input));
  return result.status;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ExternalVelocityLimit,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
