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

#include "autoware/trajectory_processor/trajectory_modifier_plugins/external_velocity_limit.hpp"

#include "autoware/trajectory_processor/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <memory>
#include <optional>

namespace autoware::trajectory_processor::plugin
{

namespace detail
{
double get_external_velocity_limit_deceleration(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit,
  const double nominal_deceleration)
{
  return velocity_limit.use_constraints
           ? std::abs(static_cast<double>(velocity_limit.constraints.min_acceleration))
           : std::abs(nominal_deceleration);
}

double get_external_velocity_limit_min_jerk(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit,
  const double nominal_jerk)
{
  return velocity_limit.use_constraints
           ? std::abs(static_cast<double>(velocity_limit.constraints.min_jerk))
           : std::abs(nominal_jerk);
}
}  // namespace detail

void ExternalVelocityLimit::on_initialize(const TrajectoryProcessorParams & params)
{
  update_params(params);
  velocity_limit_sub_ =
    make_polling_subscriber<VelocityLimit>("~/input/external_velocity_limit_mps", rclcpp::QoS{1});
}

void ExternalVelocityLimit::update_params(const TrajectoryProcessorParams & params)
{
  enabled_ = params.use_external_velocity_limit;
  nominal_deceleration_ = params.stopping_constraints.nominal_deceleration;
  nominal_jerk_ = params.stopping_constraints.jerk_limit;
}

ProcessingResult ExternalVelocityLimit::process(
  TrajectoryPoints & traj_points, TrajectoryProcessorData & input)
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

  const double deceleration =
    detail::get_external_velocity_limit_deceleration(*velocity_limit, nominal_deceleration_);
  const double max_velocity = velocity_limit->max_velocity;
  detail::VelocityLimitOptions options;
  options.current_ego_velocity = input.current_odometry->twist.twist.linear.x;
  options.current_ego_acceleration = input.current_acceleration->accel.accel.linear.x;
  const double min_jerk =
    detail::get_external_velocity_limit_min_jerk(*velocity_limit, nominal_jerk_);
  const auto result = detail::apply_velocity_limits(
    traj_points, deceleration, min_jerk,
    [max_velocity](const geometry_msgs::msg::Point &) {
      return std::optional<double>{max_velocity};
    },
    options);
  return result.status;
}

}  // namespace autoware::trajectory_processor::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_processor::plugin::ExternalVelocityLimit,
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase)
