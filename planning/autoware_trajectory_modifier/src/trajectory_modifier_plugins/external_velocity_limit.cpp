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
#include <string>

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
  return velocity_limit.use_constraints && std::isfinite(deceleration) && deceleration > 0.0
           ? deceleration
           : std::abs(default_deceleration);
}

double get_external_velocity_limit_min_jerk(
  const autoware_internal_planning_msgs::msg::VelocityLimit & velocity_limit,
  const double default_jerk)
{
  const double jerk = std::abs(velocity_limit.constraints.min_jerk);
  return velocity_limit.use_constraints && std::isfinite(jerk) && jerk > 0.0
           ? jerk
           : std::abs(default_jerk);
}
}  // namespace detail

void ExternalVelocityLimit::on_initialize(const TrajectoryModifierParams & params)
{
  update_params(params);
  profile_reset_pub_ =
    get_node_ptr()->create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
      "~/debug/velocity_profile_reset", rclcpp::QoS{10}.reliable().durability_volatile());
  debug_trajectory_pub_ = get_node_ptr()->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/external_velocity_limit/trajectory", rclcpp::QoS{1});
}

void ExternalVelocityLimit::update_params(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_external_velocity_limit;
  constraints_ = detail::make_velocity_limit_constraints(params);
  fixed_profile_.set_parameters(detail::make_fixed_profile_parameters(params));
  if (!enabled_) {
    fixed_profile_.clear();
  }
}

ProcessingResult ExternalVelocityLimit::process(
  TrajectoryPoints & traj_points, TrajectoryModifierData & input)
{
  last_profile_start_ = detail::ProfileStart::Measured;
  if (!input.has_external_velocity_limit_snapshot) {
    context_->refresh_external_velocity_limit();
  }
  const auto clear_revision = input.has_external_velocity_limit_snapshot
                                ? input.external_velocity_limit_clear_revision
                                : context_->external_velocity_limit.clear_revision();
  if (last_clear_revision_ != clear_revision) {
    fixed_profile_.clear();
    last_clear_revision_ = clear_revision;
  }
  if (!enabled_ || traj_points.empty() || !input.current_odometry) {
    publish_selected_debug_trajectory(debug_trajectory_pub_, traj_points, input);
    return ProcessingResult::Unchanged;
  }

  auto stop_state = detail::make_fixed_profile_stop_state(traj_points);
  const auto velocity_limit = input.has_external_velocity_limit_snapshot
                                ? input.external_velocity_limit
                                : context_->external_velocity_limit.active_limit();
  if (!velocity_limit) {
    fixed_profile_.clear();
    publish_selected_debug_trajectory(debug_trajectory_pub_, traj_points, input);
    return ProcessingResult::Unchanged;
  }

  auto constraints = constraints_;
  constraints.max_deceleration = detail::get_external_velocity_limit_deceleration(
    *velocity_limit, constraints_.max_deceleration);
  constraints.max_jerk =
    detail::get_external_velocity_limit_min_jerk(*velocity_limit, constraints_.max_jerk);
  const double max_velocity = velocity_limit->max_velocity;
  stop_state.zero_speed_limit = max_velocity <= detail::k_fixed_profile_stop_velocity_threshold_mps;
  const auto start = fixed_profile_.start(input, stop_state, traj_points);
  last_profile_start_ = start.start;
  if (detail::is_reset(start.start)) {
    RCLCPP_DEBUG(
      get_node_ptr()->get_logger(), "%s: candidate %zu restarted from the measured ego state (%s)",
      get_short_name().c_str(), input.candidate_index, detail::to_string(start.start));
    autoware_internal_debug_msgs::msg::StringStamped reset;
    reset.stamp = get_clock()->now();
    reset.data = "plugin=" + get_name() + " candidate=" + std::to_string(input.candidate_index) +
                 " reason=" + detail::to_string(start.start);
    profile_reset_pub_->publish(reset);
  }
  const auto result = detail::apply_velocity_limits(
    traj_points, constraints,
    [max_velocity](const geometry_msgs::msg::Point &) {
      return std::optional<double>{max_velocity};
    },
    start.options);
  if (!result.error.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000, "%s: candidate %zu: %s",
      get_short_name().c_str(), input.candidate_index, result.error.c_str());
  }
  // An unchanged result with a diagnostic rejected the proposed profile. Forget the previous
  // plan so the next cycle starts from the measured state instead of reusing an invalid one.
  if (result.status == ProcessingResult::Unchanged && !result.error.empty()) {
    fixed_profile_.store(input, TrajectoryPoints{}, stop_state);
  } else {
    fixed_profile_.store(input, traj_points, stop_state);
  }
  publish_selected_debug_trajectory(debug_trajectory_pub_, traj_points, input);
  return result.status;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ExternalVelocityLimit,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
