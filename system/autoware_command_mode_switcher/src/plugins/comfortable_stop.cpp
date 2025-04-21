//  Copyright 2025 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "comfortable_stop.hpp"

namespace autoware::command_mode_switcher
{

void ComfortableStopSwitcher::initialize()
{
  pub_velocity_limit_ = node_->create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "/planning/scenario_planning/max_velocity_candidates", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_clear_command_ =
    node_->create_publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>(
      "/planning/scenario_planning/clear_velocity_limit", rclcpp::QoS{1}.transient_local());
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_.emplace(*msg); });

  mrm_state_ = MrmState::Normal;
}

TriState ComfortableStopSwitcher::update_source_state(bool request)
{
  if (request && mrm_state_ == MrmState::Operating) return TriState::Enabled;
  if (request && mrm_state_ == MrmState::Succeeded) return TriState::Enabled;
  if (!request && mrm_state_ == MrmState::Normal) return TriState::Disabled;

  if (request) {
    publish_velocity_limit();
    mrm_state_ = MrmState::Operating;
    return TriState::Enabled;
  } else {
    publish_velocity_limit_clear_command();
    mrm_state_ = MrmState::Normal;
    return TriState::Disabled;
  }
}

MrmState ComfortableStopSwitcher::update_mrm_state()
{
  if (mrm_state_ != MrmState::Operating) {
    return mrm_state_;
  }

  if (is_stopped()) mrm_state_ = MrmState::Succeeded;
  return mrm_state_;
}

void ComfortableStopSwitcher::publish_velocity_limit()
{
  auto velocity_limit = tier4_planning_msgs::msg::VelocityLimit();
  velocity_limit.stamp = node_->now();
  velocity_limit.max_velocity = 0;
  velocity_limit.use_constraints = true;
  velocity_limit.constraints.min_acceleration = static_cast<float>(-1.0);
  velocity_limit.constraints.max_jerk = static_cast<float>(0.6);
  velocity_limit.constraints.min_jerk = static_cast<float>(-0.6);
  velocity_limit.sender = node_->get_name();

  pub_velocity_limit_->publish(velocity_limit);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Comfortable stop is requested.");
}

void ComfortableStopSwitcher::publish_velocity_limit_clear_command()
{
  auto velocity_limit_clear_command = tier4_planning_msgs::msg::VelocityLimitClearCommand();
  velocity_limit_clear_command.stamp = node_->now();
  velocity_limit_clear_command.command = true;
  velocity_limit_clear_command.sender = node_->get_name();

  pub_velocity_limit_clear_command_->publish(velocity_limit_clear_command);
  RCLCPP_INFO_STREAM(node_->get_logger(), "Comfortable stop is canceled.");
}

bool ComfortableStopSwitcher::is_stopped()
{
  if (!odom_.has_value()) return false;
  constexpr auto th_stopped_velocity = 0.001;
  return (std::abs(odom_->twist.twist.linear.x) < th_stopped_velocity);
}

}  // namespace autoware::command_mode_switcher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::command_mode_switcher::ComfortableStopSwitcher,
  autoware::command_mode_switcher::CommandPlugin)
