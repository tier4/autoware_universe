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

#include "sub_ecu_in_lane_stop_0_3g.hpp"

namespace autoware::command_mode_switcher
{

void SubEcuInLaneStop03GSwitcher::initialize()
{
  pub_trigger_ =
    node_->create_publisher<JerkConstantDecelerationTrigger>(
      "/control/jerk_constant_deceleration_trigger", rclcpp::QoS{1});
  sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { odom_.emplace(*msg); });

  mrm_state_ = MrmState::Normal;
}

TriState SubEcuInLaneStop03GSwitcher::update_source_state(bool request)
{
  if (request && mrm_state_ == MrmState::Operating) return TriState::Enabled;
  if (request && mrm_state_ == MrmState::Succeeded) return TriState::Enabled;
  if (!request && mrm_state_ == MrmState::Normal) return TriState::Disabled;

  if (request) {
    publish_jerk_constant_deceleration_trigger(request);
    mrm_state_ = MrmState::Operating;
    return TriState::Enabled;
  } else {
    publish_jerk_constant_deceleration_trigger(request);
    mrm_state_ = MrmState::Normal;
    return TriState::Disabled;
  }
}

MrmState SubEcuInLaneStop03GSwitcher::update_mrm_state()
{
  if (mrm_state_ != MrmState::Operating) {
    return mrm_state_;
  }

  if (is_stopped()) mrm_state_ = MrmState::Succeeded;
  return mrm_state_;
}

void SubEcuInLaneStop03GSwitcher::publish_jerk_constant_deceleration_trigger(bool turn_on)
{
  auto trigger = JerkConstantDecelerationTrigger();
  trigger.stamp = node_->now();
  trigger.trigger = turn_on;
  trigger.target_acceleration = -3.0;
  trigger.target_jerk = -5.0;

  pub_trigger_->publish(trigger);
}

bool SubEcuInLaneStop03GSwitcher::is_stopped()
{
  if (!odom_.has_value()) return false;
  constexpr auto th_stopped_velocity = 0.001;
  return (std::abs(odom_->twist.twist.linear.x) < th_stopped_velocity);
}

}  // namespace autoware::command_mode_switcher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::command_mode_switcher::SubEcuInLaneStop03GSwitcher,
  autoware::command_mode_switcher::CommandPlugin)
