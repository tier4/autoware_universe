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

#ifndef PLUGINS__COMFORTABLE_STOP_HPP_
#define PLUGINS__COMFORTABLE_STOP_HPP_

#include "autoware_command_mode_switcher/common/command_plugin.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>

#include <optional>
#include <string>

namespace autoware::command_mode_switcher
{

class ComfortableStopSwitcher : public ControlCommandPlugin
{
public:
  std::string mode_name() const override { return "comfortable_stop"; }
  std::string source_name() const override { return "main"; }
  bool autoware_control() const override { return true; }
  void initialize() override;

  TriState update_source_state(bool request) override;
  MrmState update_mrm_state() override;

  bool get_transition_available() override { return true; }
  bool get_transition_completed() override { return true; }

private:
  void publish_velocity_limit();
  void publish_velocity_limit_clear_command();
  bool is_stopped();

  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    pub_velocity_limit_clear_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  std::optional<nav_msgs::msg::Odometry> odom_;
  MrmState mrm_state_;
};

}  // namespace autoware::command_mode_switcher

#endif  // PLUGINS__COMFORTABLE_STOP_HPP_
