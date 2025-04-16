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

#ifndef PLUGINS__AUTONOMOUS_HPP_
#define PLUGINS__AUTONOMOUS_HPP_

#include "autoware_command_mode_switcher/common/command_plugin.hpp"

#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

class AutonomousSwitcher : public ControlCommandPlugin
{
public:
  std::string mode_name() const override { return "autonomous"; }
  std::string source_name() const override { return "main"; }
  bool autoware_control() const override { return true; }
  void initialize() override;

  bool get_transition_available() override { return transition_available_; }
  bool get_transition_completed() override { return transition_completed_; }

private:
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_transition_available_;
  rclcpp::Subscription<ModeChangeAvailable>::SharedPtr sub_transition_completed_;

  bool transition_available_ = false;
  bool transition_completed_ = false;
};

}  // namespace autoware::command_mode_switcher

#endif  // PLUGINS__AUTONOMOUS_HPP_
