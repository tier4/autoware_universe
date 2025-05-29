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

#include "autonomous.hpp"

namespace autoware::command_mode_switcher
{

void AutonomousSwitcher::initialize()
{
  sub_transition_available_ = node_->create_subscription<ModeChangeAvailable>(
    "~/command_mode/transition/available", rclcpp::QoS(1),
    [this](const ModeChangeAvailable & msg) { transition_available_ = msg.available; });
  sub_transition_completed_ = node_->create_subscription<ModeChangeAvailable>(
    "~/command_mode/transition/completed", rclcpp::QoS(1),
    [this](const ModeChangeAvailable & msg) { transition_completed_ = msg.available; });
}

}  // namespace autoware::command_mode_switcher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::command_mode_switcher::AutonomousSwitcher,
  autoware::command_mode_switcher::CommandPlugin)
