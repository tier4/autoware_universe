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

#include "autoware_command_mode_types/adapters/command_mode_status.hpp"

using TargetAdapter = autoware::command_mode_types::CommandModeStatusAdapter;

void TargetAdapter::convert_to_ros_message(const custom_type & custom, ros_message_type & ros)
{
  ros = autoware::command_mode_types::to_msg(custom);
}

void TargetAdapter::convert_to_custom(const ros_message_type & ros, custom_type & custom)
{
  custom = autoware::command_mode_types::from_msg(ros);
}

namespace autoware::command_mode_types
{

tier4_system_msgs::msg::CommandModeStatus to_msg(const CommandModeStatus & status)
{
  tier4_system_msgs::msg::CommandModeStatus msg;
  msg.stamp = status.stamp;
  for (const auto & item : status.items) {
    msg.items.push_back(to_msg(item));
  }
  return msg;
}

CommandModeStatus from_msg(const tier4_system_msgs::msg::CommandModeStatus & status)
{
  CommandModeStatus custom;
  custom.stamp = status.stamp;
  for (const auto & item : status.items) {
    custom.items.push_back(from_msg(item));
  }
  return custom;
}

tier4_system_msgs::msg::CommandModeStatusItem to_msg(const CommandModeStatusItem & item)
{
  tier4_system_msgs::msg::CommandModeStatusItem msg;
  msg.mode = item.mode;
  msg.mrm = to_mrm_state(item.mrm);
  msg.transition = item.transition;
  msg.transition_completed = item.transition_completed;
  msg.request = item.request;
  msg.vehicle_selected = item.vehicle_selected;
  msg.network_selected = item.network_selected;
  msg.command_selected = item.command_selected;
  msg.command_exclusive = item.command_exclusive;
  msg.command_enabled = item.command_enabled;
  msg.command_disabled = item.command_disabled;
  return msg;
}

CommandModeStatusItem from_msg(const tier4_system_msgs::msg::CommandModeStatusItem & item)
{
  CommandModeStatusItem custom;
  custom.mode = item.mode;
  custom.mrm = from_mrm_state(item.mrm);
  custom.transition = item.transition;
  custom.transition_completed = item.transition_completed;
  custom.request = item.request;
  custom.vehicle_selected = item.vehicle_selected;
  custom.network_selected = item.network_selected;
  custom.command_selected = item.command_selected;
  custom.command_exclusive = item.command_exclusive;
  custom.command_enabled = item.command_enabled;
  custom.command_disabled = item.command_disabled;
  return custom;
}

uint8_t to_mrm_state(const MrmState & state)
{
  // clang-format off
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  switch (state) {
    case MrmState::Normal:     return Message::NORMAL;
    case MrmState::Operating:  return Message::OPERATING;
    case MrmState::Succeeded:  return Message::SUCCEEDED;
    case MrmState::Failed:     return Message::FAILED;
    default:                   return Message::UNDEFINED;
  }
  // clang-format on
}

MrmState from_mrm_state(const uint8_t msg)
{
  // clang-format off
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  switch (msg) {
    case Message::NORMAL:     return MrmState::Normal;
    case Message::OPERATING:  return MrmState::Operating;
    case Message::SUCCEEDED:  return MrmState::Succeeded;
    case Message::FAILED:     return MrmState::Failed;
    default:                  return MrmState::Failed;
  }
  // clang-format on
}

}  // namespace autoware::command_mode_types
