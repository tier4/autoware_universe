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

#ifndef AUTOWARE_COMMAND_MODE_TYPES__ADAPTERS__COMMAND_MODE_STATUS_HPP_
#define AUTOWARE_COMMAND_MODE_TYPES__ADAPTERS__COMMAND_MODE_STATUS_HPP_

#include "autoware_command_mode_types/types/command_mode_status.hpp"

#include <rclcpp/type_adapter.hpp>

#include <tier4_system_msgs/msg/command_mode_status.hpp>

template <>
struct rclcpp::TypeAdapter<
  autoware::command_mode_types::CommandModeStatus, tier4_system_msgs::msg::CommandModeStatus>
{
  using is_specialized = std::true_type;
  using custom_type = autoware::command_mode_types::CommandModeStatus;
  using ros_message_type = tier4_system_msgs::msg::CommandModeStatus;
  static void convert_to_ros_message(const custom_type & custom, ros_message_type & ros);
  static void convert_to_custom(const ros_message_type & ros, custom_type & custom);
};

namespace autoware::command_mode_types
{

using CommandModeStatusAdapter = rclcpp::TypeAdapter<
  autoware::command_mode_types::CommandModeStatus, tier4_system_msgs::msg::CommandModeStatus>;

tier4_system_msgs::msg::CommandModeStatus to_msg(const CommandModeStatus & status);
tier4_system_msgs::msg::CommandModeStatusItem to_msg(const CommandModeStatusItem & item);
uint8_t to_tri_state(const TriState & state);
uint8_t to_mrm_state(const MrmState & state);
uint8_t to_gate_type(const GateType & type);

CommandModeStatus from_msg(const tier4_system_msgs::msg::CommandModeStatus & status);
CommandModeStatusItem from_msg(const tier4_system_msgs::msg::CommandModeStatusItem & item);
TriState from_tri_state(const uint8_t msg);
MrmState from_mrm_state(const uint8_t msg);
GateType from_gate_type(const uint8_t msg);

}  // namespace autoware::command_mode_types

#endif  // AUTOWARE_COMMAND_MODE_TYPES__ADAPTERS__COMMAND_MODE_STATUS_HPP_
