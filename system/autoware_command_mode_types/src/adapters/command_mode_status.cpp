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
  msg.state = to_tri_state(item.state);
  msg.mrm = to_mrm_state(item.mrm);
  msg.transition_state = to_tri_state(item.transition_state);
  msg.gate_state = to_tri_state(item.gate_state);
  msg.request_phase = to_request_phase(item.request_phase);
  msg.current_phase = to_request_phase(item.current_phase);
  msg.vehicle_gate_state = to_tri_state(item.vehicle_gate_state);
  msg.network_gate_state = to_tri_state(item.network_gate_state);
  msg.control_gate_state = to_tri_state(item.control_gate_state);
  msg.source_state = to_tri_state(item.source_state);
  msg.source_group = to_tri_state(item.source_group);
  msg.mode_continuable = item.mode_continuable;
  msg.mode_available = item.mode_available;
  msg.transition_available = item.transition_available;
  msg.transition_completed = item.transition_completed;
  return msg;
}

CommandModeStatusItem from_msg(const tier4_system_msgs::msg::CommandModeStatusItem & item)
{
  CommandModeStatusItem custom;
  custom.mode = item.mode;

  custom.state = from_tri_state(item.state);
  custom.mrm = from_mrm_state(item.mrm);
  custom.transition_state = from_tri_state(item.transition_state);
  custom.gate_state = from_tri_state(item.gate_state);
  custom.request_phase = from_request_phase(item.request_phase);
  custom.current_phase = from_request_phase(item.current_phase);

  custom.mode_continuable = item.mode_continuable;
  custom.mode_available = item.mode_available;
  custom.transition_available = item.transition_available;
  custom.transition_completed = item.transition_completed;

  custom.vehicle_gate_state = from_tri_state(item.vehicle_gate_state);
  custom.network_gate_state = from_tri_state(item.network_gate_state);
  custom.control_gate_state = from_tri_state(item.control_gate_state);
  custom.source_state = from_tri_state(item.source_state);
  custom.source_group = from_tri_state(item.source_group);

  return custom;
}

uint8_t to_tri_state(const TriState & state)
{
  // clang-format off
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  switch (state) {
    case TriState::Disabled:   return Message::DISABLED;
    case TriState::Enabled:    return Message::ENABLED;
    case TriState::Transition: return Message::TRANSITION;
    default:                   return Message::UNDEFINED;
  }
  // clang-format on
}

TriState from_tri_state(const uint8_t msg)
{
  // clang-format off
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  switch (msg) {
    case Message::DISABLED:   return TriState::Disabled;
    case Message::ENABLED:    return TriState::Enabled;
    case Message::TRANSITION: return TriState::Transition;
    default:                  return TriState::Transition;
  }
  // clang-format on
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

uint8_t to_request_phase(const RequestPhase & phase)
{
  // clang-format off
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  switch (phase) {
    case RequestPhase::NotSelected: return Message::NOT_SELECTED;
    case RequestPhase::ControlGate: return Message::CONTROL_GATE;
    case RequestPhase::NetworkGate: return Message::NETWORK_GATE;
    case RequestPhase::VehicleGate: return Message::VEHICLE_GATE;
    default:                        return Message::UNDEFINED;
  }
  // clang-format on
}

RequestPhase from_request_phase(const uint8_t msg)
{
  // clang-format off
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  switch (msg) {
    case Message::NOT_SELECTED: return RequestPhase::NotSelected;
    case Message::CONTROL_GATE: return RequestPhase::ControlGate;
    case Message::NETWORK_GATE: return RequestPhase::NetworkGate;
    case Message::VEHICLE_GATE: return RequestPhase::VehicleGate;
    default:                    return RequestPhase::NotSelected;
  }
  // clang-format on
}
}  // namespace autoware::command_mode_types
