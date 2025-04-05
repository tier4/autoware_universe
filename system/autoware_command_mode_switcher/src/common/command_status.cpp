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

#include "command_status.hpp"

#include <tier4_system_msgs/msg/command_mode_status_item.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

std::string to_string(const SourceState & state)
{
  // clang-format off
  switch (state) {
    case SourceState::Disabled:   return "D";
    case SourceState::Transition: return "T";
    case SourceState::Enabled:    return "E";
    default:                      return "?";
  }
  // clang-format on
}

std::string to_string(const SourceGroup & group)
{
  // clang-format off
  switch (group) {
    case SourceGroup::Shared:    return "D";
    case SourceGroup::Exclusive: return "E";
    default:                     return "?";
  }
  // clang-format on
}

std::string to_string(const TransitionState & state)
{
  // clang-format off
  switch (state) {
    case TransitionState::Transition: return "D";
    case TransitionState::Completed:  return "E";
    default:                          return "?";
  }
  // clang-format on
}

std::string to_string(bool state)
{
  return state ? "E" : "D";
}

std::string convert_debug_string(const CommandStatus & status)
{
  std::string result;
  result += to_string(status.source_state);
  result += to_string(status.source_group);
  result += to_string(status.control_gate_selected);
  result += to_string(status.vehicle_gate_selected);
  result += to_string(status.transition_state);
  return result;
}

uint8_t convert_main_state(const MainState & state)
{
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  // clang-format off
  switch (state) {
    case MainState::Disabled:   return Message::DISABLED;
    case MainState::Transition: return Message::TRANSITION;
    case MainState::Enabled:    return Message::ENABLED;
    default:                    return Message::UNDEFINED;
  }
  // clang-format on
}

uint8_t convert_mrm_state(const MrmState & mrm)
{
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  // clang-format off
  switch (mrm) {
    case MrmState::Normal:     return Message::NORMAL;
    case MrmState::Operating:  return Message::OPERATING;
    case MrmState::Succeeded:  return Message::SUCCEEDED;
    case MrmState::Failed:     return Message::FAILED;
    default:                   return Message::UNDEFINED;
  }
  // clang-format on
}

MainState update_main_state(const CommandStatus & status)
{
  if (status.transition_state == TransitionState::Transition) {
    return MainState::Transition;
  }
  bool is_enabled = true;
  is_enabled &= status.source_state == SourceState::Enabled;
  is_enabled &= status.source_group == SourceGroup::Exclusive;
  is_enabled &= status.control_gate_selected;
  is_enabled &= status.vehicle_gate_selected;
  is_enabled &= status.transition_state == TransitionState::Completed;
  return is_enabled ? MainState::Enabled : MainState::Disabled;
}

}  // namespace autoware::command_mode_switcher
