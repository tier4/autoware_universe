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
    case SourceGroup::Shared:    return "S";
    case SourceGroup::Exclusive: return "E";
    default:                     return "?";
  }
  // clang-format on
}

std::string to_string(const ControlGateState & state)
{
  // clang-format off
  switch (state) {
    case ControlGateState::Unselected: return "U";
    case ControlGateState::Requesting: return "R";
    case ControlGateState::Selected:   return "S";
    default:                           return "?";
  }
  // clang-format on
}

std::string to_string(const VehicleGateState & state)
{
  // clang-format off
  switch (state) {
    case VehicleGateState::Unselected: return "U";
    case VehicleGateState::Requesting: return "R";
    case VehicleGateState::Selected:   return "S";
    default:                           return "?";
  }
  // clang-format on
}

std::string convert_debug_string(const CommandStatus & status)
{
  std::string result;
  result += to_string(status.source_state);
  result += to_string(status.source_group);
  result += to_string(status.control_gate_state);
  result += to_string(status.vehicle_gate_state);
  return result;
}

}  // namespace autoware::command_mode_switcher
