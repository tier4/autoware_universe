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

#ifndef COMMON__COMMAND_STATUS_HPP_
#define COMMON__COMMAND_STATUS_HPP_

#include <string>

namespace autoware::command_mode_switcher
{

enum class SourceState {
  Disabled,
  Transition,
  Enabled,
};

enum class SourceGroup {
  Shared,
  Exclusive,
};

enum class ControlGateState {
  Unselected,
  Requesting,
  Selected,
};

enum class VehicleGateState {
  Unselected,
  Requesting,
  Selected,
};

struct CommandStatus
{
  SourceState source_state;
  SourceGroup source_group;
  ControlGateState control_gate_state;
  VehicleGateState vehicle_gate_state;
  bool vehicle_gate_request;
  bool control_gate_request;
  bool mode_continuable;
  bool mode_available;
  bool control_gate_ready;
  bool vehicle_gate_ready;
  bool transition_completed;
};

std::string convert_debug_string(const CommandStatus & status);

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__COMMAND_STATUS_HPP_
