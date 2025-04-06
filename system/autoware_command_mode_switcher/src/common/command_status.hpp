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

#include <autoware_command_mode_types/types/command_mode_status.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

using command_mode_types::MrmState;
using command_mode_types::RequestPhase;
using command_mode_types::TriState;

struct CommandStatus
{
  TriState state;
  MrmState mrm;
  RequestPhase request_phase;
  RequestPhase current_phase;

  TriState command_mode_state;
  TriState vehicle_gate_state;
  TriState network_gate_state;
  TriState control_gate_state;
  TriState source_state;
  TriState source_group;

  bool mode_continuable;
  bool mode_available;
  bool transition_available;
  bool transition_completed;
};

// For internal use.
TriState to_tri_state(bool state);
TriState update_main_state(const CommandStatus & status);
RequestPhase update_current_phase(const CommandStatus & status);

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__COMMAND_STATUS_HPP_
