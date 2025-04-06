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

namespace autoware::command_mode_switcher
{

TriState to_tri_state(bool state)
{
  return state ? TriState::Enabled : TriState::Disabled;
}

TriState merge_state(const TriState & s1, const TriState & s2)
{
  if (s1 == TriState::Disabled && s2 == TriState::Disabled) return TriState::Disabled;
  if (s1 == TriState::Enabled && s2 == TriState::Enabled) return TriState::Enabled;
  return TriState::Transition;
};

TriState update_main_state(const CommandStatus & status)
{
  TriState state = merge_state(status.command_mode_state, status.source_state);
  switch (status.request_phase) {
    case RequestPhase::CommandMode:
    case RequestPhase::VehicleGate:
      state = merge_state(state, status.vehicle_gate_state);  // fall-through
    case RequestPhase::NetworkGate:
      state = merge_state(state, status.network_gate_state);  // fall-through
    case RequestPhase::ControlGate:
      state = merge_state(state, status.source_group);
      state = merge_state(state, status.control_gate_state);
      break;
    case RequestPhase::NoRequest:
      break;
  }
  return state;
}

RequestPhase update_current_phase(const CommandStatus & status)
{
  TriState state = TriState::Enabled;
  state = merge_state(state, status.source_group);
  state = merge_state(state, status.source_state);
  state = merge_state(state, status.control_gate_state);
  if (state != TriState::Enabled) return RequestPhase::NoRequest;
  state = merge_state(state, status.network_gate_state);
  if (state != TriState::Enabled) return RequestPhase::ControlGate;
  state = merge_state(state, status.vehicle_gate_state);
  if (state != TriState::Enabled) return RequestPhase::NetworkGate;
  state = merge_state(state, status.command_mode_state);
  if (state != TriState::Enabled) return RequestPhase::VehicleGate;
  return RequestPhase::CommandMode;
}

}  // namespace autoware::command_mode_switcher
