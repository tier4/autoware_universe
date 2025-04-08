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

#include <string>

namespace autoware::command_mode_types
{

CommandModeStatusItem::CommandModeStatusItem(const std::string & mode) : mode(mode)
{
  mode_state = TriState::Disabled;
  gate_state = TriState::Disabled;
  mrm = MrmState::Normal;
  request_phase = GateType::NotSelected;
  current_phase = GateType::NotSelected;

  mode_continuable = false;
  mode_available = false;
  transition_available = false;
  transition_completed = false;

  transition_state = TriState::Disabled;
  vehicle_gate_state = TriState::Disabled;
  network_gate_state = TriState::Disabled;
  control_gate_state = TriState::Disabled;
  source_state = TriState::Disabled;
  source_group = TriState::Disabled;
}

TriState merge_state(const TriState & s1, const TriState & s2)
{
  if (s1 == TriState::Disabled && s2 == TriState::Disabled) return TriState::Disabled;
  if (s1 == TriState::Enabled && s2 == TriState::Enabled) return TriState::Enabled;
  return TriState::Transition;
};

bool CommandModeStatusItem::check_mode_ready() const
{
  return check_gate_ready(GateType::VehicleGate) && (transition_state == TriState::Enabled);
}

bool CommandModeStatusItem::check_gate_ready(GateType gate) const
{
  TriState state = TriState::Enabled;
  state = merge_state(state, source_state);
  state = merge_state(state, source_group);
  state = merge_state(state, control_gate_state);
  if (gate == GateType::ControlGate) return state == TriState::Enabled;
  state = merge_state(state, network_gate_state);
  if (gate == GateType::NetworkGate) return state == TriState::Enabled;
  state = merge_state(state, vehicle_gate_state);
  if (gate == GateType::VehicleGate) return state == TriState::Enabled;
  return false;  // For invalid gate type.
}

}  // namespace autoware::command_mode_types
