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

namespace autoware::command_mode_types
{

CommandModeStatusItem::CommandModeStatusItem(uint16_t mode) : mode(mode)
{
  transition = false;
  request = false;
  vehicle_selected = false;
  command_selected = false;
  command_exclusive = false;
  command_enabled = false;
  command_disabled = false;

  mode_state = TriState::Disabled;
  gate_state = TriState::Disabled;
  mrm = MrmState::Normal;
  request_phase = GateType::NotSelected;
  current_phase = GateType::NotSelected;

  continuable = false;
  available = false;
  drivable = false;
  transition_completed = false;

  transition_state = TriState::Disabled;
  vehicle_gate_state = TriState::Disabled;
  network_gate_state = TriState::Disabled;
  control_gate_state = TriState::Disabled;
  source_state = TriState::Disabled;
  source_group = TriState::Disabled;
}

bool CommandModeStatusItem::is_completed() const
{
  return !transition && is_vehicle_ready();
}

bool CommandModeStatusItem::is_vehicle_ready() const
{
  return vehicle_selected && is_command_ready();
}

bool CommandModeStatusItem::is_command_ready() const
{
  return command_selected && command_exclusive;
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
