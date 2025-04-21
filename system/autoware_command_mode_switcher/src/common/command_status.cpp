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

#include "autoware_command_mode_switcher/common/command_status.hpp"

#include <tier4_system_msgs/msg/command_mode_status_item.hpp>

namespace autoware::command_mode_switcher
{

TriState merge_state(const TriState & s1, const TriState & s2)
{
  if (s1 == TriState::Disabled && s2 == TriState::Disabled) return TriState::Disabled;
  if (s1 == TriState::Enabled && s2 == TriState::Enabled) return TriState::Enabled;
  return TriState::Transition;
};

TriState update_mode_state(const CommandStatus & status)
{
  return merge_state(status.gate_state, status.transition_state);
}

TriState update_gate_state(const CommandStatus & status)
{
  const bool is_vehicle_gate = status.request_phase == GateType::VehicleGate;
  const bool is_network_gate = status.request_phase == GateType::NetworkGate;
  const bool is_control_gate = status.request_phase == GateType::ControlGate;
  TriState state = status.source_state;
  if (is_vehicle_gate || is_network_gate || is_control_gate) {
    state = merge_state(state, status.source_group);
    state = merge_state(state, status.control_gate_state);
  }
  if (is_vehicle_gate || is_network_gate) {
    state = merge_state(state, status.network_gate_state);
  }
  if (is_vehicle_gate) {
    state = merge_state(state, status.vehicle_gate_state);
  }
  return state;
}

GateType update_current_phase(const CommandStatus & status)
{
  TriState state = TriState::Enabled;
  state = merge_state(state, status.source_state);
  state = merge_state(state, status.source_group);
  state = merge_state(state, status.control_gate_state);
  if (state != TriState::Enabled) return GateType::NotSelected;
  state = merge_state(state, status.network_gate_state);
  if (state != TriState::Enabled) return GateType::ControlGate;
  state = merge_state(state, status.vehicle_gate_state);
  if (state != TriState::Enabled) return GateType::NetworkGate;
  return GateType::VehicleGate;
}

}  // namespace autoware::command_mode_switcher
