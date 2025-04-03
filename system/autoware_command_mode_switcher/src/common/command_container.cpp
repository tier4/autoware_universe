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

#include "command_container.hpp"

namespace autoware::command_mode_switcher
{

Command::Command(std::shared_ptr<CommandPlugin> plugin) : plugin(plugin)
{
  status.source_state = SourceState::Disabled;
  status.source_group = SourceGroup::Shared;
  status.control_gate_state = ControlGateState::Unselected;
  status.vehicle_gate_state = VehicleGateState::Unselected;
  status.vehicle_gate_request = false;
  status.control_gate_request = false;
  status.mode_continuable = false;
  status.mode_available = false;
  status.control_gate_ready = false;
  status.vehicle_gate_ready = false;
  status.transition_completed = false;
}

void Command::update_status()
{
  status.control_gate_ready = plugin->get_control_gate_ready();
  status.vehicle_gate_ready = plugin->get_vehicle_gate_ready();
}

}  // namespace autoware::command_mode_switcher
