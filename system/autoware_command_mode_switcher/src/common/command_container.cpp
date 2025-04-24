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
  status.mode = plugin->mode_name();

  status.mode_state = TriState::Disabled;
  status.gate_state = TriState::Disabled;
  status.mrm = MrmState::Normal;
  status.request_phase = GateType::NotSelected;
  status.current_phase = GateType::NotSelected;

  status.mode_continuable = false;
  status.mode_available = false;
  status.transition_available = false;
  status.transition_completed = false;

  status.transition_state = TriState::Disabled;
  status.vehicle_gate_state = TriState::Disabled;
  status.network_gate_state = TriState::Enabled;
  status.control_gate_state = TriState::Disabled;
  status.source_state = TriState::Disabled;
  status.source_group = TriState::Disabled;
}

}  // namespace autoware::command_mode_switcher
