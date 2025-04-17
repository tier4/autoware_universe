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

#ifndef AUTOWARE_COMMAND_MODE_SWITCHER__COMMON__COMMAND_STATUS_HPP_
#define AUTOWARE_COMMAND_MODE_SWITCHER__COMMON__COMMAND_STATUS_HPP_

#include <autoware_command_mode_types/types/command_mode_status.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

using CommandStatus = command_mode_types::CommandModeStatusItem;
using command_mode_types::GateType;
using command_mode_types::MrmState;
using command_mode_types::TriState;

TriState update_mode_state(const CommandStatus & status);
TriState update_gate_state(const CommandStatus & status);
GateType update_current_phase(const CommandStatus & status);

}  // namespace autoware::command_mode_switcher

#endif  // AUTOWARE_COMMAND_MODE_SWITCHER__COMMON__COMMAND_STATUS_HPP_
