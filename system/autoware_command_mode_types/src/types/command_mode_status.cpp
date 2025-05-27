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
  mrm = MrmState::Normal;
  transition_completed = false;
  transition = false;
  request = false;
  vehicle_selected = false;
  command_selected = false;
  command_exclusive = false;
  command_enabled = false;
  command_disabled = false;
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

}  // namespace autoware::command_mode_types
