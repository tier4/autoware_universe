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

#include "command_mode_conversion.hpp"

#include <autoware_command_mode_types/constants/modes.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_system_msgs/srv/change_operation_mode.hpp>

#include <string>

namespace autoware::command_mode_decider
{

namespace modes = autoware::command_mode_types::modes;
using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_system_msgs::srv::ChangeOperationMode;

uint16_t operation_mode_to_command(uint32_t operation_mode)
{
  // clang-format off
  switch (operation_mode) {
    case ChangeOperationMode::Request::STOP:       return modes::stop;
    case ChangeOperationMode::Request::AUTONOMOUS: return modes::autonomous;
    case ChangeOperationMode::Request::LOCAL:      return modes::local;
    case ChangeOperationMode::Request::REMOTE:     return modes::remote;
    default:                                       return modes::unknown;
  }
  // clang-format on
}

uint32_t command_to_operation_mode(uint16_t command_mode)
{
  // clang-format off
  if (command_mode == modes::stop)       return OperationModeState::STOP;
  if (command_mode == modes::autonomous) return OperationModeState::AUTONOMOUS;
  if (command_mode == modes::local)      return OperationModeState::LOCAL;
  if (command_mode == modes::remote)     return OperationModeState::REMOTE;
  // clang-format on
  return OperationModeState::UNKNOWN;
}

uint32_t command_to_mrm_behavior(uint16_t command_mode)
{
  // clang-format off
  if (command_mode == modes::emergency_stop)   return MrmState::EMERGENCY_STOP;
  if (command_mode == modes::comfortable_stop) return MrmState::COMFORTABLE_STOP;
  if (command_mode == modes::pull_over)        return MrmState::PULL_OVER;
  // clang-format on
  return MrmState::NONE;
}

}  // namespace autoware::command_mode_decider
