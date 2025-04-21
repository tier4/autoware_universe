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

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_system_msgs/srv/change_operation_mode.hpp>

#include <string>

namespace autoware::command_mode_decider
{

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_system_msgs::srv::ChangeOperationMode;

std::string operation_mode_to_command(uint32_t mode)
{
  // clang-format off
  switch (mode) {
    case ChangeOperationMode::Request::STOP:       return "stop";
    case ChangeOperationMode::Request::AUTONOMOUS: return "autonomous";
    case ChangeOperationMode::Request::LOCAL:      return "local";
    case ChangeOperationMode::Request::REMOTE:     return "remote";
    default:                                       return "";
  }
  // clang-format on
}

uint32_t command_to_operation_mode(const std::string & text)
{
  // clang-format off
  if (text == "stop")       return OperationModeState::STOP;
  if (text == "autonomous") return OperationModeState::AUTONOMOUS;
  if (text == "local")      return OperationModeState::LOCAL;
  if (text == "remote")     return OperationModeState::REMOTE;
  // clang-format on
  return OperationModeState::UNKNOWN;
}

uint32_t command_to_mrm_behavior(const std::string & text)
{
  // clang-format off
  if (text == "emergency_stop")   return MrmState::EMERGENCY_STOP;
  if (text == "comfortable_stop") return MrmState::COMFORTABLE_STOP;
  if (text == "pull_over")        return MrmState::PULL_OVER;

  // tmp mapping
  if (text == "main_ecu_in_lane_stop_0_3g") return MrmState::PULL_OVER;
  if (text == "main_ecu_in_lane_stop_0_6g") return MrmState::EMERGENCY_STOP;
  // clang-format on
  return MrmState::NONE;
}

}  // namespace autoware::command_mode_decider
