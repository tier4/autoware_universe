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

uint8_t convert_tri_state(const TriState & state)
{
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  // clang-format off
  switch (state) {
    case TriState::Disabled:   return Message::DISABLED;
    case TriState::Enabled:    return Message::ENABLED;
    case TriState::Transition: return Message::TRANSITION;
    default:                   return Message::UNDEFINED;
  }
  // clang-format on
}

uint8_t convert_mrm_state(const MrmState & state)
{
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  // clang-format off
  switch (state) {
    case MrmState::Normal:     return Message::NORMAL;
    case MrmState::Operating:  return Message::OPERATING;
    case MrmState::Succeeded:  return Message::SUCCEEDED;
    case MrmState::Failed:     return Message::FAILED;
    default:                   return Message::UNDEFINED;
  }
  // clang-format on
}

uint8_t convert_request_stage(const RequestStage & stage)
{
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  // clang-format off
  switch (stage) {
    case RequestStage::NoRequest:   return Message::NO_REQUEST;
    case RequestStage::CommandMode: return Message::COMMAND_MODE;
    case RequestStage::VehicleGate: return Message::VEHICLE_GATE;
    case RequestStage::NetworkGate: return Message::NETWORK_GATE;
    case RequestStage::ControlGate: return Message::CONTROL_GATE;
    default:                        return Message::UNDEFINED;
  }
  // clang-format on
}

}  // namespace autoware::command_mode_types
