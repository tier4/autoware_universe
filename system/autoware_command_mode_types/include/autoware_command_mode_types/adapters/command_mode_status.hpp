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

#ifndef AUTOWARE_COMMAND_MODE_TYPES__ADAPTERS__COMMAND_MODE_STATUS_HPP_
#define AUTOWARE_COMMAND_MODE_TYPES__ADAPTERS__COMMAND_MODE_STATUS_HPP_

#include "autoware_command_mode_types/types/command_mode_status.hpp"

#include <tier4_system_msgs/msg/command_mode_status_item.hpp>

namespace autoware::command_mode_types
{

uint8_t convert_request_stage(const RequestStage & request_stage)
{
  using Message = tier4_system_msgs::msg::CommandModeStatusItem;
  // clang-format off
  switch (request_stage) {
    case RequestStage::CommandModeReady: return Message::COMMAND_MODE_READY;
    case RequestStage::VehicleGateReady: return Message::VEHICLE_GATE_READY;
    case RequestStage::NetworkGateReady: return Message::NETWORK_GATE_READY;
    case RequestStage::ControlGateReady: return Message::CONTROL_GATE_READY;
    default:                             return Message::UNDEFINED;
  }
  // clang-format on
}

}  // namespace autoware::command_mode_types

#endif  // AUTOWARE_COMMAND_MODE_TYPES__ADAPTERS__COMMAND_MODE_STATUS_HPP_
