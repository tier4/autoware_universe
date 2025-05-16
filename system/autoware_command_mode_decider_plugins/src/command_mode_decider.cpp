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

#include "command_mode_decider.hpp"

#include <autoware_command_mode_types/constants/modes.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <tier4_system_msgs/srv/change_operation_mode.hpp>

namespace autoware::command_mode_decider
{

namespace modes = autoware::command_mode_types::modes;
using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_system_msgs::srv::ChangeOperationMode;

uint16_t CommandModeDecider::from_operation_mode(uint16_t operation_mode)
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

uint16_t CommandModeDecider::to_operation_mode(uint16_t command_mode)
{
  // clang-format off
  switch(command_mode) {
    case modes::stop:       return OperationModeState::STOP;
    case modes::autonomous: return OperationModeState::AUTONOMOUS;
    case modes::local:      return OperationModeState::LOCAL;
    case modes::remote:     return OperationModeState::REMOTE;
    default:                return OperationModeState::UNKNOWN;
  }
  // clang-format on
}

uint16_t CommandModeDecider::to_mrm_behavior(uint16_t command_mode)
{
  // clang-format off
  switch(command_mode) {
    case modes::emergency_stop:   return MrmState::EMERGENCY_STOP;
    case modes::comfortable_stop: return MrmState::COMFORTABLE_STOP;
    case modes::pull_over:        return MrmState::PULL_OVER;
    default:                      return MrmState::NONE;
  }
  // clang-format on
}

uint16_t CommandModeDecider::decide(
  const RequestModeStatus & request, const CommandModeStatusTable & status)
{
  const auto command_mode_status = status;
  const auto request_mode_status = request;

  const auto background = !request_mode_status.autoware_control;
  const auto is_available = [background](const auto & status) {
    return status.mode_available && (status.transition_available || background);
  };

  // Use the specified operation mode if available.
  {
    const auto status = command_mode_status.get(request_mode_status.operation_mode);
    if (is_available(status)) {
      return request_mode_status.operation_mode;
    }
  }

  // TODO(Takagi, Isamu): Use the available MRM according to the state transitions.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/#behavior
  // use_pull_over_
  // use_comfortable_stop_

  namespace modes = autoware::command_mode_types::modes;

  if (command_mode_status.get(modes::pull_over).mode_available) {
    return modes::pull_over;
  }
  if (command_mode_status.get(modes::comfortable_stop).mode_available) {
    return modes::comfortable_stop;
  }
  if (command_mode_status.get(modes::emergency_stop).mode_available) {
    return modes::emergency_stop;
  }

  // RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no mrm available");
  return modes::unknown;
}

}  // namespace autoware::command_mode_decider

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::command_mode_decider::CommandModeDecider, autoware::command_mode_decider::DeciderPlugin)
