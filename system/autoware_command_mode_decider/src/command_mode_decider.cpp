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

#include <string>

namespace autoware::command_mode_decider
{

CommandModeDecider::CommandModeDecider(const rclcpp::NodeOptions & options)
: CommandModeDeciderBase(options)
{
}

std::string CommandModeDecider::decide_command_mode()
{
  const auto command_mode_status = get_command_mode_status();
  const auto request_mode_status = get_request_mode_status();
  const auto background = !request_mode_status.autoware_control;
  const auto is_available = [background](const auto & status) {
    return status.mode_available && (status.transition_available || background);
  };

  // Use the requested MRM if available.
  {
    const auto status = command_mode_status.get(request_mode_status.mrm);
    if (is_available(status)) {
      return request_mode_status.mrm;
    }
  }

  // Use the specified operation mode if available.
  {
    const auto status = command_mode_status.get(request_mode_status.operation_mode);
    if (is_available(status)) {
      return request_mode_status.operation_mode;
    }
  }

  // TODO(Takagi, Isamu): Use the available MRM according to the state transitions at the
  // following.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/#behavior
  const auto pull_over = "pull_over";
  const auto comfortable_stop = "comfortable_stop";
  const auto emergency_stop = "emergency_stop";

  // TODO(Takagi, Isamu): Create state transition table.
  // use_pull_over_
  // use_comfortable_stop_
  if (command_mode_status.get(pull_over).mode_available) {
    return pull_over;
  }
  if (command_mode_status.get(comfortable_stop).mode_available) {
    return comfortable_stop;
  }
  if (command_mode_status.get(emergency_stop).mode_available) {
    return emergency_stop;
  }

  // Use an empty string to delegate to switcher node.
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no mrm available: delegate to switcher");
  return std::string();
}

}  // namespace autoware::command_mode_decider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_decider::CommandModeDecider)
