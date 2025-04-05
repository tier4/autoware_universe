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

#include "command_mode_decider_redundancy.hpp"

#include <string>

namespace autoware::command_mode_decider
{

  CommandModeDeciderRedundancy::CommandModeDeciderRedundancy(const rclcpp::NodeOptions & options)
: CommandModeDeciderBase(options)
{
}

std::string CommandModeDeciderRedundancy::decide_command_mode()
{
  const auto command_mode_status = get_command_mode_status();
  const auto request_mode_status = get_request_mode_status();

  // Use the requested MRM if available.
  {
    const auto status = command_mode_status.get(request_mode_status.mrm);
    if (status.mode_available) {
      return request_mode_status.mrm;
    }
  }

  // Use the specified operation mode if available.
  {
    const auto status = command_mode_status.get(request_mode_status.operation_mode);
    if (status.mode_available) {
      return request_mode_status.operation_mode;
    }
  }

  // TODO(Takagi, Isamu): Use the available MRM according to the state transitions at the
  // following.
  // https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/ad-api/features/fail-safe/#behavior
  const auto comfortable_stop = "comfortable_stop";
  const auto main_ecu_in_lane_stop_0_4g = "main_ecu_in_lane_stop_0_4g";
  const auto main_ecu_in_lane_stop_0_6g = "main_ecu_in_lane_stop_0_6g";
  const auto sub_ecu_in_lane_stop_0_4g = "sub_ecu_in_lane_stop_0_4g";

  // TODO(Takagi, Isamu): check command_modes parameter
  if (command_mode_status.get(comfortable_stop).mode_available /*&& use_comfortable_stop_*/) {
    return comfortable_stop;
  }
  if (command_mode_status.get(main_ecu_in_lane_stop_0_4g).mode_available /*&& use_main_ecu_in_lane_stop_0_4g_*/) {
    return main_ecu_in_lane_stop_0_4g;
  }
  if (command_mode_status.get(main_ecu_in_lane_stop_0_6g).mode_available /*&& use_main_ecu_in_lane_stop_0_6g_*/) {
    return main_ecu_in_lane_stop_0_6g;
  }
  if (command_mode_status.get(sub_ecu_in_lane_stop_0_4g).mode_available /*&& use_sub_ecu_in_lane_stop_0_4g_*/) {
    return sub_ecu_in_lane_stop_0_4g;
  }
  // FIXME(TetsuKawa): How to handle the case where no MRM is available.

  // Use an empty string to delegate to switcher node.
  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 5000, "no mrm available: delegate to switcher node");
  return std::string();
}

}  // namespace autoware::command_mode_decider

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::command_mode_decider::CommandModeDeciderRedundancy)
