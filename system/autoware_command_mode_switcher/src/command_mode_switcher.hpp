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

#ifndef COMMAND_MODE_SWITCHER_HPP_
#define COMMAND_MODE_SWITCHER_HPP_

#include "common/command_container.hpp"
#include "common/manual.hpp"
#include "common/selector_interface.hpp"

#include <autoware_command_mode_types/adapters/command_mode_status.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/command_mode_request.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::command_mode_switcher
{

using autoware::command_mode_types::CommandModeStatus;
using autoware::command_mode_types::CommandModeStatusAdapter;
using tier4_system_msgs::msg::CommandModeAvailability;
using tier4_system_msgs::msg::CommandModeRequest;

class CommandModeSwitcher : public rclcpp::Node
{
public:
  explicit CommandModeSwitcher(const rclcpp::NodeOptions & options);

private:
  void on_availability(const CommandModeAvailability & msg);
  void on_request(const CommandModeRequest & msg);
  void update();

  void detect_override();
  void update_status();
  void handle_foreground_transition();
  void handle_background_transition();
  void publish_command_mode_status();

  // ROS interfaces.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<CommandModeAvailability>::SharedPtr sub_availability_;
  rclcpp::Subscription<CommandModeRequest>::SharedPtr sub_request_;
  rclcpp::Publisher<CommandModeStatusAdapter>::SharedPtr pub_status_;

  // Mode switching.
  pluginlib::ClassLoader<CommandPlugin> loader_;
  std::vector<std::shared_ptr<Command>> commands_;
  std::unordered_map<std::string, std::shared_ptr<Command>> platform_commands_;
  std::unordered_map<std::string, std::shared_ptr<Command>> autoware_commands_;
  std::shared_ptr<Command> manual_command_;
  std::shared_ptr<Command> foreground_;
  std::shared_ptr<Command> background_;
  ControlGateInterface control_gate_interface_;
  VehicleGateInterface vehicle_gate_interface_;

  bool is_ready_ = false;
  bool is_autoware_control_ = false;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMAND_MODE_SWITCHER_HPP_
