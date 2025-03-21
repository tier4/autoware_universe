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

#ifndef COMMON__SELECTOR_INTERFACE_HPP_
#define COMMON__SELECTOR_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>
#include <tier4_system_msgs/srv/select_command_source.hpp>

#include <string>
#include <vector>

namespace autoware::command_mode_switcher
{

class SelectorInterface
{
public:
  using Callback = std::function<void()>;
  SelectorInterface(rclcpp::Node & node, Callback callback);
  bool select_source(const std::string & source);
  bool select_control(const bool autoware_control);

  const std::string & source_name() const { return source_status_.source; }
  std::optional<bool> autoware_control() const;

private:
  using SelectCommandSource = tier4_system_msgs::srv::SelectCommandSource;
  using CommandSourceStatus = tier4_system_msgs::msg::CommandSourceStatus;
  using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;

  void on_source_status(const CommandSourceStatus & msg);
  void on_control_mode(const ControlModeReport & msg);

  rclcpp::Node & node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  Callback notification_callback_;

  rclcpp::Client<SelectCommandSource>::SharedPtr cli_source_select_;
  rclcpp::Subscription<CommandSourceStatus>::SharedPtr sub_source_status_;
  bool waiting_source_select_ = false;
  CommandSourceStatus source_status_;

  rclcpp::Client<ControlModeCommand>::SharedPtr cli_control_mode_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_;
  bool waiting_control_mode_ = false;
  ControlModeReport control_mode_;
};

}  // namespace autoware::command_mode_switcher

#endif  // COMMON__SELECTOR_INTERFACE_HPP_
