// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROL_CMD_SWITCHER__CONTROL_CMD_SWITCHER_HPP_
#define CONTROL_CMD_SWITCHER__CONTROL_CMD_SWITCHER_HPP_

// Core
#include <atomic>
#include <memory>
#include <string>

// Autoware
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_system_msgs/msg/election_status.hpp>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>

class ControlCmdSwitcher : public rclcpp::Node
{
public:
  explicit ControlCmdSwitcher(const rclcpp::NodeOptions & node_options);

private:
  // Subscribers
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_main_control_cmd_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_sub_control_cmd_;
  rclcpp::Subscription<tier4_system_msgs::msg::ElectionStatus>::SharedPtr sub_election_status_main_;
  rclcpp::Subscription<tier4_system_msgs::msg::ElectionStatus>::SharedPtr sub_election_status_sub_;
  void onMainControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onSubControlCmd(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onElectionStatus(const tier4_system_msgs::msg::ElectionStatus::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    pub_control_cmd_;

  std::atomic<bool> use_main_control_cmd_;
};

#endif  // CONTROL_CMD_SWITCHER__CONTROL_CMD_SWITCHER_HPP_
