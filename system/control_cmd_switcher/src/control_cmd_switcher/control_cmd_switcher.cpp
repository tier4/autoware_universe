// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language
// governing permissions and limitations under the License.

#include "control_cmd_switcher.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>

ControlCmdSwitcher::ControlCmdSwitcher(const rclcpp::NodeOptions & node_options)
: Node("control_cmd_switcher", node_options)
{
  // Subscriber
  sub_main_control_cmd_ =
    create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "~/input/main/control_cmd", rclcpp::QoS{10},
      std::bind(&ControlCmdSwitcher::onMainControlCmd, this, std::placeholders::_1));

  sub_sub_control_cmd_ =
    create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "~/input/sub/control_cmd", rclcpp::QoS{10},
      std::bind(&ControlCmdSwitcher::onSubControlCmd, this, std::placeholders::_1));

  sub_election_status_main_ = create_subscription<tier4_system_msgs::msg::ElectionStatus>(
    "~/input/election/status/main", rclcpp::QoS{10},
    std::bind(&ControlCmdSwitcher::onElectionStatus, this, std::placeholders::_1));

  sub_election_status_sub_ = create_subscription<tier4_system_msgs::msg::ElectionStatus>(
    "~/input/election/status/sub", rclcpp::QoS{10},
    std::bind(&ControlCmdSwitcher::onElectionStatus, this, std::placeholders::_1));

  // Publisher
  pub_control_cmd_ = create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
    "~/output/control_cmd", rclcpp::QoS{1});

  // Initialize
  use_main_control_cmd_ = true;
}

void ControlCmdSwitcher::onMainControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  if (use_main_control_cmd_) {
    pub_control_cmd_->publish(*msg);
  }
}

void ControlCmdSwitcher::onSubControlCmd(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  if (!use_main_control_cmd_) {
    pub_control_cmd_->publish(*msg);
  }
}

void ControlCmdSwitcher::onElectionStatus(
  const tier4_system_msgs::msg::ElectionStatus::ConstSharedPtr msg)
{
  if (msg->election_start_count <= 0) return;
  if (msg->in_election) return;
  if (((msg->path_info >> 3) & 0x01) == 1) {
    use_main_control_cmd_ = true;
  } else if (((msg->path_info >> 2) & 0x01) == 1) {
    use_main_control_cmd_ = false;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ControlCmdSwitcher)
