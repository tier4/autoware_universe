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

#include "dummy_gear_cmd_publisher.hpp"

namespace dummy_gear_cmd_publisher
{

DummyGearCmdPublisher::DummyGearCmdPublisher(const rclcpp::NodeOptions & node_options)
: Node("dummy_gear_cmd_publisher", node_options)
{
  // Parameter

  // Subscriber

  // Publisher
  pub_gear_cmd_ = create_publisher<autoware_auto_vehicle_msgs::msg::GearCommand>(
    "~/output/gear_cmd", 10);

  // Service

  // Client

  // Timer
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&DummyGearCmdPublisher::onTimer, this));

  // State

  // Diagnostics

}

void DummyGearCmdPublisher::onTimer()
{
  autoware_auto_vehicle_msgs::msg::GearCommand msg;
  msg.stamp = this->now();
  msg.command = autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE;

  pub_gear_cmd_->publish(msg);
}

}  // namespace dummy_gear_cmd_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dummy_gear_cmd_publisher::DummyGearCmdPublisher)
