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

#include "dummy_operation_mode_publisher.hpp"

namespace dummy_operation_mode_publisher
{

DummyOperationModePublisher::DummyOperationModePublisher(const rclcpp::NodeOptions & node_options)
: Node("dummy_operation_mode_publisher", node_options)
{
  // Parameter

  // Subscriber

  // Publisher
  pub_operation_mode_state_ = create_publisher<autoware_adapi_v1_msgs::msg::OperationModeState>(
    "~/output/operation_mode_state", 10);

  // Service

  // Client

  // Timer
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 1s, std::bind(&DummyOperationModePublisher::onTimer, this));

  // State

  // Diagnostics
}

void DummyOperationModePublisher::onTimer()
{
  autoware_adapi_v1_msgs::msg::OperationModeState msg;
  msg.stamp = this->now();
  msg.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
  msg.is_autonomous_mode_available = true;
  msg.is_in_transition = false;
  msg.is_stop_mode_available = true;
  msg.is_autonomous_mode_available = true;
  msg.is_local_mode_available = true;
  msg.is_remote_mode_available = true;

  pub_operation_mode_state_->publish(msg);
}

}  // namespace dummy_operation_mode_publisher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dummy_operation_mode_publisher::DummyOperationModePublisher)
