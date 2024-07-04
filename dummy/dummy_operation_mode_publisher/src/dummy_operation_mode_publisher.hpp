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

#ifndef DUMMY_OPERATION_MODE_PUBLISHER_HPP_
#define DUMMY_OPERATION_MODE_PUBLISHER_HPP_

// include
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>

namespace dummy_operation_mode_publisher
{

class DummyOperationModePublisher : public rclcpp::Node
{
public:
  explicit DummyOperationModePublisher(const rclcpp::NodeOptions & node_options);
  ~DummyOperationModePublisher() = default;

private:
  // Parameter

  // Subscriber

  // Publisher
  rclcpp::Publisher<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    pub_operation_mode_state_;

  // Service

  // Client

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // State

  // Diagnostics
};
}  // namespace dummy_operation_mode_publisher

#endif  // DUMMY_OPERATION_MODE_PUBLISHER_HPP_
