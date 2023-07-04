// Copyright 2023 Tier IV, Inc. All rights reserved.
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

#include <constant_acceleration_controller/constant_acceleration_controller.hpp>

namespace autoware::motion::control::constant_acceleration_controller
{
ConstantAccelerationController::ConstantAccelerationController(rclcpp::Node & node)
: node_{&node}
{
  // Subscriber
  using std::placeholders::_1;
  sub_target_acceleration_ = node.create_subscription<Float32Stamped>(
    "/control/tester/target_acceleration", 1,
    std::bind(&ConstantAccelerationController::onTargetAcceleration, this, _1));

  // Initialize
  current_target_acceleration_ = 0.0;
}

void ConstantAccelerationController::onTargetAcceleration(Float32Stamped::ConstSharedPtr msg)
{
  current_target_acceleration_ = msg->data;
  RCLCPP_INFO(node_->get_logger(), "target acceleration: %.2f", current_target_acceleration_);
}

bool ConstantAccelerationController::isReady(
  [[maybe_unused]] const trajectory_follower::InputData & input_data)
{
  return true;
}

trajectory_follower::LongitudinalOutput ConstantAccelerationController::run(
  [[maybe_unused]] const trajectory_follower::InputData & input_data)
{
  trajectory_follower::LongitudinalOutput output;
  output.control_cmd.stamp = node_->now();
  output.control_cmd.speed = 10.0;  // To prevent error detection
  output.control_cmd.acceleration = current_target_acceleration_;

  return output;
}
} // namespace autoware::motion::control::constant_acceleration_controller
