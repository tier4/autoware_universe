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

#ifndef CONSTANT_ACCELERATION_CONTROLLER__CONSTANT_ACCELERATION_CONTROLLER_HPP_
#define CONSTANT_ACCELERATION_CONTROLLER__CONSTANT_ACCELERATION_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <trajectory_follower_base/longitudinal_controller_base.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>

namespace autoware::motion::control::constant_acceleration_controller
{
using tier4_debug_msgs::msg::Float32Stamped;

class ConstantAccelerationController : public trajectory_follower::LongitudinalControllerBase
{
public:
  explicit ConstantAccelerationController(rclcpp::Node & node);

private:
  rclcpp::Node * node_;

  // Subscriber
  rclcpp::Subscription<Float32Stamped>::SharedPtr sub_target_acceleration_;

  void onTargetAcceleration(Float32Stamped::ConstSharedPtr msg);
  float current_target_acceleration_;

  // Function
  bool isReady(const trajectory_follower::InputData & input_data) override;
  trajectory_follower::LongitudinalOutput run(
    trajectory_follower::InputData const & input_data) override;
};
}  // namespace autoware::motion::control::constant_acceleration_controller

#endif  // CONSTANT_ACCELERATION_CONTROLLER__CONSTANT_ACCELERATION_CONTROLLER_HPP_
