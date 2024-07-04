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

#ifndef TRAJECTORY_REPEATER__TRAJECTORY_REPEATER_HPP_
#define TRAJECTORY_REPEATER__TRAJECTORY_REPEATER_HPP_

// include
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

namespace trajectory_repeater
{

class TrajectoryRepeater : public rclcpp::Node
{
public:
  explicit TrajectoryRepeater(const rclcpp::NodeOptions & node_options);
  ~TrajectoryRepeater() = default;

private:
  // Parameter

  // Subscriber
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;

  void onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;

  // Service

  // Client

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  void onTimer();

  // State
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr last_trajectory_;

  // Diagnostics

};
}  // namespace trajectory_repeater

#endif  // TRAJECTORY_REPEATER__TRAJECTORY_REPEATER_HPP_