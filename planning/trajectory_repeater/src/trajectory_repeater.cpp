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

#include "trajectory_repeater.hpp"

namespace trajectory_repeater
{

TrajectoryRepeater::TrajectoryRepeater(const rclcpp::NodeOptions & node_options)
: Node("trajectory_repeater", node_options)
{
  // Parameter

  // Subscriber
  sub_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "~/input/trajectory", 10, std::bind(&TrajectoryRepeater::onTrajectory, this, std::placeholders::_1));

  // Publisher
  pub_trajectory_ = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/trajectory", 10);

  // Service

  // Client

  // Timer
  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrajectoryRepeater::onTimer, this));

  // State

  // Diagnostics

}

void TrajectoryRepeater::onTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  last_trajectory_ = msg;
}

void TrajectoryRepeater::onTimer()
{
  if (!last_trajectory_) {
    RCLCPP_DEBUG(get_logger(), "No trajectory received");
    return;
  }
  
  pub_trajectory_->publish(*last_trajectory_);
}

}  // namespace trajectory_repeater

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_repeater::TrajectoryRepeater)
