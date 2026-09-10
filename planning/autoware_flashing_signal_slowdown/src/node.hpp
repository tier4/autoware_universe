// Copyright 2026 TIER IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "flashing_detector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_internal_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <cstdint>
#include <map>

namespace autoware::flashing_signal_slowdown
{

using autoware_internal_planning_msgs::msg::VelocityLimit;
using autoware_internal_planning_msgs::msg::VelocityLimitClearCommand;
using autoware_perception_msgs::msg::TrafficLightGroupArray;

class FlashingSignalSlowdownNode : public rclcpp::Node
{
public:
  explicit FlashingSignalSlowdownNode(const rclcpp::NodeOptions & node_options);

private:
  void on_traffic_signals(const TrafficLightGroupArray::ConstSharedPtr msg);
  void on_timer();
  void publish_velocity_limit();
  void publish_clear_command();

  double slowdown_velocity_mps_{0.0};
  std::map<std::int64_t, FlashingDetector> detectors_;

  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr sub_traffic_signals_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool slowdown_active_{false};
};

}  // namespace autoware::flashing_signal_slowdown

#endif  // NODE_HPP_
