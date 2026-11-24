// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ml_planner/preprocessing/items/traffic_signals.hpp"

namespace autoware::ml_planner::preprocess
{
std::map<lanelet::Id, TrafficSignalStamped> create_traffic_signal_map(
  const std::deque<autoware_perception_msgs::msg::TrafficLightGroupArray> & msgs,
  const rclcpp::Time & current_time, const double time_threshold_seconds)
{
  std::map<lanelet::Id, TrafficSignalStamped> traffic_signal_id_map;

  // Keep the latest signal per traffic light group id
  for (const auto & msg : msgs) {
    const rclcpp::Time msg_time(msg.stamp);
    if (msg_time > current_time || (current_time - msg_time).seconds() > time_threshold_seconds) {
      continue;
    }
    for (const auto & signal : msg.traffic_light_groups) {
      auto & curr = traffic_signal_id_map[signal.traffic_light_group_id];
      if (curr.signal.elements.empty() || msg_time > rclcpp::Time(curr.stamp)) {
        curr.signal = signal;
        curr.stamp = msg_time;
      }
    }
  }

  return traffic_signal_id_map;
}

}  // namespace autoware::ml_planner::preprocess
