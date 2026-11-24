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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TRAFFIC_SIGNALS_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TRAFFIC_SIGNALS_HPP_

#include "autoware/ml_planner/preprocessing/items/traffic_signal_stamped.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/primitives/Traits.h>

#include <deque>
#include <map>

namespace autoware::ml_planner::preprocess
{
/**
 * @brief Builds a traffic signal map from a window of TrafficLightGroupArray messages.
 *
 * Stateless: the result is fully determined by the given messages and time.
 * For each traffic light group id, the latest signal in the window is kept;
 * signals older than time_threshold_seconds relative to current_time are
 * discarded.
 *
 * @param msgs Chronological window of TrafficLightGroupArray messages.
 * @param current_time The reference time used for the age check.
 * @param time_threshold_seconds Signals older than this threshold (in seconds) are discarded.
 * @return Map of the latest valid traffic signal per lanelet ID.
 */
std::map<lanelet::Id, TrafficSignalStamped> create_traffic_signal_map(
  const std::deque<autoware_perception_msgs::msg::TrafficLightGroupArray> & msgs,
  const rclcpp::Time & current_time, const double time_threshold_seconds);

}  // namespace autoware::ml_planner::preprocess
#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TRAFFIC_SIGNALS_HPP_
