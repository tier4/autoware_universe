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

#include "autoware/ml_planner/preprocessing/message_view.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <xtensor/xarray.hpp>

#include <autoware_perception_msgs/msg/detail/traffic_light_group__struct.hpp>
#include <autoware_perception_msgs/msg/detail/traffic_light_group_array__struct.hpp>
#include <autoware_perception_msgs/msg/traffic_signal.hpp>

#include <lanelet2_traffic_rules/TrafficRules.h>

#include <cstdint>
#include <deque>
#include <map>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{
/**
 * @brief Represents a traffic signal with a timestamp.
 */
struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;                      ///< Timestamp of the signal.
  autoware_perception_msgs::msg::TrafficLightGroup signal;  ///< Traffic light group.
};

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

/**
 * @brief Build the one-hot traffic light state history tensor per lane segment.
 *
 * Stateless: samples the messages on a fixed time grid ending at current_time
 * (zero-order hold on message stamps).
 * Output layout: (max_segments, num_timesteps, TRAFFIC_LIGHT_ONE_HOT_DIM).
 * Slot layout per timestep: [GREEN, AMBER, RED, UNKNOWN/no-data, WHITE or no
 * light]. Rows beyond segment_lights.size() stay all-zero (padding).
 *
 * @param traffic_signals_msgs Chronological window of TrafficLightGroupArray messages.
 * @param segment_lights (traffic light id, turn_direction) per lane segment.
 * @param max_segments Number of segment rows in the output tensor.
 * @param current_time Time of the newest grid sample.
 * @param num_timesteps Number of grid samples.
 * @param time_step_s Grid interval in seconds.
 * @param timeout_s Observations older than this relative to a grid time are treated as no-data.
 * @return Tensor with shape [max_segments, num_timesteps, TRAFFIC_LIGHT_ONE_HOT_DIM].
 */
xt::xarray<float> create_traffic_light_past(
  const MessageView<autoware_perception_msgs::msg::TrafficLightGroupArray> & traffic_signals_msgs,
  const std::vector<std::pair<lanelet::Id, int64_t>> & segment_lights, const int64_t max_segments,
  const rclcpp::Time & current_time, const int64_t num_timesteps, const double time_step_s,
  const double timeout_s);

/**
 * @brief Infer the 8-second traffic-light sequence used during online inference.
 *
 * Green, red, unknown, and white states are held constant. Amber is assumed
 * to last three seconds in total; its elapsed duration is estimated from the
 * history and the remaining sequence transitions to red.
 */
xt::xarray<float> infer_traffic_light_future(const xt::xarray<float> & past);

}  // namespace autoware::ml_planner::preprocess
#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__TRAFFIC_SIGNALS_HPP_
