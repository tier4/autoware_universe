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

#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/preprocessing/items/map.hpp"

#include <xtensor/xbuilder.hpp>

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

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
    if ((current_time - msg_time).seconds() > time_threshold_seconds) {
      continue;  // outdated message
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

xt::xarray<float> create_traffic_light_past(
  const MessageView<autoware_perception_msgs::msg::TrafficLightGroupArray> & traffic_signals_msgs,
  const std::vector<std::pair<lanelet::Id, int64_t>> & segment_lights, const int64_t max_segments,
  const rclcpp::Time & current_time, const int64_t num_timesteps, const double time_step_s,
  const double timeout_s)
{
  constexpr int64_t green_idx = 0;
  constexpr int64_t amber_idx = 1;
  constexpr int64_t red_idx = 2;
  constexpr int64_t unknown_idx = 3;
  constexpr int64_t white_or_none_idx = 4;
  constexpr int64_t arrow_idx = 5;

  using autoware_perception_msgs::msg::TrafficLightElement;

  xt::xarray<float> history = xt::zeros<float>(
    {static_cast<size_t>(max_segments), static_cast<size_t>(num_timesteps),
     static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM)});

  // Collect the observations of every traffic light group in chronological
  // order, keyed by group id.
  struct Observation
  {
    double stamp_sec;
    const std::vector<TrafficLightElement> * elements;
  };
  std::unordered_map<int64_t, std::vector<Observation>> observations_map;
  for (const auto & msg : traffic_signals_msgs) {
    const double stamp_sec = rclcpp::Time(msg.stamp).seconds();
    for (const auto & group : msg.traffic_light_groups) {
      observations_map[group.traffic_light_group_id].push_back({stamp_sec, &group.elements});
    }
  }

  const double current_sec = current_time.seconds();
  constexpr double stamp_tolerance_s = 1e-6;

  for (size_t seg = 0; seg < segment_lights.size() && seg < static_cast<size_t>(max_segments);
       ++seg) {
    const auto & [traffic_light_id, turn_direction] = segment_lights[seg];
    if (traffic_light_id == LaneSegment::TRAFFIC_LIGHT_ID_NONE) {
      for (int64_t t = 0; t < num_timesteps; ++t) {
        history(seg, t, white_or_none_idx) = 1.0f;
      }
      continue;
    }

    const auto observations_it = observations_map.find(static_cast<int64_t>(traffic_light_id));
    const std::vector<Observation> * observations =
      (observations_it != observations_map.end()) ? &observations_it->second : nullptr;

    size_t obs_idx = 0;  // grid times are increasing, so carry the index forward
    for (int64_t t = 0; t < num_timesteps; ++t) {
      const double grid_sec =
        current_sec - static_cast<double>(num_timesteps - 1 - t) * time_step_s;
      if (observations) {
        while (obs_idx + 1 < observations->size() &&
               (*observations)[obs_idx + 1].stamp_sec <= grid_sec + stamp_tolerance_s) {
          ++obs_idx;
        }
      }

      const bool has_valid_observation =
        observations && !observations->empty() &&
        (*observations)[obs_idx].stamp_sec <= grid_sec + stamp_tolerance_s &&
        grid_sec - (*observations)[obs_idx].stamp_sec <= timeout_s;
      if (!has_valid_observation) {
        history(seg, t, unknown_idx) = 1.0f;
        continue;
      }

      const TrafficLightStatus status =
        identify_current_light_status(turn_direction, *(*observations)[obs_idx].elements);
      history(seg, t, green_idx) = status.color == TrafficLightElement::GREEN;
      history(seg, t, amber_idx) = status.color == TrafficLightElement::AMBER;
      history(seg, t, red_idx) = status.color == TrafficLightElement::RED;
      history(seg, t, unknown_idx) = status.color == TrafficLightElement::UNKNOWN;
      history(seg, t, white_or_none_idx) = status.color == TrafficLightElement::WHITE;
      history(seg, t, arrow_idx) = status.is_arrow ? 1.0f : 0.0f;
    }
  }

  return history;
}

xt::xarray<float> infer_traffic_light_future(const xt::xarray<float> & past)
{
  const size_t segments = past.shape().at(0);
  xt::xarray<float> future = xt::zeros<float>(
    {segments, static_cast<size_t>(OUTPUT_T), static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM)});
  constexpr size_t amber_index = 1;
  constexpr size_t red_index = 2;
  constexpr size_t amber_duration_steps = 30;

  for (size_t segment = 0; segment < segments; ++segment) {
    const size_t current_t = past.shape().at(1) - 1;
    bool has_state = false;
    for (size_t d = 0; d < static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM); ++d) {
      has_state = has_state || past(segment, current_t, d) != 0.0F;
    }
    if (!has_state) {
      continue;
    }

    size_t remaining_amber_steps = 0;
    if (past(segment, current_t, amber_index) > 0.5F) {
      size_t elapsed_steps = 0;
      for (size_t t = current_t + 1; t > 0; --t) {
        if (past(segment, t - 1, amber_index) <= 0.5F) {
          break;
        }
        ++elapsed_steps;
      }
      remaining_amber_steps =
        amber_duration_steps > elapsed_steps ? amber_duration_steps - elapsed_steps : 0;
    }

    for (size_t t = 0; t < static_cast<size_t>(OUTPUT_T); ++t) {
      if (past(segment, current_t, amber_index) > 0.5F && t >= remaining_amber_steps) {
        future(segment, t, red_index) = 1.0F;
      } else {
        for (size_t d = 0; d < static_cast<size_t>(TRAFFIC_LIGHT_ONE_HOT_DIM); ++d) {
          future(segment, t, d) = past(segment, current_t, d);
        }
      }
    }
  }
  return future;
}

}  // namespace autoware::ml_planner::preprocess
