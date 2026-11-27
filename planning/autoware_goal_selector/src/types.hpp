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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::goal_selector
{

/// @brief Why a goal candidate can or cannot be used.
enum class GoalState {
  AVAILABLE,   // no object in the area and the signal is not red
  OCCUPIED,    // an object overlaps the goal footprint
  RED_SIGNAL,  // the traffic signal assigned to the goal is red
};

struct GoalCandidate
{
  std::string name;
  geometry_msgs::msg::Pose pose;
  std::vector<int64_t> lane_ids;                  // lanelet IDs in travel order
  std::optional<int64_t> traffic_light_group_id;  // the goal is not used while this signal is red
};

struct Trigger
{
  std::string name;
  double x{};
  double y{};
  double z{};  // elevation of the first goal candidate, used to tell floors apart
  double radius{};
  std::vector<GoalCandidate> goal_candidates;  // in priority order
  // Goal to go to when no goal candidate is available
  std::optional<GoalCandidate> fallback_goal;
};

}  // namespace autoware::goal_selector

#endif  // TYPES_HPP_
