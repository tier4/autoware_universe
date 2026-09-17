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

#include <string>
#include <vector>

namespace autoware::goal_selector
{

struct GoalCandidate
{
  std::string name;
  geometry_msgs::msg::Pose pose;
};

struct Trigger
{
  std::string name;
  double x{};
  double y{};
  double radius{};
  std::vector<GoalCandidate> goal_candidates;  // in priority order
};

}  // namespace autoware::goal_selector

#endif  // TYPES_HPP_
