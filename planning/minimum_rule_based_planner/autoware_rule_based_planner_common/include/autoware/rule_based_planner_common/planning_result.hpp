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

#ifndef AUTOWARE__RULE_BASED_PLANNER_COMMON__PLANNING_RESULT_HPP_
#define AUTOWARE__RULE_BASED_PLANNER_COMMON__PLANNING_RESULT_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <vector>

namespace autoware::rule_based_planner
{

/**
 * @brief Represents a slowdown interval on the trajectory
 */
struct SlowdownInterval
{
  geometry_msgs::msg::Point from{};  ///< Start point of slowdown
  geometry_msgs::msg::Point to{};    ///< End point of slowdown
  double velocity{0.0};              ///< Target velocity in m/s
};

/**
 * @brief Result of a planning module's plan() function
 *
 * Contains stop points, slowdown intervals, and other modifications
 * that should be applied to the trajectory.
 */
struct PlanningResult
{
  /// Stop points to insert into trajectory (vehicle should stop at these points)
  std::vector<geometry_msgs::msg::Point> stop_points{};

  /// Intervals where vehicle should slow down
  std::vector<SlowdownInterval> slowdown_intervals{};

  /// Optional velocity limit to apply
  std::optional<double> velocity_limit{std::nullopt};

  /// Flag indicating if this module requires a stop
  bool stop_required{false};

  /// Distance to the stop point (if stop_required is true)
  std::optional<double> distance_to_stop{std::nullopt};
};

}  // namespace autoware::rule_based_planner

#endif  // AUTOWARE__RULE_BASED_PLANNER_COMMON__PLANNING_RESULT_HPP_
