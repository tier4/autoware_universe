// Copyright 2022 TIER IV, Inc.
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

#ifndef START_GOAL_PLANNER__CLOTHOID_PULL_GENERATER_HPP_
#define START_GOAL_PLANNER__CLOTHOID_PULL_GENERATER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

/**
 * @brief Generate a clothoid-smoothed connector between two poses.
 *
 * Pure-geometry helper: tries each candidate steering angle (largest first is not assumed;
 * callers should order them by preference) and returns the point sequence of the first
 * feasible entry-clothoid -> circular-arc -> exit-clothoid connection from start_pose to
 * target_pose. Returns std::nullopt if no candidate steering angle yields a feasible path.
 */
std::optional<std::vector<std::vector<geometry_msgs::msg::Point>>> plan_clothoid_pull(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & target_pose,
  double wheel_base_m, const double & max_steer_angle, double max_steer_angle_rate_rad_per_sec,
  double reference_velocity_mps);

}  // namespace autoware::minimum_rule_based_planner

#endif  // START_GOAL_PLANNER__CLOTHOID_PULL_GENERATER_HPP_
