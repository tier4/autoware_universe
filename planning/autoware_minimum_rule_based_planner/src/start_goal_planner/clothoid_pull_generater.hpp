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

#ifndef START_GOAL_PLANNER__CLOTHOID_PULL_GENERATER_HPP_
#define START_GOAL_PLANNER__CLOTHOID_PULL_GENERATER_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

/**
 * @brief Generate a clothoid-smoothed connector between two poses.
 *
 * Pure-geometry helper: try to generate 4 types of arc (starts from bigger/smaller circle and
 * left/right pull) and returns the point sequence of the first feasible entry-clothoid ->
 * circular-arc -> exit-clothoid connection from start_pose to target_pose. Returns std::nullopt if
 * no candidate steering angle yields a feasible path.
 */
std::optional<std::vector<std::vector<geometry_msgs::msg::Point>>> plan_clothoid_pull(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & target_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double & first_steer_angle,
  double max_steer_angle_rate_rad_per_sec, double reference_velocity_mps);

/**
 * @brief Generate candidate max-steer-angles for the clothoid goal connector.
 *
 * Evenly spaces `trial_count` angles between the minimum steer angle and `max_steer_angle_rad`, in
 * ascending order. If trial_count <= 1, only max_steer_angle_rad is returned.
 */
std::vector<double> generate_candidate_steer_angles_rad(
  double max_steer_angle_rad, int trial_count);

}  // namespace autoware::minimum_rule_based_planner

#endif  // START_GOAL_PLANNER__CLOTHOID_PULL_GENERATER_HPP_
