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

#ifndef PATH_SHIFT_TO_EGO_HPP_
#define PATH_SHIFT_TO_EGO_HPP_

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace autoware::minimum_rule_based_planner
{

struct TrajectoryShiftParams
{
  double minimum_shift_length{0.1};    // [m] below this, no shift is applied
  double minimum_shift_distance{5.0};  // [m] base skip distance ahead of ego
  double shift_length_to_distance_ratio{
    8.0};  // [-] ratio to scale shift distance by lateral offset
};

/// @brief Shift a centerline-based trajectory so that it starts from ego position+yaw
///        and smoothly transitions to the centerline using spline interpolation.
/// @param trajectory The centerline-based trajectory to shift
/// @param ego_pose The current ego pose (position + orientation)
/// @param params Trajectory shift parameters
/// @param delta_arc_length Resampling interval [m]
/// @return The shifted trajectory, or the original trajectory if shifting is not needed or fails
autoware_planning_msgs::msg::Trajectory shift_trajectory_to_ego(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, const TrajectoryShiftParams & params,
  double delta_arc_length);

}  // namespace autoware::minimum_rule_based_planner

#endif  // PATH_SHIFT_TO_EGO_HPP_
