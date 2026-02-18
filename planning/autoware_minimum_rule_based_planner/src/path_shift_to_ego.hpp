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

#include <vector>

namespace autoware::minimum_rule_based_planner
{

struct TrajectoryShiftParams
{
  double minimum_shift_length{0.1};    // [m] lateral offset threshold to trigger shift
  double minimum_shift_yaw{0.1};      // [rad] yaw deviation threshold to trigger shift
  double minimum_shift_distance{5.0};  // [m] floor for shift distance

  // LUT for velocity-dependent parameters.
  // velocity_breakpoints must be sorted in ascending order.
  // No extrapolation: values are clamped at the table boundaries.
  std::vector<double> velocity_breakpoints;                // [m/s]
  std::vector<double> shift_length_to_distance_ratio_table;  // [-] ratio per velocity breakpoint
  std::vector<double> guide_distance_table;                // [m] guide distance per velocity breakpoint
};

/// @brief Look up a value from a 1-D LUT (piecewise-linear interpolation, no extrapolation).
double lookup_table(
  const std::vector<double> & breakpoints, const std::vector<double> & values, double query);

/// @brief Shift a centerline-based trajectory so that it starts from ego position+yaw
///        and smoothly transitions to the centerline using spline interpolation.
/// @param trajectory The centerline-based trajectory to shift
/// @param ego_pose The current ego pose (position + orientation)
/// @param ego_velocity The current ego velocity [m/s]
/// @param params Trajectory shift parameters
/// @param delta_arc_length Resampling interval [m]
/// @return The shifted trajectory, or the original trajectory if shifting is not needed or fails
autoware_planning_msgs::msg::Trajectory shift_trajectory_to_ego(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, double ego_velocity,
  const TrajectoryShiftParams & params, double delta_arc_length);

/// @brief Shift a centerline-based trajectory so that it starts from ego position+yaw
///        and smoothly transitions to the centerline using a quintic polynomial.
autoware_planning_msgs::msg::Trajectory shift_trajectory_to_ego_quintic(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const geometry_msgs::msg::Pose & ego_pose, double ego_velocity, const double ego_yaw_rate,
  const TrajectoryShiftParams & params, double delta_arc_length);

}  // namespace autoware::minimum_rule_based_planner

#endif  // PATH_SHIFT_TO_EGO_HPP_
