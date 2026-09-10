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

#ifndef PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__UTILS__TRAJECTORY_POLYGON_UTILS_HPP_
#define PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__UTILS__TRAJECTORY_POLYGON_UTILS_HPP_

#include <autoware/trajectory/trajectory_point.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::minimum_rule_based_planner::plugin::trajectory_polygon_utils
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::Polygon2d;

/// @brief sample the trajectory from the ego position to the goal with
/// `decimate_trajectory_step_length` (extending it behind the goal by
/// `goal_extended_trajectory_length`), then create the convex footprint polygon swept between each
/// pair of successive sampled points, expanded laterally by `lat_margin`.
/// `additional_front_outer_wheel_off_track_scale` additionally widens the outer side of each
/// footprint by that ratio of the front outer wheel off-tracking on curves.
std::vector<Polygon2d> create_one_step_polygons(
  const autoware::experimental::trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const VehicleInfo & vehicle_info, const geometry_msgs::msg::Pose & current_ego_pose,
  const double lat_margin, const bool enable_to_consider_current_pose,
  const double time_to_convergence, const double decimate_trajectory_step_length,
  const double goal_extended_trajectory_length,
  const double additional_front_outer_wheel_off_track_scale = 0.0);

}  // namespace autoware::minimum_rule_based_planner::plugin::trajectory_polygon_utils

// clang-format off
#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__UTILS__TRAJECTORY_POLYGON_UTILS_HPP_  // NOLINT
// clang-format on
