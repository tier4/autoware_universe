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

#ifndef PLUGINS__POLYGON_UTILS_HPP_
#define PLUGINS__POLYGON_UTILS_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner::polygon_utils
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::Polygon2d;

/// @brief Create vehicle footprint polygons at each trajectory point
/// @note Simplified version of
/// autoware::motion_velocity_planner::polygon_utils::create_one_step_polygons
std::vector<Polygon2d> create_one_step_polygons(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const double lat_margin);

/// @brief Find collision point between trajectory polygons and object polygon
/// @param decimated_s_values Pre-computed arc lengths on the main trajectory for each decimated
/// point
/// @return pair of (collision_point, dist_to_collide_on_main_trajectory)
/// @note Simplified version of
/// autoware::motion_velocity_planner::polygon_utils::get_collision_point
std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const std::vector<double> & decimated_s_values, const geometry_msgs::msg::Point & obj_position,
  const Polygon2d & obj_polygon, const double x_offset_to_bumper);

}  // namespace autoware::minimum_rule_based_planner::polygon_utils

#endif  // PLUGINS__POLYGON_UTILS_HPP_
