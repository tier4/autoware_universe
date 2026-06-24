// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__BOUNDARY_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__BOUNDARY_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>

#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{

using autoware::route_handler::RouteHandler;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;

/** Drivable area represented by left and right boundary polylines. */
struct DrivableAreaResult
{
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;
};

/**
 * @brief Create a drivable area from route lanelets covered by the trajectory.
 * @details For each trajectory point, a vehicle footprint is checked against route lanelets.
 *          Neighbor lanelets are expanded when any overlapping footprint intersects them.
 *          Only lanelet segments where the trajectory lies are included.
 * @param trajectory Reference trajectory used for lanelet overlap and expansion.
 * @param vehicle_info Vehicle dimensions for footprint calculation.
 * @param route Route used to relax route-lanelet filtering near the goal.
 * @param route_handler Route handler with map and route set.
 * @param routing_graph Cached routing graph built with goal_purpose traffic rules.
 * @return Drivable area if generation succeeds, otherwise std::nullopt.
 */
std::optional<DrivableAreaResult> create_drivable_area(
  const Trajectory & trajectory, const VehicleInfo & vehicle_info, const LaneletRoute & route,
  const RouteHandler & route_handler, const lanelet::routing::RoutingGraph & routing_graph);

/**
 * @brief Convert a drivable area result into an autoware_planning_msgs/Path for publishing.
 * @param area Generated left/right bounds.
 * @param trajectory Reference trajectory copied into the path centerline.
 * @return Path message with left_bound and right_bound populated.
 */
Path to_path_msg(const DrivableAreaResult & area, const Trajectory & trajectory);

/**
 * @brief Convert left/right lanelet bounds into a drivable area result.
 * @param bounds Left and right boundary linestrings.
 * @param header Header copied into the result.
 * @return Drivable area with left_bound and right_bound populated.
 */
DrivableAreaResult to_drivable_area_result(
  const std::pair<lanelet::LineString2d, lanelet::LineString2d> & bounds,
  const std_msgs::msg::Header & header);

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__BOUNDARY_HPP_
