// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_searcher_base.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/lane_departure_checker/lane_departure_checker.hpp>

#include "visualization_msgs/msg/detail/marker_array__struct.hpp"
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/Forward.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner::goal_planner_utils
{
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using tier4_planning_msgs::msg::PathWithLaneId;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using Shape = autoware_perception_msgs::msg::Shape;
using Polygon2d = autoware::universe_utils::Polygon2d;

lanelet::ConstLanelets getPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance);

/*
 * @brief expand pull_over_lanes to the opposite side of drivable roads by bound_offset.
 * bound_offset must be positive regardless of left_side is true/false
 */
lanelet::ConstLanelets generateExpandedPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance, const double bound_offset);

lanelet::ConstLanelets generateBetweenEgoAndExpandedPullOverLanes(
  const lanelet::ConstLanelets & pull_over_lanes, const bool left_side,
  const geometry_msgs::msg::Pose ego_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double outer_road_offset,
  const double inner_road_offset);
PredictedObjects extractObjectsInExpandedPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance, double bound_offset, const PredictedObjects & objects);
PredictedObjects filterObjectsByLateralDistance(
  const Pose & ego_pose, const double vehicle_width, const PredictedObjects & objects,
  const double distance_thresh, const bool filter_inside);
PredictedObjects extractStaticObjectsInExpandedPullOverLanes(
  const RouteHandler & route_handler, const bool left_side, const double backward_distance,
  const double forward_distance, double bound_offset, const PredictedObjects & objects,
  const double velocity_thresh);

double calcLateralDeviationBetweenPaths(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path);
bool isReferencePath(
  const PathWithLaneId & reference_path, const PathWithLaneId & target_path,
  const double lateral_deviation_thresh);

std::optional<PathWithLaneId> cropPath(const PathWithLaneId & path, const Pose & end_pose);
PathWithLaneId cropForwardPoints(
  const PathWithLaneId & path, const size_t target_seg_idx, const double forward_length);

/**
 * @brief extend target_path by extend_length
 * @param target_path original target path to extend
 * @param reference_path reference path to extend
 * @param extend_length length to extend
 * @param remove_connected_zero_velocity flag to remove zero velocity if the last point of
 *                                       target_path has zero velocity
 * @return extended path
 */
PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const double extend_length, const bool remove_connected_zero_velocity);
/**
 * @brief extend target_path to extend_pose
 * @param target_path original target path to extend
 * @param reference_path reference path to extend
 * @param extend_pose pose to extend
 * @param remove_connected_zero_velocity flag to remove zero velocity if the last point of
 *                                       target_path has zero velocity
 * @return extended path
 */
PathWithLaneId extendPath(
  const PathWithLaneId & target_path, const PathWithLaneId & reference_path,
  const Pose & extend_pose, const bool remove_connected_zero_velocity);

std::vector<Polygon2d> createPathFootPrints(
  const PathWithLaneId & path, const double base_to_front, const double base_to_rear,
  const double width);

// debug
MarkerArray createPullOverAreaMarkerArray(
  const autoware::universe_utils::MultiPolygon2d area_polygons,
  const std_msgs::msg::Header & header, const std_msgs::msg::ColorRGBA & color, const double z);
MarkerArray createPosesMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createTextsMarkerArray(
  const std::vector<Pose> & poses, std::string && ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createGoalCandidatesMarkerArray(
  const GoalCandidates & goal_candidates, const std_msgs::msg::ColorRGBA & color);
MarkerArray createLaneletPolygonMarkerArray(
  const lanelet::CompoundPolygon3d & polygon, const std_msgs::msg::Header & header,
  const std::string & ns, const std_msgs::msg::ColorRGBA & color);
MarkerArray createNumObjectsToAvoidTextsMarkerArray(
  const GoalCandidates & goal_candidates, std::string && ns,
  const std_msgs::msg::ColorRGBA & color);
}  // namespace autoware::behavior_path_planner::goal_planner_utils

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__UTIL_HPP_
