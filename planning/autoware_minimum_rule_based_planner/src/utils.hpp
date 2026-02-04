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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "minimum_rule_based_planner.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <optional>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using PathPointTrajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

struct WaypointGroup
{
  struct Waypoint
  {
    lanelet::ConstPoint3d point;
    lanelet::Id lane_id;
  };

  struct Interval
  {
    double start;
    double end;
  };

  std::vector<Waypoint> waypoints;
  Interval interval;
};

template <typename T>
struct PathRange
{
  T left;
  T right;
};

namespace utils
{

/**
 * @brief get lanelets within route that are in specified distance backward from target lanelet
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_up_to(
  const lanelet::ConstLanelet & lanelet, const InternalPlannerData & planner_data,
  const double distance);

/**
 * @brief get lanelets within route that are in specified distance forward from target lanelet
 */
std::optional<lanelet::ConstLanelets> get_lanelets_within_route_after(
  const lanelet::ConstLanelet & lanelet, const InternalPlannerData & planner_data,
  const double distance);

/**
 * @brief get previous lanelet within route
 */
std::optional<lanelet::ConstLanelet> get_previous_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const InternalPlannerData & planner_data);

/**
 * @brief get next lanelet within route
 */
std::optional<lanelet::ConstLanelet> get_next_lanelet_within_route(
  const lanelet::ConstLanelet & lanelet, const InternalPlannerData & planner_data);

/**
 * @brief generate path from lanelet sequence centerline
 */
PathWithLaneId generate_centerline_path(
  const lanelet::ConstLanelets & lanelets, const InternalPlannerData & planner_data,
  const geometry_msgs::msg::Pose & current_pose, const double s_start, const double s_end);

/**
 * @brief get waypoints in lanelet sequence and group them
 */
std::vector<WaypointGroup> get_waypoint_groups(
  const lanelet::LaneletSequence & lanelet_sequence, const lanelet::LaneletMap & lanelet_map,
  const double group_separation_threshold, const double interval_margin_ratio);

/**
 * @brief get position of first intersection (including self-intersection) in lanelet sequence
 */
std::optional<double> get_first_intersection_arc_length(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const double vehicle_length);

/**
 * @brief get position of first self-intersection of line string in arc length
 */
std::optional<double> get_first_self_intersection_arc_length(
  const lanelet::BasicLineString2d & line_string);

/**
 * @brief get position of given point on centerline projected to path in arc length
 */
double get_arc_length_on_path(
  const lanelet::LaneletSequence & lanelet_sequence, const std::vector<PathPointWithLaneId> & path,
  const double s_centerline);

/**
 * @brief get path bounds for PathWithLaneId cropped within specified range
 */
PathRange<std::vector<geometry_msgs::msg::Point>> get_path_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end);

/**
 * @brief crop line string
 */
std::vector<geometry_msgs::msg::Point> crop_line_string(
  const std::vector<geometry_msgs::msg::Point> & line_string, const double s_start,
  const double s_end);

/**
 * @brief get positions of given point on centerline projected to left / right bound in arc length
 */
PathRange<double> get_arc_length_on_bounds(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_centerline);

/**
 * @brief get positions of given point on left / right bound projected to centerline in arc length
 */
PathRange<std::optional<double>> get_arc_length_on_centerline(
  const lanelet::LaneletSequence & lanelet_sequence, const std::optional<double> & s_left_bound,
  const std::optional<double> & s_right_bound);

/**
 * @brief Recreate the path with a given goal pose
 */
PathPointTrajectory refine_path_for_goal(
  const PathPointTrajectory & input, const geometry_msgs::msg::Pose & goal_pose,
  const lanelet::Id goal_lane_id, const double search_radius_range, const double pre_goal_offset);

/**
 * @brief Extract lanelets from the trajectory
 */
lanelet::ConstLanelets extract_lanelets_from_trajectory(
  const PathPointTrajectory & trajectory, const InternalPlannerData & planner_data);

/**
 * @brief Check if the pose is in the lanelets
 */
bool is_in_lanelets(const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & lanes);

/**
 * @brief Check if the trajectory is inside the lanelets
 */
bool is_trajectory_inside_lanelets(
  const PathPointTrajectory & refined_path, const lanelet::ConstLanelets & lanelets);

/**
 * @brief Modify path for smooth goal connection
 */
std::optional<PathPointTrajectory> modify_path_for_smooth_goal_connection(
  const PathPointTrajectory & trajectory, const InternalPlannerData & planner_data,
  const double search_radius_range, const double pre_goal_offset);

/**
 * @brief Convert PathWithLaneId to Trajectory with resampling
 */
autoware_planning_msgs::msg::Trajectory convert_path_to_trajectory(
  const PathWithLaneId & path, double resample_interval);

}  // namespace utils
}  // namespace autoware::minimum_rule_based_planner

#endif  // UTILS_HPP_
