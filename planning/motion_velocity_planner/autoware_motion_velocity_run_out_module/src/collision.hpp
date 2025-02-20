// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef COLLISION_HPP_
#define COLLISION_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <limits>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

/// @brief calculate the rtree for segments of a trajectory
inline SegmentRtree prepare_trajectory_footprint_rtree(const TrajectoryCornerFootprint & footprint)
{
  SegmentRtree rtree;
  std::vector<SegmentNode> nodes;
  for (const auto & ls :
       {footprint.corner_footprint.front_left_ls, footprint.corner_footprint.front_right_ls,
        footprint.corner_footprint.rear_left_ls, footprint.corner_footprint.rear_right_ls}) {
    for (auto i = 0UL; i + 1 < ls.size(); ++i) {
      nodes.emplace_back(universe_utils::Segment2d{ls[i], ls[i + 1]}, i);
    }
  }
  return SegmentRtree(nodes);
}

inline FootprintIntersection calculate_footprint_intersection(
  const universe_utils::Segment2d & object_segment,
  const universe_utils::Point2d & intersection_point, const SegmentNode & ego_query_result,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const std::pair<double, double> object_segment_times)
{
  const auto ego_trajectory_index = ego_query_result.second;
  const auto & ego_segment = ego_query_result.first;
  FootprintIntersection footprint_intersection;
  footprint_intersection.intersection = intersection_point;
  const auto & ego_traj_from = ego_trajectory[ego_trajectory_index];
  const auto & ego_traj_to = ego_trajectory[ego_trajectory_index + 1];

  const auto object_segment_length = boost::geometry::length(object_segment);
  const auto object_segment_intersection_ratio =
    object_segment_length > 1e-3
      ? boost::geometry::distance(object_segment.first, intersection_point) / object_segment_length
      : 0.0;
  const auto ego_segment_length = boost::geometry::length(ego_segment);
  const auto ego_segment_intersection_ratio =
    ego_segment_length > 1e-3
      ? universe_utils::calcDistance2d(ego_segment.first, intersection_point) / ego_segment_length
      : 0.0;

  footprint_intersection.ego_time = interpolation::lerp(
    rclcpp::Duration(ego_traj_from.time_from_start).seconds(),
    rclcpp::Duration(ego_traj_to.time_from_start).seconds(),
    static_cast<double>(ego_segment_intersection_ratio));
  footprint_intersection.object_time = interpolation::lerp(
    object_segment_times.first, object_segment_times.second,
    static_cast<double>(object_segment_intersection_ratio));
  return footprint_intersection;
}

inline std::vector<std::vector<FootprintIntersection>>
calculate_footprint_intersections_per_predicted_path(
  const SegmentRtree & footprint_rtree, const Object & object,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory)
{
  std::vector<std::vector<FootprintIntersection>> intersections_per_predicted_path;
  for (const auto & corner_footprint : object.corner_footprints) {
    const auto & footprint = corner_footprint.corner_footprint;
    std::vector<FootprintIntersection> intersections;
    for (auto i = 0UL; i + 1 < footprint.size(); ++i) {
      universe_utils::Segment2d segment;
      for (const auto & corner_ls : {
             footprint.front_left_ls,
             footprint.front_right_ls,
             footprint.rear_right_ls,
             footprint.rear_left_ls,
           }) {
        segment.first = corner_ls[i];
        segment.second = corner_ls[i + 1];
        std::vector<SegmentNode> query_results;
        footprint_rtree.query(
          boost::geometry::index::intersects(segment), std::back_inserter(query_results));
        for (const auto & query_result : query_results) {
          const auto intersection = universe_utils::intersect(
            segment.first, segment.second, query_result.first.first, query_result.first.second);
          intersections.push_back(calculate_footprint_intersection(
            segment, *intersection, query_result, ego_trajectory,
            {corner_footprint.time_step * static_cast<double>(i),
             corner_footprint.time_step * (static_cast<double>(i) + 1)}));
        }
      }
    }
    // TODO(Maxime): group intersections in groups of 2 (enter / exit), possibly starting/ending
    // with a group of 1 (e.g., 1st point is already "inside") currently we get issues with the same
    // linestring intersects different parts of the ego trajectory (or we can just set a maximum
    // duration between intersections)
    intersections_per_predicted_path.push_back(intersections);
  }
  return intersections_per_predicted_path;
}

inline Collision calculate_collision(
  const std::vector<FootprintIntersection> & intersections, const Parameters & params)
{
  FootprintIntersection first_ego_intersection;
  first_ego_intersection.ego_time = std::numeric_limits<double>::max();
  FootprintIntersection last_ego_intersection;
  last_ego_intersection.ego_time = 0.0;
  FootprintIntersection first_object_intersection;
  first_object_intersection.object_time = std::numeric_limits<double>::max();
  FootprintIntersection last_object_intersection;
  last_object_intersection.object_time = 0.0;
  for (const auto & i : intersections) {
    if (i.ego_time < first_ego_intersection.ego_time) {
      first_ego_intersection = i;
    }
    if (i.ego_time > last_ego_intersection.ego_time) {
      last_ego_intersection = i;
    }
    if (i.object_time < first_object_intersection.object_time) {
      first_object_intersection = i;
    }
    if (i.object_time > last_object_intersection.object_time) {
      last_object_intersection = i;
    }
  }
  return {
    TimeCollisionInterval(
      first_ego_intersection.ego_time, last_ego_intersection.ego_time, first_ego_intersection,
      last_ego_intersection),
    TimeCollisionInterval(
      first_object_intersection.object_time, last_object_intersection.object_time,
      first_object_intersection, last_object_intersection),
    params};
}

inline void calculate_collisions(
  std::vector<Object> & objects, const run_out::SegmentRtree & ego_footprint_rtree,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const run_out::Parameters & params)
{
  for (auto & object : objects) {
    const auto intersections_per_predicted_path =
      run_out::calculate_footprint_intersections_per_predicted_path(
        ego_footprint_rtree, object, ego_trajectory);
    for (const auto & intersections : intersections_per_predicted_path) {
      if (intersections.empty()) {
        continue;
      }
      object.collisions.push_back(run_out::calculate_collision(intersections, params));
    }
  }
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // COLLISION_HPP_
