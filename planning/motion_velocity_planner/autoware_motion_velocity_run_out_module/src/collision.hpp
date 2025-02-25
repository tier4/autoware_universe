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
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

inline FootprintIntersection calculate_footprint_intersection(
  const universe_utils::Segment2d & object_segment,
  const universe_utils::Point2d & intersection_point, const FootprintSegmentNode & ego_query_result,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const std::pair<double, double> object_segment_times)
{
  const auto ego_trajectory_index = ego_query_result.second.second;
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
  footprint_intersection.position = ego_query_result.second.first;
  return footprint_intersection;
}

inline double calculate_closest_interpolated_time(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const universe_utils::Point2d & p)
{
  geometry_msgs::msg::Point pt;
  pt.x = p.x();
  pt.y = p.y();
  const auto segment_idx = motion_utils::findNearestSegmentIndex(trajectory, pt);
  const auto segment_length =
    universe_utils::calcDistance2d(trajectory[segment_idx], trajectory[segment_idx + 1]);
  const auto point_distance = universe_utils::calcDistance2d(trajectory[segment_idx], pt);
  return interpolation::lerp(
    rclcpp::Duration(trajectory[segment_idx].time_from_start).seconds(),
    rclcpp::Duration(trajectory[segment_idx + 1].time_from_start).seconds(),
    point_distance / segment_length);
}

inline std::vector<FootprintIntersection> calculate_intersections(
  const universe_utils::LineString2d & ls, const TrajectoryCornerFootprint & footprint,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const double ls_time_step)
{
  std::vector<FootprintIntersection> intersections;
  if (ls.empty()) {
    return intersections;
  }
  const auto front_inside_front_polygon =
    boost::geometry::within(ls.front(), footprint.front_polygon);
  const auto front_inside_rear_polygon =
    boost::geometry::within(ls.front(), footprint.rear_polygon);
  if (front_inside_front_polygon || front_inside_rear_polygon) {
    // special case where 1st point is already inside the footprint
    FootprintIntersection fi;
    fi.intersection = ls.front();
    fi.ego_time = calculate_closest_interpolated_time(ego_trajectory, ls.front());
    fi.object_time = 0.0;
    if (front_inside_front_polygon && front_inside_rear_polygon)
      fi.position = inside_both_polygons;
    else if (front_inside_front_polygon)
      fi.position = inside_front_polygon;
    else
      fi.position = inside_rear_polygon;
    intersections.push_back(fi);
  }
  for (auto i = 0UL; i + 1 < ls.size(); ++i) {
    universe_utils::Segment2d segment;
    segment.first = ls[i];
    segment.second = ls[i + 1];
    std::vector<FootprintSegmentNode> query_results;
    footprint.rtree.query(
      boost::geometry::index::intersects(segment), std::back_inserter(query_results));
    for (const auto & query_result : query_results) {
      const auto intersection = universe_utils::intersect(
        segment.first, segment.second, query_result.first.first, query_result.first.second);
      if (intersection) {
        const auto footprint_intersection = calculate_footprint_intersection(
          segment, *intersection, query_result, ego_trajectory,
          {ls_time_step * static_cast<double>(i), ls_time_step * (static_cast<double>(i) + 1)});
        intersections.push_back(footprint_intersection);
      }
    }
  }
  const auto back_inside_front_polygon =
    boost::geometry::within(ls.back(), footprint.front_polygon);
  const auto back_inside_rear_polygon = boost::geometry::within(ls.back(), footprint.rear_polygon);
  if (back_inside_front_polygon || back_inside_rear_polygon) {
    // special case where last point is still inside the footprint
    FootprintIntersection fi;
    fi.intersection = ls.back();
    fi.ego_time = calculate_closest_interpolated_time(ego_trajectory, ls.back());
    fi.object_time = (static_cast<double>(ls.size()) - 1.0) * ls_time_step;
    if (back_inside_front_polygon && back_inside_rear_polygon)
      fi.position = inside_both_polygons;
    else if (back_inside_front_polygon)
      fi.position = inside_front_polygon;
    else
      fi.position = inside_rear_polygon;
    intersections.push_back(fi);
  }
  return intersections;
}

struct TimeCollisionIntervalPair
{
  TimeCollisionInterval ego;
  TimeCollisionInterval object;

  TimeCollisionIntervalPair(TimeCollisionInterval e, TimeCollisionInterval o)
  : ego(std::move(e)), object(std::move(o))
  {
  }
};

/// @brief group the intersections into overlap intervals of increasing object time
inline std::vector<TimeCollisionIntervalPair> calculate_overlap_intervals(
  std::vector<FootprintIntersection> intersections)
{
  std::vector<TimeCollisionIntervalPair> overlap_intervals;
  if (intersections.empty()) {
    return overlap_intervals;
  }
  std::sort(
    intersections.begin(), intersections.end(),
    [&](const FootprintIntersection & fi1, const FootprintIntersection & fi2) {
      return fi1.object_time < fi2.object_time;
    });
  std::unordered_map<IntersectionPosition, bool> already_entered;
  // TODO(Maxime): edge case when crossing rear / front segments ?
  const auto & first_position = intersections.front().position;
  if (intersections.front().position == inside_both_polygons) {
    const auto already_entered_front =
      first_position == inside_both_polygons || first_position == inside_front_polygon;
    const auto already_entered_rear =
      first_position == inside_both_polygons || first_position == inside_rear_polygon;
    already_entered[front_left] = already_entered_front;
    already_entered[front_right] = already_entered_front;
    already_entered[rear_left] = already_entered_rear;
    already_entered[rear_right] = already_entered_rear;
  }
  const auto create_overlap = [&](const size_t entering_id, const size_t exiting_id) {
    TimeCollisionInterval object_interval(
      intersections[entering_id].object_time, intersections[exiting_id].object_time,
      intersections[entering_id], intersections[exiting_id]);
    auto ego_entering = intersections[entering_id];
    auto ego_exiting = intersections[exiting_id];
    for (auto i = entering_id + 1; i <= exiting_id; ++i) {
      const auto & intersection = intersections[i];
      if (intersection.ego_time < ego_entering.ego_time) {
        ego_entering = intersection;
      }
      if (intersection.ego_time > ego_exiting.ego_time) {
        ego_exiting = intersection;
      }
    }
    TimeCollisionInterval ego_interval(
      ego_entering.ego_time, ego_exiting.ego_time, ego_entering, ego_exiting);
    overlap_intervals.emplace_back(ego_interval, object_interval);
  };
  size_t entering_intersection_id = 0UL;
  for (auto i = 1UL; i < intersections.size(); ++i) {
    const auto & intersection = intersections[i];
    if (already_entered[intersection.position]) {
      create_overlap(entering_intersection_id, i);
      entering_intersection_id = i + 1;
    } else {
      already_entered[intersection.position] = true;
    }
  }
  if (entering_intersection_id < intersections.size()) {
    create_overlap(entering_intersection_id, intersections.size() - 1);
  }
  return overlap_intervals;
}

inline std::vector<TimeCollisionIntervalPair> calculate_overlap_intervals(
  const TrajectoryCornerFootprint & ego_footprint, const ObjectCornerFootprint & object_footprint,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory)
{
  std::vector<TimeCollisionIntervalPair> overlap_intervals;
  const auto & footprint = object_footprint.corner_footprint;
  for (const auto & corner_ls : footprint.corner_linestrings) {
    const std::vector<FootprintIntersection> footprint_intersections =
      calculate_intersections(corner_ls, ego_footprint, ego_trajectory, object_footprint.time_step);
    const auto intervals = calculate_overlap_intervals(footprint_intersections);
    overlap_intervals.insert(overlap_intervals.end(), intervals.begin(), intervals.end());
  }
  return overlap_intervals;
}

inline std::vector<Collision> calculate_collisions(
  std::vector<TimeCollisionIntervalPair> intervals, const Parameters & params)
{
  std::vector<Collision> collisions;
  if (intervals.empty()) {
    return collisions;
  }
  std::sort(
    intervals.begin(), intervals.end(),
    [&](const TimeCollisionIntervalPair & i1, const TimeCollisionIntervalPair & i2) {
      return i1.object.from < i2.object.from;
    });
  TimeCollisionInterval object_combined = intervals.front().object;
  TimeCollisionInterval ego_combined = intervals.front().ego;
  for (const auto & interval : intervals) {
    if (interval.object.succeeds(object_combined)) {
      collisions.emplace_back(ego_combined, object_combined, params);
      object_combined = interval.object;
      ego_combined = interval.ego;
    } else if (interval.object.overlaps(object_combined)) {
      object_combined.expand(interval.object);
      ego_combined.expand(interval.ego);
    } else {
      throw(std::runtime_error("encountered a preceding interval"));
    }
  }
  return collisions;
}

inline void calculate_collisions(
  std::vector<Object> & objects, const TrajectoryCornerFootprint & ego_footprint,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const Parameters & params)
{
  for (auto & object : objects) {
    for (const auto & corner_footprint : object.corner_footprints) {
      const auto overlap_intervals =
        calculate_overlap_intervals(ego_footprint, corner_footprint, ego_trajectory);
      const auto collisions = calculate_collisions(overlap_intervals, params);
      object.collisions.insert(object.collisions.end(), collisions.begin(), collisions.end());
    }
  }
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // COLLISION_HPP_
