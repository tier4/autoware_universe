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
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/normalization.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <magic_enum.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

/// @brief calculate the footprint intersection information between an object segment and an ego
/// vehicle footprint segment This function determines the intersection point between a segment of
/// the predicted object's path and a segment of the ego vehicle's trajectory footprint.  It
/// calculates the times of the intersection for both the ego vehicle and the predicted object, as
/// well as the position of the intersection relative to the ego vehicle's footprint.
///
/// @param[in] object_segment The segment of the predicted object's path (a line segment).
/// @param[in] intersection_point The point where the object segment intersects the ego vehicle's
/// footprint segment.
/// @param[in] ego_query_result The result of a spatial query on the ego vehicle's footprint,
/// containing the intersecting ego vehicle footprint segment and related data.
/// @param[in] ego_trajectory The ego vehicle's trajectory, represented as a vector of trajectory
/// points.
/// @param[in] object_segment_times The start and end times of the object segment.
/// @return A FootprintIntersection struct containing information about the intersection.
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

  const auto object_segment_length = static_cast<double>(boost::geometry::length(object_segment));
  const auto object_segment_intersection_ratio =
    object_segment_length > 1e-3
      ? boost::geometry::distance(object_segment.first, intersection_point) / object_segment_length
      : 0.0;
  const auto ego_segment_length = static_cast<double>(boost::geometry::length(ego_segment));
  const auto ego_segment_offset =
    universe_utils::calcDistance2d(ego_segment.first, intersection_point);
  footprint_intersection.arc_length =
    motion_utils::calcSignedArcLength(ego_trajectory, 0, ego_trajectory_index) + ego_segment_offset;
  const auto ego_segment_intersection_ratio =
    ego_segment_length > 1e-3 ? ego_segment_offset / ego_segment_length : 0.0;

  footprint_intersection.ego_time = interpolation::lerp(
    rclcpp::Duration(ego_traj_from.time_from_start).seconds(),
    rclcpp::Duration(ego_traj_to.time_from_start).seconds(), ego_segment_intersection_ratio);
  const auto ego_yaw = interpolation::lerp(
    tf2::getYaw(ego_traj_from.pose.orientation), tf2::getYaw(ego_traj_to.pose.orientation),
    ego_segment_intersection_ratio);
  const auto obj_yaw = std::atan2(
    object_segment.second.y() - object_segment.first.y(),
    object_segment.second.x() - object_segment.first.x());
  footprint_intersection.yaw_diff = autoware::universe_utils::normalizeRadian(ego_yaw - obj_yaw);
  footprint_intersection.ego_vel = interpolation::lerp(
    ego_traj_from.longitudinal_velocity_mps, ego_traj_to.longitudinal_velocity_mps,
    ego_segment_intersection_ratio);
  const auto obj_vel =
    object_segment_length / (object_segment_times.second - object_segment_times.first);
  footprint_intersection.vel_diff = footprint_intersection.ego_vel - obj_vel;
  footprint_intersection.object_time = interpolation::lerp(
    object_segment_times.first, object_segment_times.second, object_segment_intersection_ratio);
  footprint_intersection.position = ego_query_result.second.first;
  return footprint_intersection;
}

/// @brief calculate the interpolated trajectory point closest to the given position
/// @details the pose (position + orientation), time from start, and velocities are interpolated
/// @param [in] trajectory trajectory with time information
/// @param [in] p target position
/// @param [in] longitudinal_offset [m] arc length offset for which to calculate the interpolation
/// (default to 0)
inline std::pair<autoware_planning_msgs::msg::TrajectoryPoint, double>
calculate_closest_interpolated_point_and_arc_length(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const universe_utils::Point2d & p, const double longitudinal_offset = 0.0)
{
  autoware_planning_msgs::msg::TrajectoryPoint trajectory_point;
  geometry_msgs::msg::Point pt;
  pt.x = p.x();
  pt.y = p.y();
  const auto arc_length = motion_utils::calcSignedArcLength(trajectory, 0, pt);
  const auto offset_arc_length = arc_length - longitudinal_offset;
  const auto pose = motion_utils::calcInterpolatedPose(trajectory, offset_arc_length);
  trajectory_point.pose = pose;
  const auto segment_idx = motion_utils::findNearestSegmentIndex(trajectory, pose.position);
  const auto segment_length =
    universe_utils::calcDistance2d(trajectory[segment_idx], trajectory[segment_idx + 1]);
  const auto point_distance = universe_utils::calcDistance2d(trajectory[segment_idx], pose);
  trajectory_point.time_from_start = rclcpp::Duration::from_seconds(interpolation::lerp(
    rclcpp::Duration(trajectory[segment_idx].time_from_start).seconds(),
    rclcpp::Duration(trajectory[segment_idx + 1].time_from_start).seconds(),
    point_distance / segment_length));
  trajectory_point.longitudinal_velocity_mps = static_cast<float>(interpolation::lerp(
    trajectory[segment_idx].longitudinal_velocity_mps,
    trajectory[segment_idx + 1].longitudinal_velocity_mps, point_distance / segment_length));
  return {trajectory_point, offset_arc_length};
}

/// @brief Calculates intersections between a predicted object's corner linestring and the ego
/// vehicle's trajectory footprint.
/// @param[in] ls The corner linestring of the predicted object.
/// @param[in] footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] ls_time_step The time step of the predicted object's linestring.
/// @return A vector of footprint intersections.
inline std::vector<FootprintIntersection> calculate_intersections(
  const universe_utils::LineString2d & ls, const TrajectoryCornerFootprint & footprint,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const double ls_time_step)
{
  std::vector<FootprintIntersection> intersections;
  if (ls.size() < 2) {
    return intersections;
  }
  const auto front_inside_front_polygon =
    boost::geometry::within(ls.front(), footprint.front_polygon);
  const auto front_inside_rear_polygon =
    boost::geometry::within(ls.front(), footprint.rear_polygon);
  if (front_inside_front_polygon || front_inside_rear_polygon) {
    FootprintIntersection fi;
    fi.intersection = ls.front();
    const auto [ego_point, arc_length] = calculate_closest_interpolated_point_and_arc_length(
      ego_trajectory, fi.intersection, footprint.max_longitudinal_offset);
    fi.arc_length = arc_length;
    fi.ego_time = rclcpp::Duration(ego_point.time_from_start).seconds();
    const auto obj_yaw = std::atan2(ls[1].y() - ls[0].y(), ls[1].x() - ls[2].x());
    fi.yaw_diff =
      autoware::universe_utils::normalizeRadian(tf2::getYaw(ego_point.pose.orientation) - obj_yaw);
    const auto obj_vel = boost::geometry::distance(ls[0], ls[1]) / ls_time_step;
    fi.ego_vel = ego_point.longitudinal_velocity_mps;
    fi.vel_diff = fi.ego_vel - obj_vel;
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
    geometry_msgs::msg::Point p1;
    p1.x = segment.first.x();
    p1.y = segment.first.y();
    geometry_msgs::msg::Point p2;
    p2.x = segment.second.x();
    p2.y = segment.second.y();
    std::vector<FootprintSegmentNode> query_results;
    footprint.rtree.query(
      boost::geometry::index::intersects(segment), std::back_inserter(query_results));
    for (const auto & query_result : query_results) {
      geometry_msgs::msg::Point p3;
      p3.x = query_result.first.first.x();
      p3.y = query_result.first.first.y();
      geometry_msgs::msg::Point p4;
      p4.x = query_result.first.second.x();
      p4.y = query_result.first.second.y();
      const auto intersection = universe_utils::intersect(
        p1, p2, p3, p4);
      if (intersection) {
        universe_utils::Point2d p(intersection->x, intersection->y);
        const auto footprint_intersection = calculate_footprint_intersection(
          segment, p, query_result, ego_trajectory,
          {ls_time_step * static_cast<double>(i), ls_time_step * (static_cast<double>(i) + 1)});
        intersections.push_back(footprint_intersection);
      }
    }
  }
  const auto back_inside_front_polygon =
    boost::geometry::within(ls.back(), footprint.front_polygon);
  const auto back_inside_rear_polygon = boost::geometry::within(ls.back(), footprint.rear_polygon);
  if (back_inside_front_polygon || back_inside_rear_polygon) {
    const auto & obj_from = ls[ls.size() - 2];
    const auto & obj_to = ls.back();
    FootprintIntersection fi;
    fi.intersection = obj_to;
    const auto [ego_point, arc_length] = calculate_closest_interpolated_point_and_arc_length(
      ego_trajectory, fi.intersection, footprint.max_longitudinal_offset);
    fi.arc_length = arc_length;
    fi.ego_time = rclcpp::Duration(ego_point.time_from_start).seconds();
    const auto obj_yaw = std::atan2(obj_to.y() - obj_from.y(), obj_to.x() - obj_from.x());
    fi.yaw_diff = autoware::universe_utils::normalizeRadian(
      tf2::getYaw(ego_point.pose.orientation) - obj_yaw, 0.0);
    fi.ego_vel = ego_point.longitudinal_velocity_mps;
    const auto obj_vel = boost::geometry::distance(obj_from, obj_to) / (ls_time_step);
    fi.vel_diff = fi.ego_vel - obj_vel;
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

/// @brief time intervals for ego and the object when their predicted paths overlap
struct TimeOverlapIntervalPair
{
  TimeOverlapInterval ego;
  TimeOverlapInterval object;

  TimeOverlapIntervalPair(TimeOverlapInterval e, TimeOverlapInterval o)
  : ego(std::move(e)), object(std::move(o))
  {
  }
};

/// @brief group the intersections into overlap intervals of increasing object time
/// @param intersections intersection points calculated between one object footprint linestring and
/// the ego footprint
/// @return the corresponding overlap time intervals for both ego and the object
inline std::vector<TimeOverlapIntervalPair> calculate_overlap_intervals(
  std::vector<FootprintIntersection> intersections)
{
  std::vector<TimeOverlapIntervalPair> overlap_intervals;
  if (intersections.empty()) {
    return overlap_intervals;
  }
  std::sort(
    intersections.begin(), intersections.end(),
    [&](const FootprintIntersection & fi1, const FootprintIntersection & fi2) {
      return fi1.object_time < fi2.object_time;
    });
  // TODO(Maxime): edge case when crossing rear / front segments ?
  const auto & first_position = intersections.front().position;
  bool overlap_front =
    (first_position == inside_front_polygon) || (first_position == inside_both_polygons);
  bool overlap_rear =
    (first_position == inside_rear_polygon) || (first_position == inside_both_polygons);
  const auto create_overlap = [&](const size_t entering_id, const size_t exiting_id) {
    TimeOverlapInterval object_interval(
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
    TimeOverlapInterval ego_interval(
      ego_entering.ego_time, ego_exiting.ego_time, ego_entering, ego_exiting);
    overlap_intervals.emplace_back(ego_interval, object_interval);
  };
  size_t entering_intersection_id = 0UL;
  for (auto i = 0UL; i < intersections.size(); ++i) {
    const auto & position = intersections[i].position;
    if (position == rear_left || position == rear_right) {
      overlap_rear = !overlap_rear;
    }
    if (position == front_left || position == front_right) {
      overlap_front = !overlap_front;
    }
    const auto is_exiting_the_overlap = !overlap_front && !overlap_rear;
    if (is_exiting_the_overlap) {
      create_overlap(entering_intersection_id, i);
      entering_intersection_id = i + 1;
    }
  }
  if (overlap_front || overlap_rear) {
    create_overlap(entering_intersection_id, intersections.size() - 1);
  }
  return overlap_intervals;
}

/// @brief calculate a collision from the time when the trajectories of ego and an object overlap
/// @return a Collision object with the collision type and predicted collision time
inline Collision calculate_collision(
  const TimeOverlapInterval & ego, const TimeOverlapInterval & object, const Parameters & params)
{
  Collision c(ego, object);
  const auto is_passing_collision =
    params.enable_passing_collisions && ego.from < object.from && ego.overlaps(object) &&
    ego.from + params.passing_collisions_time_margin < object.from &&
    ego.to - ego.from <= params.passing_max_overlap_duration;
  if (is_passing_collision) {
    c.type = pass_first_collision;
    std::stringstream ss;
    ss << std::setprecision(2) << "pass first collision since ego arrives first (" << ego.from
       << " < " << object.from << "), including with margin ("
       << params.passing_collisions_time_margin << ") and ego overlap bellow max ("
       << ego.to - ego.from << " < " << params.passing_max_overlap_duration << ")";
    c.explanation = ss.str();
  } else if (ego.overlaps(object)) {
    c.type = collision;
    c.ego_collision_time = ego.first_intersection.ego_time;
    const auto is_same_direction_collision =
      -M_PI_4 < ego.first_intersection.yaw_diff && ego.first_intersection.yaw_diff < M_PI_4;
    const auto is_opposite_direction_collision = M_PI - M_PI_4 < ego.first_intersection.yaw_diff &&
                                                 ego.first_intersection.yaw_diff < M_PI + M_PI_4;
    if (is_same_direction_collision) {
      if (ego.first_intersection.vel_diff < 0.0) {  // object is faster than ego
        c.explanation = " no collision will happen because object is faster and enter first";
        c.type = no_collision;
      } else {
        c.explanation = " v_diff = " + std::to_string(ego.first_intersection.vel_diff);
        c.type = collision;
        // adjust the collision time based on the velocity difference
        const auto time_margin =
          ego.first_intersection.ego_time - ego.first_intersection.object_time;
        const auto catchup_time =
          ego.first_intersection.ego_vel * time_margin / ego.first_intersection.vel_diff;
        c.ego_collision_time += catchup_time;
      }
    } else if (is_opposite_direction_collision) {
      c.type = collision;
      // predict time when collision would occur by finding time when arc lengths are equal
      const auto overlap_length =
        ego.last_intersection.arc_length - ego.first_intersection.arc_length;
      const auto ego_overlap_duration =
        ego.last_intersection.ego_time - ego.first_intersection.ego_time;
      const auto object_overlap_duration =
        object.last_intersection.object_time - object.first_intersection.object_time;
      const auto ego_vel = overlap_length / ego_overlap_duration;
      const auto obj_vel = overlap_length / object_overlap_duration;
      const auto collision_time_within_overlap = overlap_length / (ego_vel + obj_vel);
      c.ego_collision_time += collision_time_within_overlap;
    } else {
    }
  } else if (ego.to < object.from) {
    c.type = pass_first_no_collision;
  } else {
    c.type = no_collision;
  }
  return c;
}

/// @brief Calculates collisions based on overlap intervals.
///
/// @param[in] intervals A vector of time collision interval pairs.
/// @param[in] params The parameters for collision calculation.
/// @return A vector of collisions.
inline std::vector<Collision> calculate_interval_collisions(
  std::vector<TimeOverlapIntervalPair> intervals, const Parameters & params)
{
  std::vector<Collision> collisions;
  if (intervals.empty()) {
    return collisions;
  }
  std::sort(
    intervals.begin(), intervals.end(),
    [&](const TimeOverlapIntervalPair & i1, const TimeOverlapIntervalPair & i2) {
      return i1.object.from < i2.object.from;
    });
  TimeOverlapInterval object_combined = intervals.front().object;
  TimeOverlapInterval ego_combined = intervals.front().ego;
  for (const auto & interval : intervals) {
    if (interval.object.overlaps(object_combined, params.time_overlap_tolerance)) {
      object_combined.expand(interval.object);
      ego_combined.expand(interval.ego);
    } else {
      collisions.push_back(calculate_collision(ego_combined, object_combined, params));
      object_combined = interval.object;
      ego_combined = interval.ego;
    }
  }
  collisions.push_back(calculate_collision(ego_combined, object_combined, params));
  return collisions;
}

/// @brief Calculates collisions between the ego vehicle's trajectory footprint and a predicted
/// object's footprint.
///
/// This function iterates through the corner footprints of the predicted object, calculates
/// intersections with the ego vehicle's footprint, determines overlap intervals, and calculates
/// collisions based on those intervals.
///
/// @param[in] ego_footprint The footprint of the ego vehicle's trajectory.
/// @param[in] object_footprint The footprint of the predicted object.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] min_arc_length [m] minimum arc length along the ego trajectory where collisions are
/// considered
/// @param[in] params The parameters for collision calculation.
/// @return A vector of collisions.
inline std::vector<Collision> calculate_footprint_collisions(
  const TrajectoryCornerFootprint & ego_footprint, const ObjectCornerFootprint & object_footprint,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const Parameters & params)
{
  std::vector<TimeOverlapIntervalPair> all_overlap_intervals;
  for (const auto & corner_ls : object_footprint.corner_footprint.corner_linestrings) {
    const std::vector<FootprintIntersection> footprint_intersections =
      calculate_intersections(corner_ls, ego_footprint, ego_trajectory, object_footprint.time_step);
    const auto intervals = calculate_overlap_intervals(footprint_intersections);
    all_overlap_intervals.insert(all_overlap_intervals.end(), intervals.begin(), intervals.end());
  }
  return calculate_interval_collisions(all_overlap_intervals, params);
}

/// @brief Calculates collisions between the ego vehicle's trajectory and a set of predicted
/// objects.
/// @param[inout] objects A vector of predicted objects.  Collisions are added to each object.
/// @param[in] ego_footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] params The parameters for collision calculation.
inline void calculate_collisions(
  std::vector<Object> & objects, const TrajectoryCornerFootprint & ego_footprint,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const double min_arc_length, const Parameters & params)
{
  for (auto & object : objects) {
    for (const auto & corner_footprint : object.corner_footprints) {
      const auto collisions =
        calculate_footprint_collisions(ego_footprint, corner_footprint, ego_trajectory, params);
      for (const auto & collision : collisions) {
        const auto is_on_current_ego_pose =
          collision.object_time_interval.first_intersection.arc_length <= min_arc_length;
        const auto is_after_overlap = collision.ego_collision_time > collision.ego_time_interval.to;
        if (!is_on_current_ego_pose && !is_after_overlap) {
          object.collisions.push_back(collision);
        }
      }
    }
  }
}
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // COLLISION_HPP_
