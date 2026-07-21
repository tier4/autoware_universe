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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__GEOMETRY_UTILS_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__GEOMETRY_UTILS_HPP_

#include <autoware/lanelet2_utils/intersection.hpp>
#include <autoware_lane_event_classifier/types.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

namespace autoware::lane_event_classifier
{

/**
 * @brief Returns the vehicle footprint polygon in the map frame, or a single centre point when
 * vehicle info is unavailable.
 * @param input Per-cycle input providing the ego pose and optional vehicle info.
 */
inline std::vector<lanelet::BasicPoint2d> compute_footprint(const LaneEventInput & input)
{
  const auto & pose = input.odometry_ptr->pose.pose;

  if (!input.vehicle_info_ptr) {
    return {{pose.position.x, pose.position.y}};
  }

  const auto map_footprint = autoware_utils_geometry::transform_vector(
    input.vehicle_info_ptr->createFootprint(), autoware_utils_geometry::pose2transform(pose));

  std::vector<lanelet::BasicPoint2d> map_footprint_2d;
  map_footprint_2d.reserve(map_footprint.size());
  std::transform(
    map_footprint.cbegin(), map_footprint.cend(), std::back_inserter(map_footprint_2d),
    [](const auto & footprint_point) {
      return lanelet::BasicPoint2d{footprint_point.x(), footprint_point.y()};
    });
  return map_footprint_2d;
}

/**
 * @brief Returns true only if every footprint corner lies inside the lanelet.
 * @param lane Lanelet to test against.
 * @param footprint Footprint corners in the map frame.
 */
inline bool is_footprint_fully_inside_lane(
  const lanelet::ConstLanelet & lane, const std::vector<lanelet::BasicPoint2d> & footprint)
{
  return std::all_of(
    footprint.cbegin(), footprint.cend(), [&lane](const lanelet::BasicPoint2d & footprint_corner) {
      return lanelet::geometry::inside(lane, footprint_corner);
    });
}

/**
 * @brief Returns true if the boundary linestring is virtual (attribute type == "virtual").
 * @param bound Boundary linestring to test.
 */
inline bool is_virtual_linestring(const lanelet::ConstLineString3d & bound)
{
  // std::string default: a const char* default would make Attribute::as<> compare by pointer.
  return bound.attributeOr(lanelet::AttributeName::Type, std::string{}) ==
         lanelet::AttributeValueString::Virtual;
}

/**
 * @brief Returns true if the lane is a turn-direction ("left"/"right") or intersection lanelet.
 * @param lane Lanelet to test.
 *
 * Shared onset exemption for both the lane-change and lane-crossing classifiers: turning out of a
 * turn/intersection lane is never a lane event.
 */
inline bool is_turn_direction_lane(const lanelet::ConstLanelet & lane)
{
  if (autoware::experimental::lanelet2_utils::is_intersection_lanelet(lane)) {
    return true;
  }
  const auto turn_direction = lane.attributeOr("turn_direction", std::string{});
  return turn_direction == "left" || turn_direction == "right";
}

/**
 * @brief Ordered trajectory points from the one nearest the ego, forward, until the accumulated arc
 * length reaches look_ahead_m.
 * @param trajectory Planned trajectory.
 * @param ego Ego reference point in the map frame.
 * @param look_ahead_m Arc length ahead to keep.
 *
 * Shared by the lane-change and lane-crossing geometry layers, whose onset is predictive over the
 * planned trajectory.
 */
inline std::vector<lanelet::BasicPoint2d> forward_trajectory_points(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const lanelet::BasicPoint2d & ego,
  double look_ahead_m)
{
  std::vector<lanelet::BasicPoint2d> points;
  if (trajectory.points.empty()) {
    return points;
  }
  const auto point_of = [](const auto & trajectory_point) {
    return lanelet::BasicPoint2d{
      trajectory_point.pose.position.x, trajectory_point.pose.position.y};
  };
  const auto find_nearest_index = [&]() {
    std::size_t nearest_index = 0;
    double nearest_distance_sq = std::numeric_limits<double>::max();
    for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
      const double distance_sq =
        (std::invoke(point_of, trajectory.points[index]) - ego).squaredNorm();
      if (distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_index = index;
      }
    }
    return nearest_index;
  };

  const std::size_t nearest_index = std::invoke(find_nearest_index);
  points.reserve(trajectory.points.size() - nearest_index);
  lanelet::BasicPoint2d previous = std::invoke(point_of, trajectory.points[nearest_index]);
  points.push_back(previous);
  double accumulated_length = 0.0;
  for (std::size_t index = nearest_index + 1; index < trajectory.points.size(); ++index) {
    const lanelet::BasicPoint2d current = std::invoke(point_of, trajectory.points[index]);
    accumulated_length += (current - previous).norm();
    points.push_back(current);
    previous = current;
    if (accumulated_length >= look_ahead_m) {
      break;
    }
  }
  return points;
}

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__GEOMETRY_UTILS_HPP_
