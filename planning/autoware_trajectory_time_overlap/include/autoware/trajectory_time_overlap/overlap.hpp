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

#ifndef AUTOWARE__TRAJECTORY_TIME_OVERLAP__OVERLAP_HPP_
#define AUTOWARE__TRAJECTORY_TIME_OVERLAP__OVERLAP_HPP_

#include "autoware/trajectory_time_overlap/types.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::trajectory_time_overlap
{
/// @brief calculate the footprint intersection information between an object segment and an ego
/// vehicle footprint segment This function determines the intersection point between a segment of
/// the predicted object's path and a segment of the ego vehicle's trajectory footprint.  It
/// calculates the times of the intersection for both the ego vehicle and the predicted object, as
/// well as the position of the intersection relative to the ego vehicle's footprint.
FootprintIntersection calculate_footprint_intersection(
  const universe_utils::Segment2d & object_segment,
  const universe_utils::Point2d & intersection_point, const FootprintSegmentNode & ego_query_result,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & ego_trajectory,
  const std::pair<double, double> object_segment_times);

/// @brief calculate the interpolated trajectory point closest to the given position
/// @details the pose (position + orientation), time from start, and velocities are interpolated
/// @param [in] trajectory trajectory with time information
/// @param [in] p target position
/// @param [in] longitudinal_offset [m] arc length offset for which to calculate the interpolation
/// (default to 0)
std::pair<autoware_planning_msgs::msg::TrajectoryPoint, double>
calculate_closest_interpolated_point_and_arc_length(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const universe_utils::Point2d & p, const double longitudinal_offset = 0.0);

/// @brief Calculate the intersection between the end point of a linestring and the ego vehicle's
/// trajectory footprint
/// @param[in] check_front If true, the first point of the linestring is checked, otherwise the last
/// point is checked
std::optional<FootprintIntersection> calculate_end_point_intersection(
  const universe_utils::LineString2d & ls, const TrajectoryCornerFootprint & footprint,
  const double ls_time_step, const bool check_front);

/// @brief Calculates intersections between a predicted object's outline linestring and the ego
/// vehicle's trajectory footprint.
std::vector<FootprintIntersection> calculate_intersections(
  const universe_utils::LineString2d & ls, const TrajectoryCornerFootprint & footprint,
  const double ls_time_step);

/// @brief group the intersections into overlap intervals of increasing object time
/// @param intersections intersection points calculated between one object footprint linestring and
/// the ego footprint
/// @return the corresponding overlap time intervals for both ego and the object
std::vector<TimeOverlapIntervalPair> calculate_overlap_intervals(
  std::vector<FootprintIntersection> intersections);

/// @brief merge interval pairs whose object times overlap, including the given tolerance
/// @details a merged pair has exited only when every merged pair exited
std::vector<TimeOverlapIntervalPair> combine_time_overlap_intervals(
  std::vector<TimeOverlapIntervalPair> intervals, const double collision_time_overlap_tolerance);

/// @brief overlap intervals between the ego footprint and every line of an object outline
std::vector<TimeOverlapIntervalPair> calculate_ego_and_object_time_overlap_intervals(
  const TrajectoryCornerFootprint & ego_footprint, const ObjectPathFootprint & object_footprint);

/// @brief true when every interval is a real exit that finishes before ego enters it
/// @details empty input is false: missing geometry is not evidence that the object left
bool cleared_before_ego(
  const std::vector<TimeOverlapIntervalPair> & intervals, const double time_margin);
}  // namespace autoware::trajectory_time_overlap

#endif  // AUTOWARE__TRAJECTORY_TIME_OVERLAP__OVERLAP_HPP_
