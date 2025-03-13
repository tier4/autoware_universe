// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PATH_PROJECTION_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PATH_PROJECTION_HPP_

#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/types.hpp"

namespace autoware::behavior_path_planner::drivable_area_expansion
{
/// @brief project a point to a segment
/// @param p point to project on the segment
/// @param p1 first segment point
/// @param p2 second segment point
/// @return projected point and corresponding distance
PointDistance point_to_segment_projection(
  const Point2d & p, const Point2d & p1, const Point2d & p2);

/// @brief project a point to a line
/// @param p point to project on the line
/// @param p1 first line point
/// @param p2 second line point
/// @return projected point and corresponding distance
PointDistance point_to_line_projection(const Point2d & p, const Point2d & p1, const Point2d & p2);

/// @brief project a point to a linestring
/// @param p point to project
/// @param ls linestring
/// @return projected point, corresponding distance, and arc length along the linestring
Projection point_to_linestring_projection(const Point2d & p, const LineString2d & ls);

/// @brief calculate the normal to a vector at a given distance
/// @param p1 first vector point
/// @param p2 second vector point
/// @param dist distance
/// @return point p such that (p1,p) is orthogonal to (p1,p2) at the given distance
Point2d normal_at_distance(const Point2d & p1, const Point2d & p2, const double dist);

/// @brief interpolate between two points
/// @param a first point
/// @param b second point
/// @param ratio interpolation ratio such that 0 yields a, and 1 yields b
/// @return point interpolated between a and b as per the given ratio
Point2d lerp_point(const Point2d & a, const Point2d & b, const double ratio);

/// @brief calculate the point with distance and arc length relative to a linestring
/// @param ls reference linestring
/// @param arc_length arc length along the reference linestring of the resulting point
/// @param distance distance from the reference linestring of the resulting point
/// @return point at the distance and arc length relative to the reference linestring
Segment2d linestring_to_point_projection(
  const LineString2d & ls, const double arc_length, const double distance);

/// @brief create a sub linestring between the given arc lengths
/// @param ls input linestring
/// @param from_arc_length arc length of the first point of the sub linestring
/// @param to_arc_length arc length of the last point of the sub linestring
/// @return sub linestring
LineString2d sub_linestring(
  const LineString2d & ls, const double from_arc_length, const double to_arc_length);
}  // namespace autoware::behavior_path_planner::drivable_area_expansion

// clang-format off
#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__UTILS__DRIVABLE_AREA_EXPANSION__PATH_PROJECTION_HPP_  // NOLINT
// clang-format on
