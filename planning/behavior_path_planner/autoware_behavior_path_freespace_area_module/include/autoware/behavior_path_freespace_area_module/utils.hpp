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

#ifndef AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__UTILS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__UTILS_HPP_

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <lanelet2_core/primitives/Area.h>

#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::freespace_area_utils
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

/// @brief Re-encode a freespace path so that each point orientation represents the direction of
/// travel (not the vehicle heading). Reverse points (negative longitudinal velocity, as produced by
/// utils::convertWayPointsToPathWithLaneId) get their yaw rotated by pi, and all velocities are
/// made positive. This makes forward<->reverse transitions appear as ~180 deg yaw jumps so that the
/// downstream direction_change module's detectCuspPoints() can find them.
void encodeTravelDirectionOrientation(PathWithLaneId & path);

/// @brief Detect cusps by the same criterion as direction_change::detectCuspPoints (yaw jump beyond
/// angle_threshold_deg between consecutive points). Provided so this module can self-check
/// compatibility without depending on the direction_change package.
std::vector<size_t> detectCuspIndices(const PathWithLaneId & path, double angle_threshold_deg);

/// @brief Convert a path to a PoseArray (used to re-check obstacles with the freespace algorithm).
geometry_msgs::msg::PoseArray toPoseArray(const PathWithLaneId & path);

/// @brief Build left/right drivable bounds from an Area outer polygon relative to the path
/// direction. Each polygon vertex is classified by its signed lateral offset from the path
/// (positive = left) and each side is ordered by longitudinal position along the path.
/// Returns {left_bound, right_bound}. Robust for convex-ish areas; strongly non-convex outlines
/// may be misordered.
/// NOTE: This is a documented alternative to the drivable_margin corridor actually used by the
/// module (see README); it is kept and unit-tested for future integration work.
std::pair<std::vector<geometry_msgs::msg::Point>, std::vector<geometry_msgs::msg::Point>>
generateBoundsFromAreaPolygon(const PathWithLaneId & path, const lanelet::ConstArea & area);

}  // namespace autoware::behavior_path_planner::freespace_area_utils

#endif  // AUTOWARE__BEHAVIOR_PATH_FREESPACE_AREA_MODULE__UTILS_HPP_
