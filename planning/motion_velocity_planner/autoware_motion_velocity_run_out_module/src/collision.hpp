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

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief calculate a collision from the time when the trajectories of ego and an object overlap
/// @return a Collision object with the collision type and predicted collision time
Collision calculate_collision(
  const TimeOverlapInterval & ego, const TimeOverlapInterval & object, const Parameters & params);

/// @brief calculate a collision when the ego and object are predicted to overlap at the same time
void calculate_overlapping_collision(
  Collision & c, const TimeOverlapInterval & ego, const TimeOverlapInterval & object,
  const Parameters & params);

/// @brief Calculates collisions based on overlap intervals.
///
/// @param[in] intervals A vector of time collision interval pairs.
/// @param[in] params The parameters for collision calculation.
/// @return A vector of collisions.
std::vector<Collision> calculate_interval_collisions(
  std::vector<TimeOverlapIntervalPair> intervals, const Parameters & params);

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
/// @param[in] filtering_data Data containing the polygons where collisions should be ignored
/// @param[in] min_arc_length [m] minimum arc length along the ego trajectory where collisions are
/// considered
/// @param[in] params The parameters for collision calculation.
/// @return A vector of collisions.
std::vector<Collision> calculate_footprint_collisions(
  const TrajectoryCornerFootprint & ego_footprint,
  const ObjectPredictedPathFootprint & object_footprint, const FilteringData & filtering_data,
  const double min_arc_length, const Parameters & params);

/// @brief Calculates collisions between the ego vehicle's trajectory and a predicted object.
/// @param[inout] object A predicted object.  Collisions are added to each object.
/// @param[in] ego_footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] filtering_data Data containing the polygons where collisions should be ignored
/// @param[in] min_arc_length [m] minimum arc length where collisions are considered
/// @param[in] params The parameters for collision calculation.
void calculate_object_collisions(
  std::vector<Object> & objects, const TrajectoryCornerFootprint & ego_footprint,
  const FilteringDataPerLabel & filtering_data, const double min_arc_length,
  const Parameters & params);

/// @brief Calculates collisions between the ego vehicle's trajectory and a set of predicted
/// objects.
/// @param[inout] objects A vector of predicted objects.  Collisions are added to each object.
/// @param[in] ego_footprint The footprint of the ego vehicle's trajectory.
/// @param[in] ego_trajectory The ego vehicle's trajectory.
/// @param[in] filtering_data Data containing the polygons where collisions should be ignored
/// @param[in] min_arc_length [m] minimum arc length where collisions are considered
/// @param[in] params The parameters for collision calculation.
void calculate_collisions(
  std::vector<Object> & objects, const TrajectoryCornerFootprint & ego_footprint,
  const FilteringDataPerLabel & filtering_data, const double min_arc_length,
  const Parameters & params);
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // COLLISION_HPP_
