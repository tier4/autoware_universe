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

#ifndef UTILS__REFERENCE_PATH_SMOOTHER_HPP_
#define UTILS__REFERENCE_PATH_SMOOTHER_HPP_

#include "../type_alias.hpp"

#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

//! How far a point may move to each side before the footprint reaches the bound of the lanelet it
//! lies in: {right, left}. A point outside every lanelet cannot move
std::pair<double, double> lateral_room(
  const geometry_msgs::msg::Point & position, const lanelet::ConstLanelets & lanelets,
  double vehicle_half_width_m);

//! Smooths the reference_path with an elastic band QP, formulated as in the EB of
//! autoware_path_smoother: minimize the sum of squared second differences of the points resampled
//! every 1 m, each point free to move along its normal only. The travel is limited to the smaller
//! of +-clearance_m and the distance to the lane bounds minus vehicle_half_width_m and a margin.
//! The last two points (the goal position and heading) are held fixed. Returns nullopt when the
//! QP does not solve or there are too few points.
std::optional<PathPointTrajectory> smooth_reference_path(
  const PathPointTrajectory & path, const lanelet::ConstLanelets & lanelets,
  double vehicle_half_width_m, double clearance_m);

//! Refines a lateral offset profile l_i (spacing ds) laid over a centerline of the given
//! curvature: minimizes the sum of squared second differences, keeps the value and the slope at
//! both ends of `nominal` (so the goal pose it reaches is unchanged), holds the curvature of the
//! resulting path within max_curvature and every offset inside [lower_bound, upper_bound].
//! Returns nullopt when the QP does not solve, which is also how an infeasible problem reports
//! itself.
std::optional<std::vector<double>> optimize_goal_shift(
  const std::vector<double> & nominal, const std::vector<double> & centerline_curvature, double ds,
  double max_curvature, const std::vector<double> & lower_bound,
  const std::vector<double> & upper_bound);

}  // namespace autoware::safety_planner

#endif  // UTILS__REFERENCE_PATH_SMOOTHER_HPP_
