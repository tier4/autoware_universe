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

#ifndef AUTOWARE__TRAJECTORY_TIME_OVERLAP__FOOTPRINTS_HPP_
#define AUTOWARE__TRAJECTORY_TIME_OVERLAP__FOOTPRINTS_HPP_

#include "autoware/trajectory_time_overlap/types.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::trajectory_time_overlap
{
/// @brief Create the rtree for the segments of a trajectory
void prepare_trajectory_footprint_rtree(TrajectoryCornerFootprint & footprint);

/// @brief Calculate the corner footprint of the given trajectory
/// @param lateral_margin [m] lateral expansion of the ego footprint
/// @param longitudinal_margin [m] longitudinal expansion of the ego footprint
TrajectoryCornerFootprint calculate_trajectory_corner_footprint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double lateral_margin,
  const double longitudinal_margin);

/// @brief Sweep a rigid local outline along predicted poses
/// @param local_outline outline vertices in the object frame, without a repeated closing point
ObjectPathFootprint build_object_path_footprint(
  const std::vector<geometry_msgs::msg::Pose> & poses, const double time_step,
  const std::vector<universe_utils::Point2d> & local_outline);
}  // namespace autoware::trajectory_time_overlap

#endif  // AUTOWARE__TRAJECTORY_TIME_OVERLAP__FOOTPRINTS_HPP_
