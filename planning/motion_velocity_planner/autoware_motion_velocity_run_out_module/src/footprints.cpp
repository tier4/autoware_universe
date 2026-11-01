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

#include "footprints.hpp"

#include "parameters.hpp"
#include "types.hpp"

#include <autoware/trajectory_time_overlap/footprints.hpp>

namespace autoware::motion_velocity_planner::run_out
{
TrajectoryCornerFootprint calculate_trajectory_corner_footprint(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory,
  autoware::vehicle_info_utils::VehicleInfo vehicle_info, const Parameters & params)
{
  return trajectory_time_overlap::calculate_trajectory_corner_footprint(
    trajectory, vehicle_info, params.ego_lateral_margin, params.ego_longitudinal_margin);
}
}  // namespace autoware::motion_velocity_planner::run_out
