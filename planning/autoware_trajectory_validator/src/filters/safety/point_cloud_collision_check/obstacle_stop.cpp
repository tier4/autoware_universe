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

#include "obstacle_stop.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
void ObstacleStop::update_parameters([[maybe_unused]] const Params & params)
{
}

std::vector<StopObstacle> ObstacleStop::calc_obstacle_stop(
  [[maybe_unused]] const std::vector<TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const std::shared_ptr<const PlannerData> planner_data)
{
  return {};
}
}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
