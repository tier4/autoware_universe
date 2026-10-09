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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__OBSTACLE_STOP_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__OBSTACLE_STOP_HPP_

#include "parameter.hpp"
#include "planner_data_lite.hpp"
#include "types.hpp"

#include <memory>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
// 移植元 ObstacleStopModule の点群経路。中身は後続 PR で移植する。
class ObstacleStop
{
public:
  void update_parameters(const Params & params);

  std::vector<StopObstacle> calc_obstacle_stop(
    const std::vector<TrajectoryPoint> & raw_trajectory_points,
    const std::shared_ptr<const PlannerData> planner_data);
};
}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__OBSTACLE_STOP_HPP_
