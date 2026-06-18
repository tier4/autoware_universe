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

#ifndef AUTOWARE__TARGET_LANELET_ESTIMATOR__IMPL_HPP_
#define AUTOWARE__TARGET_LANELET_ESTIMATOR__IMPL_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <vector>

namespace autoware::target_lanelet_estimator
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;

struct LaneletScore
{
  lanelet::Id id{lanelet::InvalId};
  double score{0.0};  // likelihood in [0, 1]: max footprint-overlap-area ratio over the trajectory
};

struct TargetLaneletsResult
{
  std::vector<LaneletScore> lanelets;  // route lanelets with score > 0 (>1 entry means straddling)
  bool out_of_lanelet{false};          // a footprint lies outside every lanelet
};

// Estimate which route lanelet(s) the ego is going to drive on by overlapping the ego footprint
// along the trajectory with each route lanelet, and flag driving outside every lanelet.
TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info);

}  // namespace autoware::target_lanelet_estimator

#endif  // AUTOWARE__TARGET_LANELET_ESTIMATOR__IMPL_HPP_
