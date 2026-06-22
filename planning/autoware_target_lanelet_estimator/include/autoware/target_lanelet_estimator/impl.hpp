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
#include <lanelet2_routing/Forward.h>

#include <unordered_map>
#include <vector>

namespace autoware::target_lanelet_estimator
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;

using LaneletProbabilityMap = std::unordered_map<lanelet::Id, double>;

struct LaneletScore
{
  lanelet::Id id{lanelet::InvalId};
  double score{0.0};       // posterior probability in [0, 1]
  double prior{0.0};       // prior probability after transition update
  double likelihood{0.0};  // max footprint-overlap-area ratio over the trajectory
  bool updated{false};     // true when this lanelet is in S_curr or S_next for this trajectory
};

struct TargetLaneletsResult
{
  std::vector<LaneletScore> lanelets;  // all route lanelets with posterior probability
  bool out_of_lanelet{false};          // a footprint lies outside every lanelet
};

LaneletProbabilityMap initialize_lanelet_probabilities(const LaneletRoute & route);

// Estimate posterior probabilities for route lanelets by combining:
// - prior: previous posterior and transition probability
// - likelihood: max footprint/lanelet overlap along the trajectory
TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info,
  const LaneletProbabilityMap & previous_posteriors,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph);

TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info);

}  // namespace autoware::target_lanelet_estimator

#endif  // AUTOWARE__TARGET_LANELET_ESTIMATOR__IMPL_HPP_
