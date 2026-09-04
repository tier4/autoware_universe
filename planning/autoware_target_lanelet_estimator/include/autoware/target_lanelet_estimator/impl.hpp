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
#include <autoware_utils_geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>

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

// Tunable parameters of the estimator. Defaults match the values validated by the unit tests.
struct Parameters
{
  double preferred_lanelet_initial_probability{0.8};  // initial probability of a preferred lanelet
  double other_lanelet_initial_probability{0.2};      // initial probability of a non-preferred one
  double same_segment_lane_change_probability{0.05};  // per-lane leak within the same segment
  double following_transition_weight{0.8};            // transition weight to a connected successor
  double non_following_transition_weight{0.2};        // transition weight to an unconnected lanelet
  double selection_likelihood_threshold{1.0e-3};      // min likelihood to report a target lanelet
  double out_of_lanelet_search_margin{2.0};  // [m] padding of the out-of-lanelet search box
};

struct LaneletProbability
{
  lanelet::Id id{lanelet::InvalId};
  double posterior{0.0};   // posterior probability in [0, 1]
  double prior{0.0};       // prior probability after transition update
  double likelihood{0.0};  // max footprint-overlap-area ratio over the trajectory
  bool updated{false};     // true when this lanelet is in S_curr or S_next for this trajectory
};

struct TargetLaneletsResult
{
  std::vector<LaneletProbability>
    lanelet_probabilities;  // all route lanelets with posterior probability
  std::vector<lanelet::Id>
    target_lanelet_ids;        // route lanelets overlapped by trajectory footprints
  bool out_of_lanelet{false};  // no footprint overlaps any lanelet in the map
};

LaneletProbabilityMap initialize_lanelet_probabilities(
  const LaneletRoute & route, const Parameters & params = {});

// Estimate posterior probabilities for route lanelets by combining:
// - prior: previous posterior and transition probability
// - likelihood: max footprint/lanelet overlap along the poses
// This footprint-agnostic form works for any tracked vehicle: `poses` is the vehicle's
// trajectory/predicted path and `base_footprint` its local-frame footprint.
TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const std::vector<geometry_msgs::msg::Pose> & poses,
  const autoware_utils_geometry::LinearRing2d & base_footprint,
  const lanelet::LaneletMapConstPtr & lanelet_map,
  const LaneletProbabilityMap & previous_posteriors,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, const Parameters & params = {});

// Convenience overloads for the ego vehicle: the trajectory points supply the poses and
// vehicle_info the footprint.
TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info,
  const LaneletProbabilityMap & previous_posteriors,
  const lanelet::routing::RoutingGraphConstPtr & routing_graph, const Parameters & params = {});

TargetLaneletsResult get_target_lanelets(
  const LaneletRoute & route, const Trajectory & trajectory,
  const lanelet::LaneletMapConstPtr & lanelet_map, const VehicleInfo & vehicle_info,
  const Parameters & params = {});

}  // namespace autoware::target_lanelet_estimator

#endif  // AUTOWARE__TARGET_LANELET_ESTIMATOR__IMPL_HPP_
