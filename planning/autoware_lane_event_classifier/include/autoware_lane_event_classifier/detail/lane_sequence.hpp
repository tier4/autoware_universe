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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__LANE_SEQUENCE_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__LANE_SEQUENCE_HPP_

#include <autoware/lanelet2_utils/topology.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <unordered_set>

namespace autoware::lane_event_classifier
{

/**
 * @brief Collects the "straight lane sequence": the given lane plus every lane reachable going
 * straight — forward and backward — within reach_m, including the lane itself.
 *
 * @param lane The lane to expand from.
 * @param routing_graph Routing graph to traverse (a null graph yields just the lane's own id).
 * @param reach_m Fore/aft distance walked along the routing graph.
 */
[[nodiscard]] inline std::unordered_set<lanelet::Id> get_straight_lane_sequence_ids(
  const lanelet::ConstLanelet & lane, const lanelet::routing::RoutingGraphConstPtr & routing_graph,
  double reach_m)
{
  namespace lanelet2_utils = autoware::experimental::lanelet2_utils;

  std::unordered_set<lanelet::Id> ids;
  ids.insert(lane.id());
  if (!routing_graph) {
    return ids;
  }
  for (const auto & sequence :
       lanelet2_utils::get_succeeding_lanelet_sequences(lane, routing_graph, reach_m)) {
    for (const auto & sequence_lane : sequence) {
      ids.insert(sequence_lane.id());
    }
  }
  for (const auto & sequence :
       lanelet2_utils::get_preceding_lanelet_sequences(lane, routing_graph, reach_m)) {
    for (const auto & sequence_lane : sequence) {
      ids.insert(sequence_lane.id());
    }
  }
  return ids;
}

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__LANE_SEQUENCE_HPP_
