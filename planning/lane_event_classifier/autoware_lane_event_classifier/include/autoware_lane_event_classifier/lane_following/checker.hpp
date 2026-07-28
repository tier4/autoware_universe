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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_

#include <autoware_lane_event_classifier/lane_event_classifier_parameters.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <string_view>
#include <unordered_set>

namespace autoware::lane_event_classifier
{
using LaneFollowingConfig = ::lane_event_classifier::Params::LaneFollowing;

/** @brief Which rule decided the lane-following outcome (for tracing / logging). */
enum class LaneFollowingReason {
  no_reference_lane,          // no reference lane yet -> treated as following
  inside_connected_sequence,  // reference point inside a lane of the connected sequence
  within_lateral_tolerance,   // reference point within the lateral margin of the sequence
  road_shoulder_exempt,       // reference point overlaps a road shoulder
  turn_lane_exempt,           // reference_lane / current lane is a turn / intersection lane
  virtual_boundary_exempt,    // the boundary the ego crossed is virtual
  departed                    // none matched -> a real lateral departure (not following)
};

/** @brief Lane-following verdict plus the rule that decided it. */
struct LaneFollowingResult
{
  bool is_following{true};
  LaneFollowingReason reason{LaneFollowingReason::no_reference_lane};
};

/** @brief Returns a short label for the reason (tracing / logging). */
[[nodiscard]] std::string_view to_string(LaneFollowingReason reason);

/** @brief Runs the lane-following check and reports which rule decided the outcome. See
docs/lane_following.md
 */
class LaneFollowingChecker
{
public:
  LaneFollowingChecker() = default;
  explicit LaneFollowingChecker(LaneFollowingConfig config);

  /**
   * @brief Runs the check for the ego reference point against the reference lane.
   * @param lanelet_map_ptr Owned lanelet map.
   * @param routing_graph_ptr Owned routing graph.
   * @param reference_lane_id The reference lane id.
   * @param ego_point Ego reference point (base_link) in the map frame.
   */
  [[nodiscard]] LaneFollowingResult evaluate(
    const lanelet::LaneletMapPtr & lanelet_map_ptr,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph_ptr, lanelet::Id reference_lane_id,
    const lanelet::BasicPoint2d & ego_point) const;

private:
  /**
   * @brief Connected-lane-sequence ids for the reference lane, memoized while it and the graph are
   * unchanged.
   * @param reference_lane The reference lanelet.
   * @param routing_graph_ptr Routing graph to traverse.
   */
  [[nodiscard]] const std::unordered_set<lanelet::Id> & connected_sequence_ids(
    const lanelet::ConstLanelet & reference_lane,
    const lanelet::routing::RoutingGraphConstPtr & routing_graph_ptr) const;

  LaneFollowingConfig config_{};

  mutable lanelet::routing::RoutingGraphConstPtr cached_graph_ptr_;
  mutable lanelet::Id cached_reference_lane_id_{lanelet::InvalId};
  mutable std::unordered_set<lanelet::Id> cached_sequence_ids_;
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_FOLLOWING__CHECKER_HPP_
