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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__LANE_TRACKER_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__LANE_TRACKER_HPP_

#include <autoware_lane_event_classifier/types.hpp>
#include <tl_expected/expected.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::lane_event_classifier
{

/** @brief The lane the ego was following before a lane event began; held during an event. */
struct ReferenceLane
{
  lanelet::Id reference_lane_id{lanelet::InvalId};  // lane the ego was in before departure
  bool is_reference_lane_on_route{false};  // debug only: reference lane is itself a route primitive
};

/** @brief Owns the map, routing graph, and reference lane; provides generic lane queries.
 *
 * This is a map/geometry library: it tracks the reference lane and answers stateless lane queries,
 * but knows nothing about any specific lane event. Event policy (e.g. lane-change crossing
 * geometry) lives in the classifiers, which consume these queries — see lane_change/geometry.hpp.
 */
class LaneTracker
{
public:
  LaneTracker() = default;

  tl::expected<void, std::string> set_lanelet_map(const lanelet::LaneletMapPtr & lanelet_map_ptr);

  [[nodiscard]] bool has_lanelet_map() const { return static_cast<bool>(lanelet_map_ptr_); }

  [[nodiscard]] const lanelet::LaneletMapPtr & lanelet_map_ptr() const { return lanelet_map_ptr_; }

  [[nodiscard]] const lanelet::routing::RoutingGraphConstPtr & routing_graph_ptr() const
  {
    return routing_graph_ptr_;
  }

  /**
   * @brief Refreshes the reference lane for this cycle (unless held).
   * @param input Per-cycle subscribed inputs and footprint.
   */
  void update(const LaneEventInput & input);

  void hold_reference_lane() { is_reference_lane_held_ = true; }

  void release_reference_lane();

  [[nodiscard]] bool is_reference_lane_held() const { return is_reference_lane_held_; }

  [[nodiscard]] const ReferenceLane & reference_lane() const { return reference_lane_; }

  /**
   * @brief Returns true if every footprint corner is inside the lanelet with the given id.
   * @param lane_id Lanelet id to test against.
   * @param footprint Footprint corners in the map frame.
   */
  [[nodiscard]] bool is_footprint_fully_inside_lane(
    lanelet::Id lane_id, const std::vector<lanelet::BasicPoint2d> & footprint) const;

  [[nodiscard]] bool lanelet_exists(lanelet::Id lane_id) const;

  [[nodiscard]] std::optional<lanelet::ConstLanelet> get_lanelet(lanelet::Id lane_id) const;

  /**
   * @brief 2D distance from the point to the lane's polygon (0 while inside it).
   * @param lane_id Lanelet id to measure against.
   * @param point Query point in the map frame.
   * @return The distance in metres, or nullopt if the lane id is unknown.
   */
  [[nodiscard]] std::optional<double> distance_to_lane(
    lanelet::Id lane_id, const lanelet::BasicPoint2d & point) const;

  // Debug helpers (logging / visualisation only).
  /** @brief Ids of every lanelet that contains the given point. */
  [[nodiscard]] std::vector<lanelet::Id> lanelet_ids_at(const lanelet::BasicPoint2d & point) const;

  /** @brief Ids of every lanelet that at least one footprint corner lies inside. */
  [[nodiscard]] std::vector<lanelet::Id> footprint_lane_ids(
    const std::vector<lanelet::BasicPoint2d> & footprint) const;

  /** @brief Ids of the given lane's longitudinal next lanes (routing following). */
  [[nodiscard]] std::vector<lanelet::Id> next_lane_ids(lanelet::Id lane_id) const;

  /** @brief Lane ids reachable from the given lane by driving straight (fore/aft) within
   * look_ahead_m, including the lane itself.
   *
   * Memoized on (lane id, look_ahead_m): the routing-graph expansion is skipped while the reference
   * lane is unchanged (the common case), and recomputed only when the queried lane changes or the
   * map is replaced. The returned reference is valid until the next call with different arguments.
   */
  [[nodiscard]] const std::unordered_set<lanelet::Id> & straight_lane_sequence_ids(
    const lanelet::ConstLanelet & lane, double look_ahead_m) const;

  /** @brief The ordered, forward-only on-route straight sequence starting at the given lane: the
   * lane itself followed by each route-primitive straight successor, accumulating at least
   * downstream_length_m of lane length beyond the starting lane.
   *
   * Unlike the fore/aft membership set of straight_lane_sequence_ids, this is a forward-only,
   * route-filtered, *ordered* run of lanelets, suitable for concatenating boundaries or measuring
   * forward arc length along the lane sequence. Returned by value; empty if the lane id is unknown.
   */
  [[nodiscard]] lanelet::ConstLanelets get_forward_route_lane_sequence(
    lanelet::Id reference_lane_id, double downstream_length_m) const;

  /** @brief Returns true if lane_id is a preferred primitive of the current route.
   *
   * O(1) lookup against the route-primitive set cached in update(); the set is rebuilt only when
   * the route changes (new uuid == new route_ptr). */
  [[nodiscard]] bool is_route_primitive(lanelet::Id lane_id) const;

  /** @brief The cached set of route-preferred primitive ids for the current route. */
  [[nodiscard]] const std::unordered_set<lanelet::Id> & route_primitive_ids() const
  {
    return route_primitive_ids_;
  }

  /** @brief Lanelet the ego centre was found in on the last update() (InvalId if none/held). */
  [[nodiscard]] lanelet::Id last_selected_lane_id() const { return last_selected_lane_id_; }

  /** @brief True when the last update() could not advance the reference lane
   * (stale/off-sequence). */
  [[nodiscard]] bool is_last_reanchor_blocked() const { return is_last_reanchor_blocked_; }

private:
  /** @brief Rebuilds route_primitive_ids_ when the route changes (keyed on route_ptr identity). */
  void refresh_route_primitive_cache(const LaneEventInput & input);

  /** @brief Re-anchors the reference lane only on forward progress into a next lane, never on a
   * lateral move. */
  void refresh_reference_lane(const LaneEventInput & input);

  /** @brief The lanelet the ego sits in: route primitive, else off-route lane, else nearest. */
  [[nodiscard]] std::optional<lanelet::Id> select_current_lane_id(
    const LaneEventInput & input) const;

  /** @brief Returns true if candidate_lane_id is a next lane (routing following) of from_lane_id.
   */
  [[nodiscard]] bool is_lane_directly_connected(
    lanelet::Id from_lane_id, lanelet::Id candidate_lane_id) const;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  ReferenceLane reference_lane_{};
  bool is_reference_lane_held_{false};

  // Diagnostics from the last update() (logging only).
  lanelet::Id last_selected_lane_id_{lanelet::InvalId};
  bool is_last_reanchor_blocked_{false};

  // Route-primitive cache: rebuilt only when the route changes (keyed on route_ptr identity).
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr cached_route_ptr_;

  std::unordered_set<lanelet::Id> route_primitive_ids_;

  // Straight-lane-sequence memo (mutable: filled lazily by the const straight_lane_sequence_ids
  // query).
  mutable lanelet::Id cached_lane_sequence_source_id_{lanelet::InvalId};
  mutable double cached_lane_sequence_look_ahead_m_{-1.0};
  mutable std::unordered_set<lanelet::Id> cached_lane_sequence_ids_;
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__DETAIL__LANE_TRACKER_HPP_
