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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__GEOMETRY_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__GEOMETRY_HPP_

#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>
#include <autoware_lane_event_classifier/types.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace autoware::lane_event_classifier
{

/** @brief Where the planned trajectory crosses the reference lane's lateral boundary. */
struct LaneCrossingCrossing
{
  lanelet::Id target_lane_id{lanelet::InvalId};  // off-sequence lane sharing the crossed boundary (best-effort; InvalId if none is mapped)
  lanelet::BasicPoint2d crossing_point{0.0, 0.0};  // where the trajectory first crosses the reference boundary
  bool is_to_left{false};  // crossing is toward the reference lane's left side
};

/** @brief Per-cycle lane-crossing geometry the classifier reasons over. */
struct LaneCrossingObservation
{
  // A valid onset crossing (on-route-straight scope gate + exemptions passed); nullopt otherwise.
  // Predictive — populated from the trajectory, even while the footprint is still inside the lane.
  std::optional<LaneCrossingCrossing> crossing;
  // A static obstacle overlaps the reference straight sequence ahead of the ego within the window.
  bool has_qualifying_object{false};
  // Return / completion: the footprint is fully inside a lane of the reference straight sequence.
  bool is_footprint_inside_reference_sequence{false};
  // Full-entry escape: a non-sequence lane the footprint is fully inside — the move is a lane change.
  std::optional<lanelet::Id> full_entry_lane_id;
  // Scope gate: the reference lane is on-route and going straight keeps the ego on-route.
  bool is_on_route_straight{false};
  // Human-readable breakdown of the qualifying-object check for this cycle (debug logging only).
  std::string object_diagnostic;
  // Human-readable breakdown of the crossing-detection check for this cycle (debug logging only).
  std::string crossing_diagnostic;
};

/**
 * @brief Computes the per-cycle lane-crossing observation from a LaneTracker's generic queries.
 *
 * This is the lane-crossing policy layer (scope gate, onset crossing, object qualification, return,
 * and full-entry escape): the tracker stays a map/geometry library and knows nothing about
 * crossings; this class interprets its queries. Onset is predictive, mirroring LaneChangeGeometry.
 * See docs/lane_crossing.md.
 */
class LaneCrossingGeometry
{
public:
  LaneCrossingGeometry(
    double crossing_look_ahead_m, double footprint_boundary_overshoot_m,
    double object_static_speed_threshold_mps, double object_longitudinal_window_m,
    double object_overlap_area_threshold_m2);

  /** @brief Builds the observation for this cycle from the tracker's (already refreshed) state. */
  [[nodiscard]] LaneCrossingObservation observe(
    const LaneTracker & tracker, const LaneEventInput & input) const;

private:
  /** @brief True when the reference lane is a route primitive whose straight successor is also a
   * route primitive, so going straight stays on-route (the scope gate). */
  [[nodiscard]] static bool driving_straight_stays_on_route(
    const LaneTracker & tracker, lanelet::Id reference_lane_id);

  /** @brief The valid lane-crossing crossing over the reference boundary, if any. Onset fires when
   * the predictive trajectory centerline OR the current ego footprint crosses the boundary — a
   * partial dodge may keep the path reference point in-lane while the body pokes over the line.
   * @param reference_lane The tracker's current reference lanelet.
   * @param sequence_ids The reference lane's straight sequence (fore/aft) within the look-ahead.
   * @param trajectory_points Forward trajectory samples (computed once per cycle by observe).
   * @param footprint The ego footprint corners in the map frame (the physical body this cycle).
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe).
   * @param diagnostic Filled with a per-cycle breakdown of the crossing check (debug logging only). */
  [[nodiscard]] std::optional<LaneCrossingCrossing> compute_crossing(
    const LaneTracker & tracker, const lanelet::ConstLanelet & reference_lane,
    const std::unordered_set<lanelet::Id> & sequence_ids,
    const std::vector<lanelet::BasicPoint2d> & trajectory_points,
    const std::vector<lanelet::BasicPoint2d> & footprint,
    const std::vector<lanelet::Id> & footprint_ids, std::string & diagnostic) const;

  /** @brief True when a static obstacle overlaps the reference straight sequence ahead of the ego
   * within the longitudinal window and above the overlap-area threshold.
   * @param diagnostic Filled with a per-cycle breakdown of the object check (debug logging only). */
  [[nodiscard]] bool compute_has_qualifying_object(
    const LaneTracker & tracker, const LaneEventInput & input, lanelet::Id reference_lane_id,
    std::string & diagnostic) const;

  /** @brief The ordered on-route straight corridor from the reference lane forward, walked until the
   * downstream lanes cover at least downstream_length_m beyond the reference lane. */
  [[nodiscard]] static lanelet::ConstLanelets get_forward_route_lane_sequence(
    const LaneTracker & tracker, lanelet::Id reference_lane_id, double downstream_length_m);

  /** @brief True when the footprint is fully inside a lane of the reference straight sequence.
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe). */
  [[nodiscard]] static bool compute_is_footprint_inside_reference_sequence(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::unordered_set<lanelet::Id> & sequence_ids,
    const std::vector<lanelet::Id> & footprint_ids);

  /** @brief A non-sequence lane the footprint is fully inside (the full-entry escape), if any.
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe). */
  [[nodiscard]] static std::optional<lanelet::Id> compute_full_entry_lane_id(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::unordered_set<lanelet::Id> & sequence_ids,
    const std::vector<lanelet::Id> & footprint_ids);

  double crossing_look_ahead_m_;             // arc length ahead scanned for a trajectory crossing
  double footprint_boundary_overshoot_m_;    // min body overshoot past the boundary for the footprint signal
  double object_static_speed_threshold_mps_;  // below this speed an object counts as static
  double object_longitudinal_window_m_;       // ahead-of-ego arc window for a qualifying object
  double object_overlap_area_threshold_m2_;   // min object/sequence intersection area to qualify
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__GEOMETRY_HPP_
