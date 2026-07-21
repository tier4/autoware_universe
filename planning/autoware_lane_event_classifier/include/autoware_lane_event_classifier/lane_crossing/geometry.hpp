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

#include <geometry_msgs/msg/pose.hpp>

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
  lanelet::Id target_lane_id{lanelet::InvalId};  // off-sequence lane sharing the crossed boundary
                                                 // (best-effort; InvalId if none is mapped)
  lanelet::BasicPoint2d crossing_point{
    0.0, 0.0};             // where the trajectory first crosses the reference boundary
  bool is_to_left{false};  // crossing is toward the reference lane's left side
};

/** @brief The boundary / footprint half of the per-cycle lane-crossing observation. */
struct LaneCrossingObservation
{
  // A valid onset crossing (on-route-straight scope gate + exemptions passed); nullopt otherwise.
  // Predictive — populated from the trajectory, even while the footprint is still inside the lane.
  std::optional<LaneCrossingCrossing> crossing;
  // Return / completion: the footprint is fully inside a lane of the reference straight sequence.
  bool is_footprint_inside_reference_sequence{false};
  // Full-entry escape: a non-sequence lane the footprint is fully inside — the move is a lane
  // change.
  std::optional<lanelet::Id> full_entry_lane_id;
  // Scope gate: the reference lane is on-route and going straight keeps the ego on-route.
  bool is_on_route_straight{false};
  // Human-readable breakdown of the crossing-detection check for this cycle (debug logging only).
  std::string crossing_diagnostic;
};

/**
 * @brief The boundary / footprint half of the lane-crossing policy layer.
 *
 * Scope gate, onset crossing, return, and full-entry escape over a LaneTracker's generic queries:
 * the tracker stays a map/geometry library and knows nothing about crossings; this class interprets
 * its queries. The perceived-object half lives in LaneCrossingObjects. It is a plain value type
 * holding the boundary thresholds and is injected into the classifier (dependency injection). Onset
 * is predictive, mirroring LaneChangeGeometry.
 */
class LaneCrossingGeometry
{
public:
  LaneCrossingGeometry(double crossing_look_ahead_m, double footprint_boundary_overshoot_m);

  /** @brief Builds the observation for this cycle from the tracker's (already refreshed) state.
   * @param candidate_object_poses Objects the ego might cross to avoid (from LaneCrossingObjects):
   * onset fires only when the trajectory brackets one of them. */
  [[nodiscard]] LaneCrossingObservation observe(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::vector<geometry_msgs::msg::Pose> & candidate_object_poses) const;

private:
  /** @brief A crossing plus the per-cycle diagnostic that explains it (returned by value). */
  struct CrossingResult
  {
    std::optional<LaneCrossingCrossing> crossing;
    std::string diagnostic;
  };

  /** @brief True when the reference lane is a route primitive whose straight successor is also a
   * route primitive, so going straight stays on-route (the scope gate). */
  [[nodiscard]] static bool driving_straight_stays_on_route(
    const LaneTracker & tracker, lanelet::Id reference_lane_id);

  /** @brief The valid lane-crossing crossing over the reference boundary (with its diagnostic).
   * Two sources, both gated on a candidate object to go around: (a) predictive - the planned
   * trajectory departs the lane sequence around the object (crosses a boundary out before it, back
   * in after it), detected while the ego body is still inside the lane; (b) physical - the current
   * ego footprint (the real body, so a yawed corner poking over the line counts) crosses the
   * boundary into a lane outside the sequence. A path that stays in the neighbour (never returns)
   * is left to the lane-change classifier.
   * @param reference_lane The tracker's current reference lanelet.
   * @param sequence_ids The reference lane's straight sequence (fore/aft) within the look-ahead.
   * @param trajectory_points Forward trajectory samples (computed once per cycle by observe).
   * @param footprint The ego footprint corners in the map frame (the physical body this cycle).
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe).
   * @param candidate_object_poses Objects the ego might cross to avoid; onset requires one. */
  [[nodiscard]] CrossingResult compute_crossing(
    const LaneTracker & tracker, const lanelet::ConstLanelet & reference_lane,
    const std::unordered_set<lanelet::Id> & sequence_ids,
    const std::vector<lanelet::BasicPoint2d> & trajectory_points,
    const std::vector<lanelet::BasicPoint2d> & footprint,
    const std::vector<lanelet::Id> & footprint_ids,
    const std::vector<geometry_msgs::msg::Pose> & candidate_object_poses) const;

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

  double crossing_look_ahead_m_;           // arc length ahead scanned for a trajectory crossing
  double footprint_boundary_overshoot_m_;  // min body overshoot past the boundary for the footprint
                                           // (physical) crossing source
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__GEOMETRY_HPP_
