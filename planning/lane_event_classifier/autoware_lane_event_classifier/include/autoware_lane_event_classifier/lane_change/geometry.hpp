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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CHANGE__GEOMETRY_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CHANGE__GEOMETRY_HPP_

#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>
#include <autoware_lane_event_classifier/types.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <optional>
#include <vector>

namespace autoware::lane_event_classifier
{

/** @brief Where the planned trajectory first crosses the reference lane's lateral boundary. */
struct LaneChangeCrossing
{
  lanelet::Id target_lane_id{lanelet::InvalId};  // first adjacent lane the trajectory crosses into
  lanelet::BasicPoint2d crossing_point{0.0, 0.0};  // where the trajectory exits the reference lane
  bool is_to_left{false};  // crossing is toward the reference lane's left side
};

/** @brief Per-cycle lane-change geometry the classifier reasons over. */
struct LaneChangeObservation
{
  // A valid onset crossing (heads to a route primitive, exemptions passed); nullopt otherwise.
  // Predictive — populated even while the footprint is still inside the reference lane.
  std::optional<LaneChangeCrossing> crossing;
  // Abort observation: the trajectory heads back into the reference lane (no forward crossing).
  bool trajectory_returns_to_reference{false};
  // Confidence signal (booster): the whole footprint has left the route-primitive lanes.
  bool is_footprint_off_route_primitives{false};
  // Abort completion: the footprint is fully back inside the reference lane.
  bool is_footprint_inside_reference_lane{false};
  // Settle: a route primitive other than the reference lane the footprint is fully inside.
  std::optional<lanelet::Id> settle_lane_id;
};

/**
 * @brief Computes the per-cycle lane-change observation from a LaneTracker's generic queries.
 *
 * This is the lane-change policy layer (onset, exemptions, abort, and settle): the tracker stays a
 * map/geometry library and knows nothing about lane changes; this class interprets its queries. See
 * docs/lane_change.md.
 */
class LaneChangeGeometry
{
public:
  explicit LaneChangeGeometry(double crossing_look_ahead_m);

  /** @brief Builds the observation for this cycle from the tracker's (already refreshed) state. */
  [[nodiscard]] LaneChangeObservation observe(
    const LaneTracker & tracker, const LaneEventInput & input) const;

private:
  /** @brief The valid lane-change crossing of the trajectory over the reference boundary, if any.
   * @param trajectory_points Forward trajectory samples (computed once per cycle by observe). */
  [[nodiscard]] std::optional<LaneChangeCrossing> compute_crossing(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::vector<lanelet::BasicPoint2d> & trajectory_points) const;

  /** @brief True when the trajectory heads back into the reference lane (abort observation).
   * @param has_forward_crossing Whether a valid forward crossing was found this cycle.
   * @param trajectory_points Forward trajectory samples (computed once per cycle by observe). */
  [[nodiscard]] static bool compute_trajectory_returns_to_reference(
    const LaneTracker & tracker, bool has_forward_crossing,
    const std::vector<lanelet::BasicPoint2d> & trajectory_points);

  /** @brief True when no footprint corner lies inside any route-preferred primitive (the off-route
   * confidence signal).
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe). */
  [[nodiscard]] static bool is_footprint_off_route_primitives(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::vector<lanelet::Id> & footprint_ids);

  /** @brief The route-preferred lane other than the reference lane the footprint is fully inside,
   * if any.
   * @param footprint_ids Lanes the footprint touches (computed once per cycle by observe). */
  [[nodiscard]] static std::optional<lanelet::Id> compute_settle_lane_id(
    const LaneTracker & tracker, const LaneEventInput & input,
    const std::vector<lanelet::Id> & footprint_ids);

  double crossing_look_ahead_m_;  // arc length ahead scanned for a trajectory crossing
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CHANGE__GEOMETRY_HPP_
