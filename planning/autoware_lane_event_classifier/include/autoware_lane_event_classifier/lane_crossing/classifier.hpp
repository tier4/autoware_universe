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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_

#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>
#include <autoware_lane_event_classifier/lane_crossing/geometry.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_base.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_parameters.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::lane_event_classifier
{
using LaneCrossingConfig = ::lane_event_classifier::Params::LaneCrossing;

/**
 * @brief Trajectory-driven, predictive intentional-lane-crossing classifier.
 *
 * A crossing is a partial sideways move over a lane boundary to pass a static obstacle, then a
 * return to the origin lane; a move that fully enters the adjacent lane is a lane change. Onset is
 * predictive (from the planned trajectory, mirroring LaneChangeClassifier) and only fires when the
 * ego is going straight on-route with a qualifying object to avoid. Abort is not modelled in this
 * pass. See docs/lane_crossing.md.
 */
class IntentionalCrossingClassifier : public LaneEventClassifierBase
{
public:
  IntentionalCrossingClassifier(bool enabled, LaneCrossingConfig config, const LaneTracker & tracker);
  void update(const LaneEventInput & input) final;
  [[nodiscard]] uint8_t get_state() const final;
  [[nodiscard]] bool is_enabled() const final;
  [[nodiscard]] std::string name() const final { return "lane_crossing"; }
  [[nodiscard]] std::string debug_reason() const final { return debug_reason_; }

private:
  /** @brief Internal maneuver phase; maps to the reported DrivingState. */
  enum class Phase : uint8_t { idle, crossing };

  /** @brief idle: confirm a persisted trajectory crossing with a qualifying object present →
   * INTENTIONAL_LANE_CROSSING (onset). */
  void detect_onset(
    const LaneEventInput & input, const LaneCrossingObservation & observation, double now_s);

  /** @brief crossing: complete once the footprint returns into the reference straight sequence,
   * escape to idle on a full entry into an adjacent lane (a lane change), or force-complete on the
   * backstop timer. */
  void detect_completion(const LaneCrossingObservation & observation, double now_s);

  /** @brief True when a confidence signal (blinker toward the crossing side) is present. */
  [[nodiscard]] static bool has_confidence_signal(
    const LaneEventInput & input, const LaneCrossingCrossing & crossing);

  /** @brief Accumulates a valid crossing over the crossing-persistence window (shortened by a
   * confidence signal); true on confirm. */
  [[nodiscard]] bool accumulate_crossing(
    const LaneCrossingCrossing & crossing, double now_s, bool has_confidence_signal);

  /** @brief Accumulates the footprint-back-in-reference-sequence test over the settle window. */
  [[nodiscard]] bool accumulate_return(const LaneCrossingObservation & observation, double now_s);

  /** @brief Clears the persistence timers on a phase transition. */
  void reset_timers();

  bool enabled_{false};
  LaneCrossingConfig config_;
  const LaneTracker & tracker_;    // generic lane queries (owned by the node)
  LaneCrossingGeometry geometry_;  // derives the per-cycle observation from the tracker

  Phase phase_{Phase::idle};
  std::string debug_reason_;

  // Crossing persistence (onset in idle).
  std::optional<LaneCrossingCrossing> tracked_crossing_;
  double crossing_start_s_{0.0};

  // Qualifying-object memory: the last time a qualifying static object was seen, so a transient
  // perception dropout at the crossing moment does not cancel onset (docs/lane_crossing.md, "Onset").
  bool has_seen_qualifying_object_{false};
  double last_qualifying_object_s_{0.0};

  // Return persistence (footprint back inside the reference straight sequence).
  bool return_active_{false};
  double return_start_s_{0.0};

  // Committed-crossing timestamp for the max-duration backstop.
  double crossing_committed_s_{0.0};
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_
