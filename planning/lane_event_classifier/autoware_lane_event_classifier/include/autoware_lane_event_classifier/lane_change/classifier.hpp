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

#ifndef AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CHANGE__CLASSIFIER_HPP_
#define AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CHANGE__CLASSIFIER_HPP_

#include <autoware_lane_event_classifier/detail/lane_tracker.hpp>
#include <autoware_lane_event_classifier/lane_change/geometry.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_base.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_parameters.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace autoware::lane_event_classifier
{
using LaneChangeConfig = ::lane_event_classifier::Params::LaneChange;

/**
 * @brief Trajectory-driven, predictive lane-change classifier.
 *
 * Onset is decided by where the planned trajectory crosses the reference lane's lateral boundary
 * toward a route primitive (computed by LaneChangeGeometry from the LaneTracker's generic queries);
 * it can confirm before the ego body leaves the reference lane. Completion (settle) and abort
 * completion use the full footprint. See docs/lane_change.md.
 */
class LaneChangeClassifier : public LaneEventClassifierBase
{
public:
  LaneChangeClassifier(bool enabled, LaneChangeConfig config, const LaneTracker & tracker);
  void update(const LaneEventInput & input) final;
  [[nodiscard]] uint8_t get_state() const final;
  [[nodiscard]] bool is_enabled() const final;
  [[nodiscard]] std::string name() const final { return "lane_change"; }
  [[nodiscard]] std::string debug_reason() const final { return debug_reason_; }

private:
  /** @brief Internal maneuver phase; maps to the reported DrivingState. */
  enum class Phase : uint8_t { idle, changing, aborting };

  /** @brief idle: confirm a persisted trajectory crossing → LANE_CHANGING (onset). */
  void detect_onset(
    const LaneEventInput & input, const LaneChangeObservation & observation, double now_s);

  /** @brief changing: confirm the footprint settled in the target primitive → completion, or a
   * persisted trajectory return toward the reference lane → ABORTING. */
  void detect_completion_or_abort(const LaneChangeObservation & observation, double now_s);

  /** @brief aborting: complete the abort once the footprint is back in the reference lane, settle
   * if the footprint reached the target primitive, or re-commit on a persisted crossing →
   * LANE_CHANGING.
   */
  void detect_abort_completion_or_recommit(
    const LaneEventInput & input, const LaneChangeObservation & observation, double now_s);

  /** @brief True when a confidence signal (footprint off route, or blinker toward target) is
   * present.
   */
  [[nodiscard]] static bool has_confidence_signal(
    const LaneEventInput & input, const LaneChangeObservation & observation,
    const LaneChangeCrossing & crossing);

  /** @brief Accumulates a valid crossing over the crossing-persistence window (shortened by a
   * confidence signal); true on confirm. */
  [[nodiscard]] bool accumulate_crossing(
    const LaneChangeCrossing & crossing, double now_s, bool has_confidence_signal);

  /** @brief Accumulates the footprint-settled-in-target-primitive test over the settle window. */
  [[nodiscard]] bool accumulate_settle(const LaneChangeObservation & observation, double now_s);

  /** @brief Clears all persistence timers on a phase transition. */
  void reset_timers();

  bool enabled_{false};
  LaneChangeConfig config_;
  const LaneTracker & tracker_;  // generic lane queries (owned by the node)
  LaneChangeGeometry geometry_;  // derives the per-cycle observation from the tracker

  Phase phase_{Phase::idle};
  std::string debug_reason_;

  // Crossing persistence (onset in idle, re-commit in aborting).
  std::optional<LaneChangeCrossing> tracked_crossing_;
  double crossing_start_s_{0.0};

  // Settle persistence (footprint fully inside a target route primitive).
  bool settle_active_{false};
  lanelet::Id settle_lane_id_{lanelet::InvalId};
  double settle_start_s_{0.0};

  // Abort persistence (trajectory heads back into the reference lane).
  bool abort_active_{false};
  double abort_start_s_{0.0};
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CHANGE__CLASSIFIER_HPP_
