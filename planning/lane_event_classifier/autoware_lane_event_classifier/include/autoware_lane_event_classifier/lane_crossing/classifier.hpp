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
#include <autoware_lane_event_classifier/lane_crossing/objects.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_base.hpp>
#include <autoware_lane_event_classifier/lane_event_classifier_parameters.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::lane_event_classifier
{
using LaneCrossingConfig = ::lane_event_classifier::Params::LaneCrossing;

/**
 * @brief Predictive intentional-lane-crossing classifier: a partial sideways dodge over a lane
 * boundary to pass an object ahead, then a return; a move that fully enters the neighbour is a lane
 * change. Onset is trajectory (predictive) plus footprint (physical). Abort is not modelled in this
 * pass. See docs/lane_crossing.md.
 */
class IntentionalCrossingClassifier : public LaneEventClassifierBase
{
public:
  IntentionalCrossingClassifier(
    bool enabled, LaneCrossingConfig config, const LaneTracker & tracker,
    LaneCrossingGeometry geometry, LaneCrossingObjects objects);
  void update(const LaneEventInput & input) final;
  [[nodiscard]] uint8_t get_state() const final;
  [[nodiscard]] bool is_enabled() const final;
  [[nodiscard]] std::string name() const final { return "lane_crossing"; }
  [[nodiscard]] std::string debug_reason() const final { return debug_reason_; }

private:
  /** @brief Internal maneuver phase; maps to the reported DrivingState. */
  enum class Phase : uint8_t { idle, crossing };

  /** @brief idle phase: fire onset (→ INTENTIONAL_LANE_CROSSING) when a crossing persists. See
   * docs/lane_crossing.md, "Onset". */
  void detect_onset(
    const LaneEventInput & input, const LaneCrossingObservation & observation, double now_s);

  /** @brief The candidate object poses for this cycle: perceived now, else the remembered set. See
   * docs/lane_crossing.md, "Candidate object". */
  [[nodiscard]] std::vector<geometry_msgs::msg::Pose> effective_candidate_object_poses(
    std::vector<geometry_msgs::msg::Pose> perceived_poses, double now_s);

  /** @brief crossing phase: end once the ego is fully inside one lane (return or full entry). See
   * docs/lane_crossing.md, "Finishing". */
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
  LaneCrossingGeometry geometry_;  // boundary / footprint half (injected)
  LaneCrossingObjects objects_;    // perceived-object half (injected)

  Phase phase_{Phase::idle};
  std::string debug_reason_;

  // Crossing persistence (onset in idle).
  std::optional<LaneCrossingCrossing> tracked_crossing_;
  double crossing_start_s_{0.0};

  // Candidate memory (docs/lane_crossing.md, "Candidate object"): recent poses + last-seen stamp.
  std::vector<geometry_msgs::msg::Pose> remembered_candidate_poses_;
  double last_candidate_seen_s_{0.0};

  // Return persistence (footprint back inside the reference straight sequence).
  bool return_active_{false};
  double return_start_s_{0.0};
};

}  // namespace autoware::lane_event_classifier

#endif  // AUTOWARE_LANE_EVENT_CLASSIFIER__LANE_CROSSING__CLASSIFIER_HPP_
