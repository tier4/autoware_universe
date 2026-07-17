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

#include <autoware_lane_event_classifier/lane_crossing/classifier.hpp>
#include <rclcpp/time.hpp>

#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <fmt/format.h>

namespace autoware::lane_event_classifier
{

namespace
{
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;

double stamp_to_seconds(const LaneEventInput & input)
{
  return rclcpp::Time(input.odometry_ptr->header.stamp).seconds();
}

// Confidence signal (docs/lane_crossing.md, "Confidence signal"): the blinker is on toward the side
// the crossing heads to (the driver signals toward the dodge on the way out).
bool is_blinker_toward_crossing_side(
  const LaneCrossingCrossing & crossing, uint8_t turn_indicator)
{
  if (crossing.is_to_left) {
    return turn_indicator == TurnIndicatorsReport::ENABLE_LEFT;
  }
  return turn_indicator == TurnIndicatorsReport::ENABLE_RIGHT;
}
}  // namespace

IntentionalCrossingClassifier::IntentionalCrossingClassifier(
  bool enabled, LaneCrossingConfig config, const LaneTracker & tracker)
: enabled_{enabled},
  config_{config},
  tracker_{tracker},
  geometry_{
    config.crossing_look_ahead_m, config.footprint_boundary_overshoot_m,
    config.object_static_speed_threshold_mps, config.object_longitudinal_window_m,
    config.object_overlap_area_threshold_m2}
{
}

void IntentionalCrossingClassifier::reset_timers()
{
  tracked_crossing_.reset();
  crossing_start_s_ = 0.0;
  return_active_ = false;
  return_start_s_ = 0.0;
  has_seen_qualifying_object_ = false;
  last_qualifying_object_s_ = 0.0;
}

bool IntentionalCrossingClassifier::accumulate_crossing(
  const LaneCrossingCrossing & crossing, double now_s, bool has_confidence_signal)
{
  // Both-persistence (docs/lane_crossing.md, "Onset"): the same crossing side and a crossing location
  // stable within tolerance. The side (not the best-effort target lane id, which may be InvalId for a
  // dodge over the line into open space) anchors the identity across cycles.
  const bool matches_tracked =
    tracked_crossing_ && tracked_crossing_->is_to_left == crossing.is_to_left &&
    (crossing.crossing_point - tracked_crossing_->crossing_point).norm() <=
      config_.crossing_position_tolerance_m;
  if (!matches_tracked) {
    tracked_crossing_ = crossing;  // anchor the crossing location; restart the window
    crossing_start_s_ = now_s;
  }

  // Confidence signal (docs/lane_crossing.md, "Confidence signal"): shortens (never bypasses) the
  // crossing-persistence window.
  const double effective_persist_duration =
    config_.crossing_persist_duration_s * (has_confidence_signal ? config_.confidence_factor : 1.0);
  return (now_s - crossing_start_s_) >= effective_persist_duration;
}

bool IntentionalCrossingClassifier::accumulate_return(
  const LaneCrossingObservation & observation, double now_s)
{
  if (!observation.is_footprint_inside_reference_sequence) {
    return_active_ = false;
    return false;
  }
  if (!return_active_) {
    return_active_ = true;
    return_start_s_ = now_s;
  }
  return (now_s - return_start_s_) >= config_.settle_confirm_duration_s;
}

bool IntentionalCrossingClassifier::has_confidence_signal(
  const LaneEventInput & input, const LaneCrossingCrossing & crossing)
{
  return is_blinker_toward_crossing_side(crossing, input.turn_indicator);
}

void IntentionalCrossingClassifier::update(const LaneEventInput & input)
{
  const double now_s = stamp_to_seconds(input);
  const LaneCrossingObservation observation = geometry_.observe(tracker_, input);

  switch (phase_) {
    case Phase::idle:
      detect_onset(input, observation, now_s);
      break;
    case Phase::crossing:
      detect_completion(observation, now_s);
      break;
  }
}

void IntentionalCrossingClassifier::detect_onset(
  const LaneEventInput & input, const LaneCrossingObservation & observation, double now_s)
{
  // Qualifying-object memory (docs/lane_crossing.md, "Onset"): perception velocity/overlap is noisy
  // exactly when the ego dodges past a parked object, so a static object that qualified a moment ago
  // is remembered for object_qualifying_memory_s. A genuinely moving object never sustains a
  // qualifying reading, so it never latches.
  if (observation.has_qualifying_object) {
    has_seen_qualifying_object_ = true;
    last_qualifying_object_s_ = now_s;
  }
  const bool object_recently_qualified =
    has_seen_qualifying_object_ &&
    (now_s - last_qualifying_object_s_) <= config_.object_qualifying_memory_s;

  // Onset (docs/lane_crossing.md, "Onset"): a predictive trajectory crossing AND a qualifying static
  // object to avoid (seen now or within the memory window). Without either, there is no crossing.
  if (!observation.crossing || !object_recently_qualified) {
    tracked_crossing_.reset();
    // Per-cycle diagnostic (surfaced throttled by the node): why onset did not fire this cycle.
    debug_reason_ = fmt::format(
      "idle: on_route_straight={} crossing={} qualifying_recent={} | crossing: {} | objects: {}",
      observation.is_on_route_straight ? "yes" : "no",
      observation.crossing ? (observation.crossing->is_to_left ? "left" : "right") : "none",
      object_recently_qualified ? "yes" : "no", observation.crossing_diagnostic,
      observation.object_diagnostic);
    return;
  }
  const bool confidence = has_confidence_signal(input, *observation.crossing);
  if (accumulate_crossing(*observation.crossing, now_s, confidence)) {
    debug_reason_ = fmt::format(
      "onset: trajectory crosses the {} boundary to pass a static object",
      tracked_crossing_->is_to_left ? "left" : "right");
    phase_ = Phase::crossing;
    crossing_committed_s_ = now_s;
    reset_timers();
  }
}

void IntentionalCrossingClassifier::detect_completion(
  const LaneCrossingObservation & observation, double now_s)
{
  // Full-entry escape (docs/lane_crossing.md, "Finishing"): the footprint fully entered an adjacent
  // lane, so the move is a lane change; drop to idle and let the higher-priority lane-change
  // classifier own it.
  if (observation.full_entry_lane_id) {
    debug_reason_ = fmt::format(
      "abandoned: footprint fully entered lane {} (a lane change, not a crossing)",
      *observation.full_entry_lane_id);
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // Completion (docs/lane_crossing.md, "Finishing"): the footprint returned into the reference
  // straight sequence for the settle window.
  if (accumulate_return(observation, now_s)) {
    debug_reason_ = "completed: footprint returned to the reference straight sequence";
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // Backstop (docs/lane_crossing.md, "Finishing"): a crossing that neither returns nor becomes a
  // lane change within the maximum duration is force-completed.
  if ((now_s - crossing_committed_s_) >= config_.max_crossing_duration_s) {
    debug_reason_ = "completed: maximum crossing duration reached (backstop)";
    phase_ = Phase::idle;
    reset_timers();
  }
}

uint8_t IntentionalCrossingClassifier::get_state() const
{
  switch (phase_) {
    case Phase::crossing:
      return DrivingState::INTENTIONAL_LANE_CROSSING;
    case Phase::idle:
    default:
      // No crossing event: UNKNOWN (the node falls back to the lane-following check).
      return DrivingState::UNKNOWN;
  }
}

bool IntentionalCrossingClassifier::is_enabled() const
{
  return enabled_;
}

}  // namespace autoware::lane_event_classifier
