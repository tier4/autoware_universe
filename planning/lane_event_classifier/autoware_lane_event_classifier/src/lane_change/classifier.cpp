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

#include <autoware_lane_event_classifier/lane_change/classifier.hpp>
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

// Confidence signal (docs/lane_change.md, "Persistence and confidence"): the turn blinker is on
// toward the side the crossing heads to.
bool is_blinker_toward_target(const LaneChangeCrossing & crossing, uint8_t turn_indicator)
{
  if (crossing.is_to_left) {
    return turn_indicator == TurnIndicatorsReport::ENABLE_LEFT;
  }
  return turn_indicator == TurnIndicatorsReport::ENABLE_RIGHT;
}
}  // namespace

LaneChangeClassifier::LaneChangeClassifier(
  bool enabled, LaneChangeConfig config, const LaneTracker & tracker)
: enabled_{enabled}, config_{config}, tracker_{tracker}, geometry_{config.crossing_look_ahead_m}
{
}

void LaneChangeClassifier::reset_timers()
{
  tracked_crossing_.reset();
  crossing_start_s_ = 0.0;
  settle_active_ = false;
  settle_lane_id_ = lanelet::InvalId;
  settle_start_s_ = 0.0;
  abort_active_ = false;
  abort_start_s_ = 0.0;
}

bool LaneChangeClassifier::accumulate_crossing(
  const LaneChangeCrossing & crossing, double now_s, bool has_confidence_signal)
{
  // Both-persistence (docs/lane_change.md, "Persistence and confidence"): the same target lane
  // and a crossing location stable within tolerance.
  const bool matches_tracked =
    tracked_crossing_ && tracked_crossing_->target_lane_id == crossing.target_lane_id &&
    (crossing.crossing_point - tracked_crossing_->crossing_point).norm() <=
      config_.crossing_position_tolerance_m;
  if (!matches_tracked) {
    tracked_crossing_ = crossing;  // anchor the crossing location; restart the window
    crossing_start_s_ = now_s;
  }

  // Confidence signal (docs/lane_change.md, "Persistence and confidence"): shortens
  // (never bypasses) the crossing-persistence window.
  const double effective_persist_duration =
    config_.crossing_persist_duration_s * (has_confidence_signal ? config_.confidence_factor : 1.0);
  return (now_s - crossing_start_s_) >= effective_persist_duration;
}

bool LaneChangeClassifier::accumulate_settle(
  const LaneChangeObservation & observation, double now_s)
{
  if (!observation.settle_lane_id) {
    settle_active_ = false;
    return false;
  }
  if (!settle_active_ || settle_lane_id_ != *observation.settle_lane_id) {
    settle_active_ = true;
    settle_lane_id_ = *observation.settle_lane_id;
    settle_start_s_ = now_s;
  }
  return (now_s - settle_start_s_) >= config_.settle_confirm_duration_s;
}

bool LaneChangeClassifier::has_confidence_signal(
  const LaneEventInput & input, const LaneChangeObservation & observation,
  const LaneChangeCrossing & crossing)
{
  // Confidence signal (docs/lane_change.md, "Persistence and confidence"): the whole footprint has
  // left the route primitives, or the blinker points at the target.
  return observation.is_footprint_off_route_primitives ||
         is_blinker_toward_target(crossing, input.turn_indicator);
}

void LaneChangeClassifier::update(const LaneEventInput & input)
{
  const double now_s = stamp_to_seconds(input);
  const LaneChangeObservation observation = geometry_.observe(tracker_, input);

  switch (phase_) {
    case Phase::idle:
      detect_onset(input, observation, now_s);
      break;
    case Phase::changing:
      detect_completion_or_abort(observation, now_s);
      break;
    case Phase::aborting:
      detect_abort_completion_or_recommit(input, observation, now_s);
      break;
  }
}

void LaneChangeClassifier::detect_onset(
  const LaneEventInput & input, const LaneChangeObservation & observation, double now_s)
{
  if (!observation.crossing) {
    tracked_crossing_.reset();
    return;
  }
  const bool confidence = has_confidence_signal(input, observation, *observation.crossing);
  if (accumulate_crossing(*observation.crossing, now_s, confidence)) {
    debug_reason_ = fmt::format(
      "onset: trajectory crossing to lane {} persisted", tracked_crossing_->target_lane_id);
    phase_ = Phase::changing;
    reset_timers();
  }
}

void LaneChangeClassifier::detect_completion_or_abort(
  const LaneChangeObservation & observation, double now_s)
{
  // Completion / settle (docs/lane_change.md, "Finishing or aborting"): footprint fully inside a
  // target route primitive for the settle window.
  if (accumulate_settle(observation, now_s)) {
    debug_reason_ =
      fmt::format("settle: footprint fully inside route primitive {}", settle_lane_id_);
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // Abort onset (docs/lane_change.md, "Finishing or aborting"): trajectory heads back into the
  // reference lane, persisted like onset.
  if (!observation.trajectory_returns_to_reference) {
    abort_active_ = false;
    return;
  }
  if (!abort_active_) {
    abort_active_ = true;
    abort_start_s_ = now_s;
  }
  if ((now_s - abort_start_s_) >= config_.crossing_persist_duration_s) {
    debug_reason_ = "abort: trajectory returned toward the reference lane";
    phase_ = Phase::aborting;
    reset_timers();
  }
}

void LaneChangeClassifier::detect_abort_completion_or_recommit(
  const LaneEventInput & input, const LaneChangeObservation & observation, double now_s)
{
  // Abort completion (docs/lane_change.md, "Aborting"): footprint fully back inside the reference
  // lane (geometric, no dwell).
  if (observation.is_footprint_inside_reference_lane) {
    debug_reason_ = "abort completed: footprint fully back inside the reference lane";
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // A settle stays live from ABORTING (docs/lane_change.md, "Settle still counts").
  if (accumulate_settle(observation, now_s)) {
    debug_reason_ = fmt::format(
      "settle from aborting: footprint fully inside route primitive {}", settle_lane_id_);
    phase_ = Phase::idle;
    reset_timers();
    return;
  }
  // Re-commit (docs/lane_change.md, "Aborting"): trajectory swings back toward the target lane,
  // persisted like onset.
  if (!observation.crossing) {
    tracked_crossing_.reset();
    return;
  }
  const bool confidence = has_confidence_signal(input, observation, *observation.crossing);
  if (accumulate_crossing(*observation.crossing, now_s, confidence)) {
    debug_reason_ = fmt::format(
      "re-commit: trajectory crossing to lane {} persisted", tracked_crossing_->target_lane_id);
    phase_ = Phase::changing;
    reset_timers();
  }
}

uint8_t LaneChangeClassifier::get_state() const
{
  switch (phase_) {
    case Phase::changing:
      return DrivingState::LANE_CHANGING;
    case Phase::aborting:
      return DrivingState::ABORTING_LANE_CHANGE;
    case Phase::idle:
    default:
      // No lane-change event: UNKNOWN (the node falls back to the lane-following gate).
      return DrivingState::UNKNOWN;
  }
}

bool LaneChangeClassifier::is_enabled() const
{
  return enabled_;
}

}  // namespace autoware::lane_event_classifier
