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

#include <utility>
#include <vector>

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
bool is_blinker_toward_crossing_side(const LaneCrossingCrossing & crossing, uint8_t turn_indicator)
{
  if (crossing.is_to_left) {
    return turn_indicator == TurnIndicatorsReport::ENABLE_LEFT;
  }
  return turn_indicator == TurnIndicatorsReport::ENABLE_RIGHT;
}
}  // namespace

IntentionalCrossingClassifier::IntentionalCrossingClassifier(
  bool enabled, LaneCrossingConfig config, const LaneTracker & tracker,
  LaneCrossingGeometry geometry, LaneCrossingObjects objects)
: enabled_{enabled},
  config_{config},
  tracker_{tracker},
  geometry_{std::move(geometry)},
  objects_{std::move(objects)}
{
}

void IntentionalCrossingClassifier::reset_timers()
{
  tracked_crossing_.reset();
  crossing_start_s_ = 0.0;
  return_active_ = false;
  return_start_s_ = 0.0;
  remembered_candidate_poses_.clear();
  last_candidate_seen_s_ = 0.0;
}

bool IntentionalCrossingClassifier::accumulate_crossing(
  const LaneCrossingCrossing & crossing, double now_s, bool has_confidence_signal)
{
  // Persistence (docs/lane_crossing.md, "Persistence"): same side + stable crossing point.
  const bool matches_tracked =
    tracked_crossing_ && tracked_crossing_->is_to_left == crossing.is_to_left &&
    (crossing.crossing_point - tracked_crossing_->crossing_point).norm() <=
      config_.crossing_position_tolerance_m;
  if (!matches_tracked) {
    tracked_crossing_ = crossing;  // anchor the crossing location; restart the window
    crossing_start_s_ = now_s;
  }

  // Confidence signal (docs/lane_crossing.md, "Confidence signal"): shortens the window.
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

std::vector<geometry_msgs::msg::Pose>
IntentionalCrossingClassifier::effective_candidate_object_poses(
  std::vector<geometry_msgs::msg::Pose> perceived_poses, double now_s)
{
  // Candidate memory (docs/lane_crossing.md, "Candidate object"): bridges a perception dropout.
  if (!perceived_poses.empty()) {
    last_candidate_seen_s_ = now_s;
    remembered_candidate_poses_ = perceived_poses;
    return perceived_poses;
  }
  if (
    !remembered_candidate_poses_.empty() &&
    (now_s - last_candidate_seen_s_) <= config_.object_qualifying_memory_s) {
    return remembered_candidate_poses_;
  }
  remembered_candidate_poses_.clear();
  return {};
}

void IntentionalCrossingClassifier::update(const LaneEventInput & input)
{
  const double now_s = stamp_to_seconds(input);
  const LaneCrossingObjects::Result objects = objects_.observe(tracker_, input);
  const auto candidate_poses =
    effective_candidate_object_poses(objects.candidate_object_poses, now_s);
  const LaneCrossingObservation observation = geometry_.observe(tracker_, input, candidate_poses);

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
  // Onset (docs/lane_crossing.md, "Onset"): the candidate requirement is folded into the crossing.
  if (!observation.crossing) {
    tracked_crossing_.reset();
    // Per-cycle diagnostic (surfaced throttled by the node): why onset did not fire this cycle.
    debug_reason_ = fmt::format(
      "idle: on_route_straight={} | crossing: {}", observation.is_on_route_straight ? "yes" : "no",
      observation.crossing_diagnostic);
    return;
  }
  const bool confidence = has_confidence_signal(input, *observation.crossing);
  if (accumulate_crossing(*observation.crossing, now_s, confidence)) {
    debug_reason_ = fmt::format(
      "onset: trajectory brackets a candidate object, crossing the {} boundary",
      tracked_crossing_->is_to_left ? "left" : "right");
    phase_ = Phase::crossing;
    reset_timers();
  }
}

void IntentionalCrossingClassifier::detect_completion(
  const LaneCrossingObservation & observation, double now_s)
{
  // Finishing (docs/lane_crossing.md, "Finishing"): end once fully inside one lane; full entry
  // checked before return, else hold. No time cap.

  // Full entry: the ego is fully in the neighbour, hand the move to the lane-change classifier.
  if (observation.full_entry_lane_id) {
    debug_reason_ = fmt::format(
      "ended: footprint fully entered lane {} (now a lane change)",
      *observation.full_entry_lane_id);
    phase_ = Phase::idle;
    reset_timers();
    return;
  }

  // Return: footprint fully back inside the route straight sequence for the settle window.
  if (accumulate_return(observation, now_s)) {
    debug_reason_ = "completed: footprint returned fully into the route sequence";
    phase_ = Phase::idle;
    reset_timers();
    return;
  }

  // Otherwise straddling: not fully inside any lane yet, so hold the crossing.
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
