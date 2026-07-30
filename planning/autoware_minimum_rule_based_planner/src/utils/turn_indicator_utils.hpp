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

#ifndef UTILS__TURN_INDICATOR_UTILS_HPP_
#define UTILS__TURN_INDICATOR_UTILS_HPP_

#include <initializer_list>

// ---------------------------------------------------------------------------
// Pure turn-indicator decision rules: plain numbers in, direction out, so they can be unit-tested
// in isolation. The path/lanelet geometry that feeds them lives in turn_indicator_decider.cpp.
//
// Scope (see my_docs/TASK_mrbp_turn_indicator.md): intersection turns, private area exits, and
// departure/arrival at an off-centerline stop (bus stop, road shoulder). Lane changes and
// avoidance are explicitly OUT of scope - while the upstream planner performs one, this module
// must stay dark even though ego is laterally offset from its lane centerline.
// ---------------------------------------------------------------------------

namespace autoware::minimum_rule_based_planner::turn_indicator
{

enum class TurnDirection { NONE, LEFT, RIGHT };

//! Which maneuver a signal was raised for (priority resolution + debug).
enum class ManeuverKind { NONE, INTERSECTION, PRIVATE_EXIT, PULL_OUT, PULL_OVER };

//! Tunable thresholds (mirrors the param/ `turn_signal:` section).
struct TurnSignalParams
{
  //! [m] lead distance floor for a maneuver (JP law: 30 m before a turn), and the distance-to-goal
  //! at which the pull-over signal starts.
  double search_distance{30.0};
  double min_blink_duration{3.0};          //!< [s] min on-time once lit (anti-chatter)
  double stopped_velocity_threshold{0.1};  //!< [m/s] at/below this ego counts as stopped
  double heading_align_threshold{0.15};    //!< [rad] ego-vs-exit yaw gap that ends a maneuver
};

//! [s] lead time before a maneuver; the activation distance grows with speed above the floor.
constexpr double k_search_time = 3.0;
//! [m] how far past a maneuver its exit heading is measured, and how far behind ego a maneuver
//! still being completed is tracked.
constexpr double k_exit_lookahead = 15.0;
//! [m] distance-to-goal at which a stopped ego counts as arrived.
constexpr double k_goal_arrival_distance = 1.0;
//! [m] offset at/below which a lateral shift counts as finished; also the deadzone for asserting
//! which side an offset is on.
constexpr double k_lateral_shift_threshold = 0.5;
//! [m] offset above which a STOPPED ego counts as parked clear of the lane (bus stop, shoulder).
//! Must stay above the lateral deviation a lane change or avoidance can produce, or those would
//! blink too.
constexpr double k_departure_lateral_threshold = 1.5;

//! The emitted signal plus what raised it.
struct Signal
{
  TurnDirection direction{TurnDirection::NONE};
  ManeuverKind kind{ManeuverKind::NONE};
};

//! [m] distance ahead of a maneuver at which its signal comes on.
double activation_distance(double ego_velocity, const TurnSignalParams & params);

//! Signed offset beyond the deadzone => LEFT (positive) / RIGHT (negative), else NONE.
TurnDirection direction_from_lateral_offset(double signed_offset, double deadzone);

//! First candidate asking for a light. Callers pass them in priority order:
//! intersection > private exit > pull-out > pull-over.
Signal resolve_priority(std::initializer_list<Signal> candidates);

//! Lit while the maneuver start is within `activation_distance`, cleared once ego has entered the
//! maneuver and its heading matches `exit_yaw` (the spec's "ego aligned with the centerline" end
//! condition).
//! @param dist_to_start [m] arc length from ego to the maneuver start (negative once inside)
TurnDirection decide_maneuver_signal(
  TurnDirection direction, double dist_to_start, double ego_yaw, double exit_yaw,
  double ego_velocity, const TurnSignalParams & params);

//! Departure (pull-out): latches only while ego stands still clear of the lane centerline, which
//! is what keeps this module dark during a lane change or avoidance (offset, but moving). The
//! latch releases once ego is back on the centerline.
//! @param signed_offset ego lateral offset from its lane (+ = left of the centerline)
//! @param suppressed force-clear (keeps pull-out and pull-over from fighting over the direction)
//! @param[in,out] latched direction currently latched; NONE while not latched
TurnDirection decide_pull_out(
  double signed_offset, double ego_velocity, bool suppressed, const TurnSignalParams & params,
  TurnDirection & latched);

//! Arrival (pull-over): signals toward the side an off-centerline goal sits on while approaching
//! it, and clears once ego has stopped there so the signal does not stay lit after arrival.
//! @param goal_offset goal lateral offset from the goal lane centerline (+ = left)
//! @param[in,out] arrived latched arrival flag; the caller clears it when the route changes
TurnDirection decide_pull_over(
  double dist_to_goal, double goal_offset, double ego_velocity, const TurnSignalParams & params,
  bool & arrived);

//! Minimum-on-duration hold: a lit signal stays on for at least `min_duration` before it may
//! switch off; switching directly between left and right is allowed immediately.
class BlinkHold
{
public:
  explicit BlinkHold(double min_duration = 0.0) : min_duration_(min_duration) {}

  void set_min_duration(double min_duration) { min_duration_ = min_duration; }

  //! Update with the desired direction at time `now` [s]; returns what to emit.
  TurnDirection update(TurnDirection desired, double now);

  TurnDirection current() const { return held_; }

private:
  double min_duration_;
  TurnDirection held_{TurnDirection::NONE};
  double held_since_{0.0};
};

}  // namespace autoware::minimum_rule_based_planner::turn_indicator

#endif  // UTILS__TURN_INDICATOR_UTILS_HPP_
