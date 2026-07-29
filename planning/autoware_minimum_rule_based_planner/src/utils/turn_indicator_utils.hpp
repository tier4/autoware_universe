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

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------------------
// Pure turn-indicator decision logic.
//
// This header intentionally depends on NOTHING from ROS, lanelet2, or this
// package's message types. It operates on plain numbers so the decision rules
// can be unit-tested in isolation. The lanelet/ROS-aware glue lives in
// turn_indicator_decider.{hpp,cpp}.
//
// Scope (see my_docs/TASK_mrbp_turn_indicator.md): intersection turns, private
// area exits, and departure/arrival at an off-centerline stop (bus stop, road
// shoulder). Lane changes and avoidance are explicitly OUT of scope - while the
// upstream planner performs one, this module must stay dark even though ego is
// laterally offset from its lane centerline.
// ---------------------------------------------------------------------------

namespace autoware::minimum_rule_based_planner::turn_indicator
{

enum class TurnDirection { NONE, LEFT, RIGHT };

//! Which maneuver a signal was raised for (priority resolution + debug).
enum class ManeuverKind { NONE, INTERSECTION, PRIVATE_EXIT, PULL_OUT, PULL_OVER };

//! Minimal path point: position + the lanelet ids the point belongs to.
struct PathPointLite
{
  double x{0.0};
  double y{0.0};
  std::vector<int64_t> lane_ids{};
};

//! The per-lanelet map attributes this decision needs.
struct LaneAttribute
{
  TurnDirection turn_direction{TurnDirection::NONE};  //!< `turn_direction` tag
  bool is_private{false};                             //!< `location` tag == "private"
};

using LaneAttributeMap = std::unordered_map<int64_t, LaneAttribute>;

//! Tunable thresholds (mirrors the param/ `turn_signal:` section).
struct TurnSignalParams
{
  double intersection_search_distance{30.0};  //!< [m] legal min lead distance for a turn (JP: 30m)
  double search_time{3.0};                    //!< [s] lead time; activation = max(v*t, distance)
  double lateral_shift_threshold{0.5};        //!< [m] offset at/below which a shift is "finished"
  double pull_over_search_distance{30.0};     //!< [m] distance-to-goal to start pull-over signal
  double min_blink_duration{3.0};             //!< [s] min on-time once lit (anti-chatter)
  double departure_lateral_threshold{1.5};    //!< [m] offset that qualifies as "parked off-lane"
  double stopped_velocity_threshold{0.1};     //!< [m/s] at/below this ego counts as stopped
  double heading_align_threshold{0.15};       //!< [rad] ego-vs-exit yaw gap that ends a maneuver
  double exit_lookahead{15.0};                //!< [m] look-ahead past a maneuver for its exit yaw
  double goal_arrival_distance{1.0};          //!< [m] distance-to-goal that counts as arrived
};

//! One upcoming (or currently occupied) maneuver segment along the path.
struct ManeuverSegment
{
  TurnDirection direction{TurnDirection::NONE};
  ManeuverKind kind{ManeuverKind::NONE};
  double dist_to_start{0.0};   //!< [m] arc length from ego to the segment start (>= 0)
  double dist_to_end{0.0};     //!< [m] arc length from ego to the segment end (>= dist_to_start)
  std::size_t start_index{0};  //!< index into the points vector of the segment start
  std::size_t end_index{0};    //!< index into the points vector of the segment end
  double exit_yaw{0.0};        //!< [rad] path heading once the maneuver is complete
};

//! The emitted signal plus what raised it.
struct SignalDecision
{
  TurnDirection direction{TurnDirection::NONE};
  ManeuverKind kind{ManeuverKind::NONE};
};

//! Wrap to (-pi, pi].
double normalize_angle(double angle);

//! Path heading at `index`, from the neighbouring point. 0.0 for a degenerate (< 2 point) path.
double path_yaw_at(const std::vector<PathPointLite> & points, std::size_t index);

//! First of the point's lane_ids that maps to a left/right direction, else NONE.
TurnDirection point_turn_direction(const PathPointLite & point, const LaneAttributeMap & attrs);

//! True if any lanelet the point belongs to is tagged `location=private`.
bool point_is_private(const PathPointLite & point, const LaneAttributeMap & attrs);

//! Index of the path point nearest to (x, y), ignoring points whose path heading opposes `yaw`
//! so that a route folding back on itself (U-turn, loop) cannot snap ego onto the wrong leg.
//! Falls back to the plain nearest point when no candidate agrees with `yaw`. 0 for empty input.
std::size_t nearest_index(
  const std::vector<PathPointLite> & points, double x, double y, double yaw);

//! All `turn_direction`-tagged segments along the path, ordered from ego. The scan starts
//! `exit_lookahead` metres BEHIND ego (so a turn ego is still completing, whose lanelet is already
//! behind, is still reported) and each segment's end is carried `exit_lookahead` metres past its
//! lanelet so that `exit_yaw` holds the heading the turn ends on.
std::vector<ManeuverSegment> find_turn_segments(
  const std::vector<PathPointLite> & points, std::size_t ego_index, const LaneAttributeMap & attrs,
  const TurnSignalParams & params);

//! All private-area exits (a `location=private` run that rejoins a public lane) along the path.
//! Same scan window as find_turn_segments. The direction comes from the exit lanelet's
//! `turn_direction` when tagged, otherwise from the path's yaw change across the merge; a merge
//! that is geometrically straight is skipped - there is no side to signal.
std::vector<ManeuverSegment> find_private_exit_segments(
  const std::vector<PathPointLite> & points, std::size_t ego_index, const LaneAttributeMap & attrs,
  const TurnSignalParams & params);

//! Lit while the segment start is within the activation distance, cleared once ego has entered the
//! segment and its heading matches `exit_yaw` (the spec's "ego aligned with the centerline" end
//! condition) or the segment has been passed.
TurnDirection decide_maneuver_signal(
  const ManeuverSegment & segment, double ego_velocity, double ego_yaw,
  const TurnSignalParams & params);

//! The signal a set of segments asks for, plus the segment that raised it (for debug).
struct ActiveManeuver
{
  TurnDirection direction{TurnDirection::NONE};
  std::optional<ManeuverSegment> segment{std::nullopt};
};

//! First segment of `segments` that asks for a signal, else an empty decision.
ActiveManeuver decide_maneuver_signal(
  const std::vector<ManeuverSegment> & segments, double ego_velocity, double ego_yaw,
  const TurnSignalParams & params);

//! Signed offset beyond the deadzone => LEFT (positive) / RIGHT (negative), else NONE.
TurnDirection direction_from_lateral_offset(double signed_offset, double deadzone);

//! Fixed priority: intersection > private exit > pull_out > pull_over.
SignalDecision resolve_priority(
  TurnDirection intersection, TurnDirection private_exit, TurnDirection pull_out,
  TurnDirection pull_over);

//! Departure (pull-out) detector.
//!
//! Latches only when ego is STOPPED while parked clear of the lane centerline
//! (|offset| > departure_lateral_threshold) - the signature of a bus stop / shoulder start. This
//! is what keeps the module dark during a lane change or avoidance: there ego is offset but
//! moving, so it never latches. The latch releases once ego is back within
//! lateral_shift_threshold of the centerline, i.e. when the departure is complete.
class DepartureLatch
{
public:
  //! @param signed_offset ego lateral offset from the nearest route lane (+ = left of centerline)
  //! @param suppressed force-clear (used to keep pull-out and pull-over mutually exclusive)
  TurnDirection update(
    double signed_offset, double ego_velocity, bool suppressed, const TurnSignalParams & params);

  void reset();

  bool latched() const { return latched_; }

  //! What the latch currently emits (used to ride out cycles with no usable offset).
  TurnDirection direction() const { return latched_ ? direction_ : TurnDirection::NONE; }

private:
  bool latched_{false};
  TurnDirection direction_{TurnDirection::NONE};
};

//! Arrival (pull-over) detector: signals toward the side the goal sits on while approaching it,
//! and clears once ego has stopped at the goal so the signal does not stay lit after arrival.
class ArrivalState
{
public:
  //! @param goal_offset goal lateral offset from the goal lane centerline (+ = left)
  TurnDirection update(
    double dist_to_goal, double goal_offset, double ego_velocity, const TurnSignalParams & params);

  //! Call when the route (goal) changes.
  void reset();

  bool arrived() const { return arrived_; }

private:
  bool arrived_{false};
};

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
