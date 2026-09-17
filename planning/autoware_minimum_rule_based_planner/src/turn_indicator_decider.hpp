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

#ifndef TURN_INDICATOR_DECIDER_HPP_
#define TURN_INDICATOR_DECIDER_HPP_

#include "path_planner.hpp"
#include "type_alias.hpp"

#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <initializer_list>
#include <optional>

namespace autoware::minimum_rule_based_planner
{

namespace turn_indicator
{

enum class TurnDirection { NONE, LEFT, RIGHT };

enum class ManeuverKind { NONE, TURN, PULL_OUT, PULL_OVER };

//! Tunable thresholds (mirrors the param/ `turn_signal:` section).
struct TurnSignalParams
{
  double search_distance{30.0};            //! [m] activation distance floor
  double min_blink_duration{3.0};          //!< [s] min on-time once lit (anti-chatter)
  double stopped_velocity_threshold{0.1};  //!< [m/s] at/below this ego counts as stopped
  double heading_align_threshold{0.15};    //!< [rad] ego-vs-exit yaw gap that ends a maneuver
};

constexpr double k_search_time = 3.0;            //!< [s] lead time before a maneuver
constexpr double k_exit_lookahead = 15.0;        //!< [m] exit-heading lookahead / rear scan window
constexpr double k_goal_arrival_distance = 1.0;  //!< [m] distance-to-goal counted as arrived
constexpr double k_lateral_shift_threshold = 0.5;  //!< [m] shift finished / direction deadzone
//! [m] offset above which a STOPPED ego counts as parked clear of the lane (bus stop, shoulder).
//! Must exceed the lateral deviation a lane change or avoidance produces, or those blink too.
constexpr double k_departure_lateral_threshold = 1.5;

struct Signal
{
  TurnDirection direction{TurnDirection::NONE};
  ManeuverKind kind{ManeuverKind::NONE};
};

//! [m] distance ahead of a maneuver at which its signal comes on.
double activation_distance(double ego_velocity, const TurnSignalParams & params);

//! Signed offset beyond the deadzone => LEFT (positive) / RIGHT (negative), else NONE.
TurnDirection direction_from_lateral_offset(double signed_offset, double deadzone);

//! First candidate with a direction; callers pass them in priority order.
Signal resolve_priority(std::initializer_list<Signal> candidates);

//! Lit within `activation_distance` of the start, cleared once ego is inside it and its heading
//! matches `exit_yaw`.
//! @param dist_to_start [m] arc length from ego to the maneuver start (negative once inside)
TurnDirection decide_maneuver_signal(
  TurnDirection direction, double dist_to_start, double ego_yaw, double exit_yaw,
  double ego_velocity, const TurnSignalParams & params);

//! Latches only while ego stands still clear of the centerline, which is what keeps this module
//! dark during a lane change or avoidance (offset, but moving).
//! @param signed_offset ego lateral offset from its lane (+ = left of the centerline)
//! @param suppressed force-clear (keeps pull-out and pull-over from fighting over the direction)
TurnDirection decide_pull_out(
  double signed_offset, double ego_velocity, bool suppressed, const TurnSignalParams & params,
  TurnDirection & latched);

//! Signals toward the side an off-centerline goal sits on, and clears once ego has stopped there.
//! @param goal_offset goal lateral offset from the goal lane centerline (+ = left)
//! @param[in,out] arrived latched arrival flag; the caller clears it when the route changes
TurnDirection decide_pull_over(
  double dist_to_goal, double goal_offset, double ego_velocity, const TurnSignalParams & params,
  bool & arrived);

//! A lit signal stays on for at least `min_duration` before it may switch off; switching directly
//! between left and right is allowed immediately.
class BlinkHold
{
public:
  explicit BlinkHold(double min_duration = 0.0) : min_duration_(min_duration) {}

  void set_min_duration(double min_duration) { min_duration_ = min_duration; }

  //! @param now [s] current time
  TurnDirection update(TurnDirection desired, double now);

  TurnDirection current() const { return held_; }

private:
  double min_duration_;
  TurnDirection held_{TurnDirection::NONE};
  double held_since_{0.0};
};

}  // namespace turn_indicator

class TurnIndicatorDecider
{
public:
  explicit TurnIndicatorDecider(const turn_indicator::TurnSignalParams & params);

  void update_params(const turn_indicator::TurnSignalParams & params);

  TurnIndicatorsCommand decide(
    const PathWithLaneId & path, const RouteContext & route_context,
    const geometry_msgs::msg::Pose & ego_pose, double ego_velocity, const rclcpp::Time & stamp);

private:
  turn_indicator::TurnSignalParams params_;
  turn_indicator::BlinkHold blink_hold_;
  turn_indicator::TurnDirection pull_out_latch_{turn_indicator::TurnDirection::NONE};
  bool arrived_at_goal_{false};
  std::optional<geometry_msgs::msg::Pose> latched_goal_pose_;
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // TURN_INDICATOR_DECIDER_HPP_
