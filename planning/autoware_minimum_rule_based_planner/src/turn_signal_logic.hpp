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

#ifndef TURN_SIGNAL_LOGIC_HPP_
#define TURN_SIGNAL_LOGIC_HPP_

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

// ---------------------------------------------------------------------------
// Pure turn-signal decision logic.
//
// This header intentionally depends on NOTHING from ROS, lanelet2, or this
// package's message types. It operates on plain numbers so the decision rules
// can be unit-tested in isolation (CLAUDE.md Rule 9). The lanelet/ROS-aware
// glue lives in turn_indicator_decider.{hpp,cpp}.
// ---------------------------------------------------------------------------

namespace autoware::minimum_rule_based_planner::turn_signal
{

//! Side of a turn signal, independent of any message encoding.
enum class TurnDir { kNone, kLeft, kRight };

//! Minimal path point: position + the lanelet ids the point belongs to.
struct PathPointLite
{
  double x{0.0};
  double y{0.0};
  std::vector<int64_t> lane_ids{};
};

//! Tunable thresholds (mirrors the param/ `turn_signal:` section).
struct TurnSignalParams
{
  //! [m] legal minimum lead distance before an intersection turn (JP: 30m).
  double intersection_search_distance{30.0};
  //! [s] lead time; activation distance = max(v * search_time, search_distance).
  double search_time{3.0};
  //! [m] minimum |lateral offset| from the main-lane centerline to assert a left/right direction.
  double lateral_shift_threshold{0.5};
  //! [m] distance-to-goal at which the pull-over signal starts.
  double pull_over_search_distance{30.0};
  //! [s] minimum time a turn signal is held on once lit (anti-chatter).
  double min_blink_duration{3.0};
};

//! One upcoming (or currently occupied) turn segment along the path.
struct TurnSegment
{
  TurnDir direction{TurnDir::kNone};
  double dist_to_start{0.0};   //!< [m] arc length from ego to the segment start (>= 0)
  double dist_to_end{0.0};     //!< [m] arc length from ego to the segment end (>= dist_to_start)
  std::size_t start_index{0};  //!< index into the points vector of the segment start
  std::size_t end_index{0};    //!< index into the points vector of the segment end
};

/**
 * @brief Turn direction of a single point: the first of its lane_ids that maps
 *        to a left/right direction, else kNone.
 */
TurnDir point_turn_direction(
  const PathPointLite & point, const std::unordered_map<int64_t, TurnDir> & direction_of);

/**
 * @brief Index of the path point nearest to (x, y). Returns 0 for empty input.
 */
std::size_t nearest_index(const std::vector<PathPointLite> & points, double x, double y);

/**
 * @brief Find the first turn segment at or ahead of ego_index.
 * @return nullopt if the path is empty or no forward point carries a turn direction.
 */
std::optional<TurnSegment> find_next_turn_segment(
  const std::vector<PathPointLite> & points, std::size_t ego_index,
  const std::unordered_map<int64_t, TurnDir> & direction_of);

/**
 * @brief Decide the intersection signal from the nearest turn segment and ego speed.
 *        Lit while the segment start is within the activation distance and ego has
 *        not yet passed the segment end.
 */
TurnDir decide_intersection_signal(
  const std::optional<TurnSegment> & segment, double ego_velocity, const TurnSignalParams & params);

/**
 * @brief Map a signed lateral offset to a direction using a symmetric deadzone.
 *        Positive offset beyond the deadzone => kLeft, negative => kRight.
 */
TurnDir direction_from_lateral_offset(double signed_offset, double deadzone);

/**
 * @brief Resolve simultaneous candidates by fixed priority:
 *        intersection > pull_out > pull_over.
 */
TurnDir resolve_priority(TurnDir intersection, TurnDir pull_out, TurnDir pull_over);

/**
 * @brief Minimum-on-duration hold to suppress turn-signal chatter.
 *
 * A lit signal is held for at least `min_duration` seconds before it is allowed
 * to switch off. Switching directly between left and right (a new maneuver) is
 * always allowed immediately.
 */
class BlinkHold
{
public:
  explicit BlinkHold(double min_duration = 0.0) : min_duration_(min_duration) {}

  void set_min_duration(double min_duration) { min_duration_ = min_duration; }

  /**
   * @brief Update the held command with the desired direction at time `now` [s].
   * @return the direction that should actually be emitted.
   */
  TurnDir update(TurnDir desired, double now);

  TurnDir current() const { return held_; }

private:
  double min_duration_;
  TurnDir held_{TurnDir::kNone};
  double held_since_{0.0};
};

}  // namespace autoware::minimum_rule_based_planner::turn_signal

#endif  // TURN_SIGNAL_LOGIC_HPP_
