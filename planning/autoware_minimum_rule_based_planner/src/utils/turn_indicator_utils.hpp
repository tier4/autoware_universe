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
// can be unit-tested in isolation (CLAUDE.md Rule 9). The lanelet/ROS-aware
// glue lives in turn_indicator_decider.{hpp,cpp}.
// ---------------------------------------------------------------------------

namespace autoware::minimum_rule_based_planner::turn_indicator
{

enum class TurnDirection { NONE, LEFT, RIGHT };

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
  double intersection_search_distance{30.0};  //!< [m] legal min lead distance for a turn (JP: 30m)
  double search_time{3.0};                    //!< [s] lead time; activation = max(v*t, distance)
  double lateral_shift_threshold{0.5};        //!< [m] min lateral offset to assert left/right
  double pull_over_search_distance{30.0};     //!< [m] distance-to-goal to start pull-over signal
  double min_blink_duration{3.0};             //!< [s] min on-time once lit (anti-chatter)
};

//! One upcoming (or currently occupied) turn segment along the path.
struct TurnSegment
{
  TurnDirection direction{TurnDirection::NONE};
  double dist_to_start{0.0};   //!< [m] arc length from ego to the segment start (>= 0)
  double dist_to_end{0.0};     //!< [m] arc length from ego to the segment end (>= dist_to_start)
  std::size_t start_index{0};  //!< index into the points vector of the segment start
  std::size_t end_index{0};    //!< index into the points vector of the segment end
};

//! First of the point's lane_ids that maps to a left/right direction, else NONE.
TurnDirection point_turn_direction(
  const PathPointLite & point, const std::unordered_map<int64_t, TurnDirection> & direction_of);

//! Index of the path point nearest to (x, y). Returns 0 for empty input.
std::size_t nearest_index(const std::vector<PathPointLite> & points, double x, double y);

//! First turn segment at or ahead of ego_index; nullopt if none carries a turn direction.
std::optional<TurnSegment> find_next_turn_segment(
  const std::vector<PathPointLite> & points, std::size_t ego_index,
  const std::unordered_map<int64_t, TurnDirection> & direction_of);

//! Lit while the segment start is within the activation distance and ego is before its end.
TurnDirection decide_intersection_signal(
  const std::optional<TurnSegment> & segment, double ego_velocity, const TurnSignalParams & params);

//! Signed offset beyond the deadzone => LEFT (positive) / RIGHT (negative), else NONE.
TurnDirection direction_from_lateral_offset(double signed_offset, double deadzone);

//! Fixed priority: intersection > pull_out > pull_over.
TurnDirection resolve_priority(
  TurnDirection intersection, TurnDirection pull_out, TurnDirection pull_over);

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
