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

#include "turn_indicator_utils.hpp"

#include <autoware_utils_math/normalization.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace autoware::minimum_rule_based_planner::turn_indicator
{

double activation_distance(const double ego_velocity, const TurnSignalParams & params)
{
  return std::max(ego_velocity * k_search_time, params.search_distance);
}

TurnDirection direction_from_lateral_offset(const double signed_offset, const double deadzone)
{
  if (signed_offset > deadzone) {
    return TurnDirection::LEFT;
  }
  if (signed_offset < -deadzone) {
    return TurnDirection::RIGHT;
  }
  return TurnDirection::NONE;
}

Signal resolve_priority(const std::initializer_list<Signal> candidates)
{
  for (const auto & candidate : candidates) {
    if (candidate.direction != TurnDirection::NONE) {
      return candidate;
    }
  }
  return {};
}

TurnDirection decide_maneuver_signal(
  const TurnDirection direction, const double dist_to_start, const double ego_yaw,
  const double exit_yaw, const double ego_velocity, const TurnSignalParams & params)
{
  if (direction == TurnDirection::NONE) {
    return TurnDirection::NONE;
  }
  // Spec end condition: once the maneuver has started it is over when ego's heading matches the
  // heading it exits on, i.e. ego is aligned with the centerline again. The arc length may be
  // slightly negative once ego is inside, because the start is then behind ego.
  constexpr double entered_tolerance = 1e-3;
  const double yaw_gap = std::abs(autoware_utils_math::normalize_radian(ego_yaw - exit_yaw));
  if (dist_to_start <= entered_tolerance && yaw_gap <= params.heading_align_threshold) {
    return TurnDirection::NONE;
  }
  if (dist_to_start <= activation_distance(ego_velocity, params)) {
    return direction;
  }
  return TurnDirection::NONE;
}

TurnDirection decide_pull_out(
  const double signed_offset, const double ego_velocity, const bool suppressed,
  const TurnSignalParams & params, TurnDirection & latched)
{
  const double offset_magnitude = std::abs(signed_offset);

  // Suppressed, or back on the centerline: the departure is complete (or must not start).
  if (suppressed || offset_magnitude <= k_lateral_shift_threshold) {
    latched = TurnDirection::NONE;
    return TurnDirection::NONE;
  }

  if (latched == TurnDirection::NONE) {
    // Only a vehicle standing still, clear of the lane, is departing from a stop. A vehicle that
    // is offset while moving is being pushed off the centerline by the upstream planner (lane
    // change / avoidance), which is explicitly out of scope - stay dark.
    const bool stopped = std::abs(ego_velocity) <= params.stopped_velocity_threshold;
    if (!stopped || offset_magnitude <= k_departure_lateral_threshold) {
      return TurnDirection::NONE;
    }
    // Ego pulls out towards the centerline, i.e. against its own offset.
    latched = direction_from_lateral_offset(-signed_offset, k_lateral_shift_threshold);
  }

  return latched;
}

TurnDirection decide_pull_over(
  const double dist_to_goal, const double goal_offset, const double ego_velocity,
  const TurnSignalParams & params, bool & arrived)
{
  if (dist_to_goal > params.search_distance) {
    arrived = false;  // re-arm for the next approach (e.g. a new route)
    return TurnDirection::NONE;
  }

  if (
    dist_to_goal <= k_goal_arrival_distance &&
    std::abs(ego_velocity) <= params.stopped_velocity_threshold) {
    arrived = true;
  }
  if (arrived) {
    return TurnDirection::NONE;  // maneuver finished: turn the signal off
  }

  return direction_from_lateral_offset(goal_offset, k_lateral_shift_threshold);
}

TurnDirection BlinkHold::update(const TurnDirection desired, const double now)
{
  if (desired == held_) {
    return held_;
  }

  // Hold a lit signal through an off-transition until the minimum duration has elapsed.
  const bool turning_off = desired == TurnDirection::NONE;
  if (turning_off && held_ != TurnDirection::NONE && (now - held_since_) < min_duration_) {
    return held_;
  }

  held_ = desired;
  held_since_ = now;
  return held_;
}

}  // namespace autoware::minimum_rule_based_planner::turn_indicator
