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

#include "utils/turn_indicator_utils.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace autoware::minimum_rule_based_planner::turn_indicator
{
namespace
{

//! Straight path along +x at 1 m spacing. Points [0, straight_len) carry `straight_id`,
//! the rest carry `turn_id` — modelling a turn lanelet starting at x = straight_len.
std::vector<PathPointLite> make_path_with_turn(
  std::size_t total, std::size_t straight_len, int64_t straight_id, int64_t turn_id)
{
  std::vector<PathPointLite> points;
  for (std::size_t i = 0; i < total; ++i) {
    const int64_t id = i < straight_len ? straight_id : turn_id;
    points.push_back(PathPointLite{static_cast<double>(i), 0.0, {id}});
  }
  return points;
}

TurnSignalParams default_params()
{
  return TurnSignalParams{};  // 30 m / 3 s / 0.5 m / 30 m / 3 s defaults
}

}  // namespace

// ===========================================================================
// point_turn_direction
// ===========================================================================

TEST(TurnSignalLogic, PointTurnDirectionPicksFirstNonNoneLane)
{
  // Intent: a point that overlaps several lanelets should signal a turn if ANY of
  // its lanelets is a turn lanelet, so overlap/lane-change points are not missed.
  const std::unordered_map<int64_t, TurnDirection> dir = {
    {1, TurnDirection::NONE}, {2, TurnDirection::RIGHT}};

  EXPECT_EQ(point_turn_direction(PathPointLite{0, 0, {1}}, dir), TurnDirection::NONE);
  EXPECT_EQ(point_turn_direction(PathPointLite{0, 0, {1, 2}}, dir), TurnDirection::RIGHT);
  EXPECT_EQ(point_turn_direction(PathPointLite{0, 0, {99}}, dir), TurnDirection::NONE);
}

// ===========================================================================
// nearest_index
// ===========================================================================

TEST(TurnSignalLogic, NearestIndexFindsClosestPoint)
{
  const auto points = make_path_with_turn(10, 10, 1, 1);
  EXPECT_EQ(nearest_index(points, 0.1, 0.0), 0u);
  EXPECT_EQ(nearest_index(points, 4.6, 0.0), 5u);
  EXPECT_EQ(nearest_index(points, 100.0, 0.0), 9u);
}

// ===========================================================================
// find_next_turn_segment
// ===========================================================================

TEST(TurnSignalLogic, NoTurnSegmentOnStraightPath)
{
  // Intent: a route with no turn lanelet must never raise a turn segment (no false blink).
  const auto points = make_path_with_turn(50, 50, 1, 1);
  const std::unordered_map<int64_t, TurnDirection> dir = {{1, TurnDirection::NONE}};
  EXPECT_FALSE(find_next_turn_segment(points, 0, dir).has_value());
}

TEST(TurnSignalLogic, TurnSegmentReportsDistanceFromEgo)
{
  // Straight for 40 m, then a left turn lanelet. Ego at index 0.
  const auto points = make_path_with_turn(60, 40, 1, 2);
  const std::unordered_map<int64_t, TurnDirection> dir = {
    {1, TurnDirection::NONE}, {2, TurnDirection::LEFT}};

  const auto seg = find_next_turn_segment(points, 0, dir);
  ASSERT_TRUE(seg.has_value());
  EXPECT_EQ(seg->direction, TurnDirection::LEFT);
  EXPECT_NEAR(seg->dist_to_start, 40.0, 1e-6);  // turn lanelet starts 40 m ahead
  EXPECT_NEAR(seg->dist_to_end, 59.0, 1e-6);    // and continues to the path end
  // Indices map back to the path points (used to place debug markers at the real positions).
  EXPECT_EQ(seg->start_index, 40u);
  EXPECT_EQ(seg->end_index, 59u);
}

TEST(TurnSignalLogic, TurnSegmentDistanceShrinksAsEgoAdvances)
{
  // Intent: distance-to-turn must be measured from ego, so it shrinks as ego moves up.
  const auto points = make_path_with_turn(60, 40, 1, 2);
  const std::unordered_map<int64_t, TurnDirection> dir = {
    {1, TurnDirection::NONE}, {2, TurnDirection::LEFT}};

  const auto seg = find_next_turn_segment(points, 15, dir);
  ASSERT_TRUE(seg.has_value());
  EXPECT_NEAR(seg->dist_to_start, 25.0, 1e-6);  // 40 m - 15 m travelled
}

// ===========================================================================
// decide_intersection_signal  (the "light up 30 m ahead" rule)
// ===========================================================================

TEST(TurnSignalLogic, IntersectionStaysOffBeyondActivationDistance)
{
  // Intent: at standstill the signal must NOT light until within 30 m (legal lead distance).
  const auto params = default_params();
  const TurnSegment far{TurnDirection::LEFT, 40.0, 59.0};
  EXPECT_EQ(decide_intersection_signal(far, /*ego_velocity=*/0.0, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, IntersectionLightsWithinThirtyMeters)
{
  // Intent: the core legal requirement — blink once the turn is within 30 m.
  const auto params = default_params();
  const TurnSegment near{TurnDirection::LEFT, 25.0, 44.0};
  EXPECT_EQ(decide_intersection_signal(near, /*ego_velocity=*/0.0, params), TurnDirection::LEFT);
}

TEST(TurnSignalLogic, IntersectionActivationExtendsWithSpeed)
{
  // Intent: at speed the lead distance is v * search_time, so a turn 40 m ahead lights
  // already at 20 m/s (activation = 60 m) even though it exceeds the 30 m floor.
  const auto params = default_params();
  const TurnSegment seg{TurnDirection::RIGHT, 40.0, 59.0};
  EXPECT_EQ(decide_intersection_signal(seg, /*ego_velocity=*/20.0, params), TurnDirection::RIGHT);
  EXPECT_EQ(decide_intersection_signal(seg, /*ego_velocity=*/0.0, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, IntersectionTurnsOffAfterPassingSegment)
{
  // Intent: once ego has passed the turn lanelet (end behind ego) the signal clears.
  const auto params = default_params();
  const TurnSegment passed{TurnDirection::LEFT, -5.0, -1.0};
  EXPECT_EQ(decide_intersection_signal(passed, 0.0, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, IntersectionNoSegmentIsOff)
{
  EXPECT_EQ(decide_intersection_signal(std::nullopt, 5.0, default_params()), TurnDirection::NONE);
}

// ===========================================================================
// direction_from_lateral_offset  (pull-out / pull-over direction)
// ===========================================================================

TEST(TurnSignalLogic, DirectionFromLateralOffsetSign)
{
  // Intent: positive offset (left of the main-lane centerline) => left, negative => right.
  // This is the sign rule shared by behavior_path's TurnSignalDecider relative-shift logic.
  EXPECT_EQ(direction_from_lateral_offset(1.5, 0.3), TurnDirection::LEFT);
  EXPECT_EQ(direction_from_lateral_offset(-1.5, 0.3), TurnDirection::RIGHT);
}

TEST(TurnSignalLogic, DirectionDeadzoneSuppressesSmallOffsets)
{
  // Intent: offsets within the deadzone must not assert a direction (no false blink on jitter).
  EXPECT_EQ(direction_from_lateral_offset(0.2, 0.3), TurnDirection::NONE);
  EXPECT_EQ(direction_from_lateral_offset(-0.2, 0.3), TurnDirection::NONE);
  EXPECT_EQ(direction_from_lateral_offset(0.0, 0.3), TurnDirection::NONE);
}

// ===========================================================================
// resolve_priority
// ===========================================================================

TEST(TurnSignalLogic, PriorityIntersectionOverShoulder)
{
  // Intent: an active intersection turn outranks pull-out/pull-over candidates.
  EXPECT_EQ(
    resolve_priority(TurnDirection::RIGHT, TurnDirection::LEFT, TurnDirection::LEFT),
    TurnDirection::RIGHT);
  EXPECT_EQ(
    resolve_priority(TurnDirection::NONE, TurnDirection::LEFT, TurnDirection::RIGHT),
    TurnDirection::LEFT);
  EXPECT_EQ(
    resolve_priority(TurnDirection::NONE, TurnDirection::NONE, TurnDirection::RIGHT),
    TurnDirection::RIGHT);
  EXPECT_EQ(
    resolve_priority(TurnDirection::NONE, TurnDirection::NONE, TurnDirection::NONE),
    TurnDirection::NONE);
}

// ===========================================================================
// BlinkHold  (anti-chatter hysteresis)
// ===========================================================================

TEST(TurnSignalLogic, BlinkHoldKeepsSignalForMinimumDuration)
{
  // Intent: once lit, the signal must stay on for min_duration even if the desired
  // command immediately drops to none — this is what prevents flicker.
  BlinkHold hold(3.0);
  EXPECT_EQ(hold.update(TurnDirection::LEFT, 0.0), TurnDirection::LEFT);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 1.0), TurnDirection::LEFT);  // held: only 1 s elapsed
  EXPECT_EQ(hold.update(TurnDirection::NONE, 2.9), TurnDirection::LEFT);  // still held
  EXPECT_EQ(hold.update(TurnDirection::NONE, 3.5), TurnDirection::NONE);  // released after 3 s
}

TEST(TurnSignalLogic, BlinkHoldSwitchesDirectionImmediately)
{
  // Intent: switching to a new maneuver (left -> right) is a real command change and
  // must apply at once, not wait out the minimum-on timer.
  BlinkHold hold(3.0);
  EXPECT_EQ(hold.update(TurnDirection::LEFT, 0.0), TurnDirection::LEFT);
  EXPECT_EQ(hold.update(TurnDirection::RIGHT, 0.5), TurnDirection::RIGHT);
}

TEST(TurnSignalLogic, BlinkHoldStaysOffWhenIdle)
{
  BlinkHold hold(3.0);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 0.0), TurnDirection::NONE);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 5.0), TurnDirection::NONE);
}

}  // namespace autoware::minimum_rule_based_planner::turn_indicator
