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

#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::minimum_rule_based_planner::turn_indicator
{
namespace
{

constexpr double kStraightYaw = 0.0;  //!< heading of every straight test path below (+x)

TurnSignalParams default_params()
{
  return TurnSignalParams{};
}

//! Straight path along +x at 1 m spacing. Points [0, straight_len) carry `straight_id`,
//! the rest carry `turn_id` - modelling a turn lanelet starting at x = straight_len.
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

//! Straight along +x for `straight_len` m, then a 90-deg left arc, then straight along +y.
//! Points up to `straight_len` carry `straight_id`; the arc and the tail carry `turn_id` and
//! `exit_id` respectively.
std::vector<PathPointLite> make_left_turn_path(
  std::size_t straight_len, int64_t straight_id, int64_t turn_id, int64_t exit_id)
{
  std::vector<PathPointLite> points;
  for (std::size_t i = 0; i < straight_len; ++i) {
    points.push_back(PathPointLite{static_cast<double>(i), 0.0, {straight_id}});
  }
  // Quarter circle of radius 10 m, centred at (straight_len - 1, 10).
  const double cx = static_cast<double>(straight_len) - 1.0;
  const double radius = 10.0;
  for (int deg = 1; deg <= 90; deg += 3) {
    const double rad = deg * M_PI / 180.0;
    points.push_back(
      PathPointLite{cx + radius * std::sin(rad), radius - radius * std::cos(rad), {turn_id}});
  }
  // Straight along +y after the turn.
  for (int i = 1; i <= 30; ++i) {
    points.push_back(PathPointLite{cx + radius, radius + static_cast<double>(i), {exit_id}});
  }
  return points;
}

LaneAttributeMap turn_attrs(int64_t straight_id, int64_t turn_id, TurnDirection dir)
{
  LaneAttributeMap attrs;
  attrs[straight_id] = LaneAttribute{TurnDirection::NONE, false};
  attrs[turn_id] = LaneAttribute{dir, false};
  return attrs;
}

}  // namespace

// ===========================================================================
// point attribute lookup
// ===========================================================================

TEST(TurnSignalLogic, PointTurnDirectionPicksFirstNonNoneLane)
{
  // Intent: a point that overlaps several lanelets should signal a turn if ANY of
  // its lanelets is a turn lanelet, so overlap points are not missed.
  LaneAttributeMap attrs;
  attrs[1] = LaneAttribute{TurnDirection::NONE, false};
  attrs[2] = LaneAttribute{TurnDirection::RIGHT, false};

  EXPECT_EQ(point_turn_direction(PathPointLite{0, 0, {1}}, attrs), TurnDirection::NONE);
  EXPECT_EQ(point_turn_direction(PathPointLite{0, 0, {1, 2}}, attrs), TurnDirection::RIGHT);
  EXPECT_EQ(point_turn_direction(PathPointLite{0, 0, {99}}, attrs), TurnDirection::NONE);
}

TEST(TurnSignalLogic, PointIsPrivateWhenAnyLaneIsPrivate)
{
  LaneAttributeMap attrs;
  attrs[1] = LaneAttribute{TurnDirection::NONE, false};
  attrs[2] = LaneAttribute{TurnDirection::NONE, true};

  EXPECT_FALSE(point_is_private(PathPointLite{0, 0, {1}}, attrs));
  EXPECT_TRUE(point_is_private(PathPointLite{0, 0, {1, 2}}, attrs));
  EXPECT_FALSE(point_is_private(PathPointLite{0, 0, {99}}, attrs));
}

// ===========================================================================
// nearest_index
// ===========================================================================

TEST(TurnSignalLogic, NearestIndexFindsClosestPoint)
{
  const auto points = make_path_with_turn(10, 10, 1, 1);
  EXPECT_EQ(nearest_index(points, 0.1, 0.0, kStraightYaw), 0u);
  EXPECT_EQ(nearest_index(points, 4.6, 0.0, kStraightYaw), 5u);
  EXPECT_EQ(nearest_index(points, 100.0, 0.0, kStraightYaw), 9u);
}

TEST(TurnSignalLogic, NearestIndexIgnoresOppositeDirectionLeg)
{
  // Intent: on a path that folds back on itself (U-turn / loop), ego must not snap onto the
  // leg the path traverses in the opposite direction even when it is geometrically closer.
  std::vector<PathPointLite> points;
  for (int i = 0; i < 10; ++i) {  // outbound along +x at y = 0
    points.push_back(PathPointLite{static_cast<double>(i), 0.0, {1}});
  }
  for (int i = 9; i >= 0; --i) {  // return along -x at y = 0.5
    points.push_back(PathPointLite{static_cast<double>(i), 0.5, {1}});
  }
  // Ego sits nearer the return leg but heads along +x: it belongs on the outbound leg.
  const auto index = nearest_index(points, 5.0, 0.3, kStraightYaw);
  ASSERT_LT(index, 10u);
  EXPECT_NEAR(points[index].y, 0.0, 1e-9);
}

// ===========================================================================
// find_turn_segments
// ===========================================================================

TEST(TurnSignalLogic, NoTurnSegmentOnStraightPath)
{
  // Intent: a route with no turn lanelet must never raise a turn segment (no false blink).
  const auto points = make_path_with_turn(50, 50, 1, 1);
  const auto attrs = turn_attrs(1, 1, TurnDirection::NONE);
  EXPECT_TRUE(find_turn_segments(points, 0, attrs, default_params()).empty());
}

TEST(TurnSignalLogic, TurnSegmentReportsDistanceFromEgo)
{
  // Straight for 40 m, then a left turn lanelet. Ego at index 0.
  const auto points = make_path_with_turn(60, 40, 1, 2);
  const auto attrs = turn_attrs(1, 2, TurnDirection::LEFT);

  const auto segments = find_turn_segments(points, 0, attrs, default_params());
  ASSERT_EQ(segments.size(), 1u);
  EXPECT_EQ(segments.front().direction, TurnDirection::LEFT);
  EXPECT_EQ(segments.front().kind, ManeuverKind::INTERSECTION);
  EXPECT_NEAR(segments.front().dist_to_start, 40.0, 1e-6);  // turn lanelet starts 40 m ahead
  EXPECT_EQ(segments.front().start_index, 40u);
}

TEST(TurnSignalLogic, TurnSegmentDistanceShrinksAsEgoAdvances)
{
  // Intent: distance-to-turn must be measured from ego, so it shrinks as ego moves up.
  const auto points = make_path_with_turn(60, 40, 1, 2);
  const auto attrs = turn_attrs(1, 2, TurnDirection::LEFT);

  const auto segments = find_turn_segments(points, 15, attrs, default_params());
  ASSERT_EQ(segments.size(), 1u);
  EXPECT_NEAR(segments.front().dist_to_start, 25.0, 1e-6);  // 40 m - 15 m travelled
}

TEST(TurnSignalLogic, ConsecutiveOppositeTurnsAreBothReported)
{
  // Intent: a right lanelet immediately followed by a left one must not hide the second - the
  // first may already be complete, and the upcoming one still needs a signal.
  std::vector<PathPointLite> points;
  for (int i = 0; i < 20; ++i) {
    points.push_back(PathPointLite{static_cast<double>(i), 0.0, {i < 10 ? 2 : 3}});
  }
  LaneAttributeMap attrs;
  attrs[2] = LaneAttribute{TurnDirection::RIGHT, false};
  attrs[3] = LaneAttribute{TurnDirection::LEFT, false};

  const auto segments = find_turn_segments(points, 0, attrs, default_params());
  ASSERT_EQ(segments.size(), 2u);
  EXPECT_EQ(segments[0].direction, TurnDirection::RIGHT);
  EXPECT_EQ(segments[1].direction, TurnDirection::LEFT);
}

// ===========================================================================
// decide_maneuver_signal  (activation + the heading-based end condition)
// ===========================================================================

TEST(TurnSignalLogic, IntersectionStaysOffBeyondActivationDistance)
{
  // Intent: at standstill the signal must NOT light until within 30 m (legal lead distance).
  const auto params = default_params();
  const ManeuverSegment far{
    TurnDirection::LEFT, ManeuverKind::INTERSECTION, 40.0, 59.0, 40, 59, M_PI_2};
  EXPECT_EQ(
    decide_maneuver_signal(far, /*ego_velocity=*/0.0, kStraightYaw, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, IntersectionLightsWithinThirtyMeters)
{
  // Intent: the core legal requirement - blink once the turn is within 30 m.
  const auto params = default_params();
  const ManeuverSegment near{
    TurnDirection::LEFT, ManeuverKind::INTERSECTION, 25.0, 44.0, 25, 44, M_PI_2};
  EXPECT_EQ(
    decide_maneuver_signal(near, /*ego_velocity=*/0.0, kStraightYaw, params), TurnDirection::LEFT);
}

TEST(TurnSignalLogic, IntersectionActivationExtendsWithSpeed)
{
  // Intent: at speed the lead distance is v * search_time, so a turn 40 m ahead lights
  // already at 20 m/s (activation = 60 m) even though it exceeds the 30 m floor.
  const auto params = default_params();
  const ManeuverSegment seg{
    TurnDirection::RIGHT, ManeuverKind::INTERSECTION, 40.0, 59.0, 40, 59, -M_PI_2};
  EXPECT_EQ(decide_maneuver_signal(seg, 20.0, kStraightYaw, params), TurnDirection::RIGHT);
  EXPECT_EQ(decide_maneuver_signal(seg, 0.0, kStraightYaw, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, ManeuverStaysLitWhileEgoIsStillRotating)
{
  // Intent: requirement 2 - the signal must stay on for the whole maneuver. Ego has entered the
  // segment (start behind it) but has only rotated 45 of the 90 degrees.
  const auto params = default_params();
  const ManeuverSegment seg{
    TurnDirection::LEFT, ManeuverKind::INTERSECTION, -3.0, 20.0, 0, 20, M_PI_2};
  EXPECT_EQ(decide_maneuver_signal(seg, 2.0, M_PI_4, params), TurnDirection::LEFT);
}

TEST(TurnSignalLogic, ManeuverClearsWhenEgoAlignsWithExitHeading)
{
  // Intent: requirement 3 - the signal goes out when ego's pose matches the centerline it exits
  // on, NOT when the turn lanelet happens to end.
  const auto params = default_params();
  const ManeuverSegment seg{
    TurnDirection::LEFT, ManeuverKind::INTERSECTION, -3.0, 20.0, 0, 20, M_PI_2};
  EXPECT_EQ(decide_maneuver_signal(seg, 2.0, M_PI_2, params), TurnDirection::NONE);
  // ...and it is still lit just outside the alignment tolerance.
  EXPECT_EQ(
    decide_maneuver_signal(seg, 2.0, M_PI_2 - 2.0 * params.heading_align_threshold, params),
    TurnDirection::LEFT);
}

TEST(TurnSignalLogic, AlignmentDoesNotClearAManeuverEgoHasNotStartedYet)
{
  // Intent: approaching a turn, ego is aligned with the road it is ON - that must not be read as
  // "maneuver finished" and keep the signal dark. Here exit_yaw == ego_yaw but the turn is ahead.
  const auto params = default_params();
  const ManeuverSegment seg{TurnDirection::LEFT, ManeuverKind::INTERSECTION, 20.0, 50.0, 20, 50,
                            kStraightYaw};
  EXPECT_EQ(decide_maneuver_signal(seg, 2.0, kStraightYaw, params), TurnDirection::LEFT);
}

TEST(TurnSignalLogic, ManeuverIsOffOncePassed)
{
  const auto params = default_params();
  const ManeuverSegment passed{
    TurnDirection::LEFT, ManeuverKind::INTERSECTION, -20.0, -5.0, 0, 0, M_PI_2};
  EXPECT_EQ(decide_maneuver_signal(passed, 2.0, kStraightYaw, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, EmptySegmentListIsOff)
{
  const std::vector<ManeuverSegment> no_segments;
  const auto decision = decide_maneuver_signal(no_segments, 5.0, kStraightYaw, default_params());
  EXPECT_EQ(decision.direction, TurnDirection::NONE);
  EXPECT_FALSE(decision.segment.has_value());
}

TEST(TurnSignalLogic, FirstSignallingSegmentWins)
{
  // Intent: a completed turn must not mask the next one. The first segment is behind ego and
  // aligned (done); the second is 10 m ahead and must light.
  const auto params = default_params();
  const std::vector<ManeuverSegment> segments{
    ManeuverSegment{
      TurnDirection::RIGHT, ManeuverKind::INTERSECTION, -8.0, 5.0, 0, 5, kStraightYaw},
    ManeuverSegment{TurnDirection::LEFT, ManeuverKind::INTERSECTION, 10.0, 30.0, 10, 30, M_PI_2}};

  const auto decision = decide_maneuver_signal(segments, 2.0, kStraightYaw, params);
  EXPECT_EQ(decision.direction, TurnDirection::LEFT);
  ASSERT_TRUE(decision.segment.has_value());
  EXPECT_EQ(decision.segment->start_index, 10u);
}

TEST(TurnSignalLogic, TurnSegmentEndToEndOnCurvedPath)
{
  // Intent: end-to-end over a real 90-deg left turn geometry - lit while approaching and while
  // rotating, dark once ego is aligned with the road it exits on.
  const auto params = default_params();
  const auto points = make_left_turn_path(25, 1, 2, 3);
  const auto attrs = turn_attrs(1, 2, TurnDirection::LEFT);

  // 15 m before the turn.
  {
    const auto ego = nearest_index(points, 10.0, 0.0, kStraightYaw);
    const auto segments = find_turn_segments(points, ego, attrs, params);
    ASSERT_FALSE(segments.empty());
    EXPECT_EQ(
      decide_maneuver_signal(segments, 3.0, kStraightYaw, params).direction, TurnDirection::LEFT);
  }
  // Mid-turn: ego has rotated ~45 deg.
  {
    const auto ego =
      nearest_index(points, 24.0 + 10.0 * std::sin(M_PI_4), 10.0 - 10.0 * std::cos(M_PI_4), M_PI_4);
    const auto segments = find_turn_segments(points, ego, attrs, params);
    ASSERT_FALSE(segments.empty());
    EXPECT_EQ(decide_maneuver_signal(segments, 3.0, M_PI_4, params).direction, TurnDirection::LEFT);
  }
  // Out of the turn and aligned with the exit road.
  {
    const auto ego = nearest_index(points, 34.0, 15.0, M_PI_2);
    const auto segments = find_turn_segments(points, ego, attrs, params);
    EXPECT_EQ(decide_maneuver_signal(segments, 3.0, M_PI_2, params).direction, TurnDirection::NONE);
  }
}

// ===========================================================================
// find_private_exit_segments
// ===========================================================================

TEST(TurnSignalLogic, PrivateExitDirectionFromTurnDirectionTag)
{
  // Intent: a private area exit must blink even though the module's intersection rule may not
  // cover it. Here the exit lanelet is tagged `right`.
  auto points = make_path_with_turn(60, 20, 5, 6);  // ids: 5 = private, 6 = public
  LaneAttributeMap attrs;
  attrs[5] = LaneAttribute{TurnDirection::RIGHT, /*is_private=*/true};
  attrs[6] = LaneAttribute{TurnDirection::NONE, false};

  const auto segments = find_private_exit_segments(points, 0, attrs, default_params());
  ASSERT_EQ(segments.size(), 1u);
  EXPECT_EQ(segments.front().direction, TurnDirection::RIGHT);
  EXPECT_EQ(segments.front().kind, ManeuverKind::PRIVATE_EXIT);
  EXPECT_NEAR(segments.front().dist_to_start, 0.0, 1e-6);
}

TEST(TurnSignalLogic, PrivateExitDirectionFromGeometryWhenUntagged)
{
  // Intent: the `turn_direction` tag is optional on private lanelets, so fall back to the yaw
  // change across the merge. Here the path leaves a driveway (+x) onto a road running +y => left.
  const auto points = make_left_turn_path(20, 5, 5, 6);  // driveway + arc are private (id 5)
  LaneAttributeMap attrs;
  attrs[5] = LaneAttribute{TurnDirection::NONE, /*is_private=*/true};
  attrs[6] = LaneAttribute{TurnDirection::NONE, false};

  const auto segments = find_private_exit_segments(points, 0, attrs, default_params());
  ASSERT_EQ(segments.size(), 1u);
  EXPECT_EQ(segments.front().direction, TurnDirection::LEFT);
}

TEST(TurnSignalLogic, StraightPrivateMergeRaisesNoSignal)
{
  // Intent: a merge that is geometrically straight and carries no tag has no side to signal;
  // guessing one would be worse than staying dark.
  const auto points = make_path_with_turn(60, 20, 5, 6);
  LaneAttributeMap attrs;
  attrs[5] = LaneAttribute{TurnDirection::NONE, /*is_private=*/true};
  attrs[6] = LaneAttribute{TurnDirection::NONE, false};

  EXPECT_TRUE(find_private_exit_segments(points, 0, attrs, default_params()).empty());
}

TEST(TurnSignalLogic, NoPrivateExitOnAPublicPath)
{
  const auto points = make_path_with_turn(60, 20, 5, 6);
  LaneAttributeMap attrs;
  attrs[5] = LaneAttribute{TurnDirection::NONE, false};
  attrs[6] = LaneAttribute{TurnDirection::NONE, false};

  EXPECT_TRUE(find_private_exit_segments(points, 0, attrs, default_params()).empty());
}

// ===========================================================================
// direction_from_lateral_offset
// ===========================================================================

TEST(TurnSignalLogic, DirectionFromLateralOffsetSign)
{
  // Intent: positive offset (left of the centerline) => left, negative => right.
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
// DepartureLatch  (pull-out; the lane-change / avoidance exclusion lives here)
// ===========================================================================

TEST(TurnSignalLogic, DepartureLatchesWhenStoppedOffLane)
{
  // Intent: the bus stop / shoulder start case. Ego stands 2 m left of the centerline, so it
  // pulls out to the RIGHT, and keeps blinking while it moves back towards the centerline.
  const auto params = default_params();
  DepartureLatch latch;

  EXPECT_EQ(latch.update(2.0, 0.0, false, params), TurnDirection::RIGHT);
  EXPECT_EQ(latch.update(1.8, 1.0, false, params), TurnDirection::RIGHT);  // moving off
  EXPECT_EQ(latch.update(1.0, 3.0, false, params), TurnDirection::RIGHT);
}

TEST(TurnSignalLogic, DepartureClearsOnceBackOnCenterline)
{
  // Intent: requirement 3 - the signal goes out when the shift is finished.
  const auto params = default_params();
  DepartureLatch latch;

  ASSERT_EQ(latch.update(-2.0, 0.0, false, params), TurnDirection::LEFT);
  EXPECT_EQ(latch.update(-0.4, 3.0, false, params), TurnDirection::NONE);
  EXPECT_FALSE(latch.latched());
}

TEST(TurnSignalLogic, MovingOffsetEgoNeverLatches_LaneChangeAndAvoidance)
{
  // Intent: THE out-of-scope guard. While the upstream planner runs a lane change or an
  // avoidance, ego is far off its lane centerline but MOVING, and this module plans a path back
  // to the centre. It must stay dark - blinking here would announce a maneuver we do not own.
  const auto params = default_params();
  DepartureLatch latch;

  EXPECT_EQ(latch.update(3.5, 8.0, false, params), TurnDirection::NONE);  // mid lane change
  EXPECT_EQ(latch.update(2.0, 6.0, false, params), TurnDirection::NONE);  // returning
  EXPECT_EQ(latch.update(1.0, 5.0, false, params), TurnDirection::NONE);  // avoidance offset
  EXPECT_FALSE(latch.latched());
}

TEST(TurnSignalLogic, StoppedButOnlySlightlyOffLaneDoesNotLatch)
{
  // Intent: lateral control error / localization jitter at a normal in-lane stop (traffic light,
  // obstacle) must not be mistaken for a shoulder start.
  const auto params = default_params();
  DepartureLatch latch;

  EXPECT_EQ(latch.update(0.9, 0.0, false, params), TurnDirection::NONE);
  EXPECT_FALSE(latch.latched());
}

TEST(TurnSignalLogic, DepartureIsSuppressedInThePullOverRange)
{
  // Intent: after pulling into an off-centerline goal, ego is stopped and offset - exactly the
  // pull-out latch condition. Suppression keeps it from lighting the opposite direction at the
  // goal (and from fighting the pull-over signal on the way in).
  const auto params = default_params();
  DepartureLatch latch;

  EXPECT_EQ(latch.update(2.0, 0.0, /*suppressed=*/true, params), TurnDirection::NONE);
  EXPECT_FALSE(latch.latched());
}

// ===========================================================================
// ArrivalState  (pull-over)
// ===========================================================================

TEST(TurnSignalLogic, PullOverSignalsTowardsTheGoalSide)
{
  const auto params = default_params();
  ArrivalState state;
  // Goal 1.8 m to the left, 20 m ahead => blink left.
  EXPECT_EQ(state.update(20.0, 1.8, 3.0, params), TurnDirection::LEFT);
  EXPECT_EQ(state.update(20.0, -1.8, 3.0, params), TurnDirection::RIGHT);
}

TEST(TurnSignalLogic, PullOverStaysOffFarFromTheGoal)
{
  const auto params = default_params();
  ArrivalState state;
  EXPECT_EQ(state.update(80.0, 1.8, 5.0, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, PullOverStaysOffForAnOnCenterlineGoal)
{
  // Intent: a goal on the centerline is a plain stop, not an arrival maneuver.
  const auto params = default_params();
  ArrivalState state;
  EXPECT_EQ(state.update(10.0, 0.1, 2.0, params), TurnDirection::NONE);
}

TEST(TurnSignalLogic, PullOverClearsAfterArrival)
{
  // Intent: requirement 3 - once stopped at the goal the maneuver is over, so the signal must go
  // out and stay out rather than blinking at a parked vehicle forever.
  const auto params = default_params();
  ArrivalState state;

  ASSERT_EQ(state.update(20.0, 1.8, 3.0, params), TurnDirection::LEFT);
  ASSERT_EQ(state.update(0.5, 1.8, 0.0, params), TurnDirection::NONE);  // arrived
  EXPECT_TRUE(state.arrived());
  EXPECT_EQ(state.update(0.5, 1.8, 0.0, params), TurnDirection::NONE);  // stays out
}

TEST(TurnSignalLogic, PullOverRearmsForANewApproach)
{
  const auto params = default_params();
  ArrivalState state;

  ASSERT_EQ(state.update(0.5, 1.8, 0.0, params), TurnDirection::NONE);
  ASSERT_TRUE(state.arrived());
  state.update(100.0, 1.8, 5.0, params);  // left the goal area
  EXPECT_FALSE(state.arrived());
  EXPECT_EQ(state.update(20.0, 1.8, 3.0, params), TurnDirection::LEFT);
}

// ===========================================================================
// resolve_priority
// ===========================================================================

TEST(TurnSignalLogic, PriorityOrderIsIntersectionPrivateExitPullOutPullOver)
{
  const auto none = TurnDirection::NONE;
  {
    const auto d = resolve_priority(
      TurnDirection::RIGHT, TurnDirection::LEFT, TurnDirection::LEFT, TurnDirection::LEFT);
    EXPECT_EQ(d.direction, TurnDirection::RIGHT);
    EXPECT_EQ(d.kind, ManeuverKind::INTERSECTION);
  }
  {
    const auto d =
      resolve_priority(none, TurnDirection::RIGHT, TurnDirection::LEFT, TurnDirection::LEFT);
    EXPECT_EQ(d.direction, TurnDirection::RIGHT);
    EXPECT_EQ(d.kind, ManeuverKind::PRIVATE_EXIT);
  }
  {
    const auto d = resolve_priority(none, none, TurnDirection::LEFT, TurnDirection::RIGHT);
    EXPECT_EQ(d.direction, TurnDirection::LEFT);
    EXPECT_EQ(d.kind, ManeuverKind::PULL_OUT);
  }
  {
    const auto d = resolve_priority(none, none, none, TurnDirection::RIGHT);
    EXPECT_EQ(d.direction, TurnDirection::RIGHT);
    EXPECT_EQ(d.kind, ManeuverKind::PULL_OVER);
  }
  {
    const auto d = resolve_priority(none, none, none, none);
    EXPECT_EQ(d.direction, TurnDirection::NONE);
    EXPECT_EQ(d.kind, ManeuverKind::NONE);
  }
}

// ===========================================================================
// BlinkHold  (anti-chatter hysteresis)
// ===========================================================================

TEST(TurnSignalLogic, BlinkHoldKeepsSignalForMinimumDuration)
{
  // Intent: once lit, the signal must stay on for min_duration even if the desired
  // command immediately drops to none - this is what prevents flicker.
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
