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

namespace autoware::minimum_rule_based_planner::turn_indicator
{
namespace
{
constexpr double k_exit_heading = 0.0;  //!< heading every maneuver below exits on
constexpr double k_stopped = 0.0;       //!< [m/s]
constexpr double k_moving = 5.0;        //!< [m/s]

//! Ask for the signal of a maneuver `dist_to_start` metres ahead (negative once ego is inside).
TurnDirection maneuver_signal(
  const TurnDirection direction, const double dist_to_start, const double ego_velocity = k_moving,
  const double ego_yaw = k_exit_heading)
{
  return decide_maneuver_signal(
    direction, dist_to_start, ego_yaw, k_exit_heading, ego_velocity, TurnSignalParams{});
}
}  // namespace

// --- activation distance -----------------------------------------------------------------------

TEST(TurnSignalLogic, ActivationDistanceHasALegalFloor)
{
  // Below the speed at which v * search_time reaches it, the legal 30 m floor governs.
  EXPECT_DOUBLE_EQ(activation_distance(0.0, TurnSignalParams{}), 30.0);
  EXPECT_DOUBLE_EQ(activation_distance(5.0, TurnSignalParams{}), 30.0);
}

TEST(TurnSignalLogic, ActivationDistanceGrowsWithSpeed)
{
  // 20 m/s * 3.0 s = 60 m, i.e. the spec's 3-second lead time.
  EXPECT_DOUBLE_EQ(activation_distance(20.0, TurnSignalParams{}), 60.0);
}

// --- maneuver signal (intersection turns and private-area exits share this rule) ----------------

TEST(TurnSignalLogic, ManeuverStaysOffBeyondActivationDistance)
{
  EXPECT_EQ(maneuver_signal(TurnDirection::LEFT, 30.1), TurnDirection::NONE);
}

TEST(TurnSignalLogic, ManeuverLightsWithinActivationDistance)
{
  EXPECT_EQ(maneuver_signal(TurnDirection::LEFT, 29.9), TurnDirection::LEFT);
  // Right up to the maneuver start. (At 0.0 ego counts as having entered it, which lets the
  // alignment end condition apply - see ManeuverClearsWhenEgoAlignsWithExitHeading.)
  EXPECT_EQ(maneuver_signal(TurnDirection::RIGHT, 0.5), TurnDirection::RIGHT);
}

TEST(TurnSignalLogic, ManeuverActivationExtendsWithSpeed)
{
  // 20 m/s * 3.0 s = 60 m of lead distance; at 5 m/s only the 30 m floor applies.
  EXPECT_EQ(maneuver_signal(TurnDirection::LEFT, 45.0, /*ego_velocity=*/20.0), TurnDirection::LEFT);
  EXPECT_EQ(maneuver_signal(TurnDirection::LEFT, 45.0, /*ego_velocity=*/5.0), TurnDirection::NONE);
}

TEST(TurnSignalLogic, ManeuverStaysLitWhileEgoIsStillRotating)
{
  // Ego has entered the turn (its start is behind) but is still 45 deg off the exit heading.
  EXPECT_EQ(
    maneuver_signal(TurnDirection::LEFT, -5.0, k_moving, /*ego_yaw=*/M_PI_4), TurnDirection::LEFT);
}

TEST(TurnSignalLogic, ManeuverClearsWhenEgoAlignsWithExitHeading)
{
  // The spec's end condition: ego's pose matches the centerline it exits on.
  EXPECT_EQ(
    maneuver_signal(TurnDirection::LEFT, -5.0, k_moving, /*ego_yaw=*/0.1), TurnDirection::NONE);
}

TEST(TurnSignalLogic, AlignmentDoesNotClearAManeuverEgoHasNotStartedYet)
{
  // Ego is aligned with the exit heading only because the turn is still ahead of it.
  EXPECT_EQ(
    maneuver_signal(TurnDirection::LEFT, 10.0, k_moving, /*ego_yaw=*/0.0), TurnDirection::LEFT);
}

TEST(TurnSignalLogic, ADirectionlessManeuverRaisesNoSignal)
{
  EXPECT_EQ(maneuver_signal(TurnDirection::NONE, 0.0), TurnDirection::NONE);
}

// --- lateral offset ----------------------------------------------------------------------------

TEST(TurnSignalLogic, DirectionFromLateralOffsetSign)
{
  // Positive offset (left of the centerline) => left, negative => right.
  EXPECT_EQ(direction_from_lateral_offset(1.5, 0.3), TurnDirection::LEFT);
  EXPECT_EQ(direction_from_lateral_offset(-1.5, 0.3), TurnDirection::RIGHT);
  // Offsets within the deadzone must not assert a direction (no false blink on jitter).
  EXPECT_EQ(direction_from_lateral_offset(0.2, 0.3), TurnDirection::NONE);
  EXPECT_EQ(direction_from_lateral_offset(-0.2, 0.3), TurnDirection::NONE);
}

// --- departure (pull-out) ----------------------------------------------------------------------

TEST(TurnSignalLogic, DepartureLatchesWhenStoppedOffLane)
{
  auto latched = TurnDirection::NONE;
  // Parked 2 m right of the centerline: ego pulls out towards the left.
  EXPECT_EQ(
    decide_pull_out(-2.0, k_stopped, false, TurnSignalParams{}, latched), TurnDirection::LEFT);
  // Stays lit while ego moves off, even though it no longer qualifies as stopped.
  EXPECT_EQ(
    decide_pull_out(-1.8, k_moving, false, TurnSignalParams{}, latched), TurnDirection::LEFT);
}

TEST(TurnSignalLogic, DepartureClearsOnceBackOnCenterline)
{
  auto latched = TurnDirection::NONE;
  ASSERT_EQ(
    decide_pull_out(2.0, k_stopped, false, TurnSignalParams{}, latched), TurnDirection::RIGHT);
  EXPECT_EQ(
    decide_pull_out(0.4, k_moving, false, TurnSignalParams{}, latched), TurnDirection::NONE);
  EXPECT_EQ(latched, TurnDirection::NONE);
}

TEST(TurnSignalLogic, MovingOffsetEgoNeverLatches_LaneChangeAndAvoidance)
{
  // The out-of-scope guarantee: while the upstream planner runs a lane change or an avoidance ego
  // is laterally offset but MOVING, so no signal may ever be raised.
  auto latched = TurnDirection::NONE;
  for (const double offset : {0.6, 1.6, 2.5, 1.6, 0.6}) {
    EXPECT_EQ(
      decide_pull_out(offset, k_moving, false, TurnSignalParams{}, latched), TurnDirection::NONE);
  }
  EXPECT_EQ(latched, TurnDirection::NONE);
}

TEST(TurnSignalLogic, StoppedButOnlySlightlyOffLaneDoesNotLatch)
{
  // 1.0 m is past the shift threshold but short of "parked clear of the lane" (1.5 m).
  auto latched = TurnDirection::NONE;
  EXPECT_EQ(
    decide_pull_out(1.0, k_stopped, false, TurnSignalParams{}, latched), TurnDirection::NONE);
}

TEST(TurnSignalLogic, DepartureIsSuppressedWhileApproachingTheGoal)
{
  auto latched = TurnDirection::NONE;
  ASSERT_EQ(
    decide_pull_out(-2.0, k_stopped, false, TurnSignalParams{}, latched), TurnDirection::LEFT);
  // Inside the pull-over range the two must not fight over the direction.
  EXPECT_EQ(
    decide_pull_out(-2.0, k_stopped, true, TurnSignalParams{}, latched), TurnDirection::NONE);
  EXPECT_EQ(latched, TurnDirection::NONE);
}

// --- arrival (pull-over) -----------------------------------------------------------------------

TEST(TurnSignalLogic, PullOverSignalsTowardsTheGoalSide)
{
  bool arrived = false;
  EXPECT_EQ(
    decide_pull_over(20.0, /*goal_offset=*/-1.5, k_moving, TurnSignalParams{}, arrived),
    TurnDirection::RIGHT);
  EXPECT_EQ(
    decide_pull_over(20.0, /*goal_offset=*/1.5, k_moving, TurnSignalParams{}, arrived),
    TurnDirection::LEFT);
}

TEST(TurnSignalLogic, PullOverStaysOffFarFromTheGoal)
{
  bool arrived = false;
  EXPECT_EQ(
    decide_pull_over(30.1, -1.5, k_moving, TurnSignalParams{}, arrived), TurnDirection::NONE);
}

TEST(TurnSignalLogic, PullOverStaysOffForAnOnCenterlineGoal)
{
  // Nothing to signal: the goal is not a bus stop / shoulder pull-in.
  bool arrived = false;
  EXPECT_EQ(
    decide_pull_over(10.0, /*goal_offset=*/0.2, k_moving, TurnSignalParams{}, arrived),
    TurnDirection::NONE);
}

TEST(TurnSignalLogic, PullOverClearsAfterArrival)
{
  bool arrived = false;
  ASSERT_EQ(
    decide_pull_over(10.0, -1.5, k_moving, TurnSignalParams{}, arrived), TurnDirection::RIGHT);
  EXPECT_EQ(
    decide_pull_over(0.5, -1.5, k_stopped, TurnSignalParams{}, arrived), TurnDirection::NONE);
  EXPECT_TRUE(arrived);
  // Stays off while parked at the goal, even though the offset is unchanged.
  EXPECT_EQ(
    decide_pull_over(0.5, -1.5, k_stopped, TurnSignalParams{}, arrived), TurnDirection::NONE);
}

TEST(TurnSignalLogic, PullOverRearmsForANewApproach)
{
  bool arrived = true;
  // Leaving the search range re-arms it, so the next approach signals again.
  ASSERT_EQ(
    decide_pull_over(50.0, -1.5, k_moving, TurnSignalParams{}, arrived), TurnDirection::NONE);
  EXPECT_FALSE(arrived);
  EXPECT_EQ(
    decide_pull_over(10.0, -1.5, k_moving, TurnSignalParams{}, arrived), TurnDirection::RIGHT);
}

// --- priority ----------------------------------------------------------------------------------

TEST(TurnSignalLogic, PriorityOrderIsIntersectionPrivateExitPullOutPullOver)
{
  const Signal intersection{TurnDirection::LEFT, ManeuverKind::INTERSECTION};
  const Signal private_exit{TurnDirection::RIGHT, ManeuverKind::PRIVATE_EXIT};
  const Signal pull_out{TurnDirection::LEFT, ManeuverKind::PULL_OUT};
  const Signal pull_over{TurnDirection::RIGHT, ManeuverKind::PULL_OVER};

  EXPECT_EQ(
    resolve_priority({intersection, private_exit, pull_out, pull_over}).kind,
    ManeuverKind::INTERSECTION);
  EXPECT_EQ(
    resolve_priority({{}, private_exit, pull_out, pull_over}).kind, ManeuverKind::PRIVATE_EXIT);
  EXPECT_EQ(resolve_priority({{}, {}, pull_out, pull_over}).kind, ManeuverKind::PULL_OUT);
  EXPECT_EQ(resolve_priority({{}, {}, {}, pull_over}).kind, ManeuverKind::PULL_OVER);

  const auto none = resolve_priority({{}, {}, {}, {}});
  EXPECT_EQ(none.direction, TurnDirection::NONE);
  EXPECT_EQ(none.kind, ManeuverKind::NONE);
}

// --- minimum blink duration --------------------------------------------------------------------

TEST(TurnSignalLogic, BlinkHoldKeepsSignalForMinimumDuration)
{
  BlinkHold hold(3.0);
  ASSERT_EQ(hold.update(TurnDirection::LEFT, 0.0), TurnDirection::LEFT);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 1.0), TurnDirection::LEFT);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 2.9), TurnDirection::LEFT);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 3.1), TurnDirection::NONE);
}

TEST(TurnSignalLogic, BlinkHoldSwitchesDirectionImmediately)
{
  BlinkHold hold(3.0);
  ASSERT_EQ(hold.update(TurnDirection::LEFT, 0.0), TurnDirection::LEFT);
  EXPECT_EQ(hold.update(TurnDirection::RIGHT, 0.1), TurnDirection::RIGHT);
}

TEST(TurnSignalLogic, BlinkHoldStaysOffWhenIdle)
{
  BlinkHold hold(3.0);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 0.0), TurnDirection::NONE);
  EXPECT_EQ(hold.update(TurnDirection::NONE, 10.0), TurnDirection::NONE);
}

}  // namespace autoware::minimum_rule_based_planner::turn_indicator
