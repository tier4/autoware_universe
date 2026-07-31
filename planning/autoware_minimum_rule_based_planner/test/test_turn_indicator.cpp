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

#include "turn_indicator_decider.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

// ===============================================================================================
// Part 1: the decision rules, on plain numbers.
// ===============================================================================================

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

TEST(TurnSignalLogic, PriorityOrderIsTurnPullOutPullOver)
{
  const Signal turn{TurnDirection::LEFT, ManeuverKind::TURN};
  const Signal pull_out{TurnDirection::RIGHT, ManeuverKind::PULL_OUT};
  const Signal pull_over{TurnDirection::RIGHT, ManeuverKind::PULL_OVER};

  EXPECT_EQ(resolve_priority({turn, pull_out, pull_over}).kind, ManeuverKind::TURN);
  EXPECT_EQ(resolve_priority({{}, pull_out, pull_over}).kind, ManeuverKind::PULL_OUT);
  EXPECT_EQ(resolve_priority({{}, {}, pull_over}).kind, ManeuverKind::PULL_OVER);

  const auto none = resolve_priority({{}, {}, {}});
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

// ===============================================================================================
// Part 2: TurnIndicatorDecider end to end, on a synthetic lanelet map and path.
// ===============================================================================================

namespace autoware::minimum_rule_based_planner
{
namespace
{
constexpr double k_lane_half_width = 1.75;
constexpr double k_moving = 5.0;  //!< [m/s]

//! A road lanelet running along +x over [x_start, x_start + length], centred on y = 0.
lanelet::Lanelet make_lanelet(const lanelet::Id id, const double x_start, const double length)
{
  const double x_end = x_start + length;
  lanelet::LineString3d left(
    id * 10 + 1, {lanelet::Point3d(id * 100 + 1, x_start, k_lane_half_width, 0.0),
                  lanelet::Point3d(id * 100 + 2, x_end, k_lane_half_width, 0.0)});
  lanelet::LineString3d right(
    id * 10 + 2, {lanelet::Point3d(id * 100 + 3, x_start, -k_lane_half_width, 0.0),
                  lanelet::Point3d(id * 100 + 4, x_end, -k_lane_half_width, 0.0)});
  lanelet::Lanelet lanelet(id, left, right);
  lanelet.attributes()[lanelet::AttributeNamesString::Subtype] =
    lanelet::AttributeValueString::Road;
  lanelet.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Urban;
  return lanelet;
}

RouteContext make_context(const std::vector<lanelet::Lanelet> & lanelets)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lanelet : lanelets) {
    map->add(lanelet);
  }
  RouteContext ctx;
  ctx.lanelet_map_ptr = map;
  // Goal far beyond the path, so the pull-over rule never interferes with these cases.
  ctx.goal_pose.position.x = 1000.0;
  ctx.goal_pose.orientation.w = 1.0;
  return ctx;
}

PathWithLaneId::_points_type::value_type make_point(
  const double x, const double y, const double yaw, const int64_t lane_id)
{
  PathWithLaneId::_points_type::value_type point;
  point.point.pose.position.x = x;
  point.point.pose.position.y = y;
  point.point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  point.lane_ids = {lane_id};
  return point;
}

//! Straight path along +x at 1 m spacing over [0, total). Points before `split` carry `first_id`,
//! the rest carry `second_id`.
PathWithLaneId make_straight_path(
  const std::size_t total, const std::size_t split, const int64_t first_id, const int64_t second_id)
{
  PathWithLaneId path;
  for (std::size_t i = 0; i < total; ++i) {
    path.points.push_back(
      make_point(static_cast<double>(i), 0.0, 0.0, i < split ? first_id : second_id));
  }
  return path;
}

geometry_msgs::msg::Pose make_ego_pose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  return pose;
}

//! One decision from a freshly constructed decider, so the blink hold never carries state over.
uint8_t decide_once(
  const PathWithLaneId & path, const RouteContext & ctx, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_velocity = k_moving)
{
  TurnIndicatorDecider decider(turn_indicator::TurnSignalParams{});
  return decider.decide(path, ctx, ego_pose, ego_velocity, rclcpp::Time(0)).command;
}
}  // namespace

TEST(TurnIndicatorDeciderTest, StraightPublicPathRaisesNoSignal)
{
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), make_lanelet(2, 40.0, 20.0)});
  const auto path = make_straight_path(60, 40, 1, 2);

  EXPECT_EQ(decide_once(path, ctx, make_ego_pose(0.0, 0.0, 0.0)), TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorDeciderTest, IntersectionStaysOffBeyondTheLegalDistance)
{
  auto turn = make_lanelet(2, 40.0, 20.0);
  turn.attributes()["turn_direction"] = "left";
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), turn});
  const auto path = make_straight_path(60, 40, 1, 2);

  // The turn lanelet starts ~39 m ahead (the index range is expanded by one point), past the 30 m
  // activation floor at this speed.
  EXPECT_EQ(decide_once(path, ctx, make_ego_pose(0.0, 0.0, 0.0)), TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorDeciderTest, IntersectionLightsWithinTheLegalDistance)
{
  auto turn = make_lanelet(2, 40.0, 20.0);
  turn.attributes()["turn_direction"] = "left";
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), turn});
  const auto path = make_straight_path(60, 40, 1, 2);

  // 20 m short of the turn lanelet.
  EXPECT_EQ(
    decide_once(path, ctx, make_ego_pose(20.0, 0.0, 0.0)), TurnIndicatorsCommand::ENABLE_LEFT);
}

TEST(TurnIndicatorDeciderTest, IntersectionDirectionFollowsTheTag)
{
  auto turn = make_lanelet(2, 40.0, 20.0);
  turn.attributes()["turn_direction"] = "right";
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), turn});
  const auto path = make_straight_path(60, 40, 1, 2);

  EXPECT_EQ(
    decide_once(path, ctx, make_ego_pose(20.0, 0.0, 0.0)), TurnIndicatorsCommand::ENABLE_RIGHT);
}

TEST(TurnIndicatorDeciderTest, IntersectionClearsOnceEgoIsAlignedWithTheExitHeading)
{
  // The path through the "turn" lanelet is straight here, so an ego inside it is already aligned
  // with the heading it exits on: the spec's end condition holds and the signal must be dark.
  auto turn = make_lanelet(2, 40.0, 20.0);
  turn.attributes()["turn_direction"] = "left";
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), turn});
  const auto path = make_straight_path(60, 40, 1, 2);

  EXPECT_EQ(decide_once(path, ctx, make_ego_pose(45.0, 0.0, 0.0)), TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorDeciderTest, StraightPrivateMergeRaisesNoSignal)
{
  // A private run rejoining a public lane straight ahead, with no turn tag, has no side to signal.
  auto driveway = make_lanelet(1, 0.0, 40.0);
  driveway.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Private;
  const auto ctx = make_context({driveway, make_lanelet(2, 40.0, 20.0)});
  const auto path = make_straight_path(60, 40, 1, 2);

  EXPECT_EQ(decide_once(path, ctx, make_ego_pose(20.0, 0.0, 0.0)), TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorDeciderTest, PrivateExitDirectionComesFromTheMergeGeometry)
{
  // A driveway heading +x (private, untagged) rejoining a road heading +y: the yaw change across
  // the merge is +90 deg, so the exit is to the left.
  auto driveway = make_lanelet(1, 0.0, 30.0);
  driveway.attributes()[lanelet::AttributeNamesString::Location] =
    lanelet::AttributeValueString::Private;
  const auto ctx = make_context({driveway, make_lanelet(2, 30.0, 30.0)});

  PathWithLaneId path;
  for (std::size_t i = 0; i < 30; ++i) {  // driveway: (0..29, 0), heading +x
    path.points.push_back(make_point(static_cast<double>(i), 0.0, 0.0, 1));
  }
  for (std::size_t i = 0; i < 30; ++i) {  // public road: (30, 0..29), heading +y
    path.points.push_back(make_point(30.0, static_cast<double>(i), M_PI_2, 2));
  }

  // 20 m short of the merge.
  EXPECT_EQ(
    decide_once(path, ctx, make_ego_pose(9.0, 0.0, 0.0)), TurnIndicatorsCommand::ENABLE_LEFT);
}

TEST(TurnIndicatorDeciderTest, PullOutLightsForAnEgoStoppedOffTheLane)
{
  // Ego stands still 2 m right of the centerline (bus stop / shoulder): it departs to the left.
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), make_lanelet(2, 40.0, 20.0)});
  const auto path = make_straight_path(60, 40, 1, 2);

  EXPECT_EQ(
    decide_once(path, ctx, make_ego_pose(5.0, -2.0, 0.0), /*ego_velocity=*/0.0),
    TurnIndicatorsCommand::ENABLE_LEFT);
}

TEST(TurnIndicatorDeciderTest, LaneChangeAndAvoidanceNeverBlink)
{
  // Out of scope: while the upstream planner shifts ego off the centerline it keeps MOVING, and
  // this module must stay dark for the whole excursion.
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), make_lanelet(2, 40.0, 20.0)});
  const auto path = make_straight_path(60, 40, 1, 2);

  TurnIndicatorDecider decider(turn_indicator::TurnSignalParams{});
  double x = 5.0;
  for (const double offset : {0.6, 1.6, 2.5, 1.6, 0.6}) {
    const auto cmd =
      decider.decide(path, ctx, make_ego_pose(x, -offset, 0.0), k_moving, rclcpp::Time(0));
    EXPECT_EQ(cmd.command, TurnIndicatorsCommand::DISABLE);
    x += 5.0;
  }
}

TEST(TurnIndicatorDeciderTest, PullOverLightsTowardsAnOffCenterlineGoal)
{
  const auto lane = make_lanelet(1, 0.0, 40.0);
  auto ctx = make_context({lane});
  ctx.goal_lanelets = {lane};
  ctx.goal_pose = make_ego_pose(35.0, -1.5, 0.0);  // 1.5 m right of the centerline
  const auto path = make_straight_path(40, 40, 1, 1);

  // 15 m short of the goal, inside the 30 m pull-over range.
  EXPECT_EQ(
    decide_once(path, ctx, make_ego_pose(20.0, 0.0, 0.0)), TurnIndicatorsCommand::ENABLE_RIGHT);
}

TEST(TurnIndicatorDeciderTest, PullOverClearsOnceStoppedAtTheGoal)
{
  const auto lane = make_lanelet(1, 0.0, 40.0);
  auto ctx = make_context({lane});
  ctx.goal_lanelets = {lane};
  ctx.goal_pose = make_ego_pose(35.0, -1.5, 0.0);
  const auto path = make_straight_path(40, 40, 1, 1);

  TurnIndicatorDecider decider(turn_indicator::TurnSignalParams{});
  ASSERT_EQ(
    decider.decide(path, ctx, make_ego_pose(20.0, 0.0, 0.0), k_moving, rclcpp::Time(0)).command,
    TurnIndicatorsCommand::ENABLE_RIGHT);
  // Stopped at the goal, past the minimum blink duration.
  const auto cmd = decider.decide(
    path, ctx, make_ego_pose(35.0, -1.5, 0.0), /*ego_velocity=*/0.0, rclcpp::Time(10, 0));
  EXPECT_EQ(cmd.command, TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorDeciderTest, AnEmptyPathRaisesNoSignal)
{
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0)});
  EXPECT_EQ(
    decide_once(PathWithLaneId(), ctx, make_ego_pose(0.0, 0.0, 0.0)),
    TurnIndicatorsCommand::DISABLE);
}

TEST(TurnIndicatorDeciderTest, MinimumBlinkDurationHoldsALitSignal)
{
  auto turn = make_lanelet(2, 40.0, 20.0);
  turn.attributes()["turn_direction"] = "left";
  const auto ctx = make_context({make_lanelet(1, 0.0, 40.0), turn});
  const auto path = make_straight_path(60, 40, 1, 2);

  TurnIndicatorDecider decider(turn_indicator::TurnSignalParams{});
  ASSERT_EQ(
    decider.decide(path, ctx, make_ego_pose(20.0, 0.0, 0.0), k_moving, rclcpp::Time(0)).command,
    TurnIndicatorsCommand::ENABLE_LEFT);
  // Inside the turn and aligned, so the maneuver is over - but only 1 s has passed.
  EXPECT_EQ(
    decider.decide(path, ctx, make_ego_pose(45.0, 0.0, 0.0), k_moving, rclcpp::Time(1, 0)).command,
    TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(
    decider.decide(path, ctx, make_ego_pose(45.0, 0.0, 0.0), k_moving, rclcpp::Time(4, 0)).command,
    TurnIndicatorsCommand::DISABLE);
}

}  // namespace autoware::minimum_rule_based_planner
