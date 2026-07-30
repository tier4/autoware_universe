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
