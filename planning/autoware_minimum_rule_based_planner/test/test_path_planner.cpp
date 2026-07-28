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

#include "path_planner.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{

std::shared_ptr<autoware_utils_debug::TimeKeeper> make_time_keeper()
{
  return std::make_shared<autoware_utils_debug::TimeKeeper>();
}

Params make_default_params()
{
  Params params;
  params.path_planning.ego_nearest_lanelet.dist_threshold = 3.0;
  params.path_planning.ego_nearest_lanelet.yaw_threshold = 1.57;
  params.path_planning.path_length.backward = 50.0;
  params.path_planning.path_length.forward = 100.0;
  params.path_planning.output.delta_arc_length = 1.0;
  params.path_planning.waypoint_group.separation_threshold = 1.0;
  params.path_planning.waypoint_group.interval_margin_ratio = 0.5;
  params.path_planning.path_shift.enable = false;
  params.path_planning.path_shift.minimum_shift_length = 0.1;
  params.path_planning.path_shift.minimum_shift_yaw = 0.1;
  params.path_planning.path_shift.minimum_shift_distance = 5.0;
  params.path_planning.path_shift.min_speed_for_curvature = 2.77;
  params.path_planning.path_shift.lateral_accel_limit = 0.5;
  return params;
}

autoware_planning_msgs::msg::TrajectoryPoint make_traj_point(double x, double y, float vel = 1.0f)
{
  autoware_planning_msgs::msg::TrajectoryPoint pt;
  pt.pose.position.x = x;
  pt.pose.position.y = y;
  pt.pose.position.z = 0.0;
  pt.pose.orientation.w = 1.0;
  pt.longitudinal_velocity_mps = vel;
  return pt;
}

Trajectory make_straight_trajectory(size_t num_points, double spacing, float velocity)
{
  Trajectory traj;
  traj.header.frame_id = "map";
  for (size_t i = 0; i < num_points; ++i) {
    traj.points.push_back(make_traj_point(spacing * static_cast<double>(i), 0.0, velocity));
  }
  return traj;
}

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw / 2.0);
  pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

PathPointWithLaneId make_path_point(double x, double y, double vel = 1.0, int64_t lane_id = 1)
{
  PathPointWithLaneId pt;
  pt.point.pose.position.x = x;
  pt.point.pose.position.y = y;
  pt.point.pose.position.z = 0.0;
  pt.point.pose.orientation.w = 1.0;
  pt.point.longitudinal_velocity_mps = static_cast<float>(vel);
  pt.lane_ids.push_back(lane_id);
  return pt;
}

}  // namespace

// ============================================================
// PathPlanner construction test
// ============================================================

TEST(PathPlannerTest, ConstructWithoutNode)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.vehicle_length_m = 4.89;

  EXPECT_NO_THROW(PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info));
}

// ============================================================
// shift_trajectory_to_ego tests
// ============================================================

TEST(PathPlannerTest, ShiftNotNeeded)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  auto traj = make_straight_trajectory(20, 1.0, 10.0f);
  // Ego is exactly on the trajectory → no shift needed
  auto ego_pose = make_pose(0.0, 0.0, 0.0);

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = 0.1;
  shift_params.minimum_shift_yaw = 0.1;

  const auto result = planner.shift_trajectory_to_ego(traj, ego_pose, 10.0, 0.0, shift_params, 1.0);

  // Should return the trajectory unchanged since offset and yaw are below threshold
  ASSERT_EQ(result.points.size(), traj.points.size());
  for (size_t i = 0; i < result.points.size(); ++i) {
    EXPECT_NEAR(result.points[i].pose.position.x, traj.points[i].pose.position.x, 1e-3);
    EXPECT_NEAR(result.points[i].pose.position.y, traj.points[i].pose.position.y, 1e-3);
  }
}

TEST(PathPlannerTest, ShiftShortTrajectory)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  // Trajectory with fewer than 2 points → should return as-is
  Trajectory short_traj;
  short_traj.header.frame_id = "map";
  short_traj.points.push_back(make_traj_point(0.0, 0.0, 10.0f));

  auto ego_pose = make_pose(0.0, 1.0, 0.0);  // Offset ego
  TrajectoryShiftParams shift_params;

  const auto result =
    planner.shift_trajectory_to_ego(short_traj, ego_pose, 10.0, 0.0, shift_params, 1.0);

  ASSERT_EQ(result.points.size(), 1u);
  EXPECT_NEAR(result.points[0].pose.position.x, 0.0, 1e-3);
}

TEST(PathPlannerTest, ShiftNormal)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  auto traj = make_straight_trajectory(50, 1.0, 10.0f);
  // Ego is offset laterally (large offset to trigger shift)
  auto ego_pose = make_pose(0.0, 2.0, 0.0);

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = 0.1;
  shift_params.minimum_shift_yaw = 0.1;
  shift_params.minimum_shift_distance = 5.0;
  shift_params.min_speed_for_curvature = 2.77;
  shift_params.lateral_accel_limit = 0.5;

  const auto result = planner.shift_trajectory_to_ego(traj, ego_pose, 10.0, 0.0, shift_params, 1.0);

  // First point should be at ego position
  EXPECT_NEAR(result.points.front().pose.position.x, ego_pose.position.x, 1e-3);
  EXPECT_NEAR(result.points.front().pose.position.y, ego_pose.position.y, 1e-3);

  // Last points should match original trajectory (merged back)
  const auto & last_orig = traj.points.back();
  const auto & last_result = result.points.back();
  EXPECT_NEAR(last_result.pose.position.x, last_orig.pose.position.x, 1e-3);
  EXPECT_NEAR(last_result.pose.position.y, last_orig.pose.position.y, 1e-3);
}

// Regression test for the shoulder-start path shape bug: when the trajectory starts AHEAD of
// ego (refine_path_range clamps the path start to the vehicle front offset at the beginning of
// the route), the shift section used to be skipped entirely (calcLongitudinalOffsetPose returns
// null for base points before the trajectory start), leaving a >10 m straight chord from ego to
// the merge point. The shift section must be generated by backward extrapolation instead.
TEST(PathPlannerTest, ShiftEgoBehindTrajectoryStart)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  // Straight trajectory along +x at y = 0 starting at x = 7 (ahead of ego at x = 0)
  Trajectory traj;
  traj.header.frame_id = "map";
  for (size_t i = 0; i < 100; ++i) {
    traj.points.push_back(make_traj_point(7.0 + static_cast<double>(i), 0.0, 10.0f));
  }
  // Ego stopped on the shoulder, 3 m to the right, behind the trajectory start
  const auto ego_pose = make_pose(0.0, -3.0, 0.0);

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = 0.1;
  shift_params.minimum_shift_yaw = 0.1;
  shift_params.minimum_shift_distance = 5.0;
  shift_params.min_speed_for_curvature = 2.77;
  shift_params.lateral_accel_limit = 1.0;

  const auto result = planner.shift_trajectory_to_ego(traj, ego_pose, 0.0, 0.0, shift_params, 0.5);

  // First point is ego
  ASSERT_GE(result.points.size(), 2u);
  EXPECT_NEAR(result.points.front().pose.position.x, ego_pose.position.x, 1e-3);
  EXPECT_NEAR(result.points.front().pose.position.y, ego_pose.position.y, 1e-3);

  // The shift section must be present: no large point gap (the bug produced a >10 m chord)
  // and no backward steps
  for (size_t i = 1; i < result.points.size(); ++i) {
    const auto & prev = result.points.at(i - 1).pose.position;
    const auto & curr = result.points.at(i).pose.position;
    const double gap = std::hypot(curr.x - prev.x, curr.y - prev.y);
    EXPECT_LT(gap, 2.5) << "large gap at index " << i;
    EXPECT_GT(curr.x, prev.x - 1e-6) << "backward step at index " << i;
  }

  // The trajectory must merge back onto the reference line (y = 0) at the end
  EXPECT_NEAR(result.points.back().pose.position.y, 0.0, 1e-3);
}

// ============================================================
// trajectory anchor tests (connect from the previous trajectory instead of from ego)
// ============================================================

namespace
{
// straight line along +x at the given lateral offset, from x_start to x_end (1 m spacing)
Trajectory make_line_trajectory(double x_start, double x_end, double y)
{
  Trajectory traj;
  traj.header.frame_id = "map";
  for (double x = x_start; x <= x_end + 1e-9; x += 1.0) {
    traj.points.push_back(make_traj_point(x, y, 10.0f));
  }
  return traj;
}

// lateral position of the trajectory where it passes the given x
double lateral_at_x(const Trajectory & traj, double x)
{
  const auto it =
    std::min_element(traj.points.begin(), traj.points.end(), [x](const auto & a, const auto & b) {
      return std::abs(a.pose.position.x - x) < std::abs(b.pose.position.x - x);
    });
  return it->pose.position.y;
}

TrajectoryAnchorParams make_anchor_params()
{
  TrajectoryAnchorParams params;
  params.enable = true;
  params.backward_distance = 5.0;
  params.max_lateral_deviation = 1.0;
  params.max_yaw_deviation = 0.524;
  return params;
}
}  // namespace

TEST(PathPlannerTest, AnchorPoseTakenBehindEgo)
{
  const auto previous_trajectory = make_line_trajectory(-20.0, 80.0, 0.0);
  const auto ego_pose = make_pose(0.0, 0.2, 0.05);

  const auto anchor =
    utils::find_trajectory_anchor_pose(previous_trajectory, ego_pose, make_anchor_params());

  ASSERT_TRUE(anchor.has_value());
  EXPECT_NEAR(anchor->position.x, -5.0, 1e-3);
  EXPECT_NEAR(anchor->position.y, 0.0, 1e-3);
}

// Once ego no longer tracks the previous trajectory (manual intervention, large disturbance,
// pull out from a shoulder), keeping the old line as the base would plan a trajectory the vehicle
// is not on: the caller must reconnect from ego instead.
TEST(PathPlannerTest, AnchorRejectedWhenEgoLeftPreviousTrajectory)
{
  const auto previous_trajectory = make_line_trajectory(-20.0, 80.0, 0.0);

  EXPECT_FALSE(
    utils::find_trajectory_anchor_pose(
      previous_trajectory, make_pose(0.0, 2.0, 0.0), make_anchor_params())
      .has_value());
  EXPECT_FALSE(
    utils::find_trajectory_anchor_pose(
      previous_trajectory, make_pose(0.0, 0.0, 1.0), make_anchor_params())
      .has_value());
}

TEST(PathPlannerTest, AnchorClampedToPreviousTrajectoryStart)
{
  // right after departure the previous trajectory does not reach 5 m behind ego
  const auto previous_trajectory = make_line_trajectory(-2.0, 80.0, 0.0);

  const auto anchor = utils::find_trajectory_anchor_pose(
    previous_trajectory, make_pose(0.0, 0.0, 0.0), make_anchor_params());

  ASSERT_TRUE(anchor.has_value());
  EXPECT_NEAR(anchor->position.x, -2.0, 1e-3);
}

// The point of anchoring: the planned line stays where it was instead of being pulled sideways
// onto ego every cycle. Ego closes the remaining lateral error through the controller.
TEST(PathPlannerTest, AnchorConnectionKeepsTrajectoryOnReferenceAheadOfEgo)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;
  PathPlanner planner(logger, clock, make_time_keeper(), make_default_params(), vehicle_info);

  const auto reference = make_line_trajectory(-10.0, 80.0, 0.0);
  const auto previous_trajectory = make_line_trajectory(-20.0, 80.0, 0.0);
  // ego drifted 0.6 m off the planned line (still within max_lateral_deviation)
  const auto ego_pose = make_pose(0.0, 0.6, 0.0);

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = 0.5;
  shift_params.minimum_shift_yaw = 0.349;
  shift_params.minimum_shift_distance = 5.0;
  shift_params.min_speed_for_curvature = 2.77;
  shift_params.lateral_accel_limit = 1.0;

  const auto anchor =
    utils::find_trajectory_anchor_pose(previous_trajectory, ego_pose, make_anchor_params());
  ASSERT_TRUE(anchor.has_value());

  const auto anchored =
    planner.shift_trajectory_to_pose(reference, *anchor, 0.0, 10.0, shift_params, 0.5);
  const auto to_ego =
    planner.shift_trajectory_to_ego(reference, ego_pose, 10.0, 0.0, shift_params, 0.5);

  // anchored: the trajectory at ego stays on the reference; ego-connected: it is dragged to ego
  EXPECT_NEAR(lateral_at_x(anchored, 0.0), 0.0, 1e-3);
  EXPECT_NEAR(lateral_at_x(to_ego, 0.0), 0.6, 1e-3);
}

// When the previous plan itself is laterally offset from the fresh reference, the shift is
// generated from the anchor (behind ego) so that the offset is already resolved at ego.
TEST(PathPlannerTest, AnchorConnectionMergesFromBehindEgo)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;
  PathPlanner planner(logger, clock, make_time_keeper(), make_default_params(), vehicle_info);

  const auto reference = make_line_trajectory(-10.0, 80.0, 0.0);
  const auto previous_trajectory = make_line_trajectory(-20.0, 80.0, 0.8);
  const auto ego_pose = make_pose(0.0, 0.8, 0.0);

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = 0.5;
  shift_params.minimum_shift_yaw = 0.349;
  shift_params.minimum_shift_distance = 5.0;
  shift_params.min_speed_for_curvature = 2.77;
  shift_params.lateral_accel_limit = 1.0;

  const auto anchor =
    utils::find_trajectory_anchor_pose(previous_trajectory, ego_pose, make_anchor_params());
  ASSERT_TRUE(anchor.has_value());
  EXPECT_NEAR(anchor->position.x, -5.0, 1e-3);

  const auto anchored =
    planner.shift_trajectory_to_pose(reference, *anchor, 0.0, 10.0, shift_params, 0.5);

  // the connection starts at the anchor (behind ego), not at ego ...
  ASSERT_GE(anchored.points.size(), 2u);
  EXPECT_NEAR(anchored.points.front().pose.position.x, -5.0, 1e-3);
  EXPECT_NEAR(anchored.points.front().pose.position.y, 0.8, 1e-3);
  // ... and converges onto the reference without stepping back away from it
  for (size_t i = 1; i < anchored.points.size(); ++i) {
    EXPECT_LE(
      std::abs(anchored.points.at(i).pose.position.y),
      std::abs(anchored.points.at(i - 1).pose.position.y) + 1e-6)
      << "diverging from the reference at index " << i;
  }
  EXPECT_NEAR(anchored.points.back().pose.position.y, 0.0, 1e-3);
}

// Cycle-to-cycle stability, the reason for the anchor: with the ego connection the head geometry
// is a function of the measured ego pose / yaw rate, so it is rebuilt differently every cycle
// (the deployed config even has minimum_shift_yaw = 0, i.e. the shift always runs). Anchored on
// the previous plan, the same reference yields the same trajectory regardless of that noise.
TEST(PathPlannerTest, AnchorConnectionIsUnaffectedByEgoPoseNoise)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;
  PathPlanner planner(logger, clock, make_time_keeper(), make_default_params(), vehicle_info);

  const auto reference = make_line_trajectory(-10.0, 80.0, 0.0);
  const auto previous_trajectory = make_line_trajectory(-20.0, 80.0, 0.0);

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = 0.5;
  shift_params.minimum_shift_yaw = 0.0;  // as deployed: the shift always runs
  shift_params.minimum_shift_distance = 5.0;
  shift_params.min_speed_for_curvature = 2.77;
  shift_params.lateral_accel_limit = 1.0;

  // two consecutive cycles at the same longitudinal position, differing only by ego pose noise
  const auto ego_a = make_pose(0.0, 0.1, 0.05);
  const auto ego_b = make_pose(0.0, -0.1, -0.05);

  const auto anchor_a =
    utils::find_trajectory_anchor_pose(previous_trajectory, ego_a, make_anchor_params());
  const auto anchor_b =
    utils::find_trajectory_anchor_pose(previous_trajectory, ego_b, make_anchor_params());
  ASSERT_TRUE(anchor_a.has_value() && anchor_b.has_value());

  const auto anchored_a =
    planner.shift_trajectory_to_pose(reference, *anchor_a, 0.0, 10.0, shift_params, 0.5);
  const auto anchored_b =
    planner.shift_trajectory_to_pose(reference, *anchor_b, 0.0, 10.0, shift_params, 0.5);

  ASSERT_EQ(anchored_a.points.size(), anchored_b.points.size());
  for (size_t i = 0; i < anchored_a.points.size(); ++i) {
    EXPECT_NEAR(
      anchored_a.points.at(i).pose.position.y, anchored_b.points.at(i).pose.position.y, 1e-9)
      << "trajectory differs between cycles at index " << i;
  }

  // the ego connection, in contrast, moves with the noise
  const auto to_ego_a =
    planner.shift_trajectory_to_ego(reference, ego_a, 10.0, 0.0, shift_params, 0.5);
  const auto to_ego_b =
    planner.shift_trajectory_to_ego(reference, ego_b, 10.0, 0.0, shift_params, 0.5);
  EXPECT_GT(std::abs(lateral_at_x(to_ego_a, 0.0) - lateral_at_x(to_ego_b, 0.0)), 0.1);
}

// ============================================================
// update_current_lanelet tests
// ============================================================

namespace
{

// Two successive lanelets along +x (L1: x 0..30, L2: x 30..60), 3 m wide, with shared boundary
// points so that the routing graph connects them by succession.
struct TwoLaneletRoute
{
  lanelet::LaneletMapPtr map;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;
  lanelet::ConstLanelet l1;
  lanelet::ConstLanelet l2;
};

TwoLaneletRoute make_two_lanelet_route()
{
  const auto make_lane = [](
                           lanelet::Id id, lanelet::Point3d & left_start,
                           lanelet::Point3d & right_start, double x_end,
                           lanelet::Point3d & left_end, lanelet::Point3d & right_end) {
    (void)x_end;
    lanelet::LineString3d left(id * 10 + 1, {left_start, left_end});
    lanelet::LineString3d right(id * 10 + 2, {right_start, right_end});
    lanelet::Lanelet ll(id, left, right);
    ll.attributes()[lanelet::AttributeName::Subtype] = lanelet::AttributeValueString::Road;
    ll.attributes()[lanelet::AttributeName::Location] = lanelet::AttributeValueString::Urban;
    ll.attributes()[lanelet::AttributeName::OneWay] = true;
    return ll;
  };

  lanelet::Point3d l_a(101, 0.0, 1.5, 0.0), r_a(102, 0.0, -1.5, 0.0);
  lanelet::Point3d l_b(103, 30.0, 1.5, 0.0), r_b(104, 30.0, -1.5, 0.0);
  lanelet::Point3d l_c(105, 60.0, 1.5, 0.0), r_c(106, 60.0, -1.5, 0.0);

  auto ll1 = make_lane(1, l_a, r_a, 30.0, l_b, r_b);
  auto ll2 = make_lane(2, l_b, r_b, 60.0, l_c, r_c);

  TwoLaneletRoute route;
  route.map = lanelet::utils::createMap({ll1, ll2});
  route.traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  route.routing_graph = lanelet::routing::RoutingGraph::build(*route.map, *route.traffic_rules);
  route.l1 = ll1;
  route.l2 = ll2;
  return route;
}

RouteContext make_route_context(const TwoLaneletRoute & route)
{
  RouteContext context;
  context.lanelet_map_ptr = route.map;
  context.traffic_rules_ptr = route.traffic_rules;
  context.routing_graph_ptr = route.routing_graph;
  context.route_lanelets = {route.l1, route.l2};
  context.preferred_lanelets = {route.l1, route.l2};
  context.start_lanelets = {route.l1};
  context.goal_lanelets = {route.l2};
  return context;
}

}  // namespace

// Regression test for the shoulder-start path shape bug: ego parked on the road shoulder is
// outside every route lanelet, and update_current_lanelet used to treat that as "passed the
// current lanelet" and advance to the following lanelet EVERY cycle, making the generated path
// run away forward along the route. Laterally-offset ego must keep the current lanelet.
TEST(PathPlannerTest, CurrentLaneletKeptWhileEgoOnShoulder)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), make_default_params(), vehicle_info);
  const auto route = make_two_lanelet_route();
  planner.set_route_context(make_route_context(route));

  // Ego parked on the right shoulder alongside L1 (outside the lanelet boundary y = -1.5)
  const auto shoulder_pose = make_pose(10.0, -3.5, 0.0);

  ASSERT_TRUE(planner.update_current_lanelet(shoulder_pose));
  ASSERT_TRUE(planner.current_lanelet().has_value());
  EXPECT_EQ(planner.current_lanelet()->id(), route.l1.id());

  // Repeated cycles at the same pose must NOT advance the current lanelet
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(planner.update_current_lanelet(shoulder_pose)) << "cycle " << i;
    EXPECT_EQ(planner.current_lanelet()->id(), route.l1.id()) << "cycle " << i;
  }
}

// The original intent of the advance branch must be preserved: when ego (still outside the
// lanelets laterally) has passed the END of the current lanelet, advance to the following one.
TEST(PathPlannerTest, CurrentLaneletAdvancesWhenEgoPassesEnd)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), make_default_params(), vehicle_info);
  const auto route = make_two_lanelet_route();
  planner.set_route_context(make_route_context(route));

  ASSERT_TRUE(planner.update_current_lanelet(make_pose(10.0, -3.5, 0.0)));
  ASSERT_EQ(planner.current_lanelet()->id(), route.l1.id());

  // Ego (still on the shoulder) has driven past the end of L1 (x = 30)
  ASSERT_TRUE(planner.update_current_lanelet(make_pose(35.0, -3.5, 0.0)));
  EXPECT_EQ(planner.current_lanelet()->id(), route.l2.id());
}

// ============================================================
// convert_path_to_trajectory tests
// ============================================================

TEST(PathPlannerTest, ConvertPathToTrajectoryEmpty)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  const PathWithLaneId empty_path;
  const auto result = planner.convert_path_to_trajectory(empty_path, 1.0);
  EXPECT_TRUE(result.points.empty());
}

TEST(PathPlannerTest, ConvertPathToTrajectoryVelocityPreserved)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  PathWithLaneId path;
  path.header.frame_id = "map";
  const float speed = 5.0f;
  for (int i = 0; i <= 10; ++i) {
    path.points.push_back(make_path_point(static_cast<double>(i), 0.0, speed));
  }

  const auto result = planner.convert_path_to_trajectory(path, 1.0);
  ASSERT_FALSE(result.points.empty());
  for (const auto & pt : result.points) {
    EXPECT_NEAR(pt.longitudinal_velocity_mps, speed, 1e-3f);
  }
}

TEST(PathPlannerTest, ConvertPathToTrajectoryResamplingSpacing)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  PathWithLaneId path;
  path.header.frame_id = "map";
  for (int i = 0; i <= 10; ++i) {
    path.points.push_back(make_path_point(static_cast<double>(i), 0.0, 1.0));
  }

  const double interval = 0.5;
  const auto result = planner.convert_path_to_trajectory(path, interval);
  ASSERT_GE(result.points.size(), 2u);

  for (size_t i = 1; i + 1 < result.points.size(); ++i) {
    const double dx = result.points[i].pose.position.x - result.points[i - 1].pose.position.x;
    const double dy = result.points[i].pose.position.y - result.points[i - 1].pose.position.y;
    EXPECT_NEAR(std::hypot(dx, dy), interval, 0.05);
  }
}

}  // namespace autoware::minimum_rule_based_planner
