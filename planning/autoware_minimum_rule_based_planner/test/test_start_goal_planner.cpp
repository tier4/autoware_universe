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

#include "start_goal_planner/start_goal_planner.hpp"

#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace
{

std::shared_ptr<autoware_utils_debug::TimeKeeper> make_time_keeper()
{
  return std::make_shared<autoware_utils_debug::TimeKeeper>();
}

StartGoalPlannerParams make_default_params()
{
  StartGoalPlannerParams params;
  params.reference_velocity = 4.7;
  params.steer_angle_trial_count = 5;
  params.max_steer_angle_rate = 30.0;
  params.eval_weight_curvature = 0.4;
  params.eval_weight_length = 0.3;
  params.eval_weight_diff = 0.3;
  params.traj_generation_exit_count = 5;
  params.goal_planner.search_radius_range = 20.0;
  params.goal_planner.pre_goal_offset = 1.0;
  params.goal_planner.num_start_poses = 10;
  params.start_planner.max_path_range = 25.0;
  params.start_planner.num_goal_poses = 10;
  return params;
}

VehicleInfo make_vehicle_info()
{
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;
  vehicle_info.max_steer_angle_rad = 0.7;
  vehicle_info.vehicle_length_m = 4.89;
  vehicle_info.vehicle_width_m = 1.9;
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.rear_overhang_m = 1.0;
  vehicle_info.left_overhang_m = 0.1;
  vehicle_info.right_overhang_m = 0.1;
  return vehicle_info;
}

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  return pose;
}

PathPointWithLaneId make_path_point(double x, double y, double vel, int64_t lane_id)
{
  PathPointWithLaneId pt;
  pt.point.pose = make_pose(x, y, 0.0);
  pt.point.longitudinal_velocity_mps = static_cast<float>(vel);
  pt.lane_ids.push_back(lane_id);
  return pt;
}

// Straight trajectory along +x at y = 0, referencing a single lane id.
PathPointTrajectory make_straight_trajectory(
  double x_start, double x_end, double spacing, int64_t lane_id, double vel = 3.0)
{
  std::vector<PathPointWithLaneId> points;
  const auto num_steps = static_cast<size_t>(std::floor((x_end - x_start) / spacing)) + 1;
  for (size_t i = 0; i < num_steps; ++i) {
    const double x = x_start + static_cast<double>(i) * spacing;
    points.push_back(make_path_point(x, 0.0, vel, lane_id));
  }
  const auto trajectory = autoware::experimental::trajectory::pretty_build(points);
  if (!trajectory) {
    ADD_FAILURE() << "failed to build test trajectory";
    return {};
  }
  return *trajectory;
}

// Straight lanelet spanning x in [x_start, x_end], with left/right bound linestrings supplied
// (so that adjacent lanelets can share a boundary linestring, as required by
// StartGoalPlanner's adjacency checks).
lanelet::Lanelet make_lanelet(
  lanelet::Id id, const lanelet::LineString3d & left, const lanelet::LineString3d & right)
{
  return lanelet::Lanelet(id, left, right);
}

lanelet::LineString3d make_line(lanelet::Id id, double x_start, double x_end, double y)
{
  return lanelet::LineString3d(
    id,
    {lanelet::Point3d(id * 10 + 1, x_start, y, 0.0), lanelet::Point3d(id * 10 + 2, x_end, y, 0.0)});
}

lanelet::Polygon3d make_parking_lot(
  lanelet::Id id, double x_start, double x_end, double y_min, double y_max)
{
  lanelet::Polygon3d polygon(
    id, {lanelet::Point3d(id * 10 + 1, x_start, y_min, 0.0),
         lanelet::Point3d(id * 10 + 2, x_end, y_min, 0.0),
         lanelet::Point3d(id * 10 + 3, x_end, y_max, 0.0),
         lanelet::Point3d(id * 10 + 4, x_start, y_max, 0.0)});
  polygon.attributes()[lanelet::AttributeName::Type] = "parking_lot";
  return polygon;
}

lanelet::routing::RoutingGraphPtr make_routing_graph(const lanelet::LaneletMapPtr & map)
{
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  return lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
}

// Common test fixture geometry: a straight road lanelet (id 10, y in [-2, 2], x in [0, 40])
// with an adjacent road-shoulder lanelet (id 11, y in [-6, -2]) sharing the road's right bound.
// This shoulder lanelet is picked up by get_available_area() (shares a boundary with the base
// lanelet and has the road_shoulder subtype), so ego placed inside it can trigger the start
// planner without needing a separate parking-lot polygon.
struct StartLaneletFixture
{
  lanelet::LaneletMapPtr map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Lanelet road_lanelet;
  lanelet::Lanelet shoulder_lanelet;
  lanelet::routing::RoutingGraphPtr routing_graph;

  StartLaneletFixture()
  {
    const auto left = make_line(1, 0.0, 40.0, 2.0);
    const auto right = make_line(2, 0.0, 40.0, -2.0);
    const auto shoulder_right = make_line(3, 0.0, 40.0, -8.0);

    road_lanelet = make_lanelet(10, left, right);
    road_lanelet.attributes()[lanelet::AttributeNamesString::Subtype] =
      lanelet::AttributeValueString::Road;

    shoulder_lanelet = make_lanelet(11, right, shoulder_right);
    shoulder_lanelet.attributes()[lanelet::AttributeNamesString::Subtype] = "road_shoulder";

    map->add(road_lanelet);
    map->add(shoulder_lanelet);
    routing_graph = make_routing_graph(map);
  }
};

StartGoalPlanner::RouteData make_route_data(
  const StartLaneletFixture & fixture, const geometry_msgs::msg::Pose & goal_pose)
{
  StartGoalPlanner::RouteData route_data;
  route_data.goal_pose = goal_pose;
  route_data.preferred_lanelets = {fixture.road_lanelet};
  route_data.start_lanelets = {fixture.road_lanelet};
  route_data.lanelet_map_ptr = fixture.map;
  route_data.routing_graph_ptr = fixture.routing_graph;
  return route_data;
}

}  // namespace

// ============================================================
// plan(): no start/goal condition met
// ============================================================

TEST(StartGoalPlannerTest, PlanReturnsNulloptWhenNeitherConditionIsMet)
{
  StartLaneletFixture fixture;
  auto trajectory = make_straight_trajectory(0.0, 40.0, 1.0, fixture.road_lanelet.id());

  StartGoalPlanner planner(
    rclcpp::get_logger("test_start_goal_planner"), make_time_keeper(), make_default_params(),
    make_vehicle_info());
  // Goal far outside the search radius, ego on the road centerline (no lateral offset).
  planner.set_route_data(make_route_data(fixture, make_pose(1000.0, 1000.0, 0.0)));

  auto current_lanelet = lanelet::ConstLanelet(fixture.road_lanelet);
  const auto ego_pose = make_pose(5.0, 0.0, 0.0);
  const auto result = planner.plan(trajectory, current_lanelet, trajectory.length(), ego_pose);

  EXPECT_FALSE(result.has_value());
  EXPECT_FALSE(planner.start_planner_active());
  EXPECT_FALSE(planner.goal_planner_active());
}

// ============================================================
// plan(): start planner activation / deactivation
// ============================================================

TEST(StartGoalPlannerTest, StartPlannerActivatesAndGeneratesTrajectoryFromShoulder)
{
  StartLaneletFixture fixture;
  auto trajectory = make_straight_trajectory(0.0, 40.0, 1.0, fixture.road_lanelet.id());

  StartGoalPlanner planner(
    rclcpp::get_logger("test_start_goal_planner"), make_time_keeper(), make_default_params(),
    make_vehicle_info());
  planner.set_route_data(make_route_data(fixture, make_pose(1000.0, 1000.0, 0.0)));

  auto current_lanelet = lanelet::ConstLanelet(fixture.road_lanelet);
  // Ego is inside the road-shoulder lanelet (y = -6), well beyond the 1 m centerline threshold.
  const auto ego_pose = make_pose(10.0, -6.0, 0.0);
  const auto result = planner.plan(trajectory, current_lanelet, trajectory.length(), ego_pose);

  EXPECT_TRUE(planner.start_planner_active());
  EXPECT_FALSE(planner.goal_planner_active());
  ASSERT_TRUE(result.has_value());
  // The generated trajectory must start from the ego pose.
  const auto start_point = result->compute(0.0);
  EXPECT_NEAR(start_point.point.pose.position.x, ego_pose.position.x, 1e-2);
  EXPECT_NEAR(start_point.point.pose.position.y, ego_pose.position.y, 1e-2);
}

TEST(StartGoalPlannerTest, StartPlannerDeactivatesWhenEgoReturnsToCenterline)
{
  StartLaneletFixture fixture;
  auto trajectory = make_straight_trajectory(0.0, 40.0, 1.0, fixture.road_lanelet.id());

  StartGoalPlanner planner(
    rclcpp::get_logger("test_start_goal_planner"), make_time_keeper(), make_default_params(),
    make_vehicle_info());
  planner.set_route_data(make_route_data(fixture, make_pose(1000.0, 1000.0, 0.0)));

  auto current_lanelet = lanelet::ConstLanelet(fixture.road_lanelet);
  // First call: activate the start planner from the shoulder.
  planner.plan(trajectory, current_lanelet, trajectory.length(), make_pose(10.0, -6.0, 0.0));
  ASSERT_TRUE(planner.start_planner_active());

  // Second call: ego has returned close to the lane centerline (< 1 m).
  const auto result =
    planner.plan(trajectory, current_lanelet, trajectory.length(), make_pose(15.0, 0.0, 0.0));

  EXPECT_FALSE(planner.start_planner_active());
  EXPECT_FALSE(result.has_value());
}

// ============================================================
// plan(): fallback trajectory when no available area exists
// ============================================================

TEST(StartGoalPlannerTest, StartPlannerFallsBackToStoppedTrajectoryWithoutAvailableArea)
{
  StartLaneletFixture fixture;
  auto trajectory = make_straight_trajectory(0.0, 40.0, 1.0, fixture.road_lanelet.id());

  StartGoalPlanner planner(
    rclcpp::get_logger("test_start_goal_planner"), make_time_keeper(), make_default_params(),
    make_vehicle_info());
  planner.set_route_data(make_route_data(fixture, make_pose(1000.0, 1000.0, 0.0)));

  auto current_lanelet = lanelet::ConstLanelet(fixture.road_lanelet);
  // Ego is inside the road-shoulder lanelet (y = -6), well beyond the 1 m centerline threshold.
  const auto ego_pose = make_pose(10.0, -6.0, -M_PI / 4);
  const auto result = planner.plan(trajectory, current_lanelet, trajectory.length(), ego_pose);

  EXPECT_TRUE(planner.start_planner_active());
  ASSERT_TRUE(result.has_value());
  for (const auto & point : result->restore()) {
    EXPECT_FLOAT_EQ(point.point.longitudinal_velocity_mps, 0.0f);
    EXPECT_FLOAT_EQ(point.point.lateral_velocity_mps, 0.0f);
  }
}

// ============================================================
// plan(): goal planner activation and deactivation on goal move
// ============================================================

TEST(StartGoalPlannerTest, GoalPlannerActivatesAndGeneratesTrajectoryIntoParkingLot)
{
  StartLaneletFixture fixture;
  fixture.map->add(make_parking_lot(20, 30.0, 40.0, -10.0, 10.0));

  auto trajectory = make_straight_trajectory(0.0, 40.0, 1.0, fixture.road_lanelet.id());

  StartGoalPlanner planner(
    rclcpp::get_logger("test_start_goal_planner"), make_time_keeper(), make_default_params(),
    make_vehicle_info());
  // Goal is inside the parking lot, close to the trajectory end, within the search radius.
  const auto goal_pose = make_pose(30.0, -4.0, 0.0);
  planner.set_route_data(make_route_data(fixture, goal_pose));

  auto current_lanelet = lanelet::ConstLanelet(fixture.road_lanelet);
  // Ego is on the road centerline, far from the goal / shoulder, so only the goal planner
  // condition is satisfied.
  const auto ego_pose = make_pose(5.0, 0.0, 0.0);
  const auto result = planner.plan(trajectory, current_lanelet, trajectory.length(), ego_pose);

  EXPECT_TRUE(planner.goal_planner_active());
  EXPECT_FALSE(planner.start_planner_active());
  ASSERT_TRUE(result.has_value());
  const auto end_point = result->compute(result->length());
  EXPECT_NEAR(end_point.point.pose.position.x, goal_pose.position.x, 1e-2);
  EXPECT_NEAR(end_point.point.pose.position.y, goal_pose.position.y, 1e-2);
}

TEST(StartGoalPlannerTest, GoalPlannerResetsWhenGoalPoseMoves)
{
  StartLaneletFixture fixture;
  fixture.map->add(make_parking_lot(20, 30.0, 40.0, -10.0, 10.0));

  auto trajectory = make_straight_trajectory(0.0, 40.0, 1.0, fixture.road_lanelet.id());

  StartGoalPlanner planner(
    rclcpp::get_logger("test_start_goal_planner"), make_time_keeper(), make_default_params(),
    make_vehicle_info());
  const auto goal_pose = make_pose(30.0, -4.0, 0.0);
  planner.set_route_data(make_route_data(fixture, goal_pose));

  auto current_lanelet = lanelet::ConstLanelet(fixture.road_lanelet);
  const auto ego_pose = make_pose(5.0, 0.0, 0.0);
  planner.plan(trajectory, current_lanelet, trajectory.length(), ego_pose);
  ASSERT_TRUE(planner.goal_planner_active());

  // Move the goal far away: the goal planner must reset.
  auto route_data = make_route_data(fixture, make_pose(1000.0, 1000.0, 0.0));
  planner.set_route_data(route_data);
  const auto result = planner.plan(trajectory, current_lanelet, trajectory.length(), ego_pose);

  EXPECT_FALSE(planner.goal_planner_active());
  EXPECT_FALSE(result.has_value());
}

// ============================================================
// update_params(): parameter changes take effect
// ============================================================

TEST(StartGoalPlannerTest, UpdateParamsShrinksGoalSearchRadius)
{
  StartLaneletFixture fixture;
  fixture.map->add(make_parking_lot(20, 30.0, 40.0, -10.0, 10.0));

  auto trajectory = make_straight_trajectory(0.0, 40.0, 1.0, fixture.road_lanelet.id());

  StartGoalPlanner planner(
    rclcpp::get_logger("test_start_goal_planner"), make_time_keeper(), make_default_params(),
    make_vehicle_info());
  const auto goal_pose = make_pose(38.0, -4.0, 0.0);
  planner.set_route_data(make_route_data(fixture, goal_pose));

  // Shrink the search radius so the same goal pose is no longer within range.
  auto params = make_default_params();
  params.goal_planner.search_radius_range = 1.0;
  planner.update_params(params);

  auto current_lanelet = lanelet::ConstLanelet(fixture.road_lanelet);
  const auto ego_pose = make_pose(5.0, 0.0, 0.0);
  const auto result = planner.plan(trajectory, current_lanelet, trajectory.length(), ego_pose);

  EXPECT_FALSE(planner.goal_planner_active());
  EXPECT_FALSE(result.has_value());
}

}  // namespace autoware::minimum_rule_based_planner
