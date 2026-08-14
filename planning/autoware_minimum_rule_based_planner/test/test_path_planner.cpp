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
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

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
  params.path_planning.early_stop.enable = false;
  params.path_planning.smooth_goal_connection.search_radius_range = 15.0;
  params.path_planning.smooth_goal_connection.pre_goal_offset = 3.0;
  params.path_planning.path_shift.enable = false;
  params.path_planning.path_shift.minimum_shift_length = 0.1;
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

std::vector<lanelet::Lanelet> make_straight_lanelets(size_t count, double length)
{
  std::vector<lanelet::Point3d> left_points;
  std::vector<lanelet::Point3d> right_points;
  for (size_t i = 0; i <= count; ++i) {
    const auto x = static_cast<double>(i) * length;
    left_points.emplace_back(10000 + i, x, 2.0, 0.0);
    right_points.emplace_back(20000 + i, x, -2.0, 0.0);
  }

  std::vector<lanelet::Lanelet> lanelets;
  for (size_t i = 0; i < count; ++i) {
    lanelet::LineString3d left(30000 + i, {left_points[i], left_points[i + 1]});
    lanelet::LineString3d right(40000 + i, {right_points[i], right_points[i + 1]});
    lanelet::Lanelet lanelet(1 + i, left, right);
    lanelet.attributes()[lanelet::AttributeNamesString::Subtype] =
      lanelet::AttributeValueString::Road;
    lanelet.attributes()[lanelet::AttributeNamesString::Location] =
      lanelet::AttributeValueString::Urban;
    lanelet.attributes()["one_way"] = "yes";
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

RouteContext make_route_context(const std::vector<lanelet::Lanelet> & lanelets)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  for (const auto & lanelet : lanelets) {
    map->add(lanelet);
  }
  auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  RouteContext context;
  context.lanelet_map_ptr = map;
  context.routing_graph_ptr = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);
  context.traffic_rules_ptr = std::move(traffic_rules);
  context.route_lanelets.assign(lanelets.begin(), lanelets.end());
  context.preferred_lanelets.assign(lanelets.begin(), lanelets.end());
  return context;
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

TEST(PathPlannerTest, CurrentLaneletAtVehicleFootprintCenter)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.min_longitudinal_offset_m = -1.0;
  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  lanelet::LineString3d left(
    101, {lanelet::Point3d(102, 10.0, 2.0, 0.0), lanelet::Point3d(103, 20.0, 2.0, 0.0)});
  lanelet::LineString3d right(
    104, {lanelet::Point3d(105, 10.0, -2.0, 0.0), lanelet::Point3d(106, 20.0, -2.0, 0.0)});
  const lanelet::Lanelet lanelet(1, left, right);
  RouteContext context;
  context.route_lanelets = {lanelet};
  planner.set_route_context(context);

  // base_link is behind the lanelet, while the footprint center is at x = 10.5 inside it.
  const auto base_link_pose = make_pose(9.0, 0.0);
  EXPECT_TRUE(planner.update_current_lanelet(base_link_pose));

  // Force the containment fallback after the current lanelet has been initialized.
  planner.route_context().route_lanelets.clear();
  EXPECT_TRUE(planner.update_current_lanelet(base_link_pose));
}

TEST(PathPlannerTest, PathStartIsContinuousAtFootprintCenterLaneletTransition)
{
  auto params = make_default_params();
  params.path_planning.path_length.backward = 5.0;
  params.path_planning.path_length.forward = 10.0;
  const auto lanelets = make_straight_lanelets(3, 10.0);
  const builtin_interfaces::msg::Time stamp;

  for (const auto & [max_offset, min_offset] :
       std::vector<std::pair<double, double>>{{4.0, -1.0}, {1.0, -1.0}, {1.0, -2.0}}) {
    VehicleInfo vehicle_info{};
    vehicle_info.max_longitudinal_offset_m = max_offset;
    vehicle_info.min_longitudinal_offset_m = min_offset;
    vehicle_info.vehicle_length_m = max_offset - min_offset;
    PathPlanner planner(
      rclcpp::get_logger("test_path_planner"), std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
      make_time_keeper(), params, vehicle_info);
    planner.set_route_context(make_route_context(lanelets));

    const auto center_offset = (max_offset + min_offset) / 2.0;
    const auto base_at_transition = 10.0 - center_offset;
    const auto before = planner.plan_path(make_pose(base_at_transition - 0.01, 0.0), 1.0, stamp);
    const auto after = planner.plan_path(make_pose(base_at_transition + 0.01, 0.0), 1.0, stamp);

    ASSERT_TRUE(before);
    ASSERT_TRUE(after);
    ASSERT_FALSE(before->points.empty());
    ASSERT_FALSE(after->points.empty());
    EXPECT_NEAR(
      after->points.front().point.pose.position.x - before->points.front().point.pose.position.x,
      0.02, 0.05);
    EXPECT_LE(
      std::abs(static_cast<long>(after->points.size()) - static_cast<long>(before->points.size())),
      1);
  }
}

TEST(PathPlannerTest, FootprintCenterHandlesShortLanelets)
{
  auto params = make_default_params();
  params.path_planning.path_length.backward = 2.0;
  params.path_planning.path_length.forward = 4.0;
  VehicleInfo vehicle_info{};
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.min_longitudinal_offset_m = -1.0;
  vehicle_info.vehicle_length_m = 5.0;
  PathPlanner planner(
    rclcpp::get_logger("test_path_planner"), std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
    make_time_keeper(), params, vehicle_info);
  planner.set_route_context(make_route_context(make_straight_lanelets(8, 2.0)));

  const builtin_interfaces::msg::Time stamp;
  const auto route_start = planner.plan_path(make_pose(1.0, 0.0), 1.0, stamp);
  for (double x = 1.5; x <= 6.0; x += 0.5) {
    ASSERT_TRUE(planner.plan_path(make_pose(x, 0.0), 1.0, stamp));
  }
  const auto before = planner.plan_path(make_pose(6.49, 0.0), 1.0, stamp);
  const auto exact = planner.plan_path(make_pose(6.5, 0.0), 1.0, stamp);
  const auto after = planner.plan_path(make_pose(6.51, 0.0), 1.0, stamp);
  for (double x = 7.0; x < 12.0; x += 0.5) {
    ASSERT_TRUE(planner.plan_path(make_pose(x, 0.0), 1.0, stamp));
  }
  const auto route_end = planner.plan_path(make_pose(12.0, 0.0), 1.0, stamp);

  ASSERT_TRUE(route_start);
  ASSERT_TRUE(before);
  ASSERT_TRUE(exact);
  ASSERT_TRUE(after);
  ASSERT_TRUE(route_end);
  ASSERT_FALSE(before->points.empty());
  ASSERT_FALSE(after->points.empty());
  EXPECT_NEAR(
    after->points.front().point.pose.position.x - before->points.front().point.pose.position.x,
    0.02, 0.05);
}

// ============================================================
// shift_trajectory_to_ego tests
// ============================================================

TEST(PathPlannerTest, ShiftAlwaysStartsAtEgo)
{
  auto logger = rclcpp::get_logger("test_path_planner");
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto params = make_default_params();
  VehicleInfo vehicle_info{};
  vehicle_info.wheel_base_m = 2.79;

  PathPlanner planner(logger, clock, make_time_keeper(), params, vehicle_info);

  auto traj = make_straight_trajectory(20, 1.0, 10.0f);
  // Keep the offset below the former shift threshold.
  auto ego_pose = make_pose(0.0, 0.05, 0.0);

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = 0.1;

  const auto result = planner.shift_trajectory_to_ego(traj, ego_pose, 10.0, 0.0, shift_params, 1.0);

  ASSERT_FALSE(result.points.empty());
  EXPECT_NEAR(result.points.front().pose.position.x, ego_pose.position.x, 1e-3);
  EXPECT_NEAR(result.points.front().pose.position.y, ego_pose.position.y, 1e-3);
  EXPECT_NEAR(result.points.front().pose.orientation.z, ego_pose.orientation.z, 1e-3);
  EXPECT_NEAR(result.points.front().pose.orientation.w, ego_pose.orientation.w, 1e-3);
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
