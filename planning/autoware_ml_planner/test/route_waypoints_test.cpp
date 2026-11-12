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

#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/preprocessing/items/map.hpp"

#include <Eigen/Core>
#include <xtensor/xarray.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::ml_planner::test
{
using autoware::ml_planner::preprocess::LaneSegmentContext;
using autoware::ml_planner::preprocess::sample_route_waypoints;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;

namespace
{
constexpr double tolerance = 1e-4;

void expect_waypoint(
  const xt::xarray<float> & waypoints, const int64_t k, const double x, const double y,
  const double cos_yaw, const double sin_yaw)
{
  EXPECT_NEAR(waypoints(k, 0), x, tolerance) << "waypoint " << k;
  EXPECT_NEAR(waypoints(k, 1), y, tolerance) << "waypoint " << k;
  EXPECT_NEAR(waypoints(k, 2), cos_yaw, tolerance) << "waypoint " << k;
  EXPECT_NEAR(waypoints(k, 3), sin_yaw, tolerance) << "waypoint " << k;
}

void expect_zero_from(const xt::xarray<float> & waypoints, const int64_t first)
{
  for (int64_t k = first; k < static_cast<int64_t>(waypoints.shape(0)); ++k) {
    expect_waypoint(waypoints, k, 0.0, 0.0, 0.0, 0.0);
  }
}

lanelet::Lanelet make_straight_lanelet(
  const double start_x, const double start_y, const double end_x, const double end_y)
{
  constexpr double half_width = 1.75;
  const double length = std::hypot(end_x - start_x, end_y - start_y);
  // left normal of the lanelet direction
  const double normal_x = -(end_y - start_y) / length * half_width;
  const double normal_y = (end_x - start_x) / length * half_width;

  const auto make_bound = [&](const double offset_x, const double offset_y) {
    return lanelet::LineString3d(
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), start_x + offset_x, start_y + offset_y, 0.0),
       lanelet::Point3d(lanelet::utils::getId(), end_x + offset_x, end_y + offset_y, 0.0)});
  };

  lanelet::Lanelet lanelet(
    lanelet::utils::getId(), make_bound(normal_x, normal_y), make_bound(-normal_x, -normal_y));
  lanelet.setAttribute("subtype", "road");
  return lanelet;
}

LaneletSegment make_route_segment(const lanelet::Id id)
{
  LaneletSegment segment;
  segment.preferred_primitive.id = id;
  segment.primitives.resize(1);
  segment.primitives.front().id = id;
  return segment;
}
}  // namespace

TEST(RouteWaypointsTest, StartAtEgoProjectionOnStraightLine)
{
  const std::vector<Eigen::Vector2d> line = {{-10.0, 2.0}, {190.0, 2.0}};
  const auto waypoints = sample_route_waypoints(line, NUM_ROUTE_WAYPOINTS, 5.0);

  ASSERT_EQ(waypoints.shape(0), static_cast<size_t>(NUM_ROUTE_WAYPOINTS));
  ASSERT_EQ(waypoints.shape(1), static_cast<size_t>(POSE_DIM));
  for (int64_t k = 0; k < NUM_ROUTE_WAYPOINTS; ++k) {
    expect_waypoint(waypoints, k, 5.0 * static_cast<double>(k), 2.0, 1.0, 0.0);
  }
}

TEST(RouteWaypointsTest, WaypointsBeyondRouteEndAreZero)
{
  const std::vector<Eigen::Vector2d> line = {{-10.0, 0.0}, {12.0, 0.0}};
  const auto waypoints = sample_route_waypoints(line, NUM_ROUTE_WAYPOINTS, 5.0);

  expect_waypoint(waypoints, 0, 0.0, 0.0, 1.0, 0.0);
  expect_waypoint(waypoints, 1, 5.0, 0.0, 1.0, 0.0);
  expect_waypoint(waypoints, 2, 10.0, 0.0, 1.0, 0.0);
  expect_zero_from(waypoints, 3);
}

TEST(RouteWaypointsTest, LaneChangeStepIsPartOfLine)
{
  // The route moves to the left lane at x = 10: the join between the two segments is a sideways
  // step of 3.5 m that counts as route length.
  const std::vector<Eigen::Vector2d> line = {{-10.0, 0.0}, {10.0, 0.0}, {10.0, 3.5}, {30.0, 3.5}};
  const auto waypoints = sample_route_waypoints(line, NUM_ROUTE_WAYPOINTS, 5.0);

  expect_waypoint(waypoints, 0, 0.0, 0.0, 1.0, 0.0);
  expect_waypoint(waypoints, 1, 5.0, 0.0, 1.0, 0.0);
  // At the start of the step, the point takes the direction of the step
  expect_waypoint(waypoints, 2, 10.0, 0.0, 0.0, 1.0);
  expect_waypoint(waypoints, 3, 11.5, 3.5, 1.0, 0.0);
  expect_waypoint(waypoints, 6, 26.5, 3.5, 1.0, 0.0);
  expect_zero_from(waypoints, 7);
}

TEST(RouteWaypointsTest, EgoBesideLineProjectsOntoIt)
{
  const std::vector<Eigen::Vector2d> line = {{-20.0, -3.0}, {80.0, -3.0}};
  const auto waypoints = sample_route_waypoints(line, NUM_ROUTE_WAYPOINTS, 5.0);

  expect_waypoint(waypoints, 0, 0.0, -3.0, 1.0, 0.0);
  expect_waypoint(waypoints, 15, 75.0, -3.0, 1.0, 0.0);
  expect_zero_from(waypoints, 16);
}

TEST(RouteWaypointsTest, EgoPastRouteEndGivesZeros)
{
  const std::vector<Eigen::Vector2d> line = {{-50.0, 0.0}, {-10.0, 0.0}};
  const auto waypoints = sample_route_waypoints(line, NUM_ROUTE_WAYPOINTS, 5.0);

  expect_zero_from(waypoints, 0);
}

TEST(RouteWaypointsTest, LineWithoutPartGivesZeros)
{
  for (const auto & line :
       {std::vector<Eigen::Vector2d>{}, std::vector<Eigen::Vector2d>{{1.0, 2.0}}}) {
    const auto waypoints = sample_route_waypoints(line, NUM_ROUTE_WAYPOINTS, 5.0);
    ASSERT_EQ(waypoints.shape(0), static_cast<size_t>(NUM_ROUTE_WAYPOINTS));
    expect_zero_from(waypoints, 0);
  }
}

TEST(RouteWaypointsTest, FollowsRouteSegmentCenterlinesInEgoFrame)
{
  // Two consecutive lanelets on y = 0; the ego stands at (30, 0.5), heading east
  const lanelet::Lanelet first = make_straight_lanelet(0.0, 0.0, 100.0, 0.0);
  const lanelet::Lanelet second = make_straight_lanelet(100.0, 0.0, 200.0, 0.0);
  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(first);
  lanelet_map->add(second);
  const LaneSegmentContext context(lanelet_map);

  LaneletRoute route;
  route.segments = {make_route_segment(first.id()), make_route_segment(second.id())};
  Eigen::Matrix4d map_to_ego = Eigen::Matrix4d::Identity();
  map_to_ego(0, 3) = -30.0;
  map_to_ego(1, 3) = -0.5;
  const auto segment_indices = context.select_route_segment_indices(
    route, map_to_ego, 30.0, 0.5, 0.0, 0.0, NUM_SEGMENTS_IN_ROUTE);
  ASSERT_EQ(segment_indices.size(), 2U);

  const auto waypoints = context.create_route_waypoint_tensor(
    map_to_ego, segment_indices, NUM_ROUTE_WAYPOINTS, ROUTE_WAYPOINT_SPACING_M);

  // Waypoint 14 lies where the two lanelets meet
  for (int64_t k = 0; k < NUM_ROUTE_WAYPOINTS; ++k) {
    expect_waypoint(
      waypoints, k, ROUTE_WAYPOINT_SPACING_M * static_cast<double>(k), -0.5, 1.0, 0.0);
  }
}

}  // namespace autoware::ml_planner::test
