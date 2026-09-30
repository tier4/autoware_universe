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

#include "autoware/tensorrt_e2e/providers/map_tensors.hpp"

#include <autoware/diffusion_planner/dimensions.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace
{
namespace dp = autoware::diffusion_planner;
using dp::LanePoint;
using dp::SEGMENT_POINT_DIM;

constexpr int64_t kSlots = 3;
constexpr int64_t kPointsPerSegment = dp::POINTS_PER_SEGMENT;

//! Ids double as the map order. The cloud pose is the origin, the planning pose 1.1 m ahead and
//! yawed: A is nearest to the cloud pose, B to the planning pose, C is far from both. So the
//! cloud pose orders A, B, C; the planning pose would order B, A, C.
constexpr int64_t kA = 1;
constexpr int64_t kB = 2;
constexpr int64_t kC = 3;

std::vector<LanePoint> column(const double x, const double y)
{
  std::vector<LanePoint> points;
  for (int64_t i = 0; i < kPointsPerSegment; ++i) {
    points.emplace_back(x, y + 0.01 * static_cast<double>(i), 0.0);
  }
  return points;
}

dp::LaneSegment lane(const int64_t id, const double x, const double y)
{
  const auto centerline = column(x, y);
  return dp::LaneSegment(
    id, centerline, column(x - 1.0, y), column(x + 1.0, y), LanePoint(x, y, 0.0),
    dp::LINE_TYPE_LINE_THIN, dp::LINE_TYPE_LINE_THIN, 10.0f, dp::LaneSegment::TURN_DIRECTION_NONE,
    dp::LaneSegment::TRAFFIC_LIGHT_ID_NONE);
}

dp::preprocess::LaneSegmentContext make_context()
{
  dp::LaneletMap map;
  map.lane_segments = {lane(kA, 0.0, 3.0), lane(kB, 2.0, 3.0), lane(kC, 50.0, 0.0)};
  // Polygons and line strings tie-break on the same geometry: A' before B' from the cloud pose.
  const auto polyline = [](const double x, const double y) {
    return std::vector<LanePoint>{LanePoint(x, y, 0.0), LanePoint(x, y + 1.0, 0.0)};
  };
  map.polygons = {
    {polyline(0.0, 3.0), dp::POLYGON_TYPE_INTERSECTION_AREA, 1},
    {polyline(2.0, 3.0), dp::POLYGON_TYPE_INTERSECTION_AREA, 2}};
  map.line_strings = {
    {polyline(0.0, 3.0), dp::LINE_STRING_TYPE_STOP_LINE, 1, 0},
    {polyline(2.0, 3.0), dp::LINE_STRING_TYPE_STOP_LINE, 2, 0}};
  return dp::preprocess::LaneSegmentContext(std::move(map));
}

Eigen::Matrix4d pose_matrix(const double x, const double y, const double yaw)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m.block<2, 2>(0, 0) << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  m(0, 3) = x;
  m(1, 3) = y;
  return m;
}

//! An ego frame planning at `plan` from a map cut at `cloud`; equal poses are `cloud_stamp`.
EgoFrame make_ego(const Eigen::Matrix4d & cloud, const Eigen::Matrix4d & plan)
{
  EgoFrame ego;
  ego.ego_to_map = plan;
  ego.map_to_ego = plan.inverse();
  ego.sensor_to_map = cloud;
  ego.map_to_sensor = cloud.inverse();
  return ego;
}

autoware_planning_msgs::msg::LaneletRoute make_route()
{
  autoware_planning_msgs::msg::LaneletRoute route;
  route.segments.resize(3);
  route.segments[0].preferred_primitive.id = kA;
  route.segments[1].preferred_primitive.id = kB;
  route.segments[2].preferred_primitive.id = kC;
  return route;
}

const std::map<lanelet::Id, dp::preprocess::TrafficSignalStamped> kNoLights;

//! x, y of the first point of `slot` in a `lanes` / `route_lanes` tensor.
std::pair<float, float> first_point(const std::vector<float> & data, const int64_t slot)
{
  const size_t base = static_cast<size_t>(slot * kPointsPerSegment * SEGMENT_POINT_DIM);
  return {data[base + dp::X], data[base + dp::Y]};
}

const Eigen::Matrix4d kCloud = pose_matrix(0.0, 0.0, 0.0);
const Eigen::Matrix4d kPlanning = pose_matrix(1.1, 0.0, 0.2);
}  // namespace

TEST(MapTensorsTest, CloudStampSelectsAndWritesInTheEgoFrame)
{
  const auto context = make_context();
  const EgoFrame ego = make_ego(kPlanning, kPlanning);

  const auto lanes = map_tensors::build_lanes(context, ego, kNoLights, kSlots);
  // What the node built before the cloud pose existed: both steps from map_to_ego.
  const auto old_indices = context.select_lane_segment_indices(ego.map_to_ego, 1.1f, 0.0f, kSlots);
  const auto old_data =
    context.create_tensor_data_from_indices(ego.map_to_ego, kNoLights, old_indices, kSlots);
  EXPECT_EQ(lanes.indices, old_indices);
  EXPECT_EQ(lanes.indices, (std::vector<int64_t>{1, 0, 2}));  // B, A, C
  EXPECT_EQ(lanes.data, old_data.first);
  EXPECT_EQ(lanes.speed_limit, old_data.second);

  EXPECT_EQ(
    map_tensors::build_polygons(context, ego, 2, 2),
    context.create_polygon_tensor(ego.map_to_ego, 1.1f, 0.0f, 2, 2));
  EXPECT_EQ(
    map_tensors::build_line_strings(context, ego, 2),
    context.create_line_string_tensor(ego.map_to_ego, 1.1f, 0.0f, 2));

  const auto route = make_route();
  const auto route_lanes = map_tensors::build_route_lanes(context, ego, route, kNoLights, kSlots);
  const auto old_route =
    context.select_route_segment_indices(route, 1.1, 0.0, 0.0, kSlots);
  EXPECT_EQ(route_lanes.indices, old_route);
  EXPECT_EQ(route_lanes.indices, (std::vector<int64_t>{1, 2}));  // starts at B
}

TEST(MapTensorsTest, PlanningTimeSelectsAtTheCloudPoseAndWritesInThePlanningFrame)
{
  const auto context = make_context();
  const EgoFrame ego = make_ego(kCloud, kPlanning);

  // Order and set from the cloud pose: A, B, C -- not the planning pose's B, A, C.
  const auto lanes = map_tensors::build_lanes(context, ego, kNoLights, kSlots);
  ASSERT_EQ(lanes.indices.size(), 3U);
  EXPECT_EQ(lanes.indices, (std::vector<int64_t>{0, 1, 2}));
  // Points in the planning frame: slot 0 (A) first point is (0, 3) seen from the planning pose.
  const Eigen::Vector4d a_in_plan = ego.map_to_ego * Eigen::Vector4d(0.0, 3.0, 0.0, 1.0);
  const auto [ax, ay] = first_point(lanes.data, 0);
  EXPECT_NEAR(ax, a_in_plan.x(), 1e-5);
  EXPECT_NEAR(ay, a_in_plan.y(), 1e-5);
  const Eigen::Vector4d b_in_plan = ego.map_to_ego * Eigen::Vector4d(2.0, 3.0, 0.0, 1.0);
  const auto [bx, by] = first_point(lanes.data, 1);
  EXPECT_NEAR(bx, b_in_plan.x(), 1e-5);
  EXPECT_NEAR(by, b_in_plan.y(), 1e-5);
  // The same slots the plain call gives when handed the cloud-pose order and the planning frame.
  EXPECT_EQ(
    lanes.data,
    context.create_tensor_data_from_indices(ego.map_to_ego, kNoLights, lanes.indices, kSlots)
      .first);

  // Polygons and line strings: the slot order is the cloud pose's (id 1 then 2), the written
  // points the planning frame's. Layout [slot, point, 2 + types]; padding stays exactly zero.
  const Eigen::Vector4d first = ego.map_to_ego * Eigen::Vector4d(0.0, 3.0, 0.0, 1.0);
  const Eigen::Vector4d second = ego.map_to_ego * Eigen::Vector4d(2.0, 3.0, 0.0, 1.0);
  const auto check_slots = [&](const std::vector<float> & tensor, const int64_t points) {
    const size_t point_dim = 4;  // x, y and two types
    ASSERT_EQ(tensor.size(), static_cast<size_t>(2 * points) * point_dim);
    const size_t slot1 = static_cast<size_t>(points) * point_dim;
    EXPECT_NEAR(tensor[0], first.x(), 1e-5);
    EXPECT_NEAR(tensor[1], first.y(), 1e-5);
    EXPECT_NEAR(tensor[slot1], second.x(), 1e-5);
    EXPECT_NEAR(tensor[slot1 + 1], second.y(), 1e-5);
    EXPECT_EQ(tensor[2 * point_dim], 0.0f);  // third point of slot 0: absent
    EXPECT_EQ(tensor[2 * point_dim + 1], 0.0f);
  };
  check_slots(map_tensors::build_polygons(context, ego, 2, 2), dp::POINTS_PER_POLYGON);
  check_slots(map_tensors::build_line_strings(context, ego, 2), dp::POINTS_PER_LINE_STRING);

  // The route starts at the lanelet nearest the cloud pose (A), not the planning pose's (B).
  const auto route_lanes =
    map_tensors::build_route_lanes(context, ego, make_route(), kNoLights, kSlots);
  EXPECT_EQ(route_lanes.indices, (std::vector<int64_t>{0, 1, 2}));
  const auto [rx, ry] = first_point(route_lanes.data, 0);
  EXPECT_NEAR(rx, a_in_plan.x(), 1e-5);
  EXPECT_NEAR(ry, a_in_plan.y(), 1e-5);
}

}  // namespace autoware::tensorrt_e2e
