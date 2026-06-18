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

#include "autoware/target_lanelet_estimator/impl.hpp"

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <utility>
#include <vector>

namespace autoware::target_lanelet_estimator
{
namespace
{
// A 3 m wide straight lane along x, centered at y_center.
lanelet::Lanelet make_lane(lanelet::Id id, double y_center, double x_start, double x_end)
{
  constexpr double half_width = 1.5;
  lanelet::LineString3d left{
    lanelet::utils::getId(),
    {lanelet::Point3d{lanelet::utils::getId(), x_start, y_center + half_width},
     lanelet::Point3d{lanelet::utils::getId(), x_end, y_center + half_width}}};
  lanelet::LineString3d right{
    lanelet::utils::getId(),
    {lanelet::Point3d{lanelet::utils::getId(), x_start, y_center - half_width},
     lanelet::Point3d{lanelet::utils::getId(), x_end, y_center - half_width}}};
  return lanelet::Lanelet{id, left, right};
}

LaneletRoute make_route(const std::vector<lanelet::Id> & ids)
{
  LaneletRoute route;
  autoware_planning_msgs::msg::LaneletSegment segment;
  for (const auto id : ids) {
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = id;
    segment.primitives.push_back(primitive);
  }
  segment.preferred_primitive.id = ids.front();
  route.segments.push_back(segment);
  return route;
}

// All points share yaw = 0 (orientation.w = 1).
Trajectory make_trajectory(const std::vector<std::pair<double, double>> & xy_points)
{
  Trajectory trajectory;
  for (const auto & [x, y] : xy_points) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.orientation.w = 1.0;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

VehicleInfo make_vehicle_info()
{
  VehicleInfo vehicle_info;
  vehicle_info.wheel_base_m = 2.0;
  vehicle_info.front_overhang_m = 0.5;
  vehicle_info.rear_overhang_m = 0.5;
  vehicle_info.wheel_tread_m = 1.0;  // half width 0.5 -> total width 1.0 m
  return vehicle_info;
}

double score_of(const std::vector<LaneletScore> & lanelets, lanelet::Id id)
{
  for (const auto & lanelet : lanelets) {
    if (lanelet.id == id) {
      return lanelet.score;
    }
  }
  return 0.0;
}

constexpr lanelet::Id lane_a = 1001;
constexpr lanelet::Id lane_b = 1002;
}  // namespace

class GetTargetLaneletsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    lanelet_map_ = lanelet::utils::createMap(
      {make_lane(lane_a, 0.0, -5.0, 25.0), make_lane(lane_b, 3.0, -5.0, 25.0)});
    vehicle_info_ = make_vehicle_info();
  }

  lanelet::LaneletMapConstPtr lanelet_map_;
  VehicleInfo vehicle_info_;
};

// Driving straight along one lane -> that single lanelet, not out of lanelet.
TEST_F(GetTargetLaneletsTest, SingleLanelet)
{
  const auto route = make_route({lane_a, lane_b});
  const auto trajectory = make_trajectory({{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  ASSERT_EQ(result.lanelets.size(), 1u);
  EXPECT_EQ(result.lanelets.front().id, lane_a);
  // footprint stays fully inside lane A -> likelihood 1.0
  EXPECT_NEAR(result.lanelets.front().score, 1.0, 1e-6);
  EXPECT_FALSE(result.out_of_lanelet);
}

// A trajectory crossing from lane A to lane B -> both lanelets returned.
TEST_F(GetTargetLaneletsTest, LaneChangeStraddling)
{
  const auto route = make_route({lane_a, lane_b});
  const auto trajectory = make_trajectory({{0.0, 0.0}, {5.0, 1.5}, {10.0, 3.0}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  EXPECT_EQ(result.lanelets.size(), 2u);
  // the footprint is fully inside each lane at some point -> both reach likelihood 1.0
  EXPECT_NEAR(score_of(result.lanelets, lane_a), 1.0, 1e-6);
  EXPECT_NEAR(score_of(result.lanelets, lane_b), 1.0, 1e-6);
  EXPECT_FALSE(result.out_of_lanelet);
}

// A footprint that always straddles the A/B boundary -> partial likelihood on both.
TEST_F(GetTargetLaneletsTest, PartialOverlapLikelihood)
{
  const auto route = make_route({lane_a, lane_b});
  // footprint centered at y=1.25 spans y in [0.75, 1.75]; the A/B boundary is at y=1.5,
  // so 0.75 of the footprint is on lane A and 0.25 on lane B.
  const auto trajectory = make_trajectory({{0.0, 1.25}, {5.0, 1.25}, {10.0, 1.25}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  EXPECT_NEAR(score_of(result.lanelets, lane_a), 0.75, 1e-3);
  EXPECT_NEAR(score_of(result.lanelets, lane_b), 0.25, 1e-3);
  EXPECT_FALSE(result.out_of_lanelet);
}

// A trajectory far from every lanelet -> no lanelet, flagged out of lanelet.
TEST_F(GetTargetLaneletsTest, OutOfLanelet)
{
  const auto route = make_route({lane_a, lane_b});
  const auto trajectory = make_trajectory({{0.0, 50.0}, {5.0, 50.0}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  EXPECT_TRUE(result.lanelets.empty());
  EXPECT_TRUE(result.out_of_lanelet);
}

}  // namespace autoware::target_lanelet_estimator
