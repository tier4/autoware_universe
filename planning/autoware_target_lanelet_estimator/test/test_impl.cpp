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
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <utility>
#include <vector>

namespace autoware::target_lanelet_estimator
{
namespace
{
lanelet::Lanelet make_lane(
  lanelet::Id id, const lanelet::LineString3d & left, const lanelet::LineString3d & right)
{
  lanelet::Lanelet lanelet{id, left, right};
  lanelet.setAttribute(lanelet::AttributeName::Subtype, lanelet::AttributeValueString::Road);
  return lanelet;
}

lanelet::LineString3d make_line(const lanelet::Point3d & start, const lanelet::Point3d & end)
{
  return lanelet::LineString3d{lanelet::utils::getId(), {start, end}};
}

lanelet::Point3d make_point(double x, double y)
{
  return lanelet::Point3d{lanelet::utils::getId(), x, y, 0.0};
}

LaneletRoute make_route(const std::vector<std::vector<lanelet::Id>> & segment_ids)
{
  LaneletRoute route;
  for (const auto & ids : segment_ids) {
    autoware_planning_msgs::msg::LaneletSegment segment;
    for (const auto id : ids) {
      autoware_planning_msgs::msg::LaneletPrimitive primitive;
      primitive.id = id;
      segment.primitives.push_back(primitive);
    }
    segment.preferred_primitive.id = ids.front();
    route.segments.push_back(segment);
  }
  return route;
}

LaneletRoute make_route(const std::vector<lanelet::Id> & ids)
{
  return make_route(std::vector<std::vector<lanelet::Id>>{ids});
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

double posterior_of(const std::vector<LaneletProbability> & lanelets, lanelet::Id id)
{
  for (const auto & lanelet : lanelets) {
    if (lanelet.id == id) {
      return lanelet.posterior;
    }
  }
  return 0.0;
}

double likelihood_of(const std::vector<LaneletProbability> & lanelets, lanelet::Id id)
{
  for (const auto & lanelet : lanelets) {
    if (lanelet.id == id) {
      return lanelet.likelihood;
    }
  }
  return 0.0;
}

double prior_of(const std::vector<LaneletProbability> & lanelets, lanelet::Id id)
{
  for (const auto & lanelet : lanelets) {
    if (lanelet.id == id) {
      return lanelet.prior;
    }
  }
  return 0.0;
}

bool has_target_lanelet(const TargetLaneletsResult & result, lanelet::Id id)
{
  return std::find(result.target_lanelet_ids.begin(), result.target_lanelet_ids.end(), id) !=
         result.target_lanelet_ids.end();
}

lanelet::routing::RoutingGraphConstPtr make_routing_graph(
  const lanelet::LaneletMapConstPtr & lanelet_map)
{
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  return lanelet::routing::RoutingGraph::build(*lanelet_map, *traffic_rules);
}

constexpr lanelet::Id lane_a = 1001;
constexpr lanelet::Id lane_b = 1002;
constexpr lanelet::Id lane_c = 1003;
constexpr lanelet::Id lane_d = 1004;
}  // namespace

class GetTargetLaneletsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto a_left_start = make_point(-5.0, 1.5);
    const auto a_left_end = make_point(5.0, 1.5);
    const auto a_right_start = make_point(-5.0, -1.5);
    const auto a_right_end = make_point(5.0, -1.5);
    const auto b_left_start = make_point(-5.0, 4.5);
    const auto b_left_end = make_point(5.0, 4.5);
    const auto c_left_end = make_point(25.0, 1.5);
    const auto c_right_end = make_point(25.0, -1.5);
    const auto d_left_end = make_point(25.0, 4.5);

    const auto a_left = make_line(a_left_start, a_left_end);
    const auto a_right = make_line(a_right_start, a_right_end);
    const auto b_left = make_line(b_left_start, b_left_end);
    const auto c_left = make_line(a_left_end, c_left_end);
    const auto c_right = make_line(a_right_end, c_right_end);
    const auto d_left = make_line(b_left_end, d_left_end);

    lanelet_map_ = lanelet::utils::createMap(
      {make_lane(lane_a, a_left, a_right), make_lane(lane_b, b_left, a_left),
       make_lane(lane_c, c_left, c_right), make_lane(lane_d, d_left, c_left)});
    vehicle_info_ = make_vehicle_info();
  }

  lanelet::LaneletMapConstPtr lanelet_map_;
  VehicleInfo vehicle_info_;
};

// Driving straight along one lane -> that single lanelet, not out of lanelet.
TEST_F(GetTargetLaneletsTest, SingleLanelet)
{
  const auto route = make_route({lane_a, lane_b});
  const auto trajectory = make_trajectory({{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  ASSERT_EQ(result.lanelet_probabilities.size(), 2u);
  // footprint stays fully inside lane A -> likelihood 1.0
  EXPECT_NEAR(likelihood_of(result.lanelet_probabilities, lane_a), 1.0, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_a), 1.0, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_b), 0.0, 1e-6);
  ASSERT_EQ(result.target_lanelet_ids.size(), 1u);
  EXPECT_TRUE(has_target_lanelet(result, lane_a));
  EXPECT_FALSE(result.out_of_lanelet);
}

// A trajectory crossing from lane A to lane B -> both lanelets returned.
TEST_F(GetTargetLaneletsTest, LaneChangeStraddling)
{
  const auto route = make_route({lane_a, lane_b});
  const auto trajectory = make_trajectory({{0.0, 0.0}, {1.0, 1.5}, {2.0, 3.0}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  EXPECT_EQ(result.lanelet_probabilities.size(), 2u);
  // the footprint is fully inside each lane at some point -> both reach likelihood 1.0
  EXPECT_NEAR(likelihood_of(result.lanelet_probabilities, lane_a), 1.0, 1e-6);
  EXPECT_NEAR(likelihood_of(result.lanelet_probabilities, lane_b), 1.0, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_a), 1.0, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_b), 1.0, 1e-6);
  ASSERT_EQ(result.target_lanelet_ids.size(), 2u);
  EXPECT_TRUE(has_target_lanelet(result, lane_a));
  EXPECT_TRUE(has_target_lanelet(result, lane_b));
  EXPECT_FALSE(result.out_of_lanelet);
}

// A footprint that always straddles the A/B boundary -> partial likelihood on both.
TEST_F(GetTargetLaneletsTest, PartialOverlapLikelihood)
{
  const auto route = make_route({lane_a, lane_b});
  // footprint centered at y=1.25 spans y in [0.75, 1.75]; the A/B boundary is at y=1.5,
  // so 0.75 of the footprint is on lane A and 0.25 on lane B.
  const auto trajectory = make_trajectory({{0.0, 1.25}, {2.0, 1.25}, {4.0, 1.25}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  EXPECT_NEAR(likelihood_of(result.lanelet_probabilities, lane_a), 0.75, 1e-3);
  EXPECT_NEAR(likelihood_of(result.lanelet_probabilities, lane_b), 0.25, 1e-3);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_a), 0.909, 1e-3);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_b), 0.091, 1e-3);
  ASSERT_EQ(result.target_lanelet_ids.size(), 2u);
  EXPECT_TRUE(has_target_lanelet(result, lane_a));
  EXPECT_TRUE(has_target_lanelet(result, lane_b));
  EXPECT_FALSE(result.out_of_lanelet);
}

// A trajectory far from every lanelet -> no lanelet, flagged out of lanelet.
TEST_F(GetTargetLaneletsTest, OutOfLanelet)
{
  const auto route = make_route({lane_a, lane_b});
  const auto trajectory = make_trajectory({{0.0, 50.0}, {2.0, 50.0}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  ASSERT_EQ(result.lanelet_probabilities.size(), 2u);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_a), 0.0, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_b), 0.0, 1e-6);
  EXPECT_TRUE(result.target_lanelet_ids.empty());
  EXPECT_TRUE(result.out_of_lanelet);
}

TEST_F(GetTargetLaneletsTest, KeepsInitialProbabilitiesOutsideUpdateScope)
{
  const auto route = make_route({{lane_a, lane_b}, {lane_c, lane_d}});
  const auto trajectory = make_trajectory({{0.0, 0.0}, {2.0, 0.0}});

  const auto result = get_target_lanelets(route, trajectory, lanelet_map_, vehicle_info_);

  ASSERT_EQ(result.lanelet_probabilities.size(), 4u);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_c), 0.8, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_d), 0.2, 1e-6);
}

TEST_F(GetTargetLaneletsTest, NextSegmentPriorUsesRoutingRelation)
{
  const auto route = make_route({{lane_a, lane_b}, {lane_c, lane_d}});
  const auto trajectory = make_trajectory({{0.0, 0.0}, {6.0, 0.0}});
  const auto routing_graph = make_routing_graph(lanelet_map_);
  const auto previous_posteriors = initialize_lanelet_probabilities(route);

  const auto result = get_target_lanelets(
    route, trajectory, lanelet_map_, vehicle_info_, previous_posteriors, routing_graph);

  ASSERT_EQ(result.lanelet_probabilities.size(), 4u);
  EXPECT_GT(
    prior_of(result.lanelet_probabilities, lane_c), prior_of(result.lanelet_probabilities, lane_d));
  EXPECT_NEAR(prior_of(result.lanelet_probabilities, lane_c), 0.68, 1e-6);
  EXPECT_NEAR(prior_of(result.lanelet_probabilities, lane_d), 0.32, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_c), 1.0, 1e-6);
  EXPECT_NEAR(posterior_of(result.lanelet_probabilities, lane_d), 0.0, 1e-6);
}

}  // namespace autoware::target_lanelet_estimator
