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

#include <Eigen/Geometry>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <cmath>
#include <memory>
#include <vector>

namespace autoware::ml_planner::test
{
using autoware::ml_planner::preprocess::LaneSegmentContext;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;

namespace
{
// Map-to-ego transform of an ego at (x, y) heading `yaw`.
Eigen::Matrix4d map_to_ego(const double x, const double y, const double yaw)
{
  Eigen::Isometry3d ego_to_map = Eigen::Isometry3d::Identity();
  ego_to_map.translate(Eigen::Vector3d(x, y, 0.0));
  ego_to_map.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  return ego_to_map.inverse().matrix();
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

// The route goes east on `eastbound_`, and later comes back north on `northbound_`, which crosses
// `eastbound_` at (50, 0).
class RouteSegmentSelectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    eastbound_ = make_straight_lanelet(0.0, 0.0, 100.0, 0.0);
    northbound_ = make_straight_lanelet(50.0, -50.0, 50.0, 50.0);

    auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
    lanelet_map->add(eastbound_);
    lanelet_map->add(northbound_);
    context_ = std::make_unique<LaneSegmentContext>(lanelet_map);

    route_.segments = {make_route_segment(eastbound_.id()), make_route_segment(northbound_.id())};
  }

  int64_t array_index(const lanelet::Lanelet & lanelet) const
  {
    return static_cast<int64_t>(context_->get_lanelet_id_to_array_index().at(lanelet.id()));
  }

  lanelet::Lanelet eastbound_;
  lanelet::Lanelet northbound_;
  std::unique_ptr<LaneSegmentContext> context_;
  LaneletRoute route_;
};

TEST_F(RouteSegmentSelectionTest, SelectsSegmentAlignedWithEgoAtSelfCrossing)
{
  // The ego drives east at the crossing point, where it is on both lanelets.
  const auto indices = context_->select_route_segment_indices(
    route_, map_to_ego(50.0, 0.3, 0.0), 50.0, 0.3, 0.0, 0.0, NUM_SEGMENTS_IN_ROUTE);

  ASSERT_EQ(indices.size(), 2U);
  EXPECT_EQ(indices.front(), array_index(eastbound_));
  EXPECT_EQ(indices.back(), array_index(northbound_));
}

TEST_F(RouteSegmentSelectionTest, SelectsCrossingSegmentWhenEgoHeadsAlongIt)
{
  const auto indices = context_->select_route_segment_indices(
    route_, map_to_ego(50.3, 0.0, M_PI / 2.0), 50.3, 0.0, 0.0, M_PI / 2.0, NUM_SEGMENTS_IN_ROUTE);

  ASSERT_EQ(indices.size(), 1U);
  EXPECT_EQ(indices.front(), array_index(northbound_));
}

TEST_F(RouteSegmentSelectionTest, FallsBackToClosestSegmentWhenNoSegmentIsAligned)
{
  // The ego heads west, which matches neither lanelet direction, and is only on `northbound_`.
  const auto indices = context_->select_route_segment_indices(
    route_, map_to_ego(50.0, 3.0, M_PI), 50.0, 3.0, 0.0, M_PI, NUM_SEGMENTS_IN_ROUTE);

  ASSERT_EQ(indices.size(), 1U);
  EXPECT_EQ(indices.front(), array_index(northbound_));
}

}  // namespace autoware::ml_planner::test
