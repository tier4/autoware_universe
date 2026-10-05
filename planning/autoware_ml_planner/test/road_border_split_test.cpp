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

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <vector>

namespace autoware::ml_planner::test
{

class RoadBorderSplitTest : public ::testing::Test
{
protected:
  void SetUp() override { lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>(); }

  void add_road_border(const std::vector<std::pair<double, double>> & points)
  {
    lanelet::LineString3d border(lanelet::utils::getId());
    for (const auto & [x, y] : points) {
      border.push_back(lanelet::Point3d(lanelet::utils::getId(), x, y, 0.0));
    }
    border.setAttribute("type", "road_border");
    lanelet_map_ptr_->add(border);
  }

  static std::vector<LanePoint> flatten(const std::vector<MapPolyline> & borders)
  {
    std::vector<LanePoint> points;
    for (const auto & border : borders) {
      points.insert(points.end(), border.points.begin(), border.points.end());
    }
    return points;
  }

  static bool contains(const std::vector<LanePoint> & points, const LanePoint & target)
  {
    for (const auto & point : points) {
      if ((point - target).norm() < 1e-9) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
};

TEST_F(RoadBorderSplitTest, KeepsEveryOriginalPoint)
{
  // Irregular spacing, including points closer than any resampling step would produce.
  const std::vector<std::pair<double, double>> raw{
    {0.0, 0.0}, {0.3, 0.1}, {7.0, 0.0}, {7.2, 2.0}, {30.0, 2.0}, {31.0, 2.5}, {80.0, 2.5}};
  add_road_border(raw);
  const auto map = convert_to_internal_lanelet_map(lanelet_map_ptr_);
  const auto points = flatten(map.road_borders);
  for (const auto & [x, y] : raw) {
    EXPECT_TRUE(contains(points, LanePoint(x, y, 0.0))) << "missing (" << x << ", " << y << ")";
  }
}

TEST_F(RoadBorderSplitTest, FillsGapsAndSplitsIntoFixedSizePieces)
{
  add_road_border({{0.0, 0.0}, {200.0, 0.0}});
  const auto map = convert_to_internal_lanelet_map(lanelet_map_ptr_);
  // 200 m at <= 5 m spacing gives 41 points, split into pieces of 20 sharing end points.
  ASSERT_GE(map.road_borders.size(), 2u);
  for (size_t i = 0; i < map.road_borders.size(); ++i) {
    const auto & points = map.road_borders[i].points;
    ASSERT_EQ(points.size(), static_cast<size_t>(POINTS_PER_ROAD_BORDER));
    for (size_t j = 1; j < points.size(); ++j) {
      EXPECT_LE((points[j] - points[j - 1]).norm(), 5.0 + 1e-9);
    }
  }
  EXPECT_EQ(map.road_borders.front().points.front(), LanePoint(0.0, 0.0, 0.0));
  EXPECT_EQ(map.road_borders.back().points.back(), LanePoint(200.0, 0.0, 0.0));
  for (size_t i = 1; i < map.road_borders.size(); ++i) {
    EXPECT_EQ(map.road_borders[i].points.front(), map.road_borders[i - 1].points.back());
  }
}

TEST_F(RoadBorderSplitTest, PiecesDoNotOverlap)
{
  // 30 points 1 m apart: split 15 / 16 at x = 14 and each piece is filled up to 20 points.
  std::vector<std::pair<double, double>> raw;
  for (int x = 0; x < 30; ++x) {
    raw.emplace_back(static_cast<double>(x), 0.0);
  }
  add_road_border(raw);
  const auto map = convert_to_internal_lanelet_map(lanelet_map_ptr_);
  ASSERT_EQ(map.road_borders.size(), 2u);
  const auto & first = map.road_borders[0].points;
  const auto & second = map.road_borders[1].points;
  ASSERT_EQ(first.size(), static_cast<size_t>(POINTS_PER_ROAD_BORDER));
  ASSERT_EQ(second.size(), static_cast<size_t>(POINTS_PER_ROAD_BORDER));
  EXPECT_EQ(first.front(), LanePoint(0.0, 0.0, 0.0));
  EXPECT_EQ(first.back(), LanePoint(14.0, 0.0, 0.0));
  EXPECT_EQ(second.front(), LanePoint(14.0, 0.0, 0.0));
  EXPECT_EQ(second.back(), LanePoint(29.0, 0.0, 0.0));
  for (size_t i = 1; i < first.size(); ++i) {
    EXPECT_LE(first[i].x(), 14.0);
    EXPECT_GE(second[i].x(), 14.0);
  }
}

TEST_F(RoadBorderSplitTest, ShortBorderIsPaddedWithMidpoints)
{
  add_road_border({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  const auto map = convert_to_internal_lanelet_map(lanelet_map_ptr_);
  ASSERT_EQ(map.road_borders.size(), 1u);
  const auto & points = map.road_borders[0].points;
  ASSERT_EQ(points.size(), static_cast<size_t>(POINTS_PER_ROAD_BORDER));
  EXPECT_TRUE(contains(points, LanePoint(1.0, 0.0, 0.0)));
  EXPECT_EQ(points.front(), LanePoint(0.0, 0.0, 0.0));
  EXPECT_EQ(points.back(), LanePoint(2.0, 0.0, 0.0));
}

}  // namespace autoware::ml_planner::test
