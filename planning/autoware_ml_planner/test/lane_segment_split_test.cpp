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

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <vector>

namespace autoware::ml_planner::test
{

class LaneSegmentSplitTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet_id_ = add_straight_lanelet(100.0);
  }

  // Straight lanelet along +x from the origin, sampled every metre.
  lanelet::Id add_straight_lanelet(const double length)
  {
    lanelet::LineString3d left(lanelet::utils::getId());
    lanelet::LineString3d right(lanelet::utils::getId());
    for (int x = 0; x <= static_cast<int>(length); ++x) {
      left.push_back(lanelet::Point3d(lanelet::utils::getId(), x, 1.5, 0.0));
      right.push_back(lanelet::Point3d(lanelet::utils::getId(), x, -1.5, 0.0));
    }
    lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
    lanelet.setAttribute("subtype", "road");
    lanelet.setAttribute("speed_limit", "30");
    lanelet_map_ptr_->add(lanelet);
    return lanelet.id();
  }

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  lanelet::Id lanelet_id_{};
};

TEST_F(LaneSegmentSplitTest, DisabledKeepsOneSegmentPerLanelet)
{
  const auto map = convert_to_internal_lanelet_map(lanelet_map_ptr_, 5.0, 0.0);
  ASSERT_EQ(map.lane_segments.size(), 1u);
  EXPECT_EQ(map.lane_segments[0].centerline.size(), static_cast<size_t>(POINTS_PER_SEGMENT));
}

TEST_F(LaneSegmentSplitTest, ShortLaneletIsIdenticalToUnsplit)
{
  const auto unsplit = convert_to_internal_lanelet_map(lanelet_map_ptr_, 5.0, 0.0);
  const auto split = convert_to_internal_lanelet_map(lanelet_map_ptr_, 5.0, 150.0);
  ASSERT_EQ(split.lane_segments.size(), 1u);
  for (size_t i = 0; i < unsplit.lane_segments[0].centerline.size(); ++i) {
    EXPECT_EQ(split.lane_segments[0].centerline[i], unsplit.lane_segments[0].centerline[i]);
    EXPECT_EQ(split.lane_segments[0].left_boundary[i], unsplit.lane_segments[0].left_boundary[i]);
    EXPECT_EQ(
      split.lane_segments[0].right_boundary[i], unsplit.lane_segments[0].right_boundary[i]);
  }
}

TEST_F(LaneSegmentSplitTest, LongLaneletIsSplitIntoContiguousEqualPieces)
{
  const auto map = convert_to_internal_lanelet_map(lanelet_map_ptr_, 5.0, 30.0);
  // ceil(100 / 30) = 4 pieces of 25 m.
  ASSERT_EQ(map.lane_segments.size(), 4u);
  for (size_t part = 0; part < map.lane_segments.size(); ++part) {
    const LaneSegment & segment = map.lane_segments[part];
    EXPECT_EQ(segment.id, lanelet_id_);
    ASSERT_EQ(segment.centerline.size(), static_cast<size_t>(POINTS_PER_SEGMENT));
    ASSERT_EQ(segment.left_boundary.size(), static_cast<size_t>(POINTS_PER_SEGMENT));
    ASSERT_EQ(segment.right_boundary.size(), static_cast<size_t>(POINTS_PER_SEGMENT));
    EXPECT_NEAR(segment.centerline.front().x(), 25.0 * static_cast<double>(part), 1e-6);
    EXPECT_NEAR(segment.centerline.back().x(), 25.0 * static_cast<double>(part + 1), 1e-6);
    EXPECT_NEAR(segment.left_boundary.front().x(), segment.centerline.front().x(), 1e-6);
    EXPECT_NEAR(segment.right_boundary.back().x(), segment.centerline.back().x(), 1e-6);
    EXPECT_NEAR(segment.mean_point.x(), 25.0 * static_cast<double>(part) + 12.5, 1e-6);
    ASSERT_TRUE(segment.speed_limit_mps.has_value());
    if (part > 0) {
      EXPECT_EQ(segment.centerline.front(), map.lane_segments[part - 1].centerline.back());
    }
  }
}

TEST_F(LaneSegmentSplitTest, DefaultSplitsWithHardcodedMaxLength)
{
  // The default uses constants::LANE_SEGMENT_MAX_LENGTH_M (20 m): 100 m -> 5 pieces.
  const auto map = convert_to_internal_lanelet_map(lanelet_map_ptr_);
  EXPECT_EQ(map.lane_segments.size(), 5u);
}

TEST_F(LaneSegmentSplitTest, RouteSelectionStartsFromClosestPiece)
{
  const preprocess::LaneSegmentContext context(lanelet_map_ptr_);
  const auto & id_to_indices = context.get_lanelet_id_to_array_index();
  ASSERT_EQ(id_to_indices.at(lanelet_id_).size(), 5u);

  autoware_planning_msgs::msg::LaneletRoute route;
  autoware_planning_msgs::msg::LaneletSegment route_segment;
  route_segment.preferred_primitive.id = lanelet_id_;
  route.segments.push_back(route_segment);

  // Ego at x = 70 m lies on the fourth piece [60, 80]; the pieces behind it are dropped.
  const std::vector<int64_t> selected =
    context.select_route_segment_indices(route, 70.0, 0.0, 0.0, NUM_SEGMENTS_IN_ROUTE);
  const std::vector<int64_t> expected{3, 4};
  EXPECT_EQ(selected, expected);
}

}  // namespace autoware::ml_planner::test
