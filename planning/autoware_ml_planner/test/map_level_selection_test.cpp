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

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace autoware::ml_planner::test
{
using autoware::ml_planner::preprocess::LaneSegmentContext;

namespace
{
// A straight lanelet whose height goes linearly from `start_z` to `end_z`.
lanelet::Lanelet make_lanelet(
  const double start_x, const double start_y, const double start_z, const double end_x,
  const double end_y, const double end_z)
{
  constexpr double half_width = 1.75;
  const double length = std::hypot(end_x - start_x, end_y - start_y);
  const double normal_x = -(end_y - start_y) / length * half_width;
  const double normal_y = (end_x - start_x) / length * half_width;

  const auto make_bound = [&](const double offset_x, const double offset_y) {
    return lanelet::LineString3d(
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), start_x + offset_x, start_y + offset_y, start_z),
       lanelet::Point3d(lanelet::utils::getId(), end_x + offset_x, end_y + offset_y, end_z)});
  };

  lanelet::Lanelet lanelet(
    lanelet::utils::getId(), make_bound(normal_x, normal_y), make_bound(-normal_x, -normal_y));
  lanelet.setAttribute("subtype", "road");
  return lanelet;
}

lanelet::LineString3d make_road_border(
  const double start_x, const double start_y, const double end_x, const double end_y,
  const double z)
{
  lanelet::LineString3d border(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), start_x, start_y, z),
                              lanelet::Point3d(lanelet::utils::getId(), end_x, end_y, z)});
  border.setAttribute("type", "road_border");
  return border;
}

// The transform from the map frame to the frame of an ego at `(x, y, z)` heading east, level.
Eigen::Matrix4d map_to_ego(const double x, const double y, const double z)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform(0, 3) = -x;
  transform(1, 3) = -y;
  transform(2, 3) = -z;
  return transform;
}
}  // namespace

// `ground_` runs east at height 0. `overpass_` crosses over it northward at (50, 0), 6 m up.
// `uphill_` continues `ground_` east of x = 100 and climbs at 8%, reaching 8 m at x = 200.
class MapLevelSelectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ground_ = make_lanelet(0.0, 0.0, 0.0, 100.0, 0.0, 0.0);
    overpass_ = make_lanelet(50.0, -50.0, 6.0, 50.0, 50.0, 6.0);
    uphill_ = make_lanelet(100.0, 0.0, 0.0, 200.0, 0.0, 8.0);

    auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
    lanelet_map->add(ground_);
    lanelet_map->add(overpass_);
    lanelet_map->add(uphill_);
    lanelet_map->add(make_road_border(0.0, -3.0, 100.0, -3.0, 0.0));
    lanelet_map->add(make_road_border(47.0, -50.0, 47.0, 50.0, 6.0));
    context_ = std::make_unique<LaneSegmentContext>(lanelet_map);
  }

  std::vector<lanelet::Id> selected_lanelet_ids(
    const double x, const double y, const double z) const
  {
    const auto indices =
      context_->select_lane_segment_indices(map_to_ego(x, y, z), x, y, NUM_SEGMENTS_IN_LANE);
    std::vector<lanelet::Id> ids;
    for (const auto & [id, index] : context_->get_lanelet_id_to_array_index()) {
      if (std::find(indices.begin(), indices.end(), static_cast<int64_t>(index)) != indices.end()) {
        ids.push_back(id);
      }
    }
    return ids;
  }

  // The road border rows of the input whose first point is at `(ego_frame_x, ego_frame_y)` along
  // the axis given by `along_x` (x when true, y when false).
  size_t road_border_rows_at(
    const double x, const double y, const double z, const bool along_x, const double value) const
  {
    const auto tensor = context_->create_road_border_tensor(map_to_ego(x, y, z), x, y);
    size_t count = 0;
    for (size_t i = 0; i < tensor.shape(0); ++i) {
      const bool valid = tensor(i, 0, 0) != 0.0f || tensor(i, 0, 1) != 0.0f;
      const float coordinate = along_x ? tensor(i, 0, 0) : tensor(i, 0, 1);
      if (valid && std::abs(coordinate - value) < 1e-3) {
        ++count;
      }
    }
    return count;
  }

  static bool contains(const std::vector<lanelet::Id> & ids, const lanelet::Lanelet & lanelet)
  {
    return std::find(ids.begin(), ids.end(), lanelet.id()) != ids.end();
  }

  lanelet::Lanelet ground_;
  lanelet::Lanelet overpass_;
  lanelet::Lanelet uphill_;
  std::unique_ptr<LaneSegmentContext> context_;
};

TEST_F(MapLevelSelectionTest, LeavesOutTheOverpassUnderTheEgo)
{
  const auto ids = selected_lanelet_ids(45.0, 0.0, 0.0);
  EXPECT_TRUE(contains(ids, ground_));
  EXPECT_FALSE(contains(ids, overpass_));
}

TEST_F(MapLevelSelectionTest, LeavesOutTheRoadBelowTheEgoOnTheOverpass)
{
  const auto ids = selected_lanelet_ids(50.0, -5.0, 6.0);
  EXPECT_TRUE(contains(ids, overpass_));
  EXPECT_FALSE(contains(ids, ground_));
}

TEST_F(MapLevelSelectionTest, KeepsTheRoadAheadThatClimbs)
{
  // At x = 100 the climb starts at the ego's height; its far end is 8 m up but the closest point
  // decides.
  EXPECT_TRUE(contains(selected_lanelet_ids(90.0, 0.0, 0.0), uphill_));
}

TEST_F(MapLevelSelectionTest, LeavesOutTheRoadBorderOnTheOtherLevel)
{
  // From under the overpass: the ground border runs at y = -3, the overpass border at x = 2.
  EXPECT_GT(road_border_rows_at(45.0, 0.0, 0.0, false, -3.0), 0U);
  EXPECT_EQ(road_border_rows_at(45.0, 0.0, 0.0, true, 2.0), 0U);
  // From the overpass: the ground border runs at y = 2, the overpass border at x = -3.
  EXPECT_EQ(road_border_rows_at(50.0, -5.0, 6.0, false, 2.0), 0U);
  EXPECT_GT(road_border_rows_at(50.0, -5.0, 6.0, true, -3.0), 0U);
}

}  // namespace autoware::ml_planner::test
