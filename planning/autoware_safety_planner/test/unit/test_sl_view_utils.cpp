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

#include "utils/sl_view_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::safety_planner
{

// Straight path along +x from (0, 0) to (100, 0), so s == x and l == y
PathPointTrajectory make_straight_path()
{
  std::vector<PathPointWithLaneId> points;
  for (int i = 0; i <= 10; ++i) {
    auto & p = points.emplace_back();
    p.point.pose.position.x = 10.0 * i;
    p.point.pose.orientation.w = 1.0;
  }
  return *PathPointTrajectory::Builder{}.build(points);
}

TEST(SlViewUtils, LateralOffsetAndWorldPoseRoundTrip)
{
  const auto path = make_straight_path();
  EXPECT_NEAR(lateral_offset_at(path, 30.0, Point2d{30.0, 2.5}), 2.5, 1e-6);
  EXPECT_NEAR(lateral_offset_at(path, 30.0, Point2d{30.0, -1.0}), -1.0, 1e-6);

  const auto pose = to_world_pose(path, 40.0, 1.5);
  EXPECT_NEAR(pose.position.x(), 40.0, 1e-6);
  EXPECT_NEAR(pose.position.y(), 1.5, 1e-6);
  EXPECT_NEAR(pose.yaw, 0.0, 1e-6);
}

TEST(SlViewUtils, FootprintSlBox)
{
  VehicleInfo vehicle_info;
  vehicle_info.min_longitudinal_offset_m = -1.0;
  vehicle_info.max_longitudinal_offset_m = 4.0;
  vehicle_info.min_lateral_offset_m = -0.9;
  vehicle_info.max_lateral_offset_m = 0.9;

  const auto box = footprint_sl_box(vehicle_info, 10.0, 0.5);
  EXPECT_DOUBLE_EQ(box.s_min, 9.0);
  EXPECT_DOUBLE_EQ(box.s_max, 14.0);
  EXPECT_DOUBLE_EQ(box.l_min, -0.4);
  EXPECT_DOUBLE_EQ(box.l_max, 1.4);

  const auto swept = footprint_sl_box(vehicle_info, SlBox{10.0, 20.0, -1.0, 1.0});
  EXPECT_DOUBLE_EQ(swept.s_min, 9.0);
  EXPECT_DOUBLE_EQ(swept.s_max, 24.0);
  EXPECT_DOUBLE_EQ(swept.l_min, -1.9);
  EXPECT_DOUBLE_EQ(swept.l_max, 1.9);
}

TEST(SlViewUtils, InterpolateBoundaryL)
{
  const std::vector<SlPoint> polyline{{0.0, 1.0}, {10.0, 3.0}, {20.0, 3.0}};
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, -5.0), 1.0);  // clamped before the start
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 5.0), 2.0);
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 15.0), 3.0);
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 25.0), 3.0);  // clamped after the end
}

TEST(SlViewUtils, LateralBoundExtremeL)
{
  LateralBoundEntry bound;
  bound.polyline = {{0.0, 2.0}, {10.0, 1.0}, {20.0, 2.0}};

  double extreme_l = 0.0;
  bound.forbidden_side = Side::LEFT;
  ASSERT_TRUE(lateral_bound_extreme_l(bound, 5.0, 15.0, extreme_l));
  EXPECT_DOUBLE_EQ(extreme_l, 1.0);  // min(l) including the interior vertex

  bound.forbidden_side = Side::RIGHT;
  ASSERT_TRUE(lateral_bound_extreme_l(bound, 5.0, 15.0, extreme_l));
  EXPECT_DOUBLE_EQ(extreme_l, 1.5);  // max(l) is at the interval ends

  EXPECT_FALSE(lateral_bound_extreme_l(bound, 30.0, 40.0, extreme_l));  // no overlap in s
}

TEST(SlViewUtils, ViolatesLateralBound)
{
  LateralBoundEntry bound;
  bound.polyline = {{0.0, 2.0}, {20.0, 2.0}};
  bound.forbidden_side = Side::LEFT;
  bound.margin = 0.5;

  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{5.0, 10.0, -1.0, 1.4}));
  EXPECT_TRUE(violates_lateral_bound(bound, SlBox{5.0, 10.0, -1.0, 1.6}));    // inside the margin
  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{30.0, 35.0, -1.0, 5.0}));  // beyond the polyline

  bound.forbidden_side = Side::RIGHT;
  EXPECT_TRUE(violates_lateral_bound(bound, SlBox{5.0, 10.0, 2.4, 3.0}));
  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{5.0, 10.0, 2.6, 3.0}));
}

TEST(SlViewUtils, ViolatesStopBar)
{
  StopBarEntry stop_bar;
  stop_bar.s_stop = 50.0;
  stop_bar.margin = 1.0;
  stop_bar.time = TimeWindow{2.0, 4.0};

  EXPECT_TRUE(violates_stop_bar(stop_bar, SlBox{45.0, 49.5, -1.0, 1.0}, 2.5, 3.0));
  EXPECT_FALSE(violates_stop_bar(stop_bar, SlBox{45.0, 48.5, -1.0, 1.0}, 2.5, 3.0));
  EXPECT_FALSE(violates_stop_bar(stop_bar, SlBox{45.0, 49.5, -1.0, 1.0}, 5.0, 6.0));  // inactive
}

}  // namespace autoware::safety_planner
