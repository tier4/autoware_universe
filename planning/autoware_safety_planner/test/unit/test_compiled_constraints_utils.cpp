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

#include "trajectory_planner/frenet_sampling_based_planner/compiled_constraints_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::safety_planner
{

TEST(CompiledConstraintsUtils, InterpolateBoundaryL)
{
  const std::vector<SlPoint> polyline{{0.0, 1.0}, {10.0, 3.0}, {20.0, 3.0}};
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, -5.0), 1.0);  // clamped before the start
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 5.0), 2.0);
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 15.0), 3.0);
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 25.0), 3.0);  // clamped after the end
}

TEST(CompiledConstraintsUtils, LateralBoundExtremeL)
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

TEST(CompiledConstraintsUtils, ViolatesLateralBound)
{
  LateralBoundEntry bound;
  bound.polyline = {{0.0, 2.0}, {20.0, 2.0}};
  bound.forbidden_side = Side::LEFT;

  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{5.0, 10.0, -1.0, 1.9}));
  EXPECT_TRUE(violates_lateral_bound(bound, SlBox{5.0, 10.0, -1.0, 2.1}));
  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{30.0, 35.0, -1.0, 5.0}));  // beyond the polyline

  bound.forbidden_side = Side::RIGHT;
  EXPECT_TRUE(violates_lateral_bound(bound, SlBox{5.0, 10.0, 1.9, 3.0}));
  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{5.0, 10.0, 2.1, 3.0}));
}

TEST(CompiledConstraintsUtils, ViolatesStopBar)
{
  StopBarEntry stop_bar;
  stop_bar.s_stop = 50.0;
  stop_bar.time = TimeWindow{2.0, 4.0};

  EXPECT_TRUE(violates_stop_bar(stop_bar, SlBox{45.0, 50.5, -1.0, 1.0}, 2.5, 3.0));
  EXPECT_FALSE(violates_stop_bar(stop_bar, SlBox{45.0, 49.5, -1.0, 1.0}, 2.5, 3.0));
  EXPECT_FALSE(violates_stop_bar(stop_bar, SlBox{45.0, 50.5, -1.0, 1.0}, 5.0, 6.0));  // inactive
}

}  // namespace autoware::safety_planner
