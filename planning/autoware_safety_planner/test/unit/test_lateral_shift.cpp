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

#include "constraint.hpp"
#include "utils/reference_path_smoother.hpp"

#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace autoware::safety_planner
{

// The use the reference path following planner makes of optimize_goal_shift: a zero profile
// pushed off the centerline by a one sided bound ahead of the ego, the ends held on it
TEST(OptimizeGoalShift, LeavesTheCenterlineOnlyWhereTheBoundAsks)
{
  constexpr double ds = 1.0;
  constexpr std::size_t n = 60;
  const std::vector<double> nominal(n, 0.0);
  const std::vector<double> straight(n, 0.0);
  std::vector<double> lo(n, -INF);
  std::vector<double> hi(n, INF);
  // A curb reaching 0.4 m past the left of the footprint on the centerline over s in [25, 35]
  for (std::size_t i = 25; i <= 35; ++i) {
    hi[i] = -0.4;
  }
  const auto optimized = optimize_goal_shift(nominal, straight, ds, 0.1, lo, hi, 1.0e-3);
  ASSERT_TRUE(optimized.has_value());
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_LE((*optimized)[i], hi[i] + 1.0e-4) << i;
    EXPECT_GE((*optimized)[i], lo[i] - 1.0e-4) << i;
  }
  EXPECT_NEAR(optimized->front(), 0.0, 1.0e-6);
  EXPECT_NEAR(optimized->back(), 0.0, 1.0e-6);
  // The deviation weight brings the path back to the centerline well within the window
  EXPECT_NEAR((*optimized)[5], 0.0, 0.05);
  EXPECT_NEAR((*optimized)[55], 0.0, 0.05);
}

TEST(ShiftPathPoints, MovesTheSamplesAlongTheNormalAndTurnsTheHeadings)
{
  std::vector<PathPointWithLaneId> points;
  for (int i = 0; i <= 20; ++i) {
    PathPointWithLaneId point;
    point.point.pose.position.x = static_cast<double>(i);
    point.point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
    points.push_back(point);
  }
  const auto path = experimental::trajectory::pretty_build(points);
  ASSERT_TRUE(path.has_value());

  const std::vector<double> ss{10.0, 11.0, 12.0, 13.0};
  const std::vector<double> offsets{0.0, 0.5, 1.0, 1.0};  // positive = left = +y
  const auto shifted = shift_path_points(*path, ss, offsets);
  ASSERT_EQ(shifted.size(), 10 + ss.size());
  EXPECT_DOUBLE_EQ(shifted[9].point.pose.position.x, 9.0);  // kept as it was
  EXPECT_NEAR(shifted[11].point.pose.position.x, 11.0, 1.0e-9);
  EXPECT_NEAR(shifted[11].point.pose.position.y, 0.5, 1.0e-9);
  EXPECT_NEAR(shifted[13].point.pose.position.y, 1.0, 1.0e-9);
  // The heading at a sample is the chord between its neighbors
  EXPECT_NEAR(
    autoware_utils_geometry::get_rpy(shifted[11].point.pose).z, std::atan2(1.0, 2.0), 1.0e-9);
  EXPECT_NEAR(autoware_utils_geometry::get_rpy(shifted[13].point.pose).z, 0.0, 1.0e-9);
}

}  // namespace autoware::safety_planner
