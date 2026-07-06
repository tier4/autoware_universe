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

#include "autoware/behavior_path_freespace_area_module/utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <cmath>
#include <vector>

namespace
{
using autoware::behavior_path_planner::freespace_area_utils::detectCuspIndices;
using autoware::behavior_path_planner::freespace_area_utils::encodeTravelDirectionOrientation;
using autoware::behavior_path_planner::freespace_area_utils::generateBoundsFromAreaPolygon;
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

PathPointWithLaneId makePoint(const double x, const double y, const double yaw, const double vel)
{
  PathPointWithLaneId p;
  p.point.pose.position.x = x;
  p.point.pose.position.y = y;
  p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
  p.point.longitudinal_velocity_mps = vel;
  return p;
}
}  // namespace

TEST(FreespaceAreaUtils, EncodeTravelDirectionMakesCuspDetectable)
{
  // forward -> reverse path as produced by convertWayPointsToPathWithLaneId:
  // all orientations are vehicle heading (+x), reverse points have negative velocity.
  PathWithLaneId path;
  path.points.push_back(makePoint(0.0, 0.0, 0.0, 1.0));
  path.points.push_back(makePoint(1.0, 0.0, 0.0, 1.0));
  path.points.push_back(makePoint(2.0, 0.0, 0.0, 1.0));
  path.points.push_back(makePoint(1.5, 0.0, 0.0, -1.0));  // reversing
  path.points.push_back(makePoint(1.0, 0.0, 0.0, -1.0));

  // Before encoding: no yaw jump, direction_change's detector would miss the cusp.
  EXPECT_TRUE(detectCuspIndices(path, 90.0).empty());

  encodeTravelDirectionOrientation(path);

  // After encoding: reverse points flipped by pi and velocities positive.
  const auto cusps = detectCuspIndices(path, 90.0);
  ASSERT_EQ(cusps.size(), 1u);
  EXPECT_EQ(cusps.front(), 3u);
  for (const auto & p : path.points) {
    EXPECT_GE(p.point.longitudinal_velocity_mps, 0.0);
  }
}

TEST(FreespaceAreaUtils, GenerateBoundsFromSquareArea)
{
  // Square area 0..10 x -5..5, path crossing it along +x on y=0.
  const lanelet::Point3d p1(1, 0.0, -5.0, 0.0);
  const lanelet::Point3d p2(2, 10.0, -5.0, 0.0);
  const lanelet::Point3d p3(3, 10.0, 5.0, 0.0);
  const lanelet::Point3d p4(4, 0.0, 5.0, 0.0);
  lanelet::LineString3d ring(10, {p1, p2, p3, p4});
  lanelet::Area area(100, {ring});
  const lanelet::ConstArea const_area = area;

  PathWithLaneId path;
  for (int i = 0; i <= 10; ++i) {
    path.points.push_back(makePoint(static_cast<double>(i), 0.0, 0.0, 1.0));
  }

  const auto [left_bound, right_bound] = generateBoundsFromAreaPolygon(path, const_area);

  ASSERT_FALSE(left_bound.empty());
  ASSERT_FALSE(right_bound.empty());

  // Left bound (relative to +x travel) must be on positive y, right on negative y.
  for (const auto & p : left_bound) {
    EXPECT_GT(p.y, 0.0) << "left bound point below the path";
  }
  for (const auto & p : right_bound) {
    EXPECT_LT(p.y, 0.0) << "right bound point above the path";
  }
}
