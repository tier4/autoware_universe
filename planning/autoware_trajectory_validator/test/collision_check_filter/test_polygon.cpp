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

// #include "autoware/trajectory_validator/filters/safety/collision_check_filter.hpp"
#include "../../src/filters/safety/collision_check_filter.cpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::trajectory_validator::plugin::polygon
{

class PolygonCollisionTest : public ::testing::Test
{
protected:
  Polygon2d create_rect_poly(double min_x, double min_y, double max_x, double max_y)
  {
    Polygon2d poly;
    poly.outer().push_back(Point2d(min_x, min_y));
    poly.outer().push_back(Point2d(max_x, min_y));
    poly.outer().push_back(Point2d(max_x, max_y));
    poly.outer().push_back(Point2d(min_x, max_y));
    poly.outer().push_back(Point2d(min_x, min_y));
    boost::geometry::correct(poly);
    return poly;
  }
};

TEST_F(PolygonCollisionTest, ComputeOverallEnvelope)
{
  FootprintTrajectory trajectory;
  trajectory.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));
  trajectory.push_back(create_rect_poly(2.0, 2.0, 3.0, 3.0));

  Box2d envelope = compute_overall_envelope(trajectory);

  EXPECT_DOUBLE_EQ(envelope.min_corner().x(), 0.0);
  EXPECT_DOUBLE_EQ(envelope.min_corner().y(), 0.0);
  EXPECT_DOUBLE_EQ(envelope.max_corner().x(), 3.0);
  EXPECT_DOUBLE_EQ(envelope.max_corner().y(), 3.0);
}

TEST_F(PolygonCollisionTest, ComputeOverallConvexHull)
{
  FootprintTrajectory trajectory;
  trajectory.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));
  trajectory.push_back(create_rect_poly(2.0, 2.0, 3.0, 3.0));

  Polygon2d hull = compute_overall_convex_hull(trajectory);

  double area = boost::geometry::area(hull);
  EXPECT_DOUBLE_EQ(area, 5.0);
}

TEST_F(PolygonCollisionTest, CheckPathPolygonConvexCollision)
{
  FootprintTrajectory traj1_disjoint;
  traj1_disjoint.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));

  FootprintTrajectory traj2_disjoint;
  traj2_disjoint.push_back(create_rect_poly(5.0, 5.0, 6.0, 6.0));

  EXPECT_FALSE(check_path_polygon_convex_collision(traj1_disjoint, traj2_disjoint));

  FootprintTrajectory traj1_L_shape;
  traj1_L_shape.push_back(create_rect_poly(0.0, 0.0, 1.0, 4.0));  // 縦長
  traj1_L_shape.push_back(create_rect_poly(0.0, 0.0, 4.0, 1.0));  // 横長

  FootprintTrajectory traj2_L_shape;
  traj2_L_shape.push_back(create_rect_poly(3.0, 3.0, 3.0, 7.0));
  traj2_L_shape.push_back(create_rect_poly(3.0, 3.0, 7.0, 3.0));
  EXPECT_FALSE(check_path_polygon_convex_collision(traj1_L_shape, traj2_L_shape));

  FootprintTrajectory traj1_collide;
  traj1_collide.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));
  traj1_collide.push_back(create_rect_poly(10.0, 0.0, 11.0, 1.0));

  FootprintTrajectory traj2_collide;
  traj2_collide.push_back(create_rect_poly(5.0, -5.0, 6.0, -4.0));
  traj2_collide.push_back(create_rect_poly(5.0, 5.0, 6.0, 6.0));

  EXPECT_TRUE(check_path_polygon_convex_collision(traj1_collide, traj2_collide));
}

}  // namespace autoware::trajectory_validator::plugin::polygon
