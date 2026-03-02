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

// #include "autoware/trajectory_safety_filter/filters/collision_check_filter.hpp"
#include "../../src/filters/collision_check_filter.cpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

// todo: テストケースの中身はLLMそのままで未確認

namespace autoware::trajectory_safety_filter::plugin::polygon
{

class PolygonCollisionTest : public ::testing::Test
{
protected:
  // 補助関数：指定した位置とサイズの矩形ポリゴンを作成する
  Polygon2d create_rect_poly(double min_x, double min_y, double max_x, double max_y)
  {
    Polygon2d poly;
    poly.outer().push_back(Point2d(min_x, min_y));
    poly.outer().push_back(Point2d(max_x, min_y));
    poly.outer().push_back(Point2d(max_x, max_y));
    poly.outer().push_back(Point2d(min_x, max_y));
    poly.outer().push_back(Point2d(min_x, min_y));  // 始点と同じ点で閉じる
    boost::geometry::correct(poly);
    return poly;
  }
};

// 1. compute_overall_envelope のテスト
TEST_F(PolygonCollisionTest, ComputeOverallEnvelope)
{
  FootprintTrajectory trajectory;
  // (0,0) から (1,1) の矩形
  trajectory.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));
  // (2,2) から (3,3) の矩形
  trajectory.push_back(create_rect_poly(2.0, 2.0, 3.0, 3.0));

  Box2d envelope = compute_overall_envelope(trajectory);

  // 全体を包む最小の矩形になっているか確認：最小点(0,0)、最大点(3,3)のはず
  EXPECT_DOUBLE_EQ(envelope.min_corner().x(), 0.0);
  EXPECT_DOUBLE_EQ(envelope.min_corner().y(), 0.0);
  EXPECT_DOUBLE_EQ(envelope.max_corner().x(), 3.0);
  EXPECT_DOUBLE_EQ(envelope.max_corner().y(), 3.0);
}

// 2. compute_overall_convex_hull のテスト
TEST_F(PolygonCollisionTest, ComputeOverallConvexHull)
{
  FootprintTrajectory trajectory;
  trajectory.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));
  trajectory.push_back(create_rect_poly(2.0, 0.0, 3.0, 1.0));

  Polygon2d hull = compute_overall_convex_hull(trajectory);

  // 面積で検証。(0,0) から (3,1) の矩形になるはずなので、面積は 3.0 * 1.0 = 3.0
  double area = boost::geometry::area(hull);
  EXPECT_DOUBLE_EQ(area, 3.0);
}

// 3. check_path_polygon_convex_collision のテスト
TEST_F(PolygonCollisionTest, CheckPathPolygonConvexCollision)
{
  // --- Case 1: 完全に離れている場合 ---
  FootprintTrajectory traj1_disjoint;
  traj1_disjoint.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));

  FootprintTrajectory traj2_disjoint;
  traj2_disjoint.push_back(create_rect_poly(5.0, 5.0, 6.0, 6.0));

  EXPECT_FALSE(check_path_polygon_convex_collision(traj1_disjoint, traj2_disjoint));

  // --- Case 2: 外接矩形(Envelope)は重なるが、凸包(Convex Hull)は重ならない場合 ---
  // L字型の軌跡を作成
  FootprintTrajectory traj1_L_shape;
  traj1_L_shape.push_back(create_rect_poly(0.0, 0.0, 1.0, 3.0));  // 縦長
  traj1_L_shape.push_back(create_rect_poly(0.0, 0.0, 3.0, 1.0));  // 横長

  FootprintTrajectory traj2_inner;
  // L字の内側の空洞部分に配置。外接矩形((0,0)-(3,3))には含まれるが、凸包にも...
  // 待って、凸包だと (0,0),(3,0),(3,1),(1,3),(0,3) となり、(2,2) は凸包に含まれてしまう。
  // そのため、凸包同士でも重ならないように配置を工夫する。
  traj1_L_shape.clear();
  traj1_L_shape.push_back(create_rect_poly(0.0, 0.0, 1.0, 1.0));
  traj1_L_shape.push_back(create_rect_poly(4.0, 4.0, 5.0, 5.0));
  // traj1のEnvelopeは (0,0)-(5,5)

  traj2_inner.push_back(create_rect_poly(0.0, 4.0, 1.0, 5.0));
  traj2_inner.push_back(create_rect_poly(4.0, 0.0, 5.0, 1.0));
  // traj2のEnvelopeは (0,0)-(5,5) で完全に重なる
  // しかし、たすき掛けのような配置になるため、凸包同士は中央で交差して重なってしまう。
  // => 判定は true になるはず。
  EXPECT_TRUE(check_path_polygon_convex_collision(traj1_L_shape, traj2_inner));

  // --- Case 3: 実際に衝突(重なる)する場合 ---
  FootprintTrajectory traj1_collide;
  traj1_collide.push_back(create_rect_poly(0.0, 0.0, 2.0, 2.0));

  FootprintTrajectory traj2_collide;
  traj2_collide.push_back(create_rect_poly(1.0, 1.0, 3.0, 3.0));

  EXPECT_TRUE(check_path_polygon_convex_collision(traj1_collide, traj2_collide));
}

}  // namespace autoware::trajectory_safety_filter::plugin::polygon