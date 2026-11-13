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

#include "autoware/trajectory_modifier/time_sequence_raw/road_border_avoidance.hpp"

#include <Eigen/Geometry>
#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <cmath>
#include <limits>
#include <vector>

namespace autoware::trajectory_modifier::test
{
using autoware::trajectory_modifier::time_sequence_raw::RoadBorder;
using autoware::trajectory_modifier::time_sequence_raw::RoadBorderAvoidance;
using autoware::trajectory_modifier::time_sequence_raw::RoadBorderAvoidanceParams;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_geometry::LineString2d;

class RoadBorderAvoidanceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    vehicle_info_ = autoware::vehicle_info_utils::createVehicleInfo(
      0.3, 0.2, 2.75, 1.6, 1.0, 1.0, 0.1, 0.1, 2.0, 0.7);
    params_.enable = true;
    params_.footprint_margin_m = 0.2;

    ego_pose_.orientation.w = 1.0;
  }

  static Trajectory make_straight_trajectory(const double y, const size_t num_points = 20)
  {
    Trajectory trajectory;
    trajectory.header.frame_id = "map";
    for (size_t i = 1; i <= num_points; ++i) {
      TrajectoryPoint point;
      point.pose.position.x = static_cast<double>(i);
      point.pose.position.y = y;
      point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      trajectory.points.push_back(point);
    }
    return trajectory;
  }

  static LineString2d make_parallel_border(const double y)
  {
    LineString2d border;
    border.emplace_back(-10.0, y);
    border.emplace_back(50.0, y);
    return border;
  }

  RoadBorderAvoidanceParams params_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  geometry_msgs::msg::Pose ego_pose_;
};

TEST_F(RoadBorderAvoidanceTest, NoOpWhenClear)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  avoidance.set_road_borders({make_parallel_border(5.0)});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);

  EXPECT_EQ(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
  ASSERT_EQ(result.trajectory.points.size(), raw.points.size());
  for (size_t i = 0; i < raw.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(result.trajectory.points[i].pose.position.x, raw.points[i].pose.position.x);
    EXPECT_DOUBLE_EQ(result.trajectory.points[i].pose.position.y, raw.points[i].pose.position.y);
  }
}

TEST_F(RoadBorderAvoidanceTest, ShiftsAwayFromLeftBorder)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  avoidance.set_road_borders({make_parallel_border(1.0)});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);

  EXPECT_GT(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(point.pose.position.y, 0.0);
  }

  const auto second = avoidance.adjust(result.trajectory, ego_pose_);
  EXPECT_EQ(second.num_shifted_points, 0U);
  EXPECT_EQ(second.num_unresolved_points, 0U);
}

TEST_F(RoadBorderAvoidanceTest, ShiftsAwayFromRightBorder)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  avoidance.set_road_borders({make_parallel_border(-1.0)});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);

  EXPECT_GT(result.num_shifted_points, 0U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_GT(point.pose.position.y, 0.0);
  }
}

TEST_F(RoadBorderAvoidanceTest, CapsShiftAndReportsUnresolved)
{
  params_.max_lateral_shift_m = 0.3;
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  avoidance.set_road_borders({make_parallel_border(0.0)});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);

  EXPECT_EQ(result.num_shifted_points, 0U);
  EXPECT_GT(result.num_unresolved_points, 0U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_NEAR(std::abs(point.pose.position.y), 0.3, 1e-9);
  }
}

TEST_F(RoadBorderAvoidanceTest, PropagatesShiftToSubsequentPoints)
{
  LineString2d border;
  border.emplace_back(-10.0, 1.0);
  border.emplace_back(5.0, 1.0);

  {
    params_.propagate_shift = true;
    RoadBorderAvoidance avoidance(params_, vehicle_info_);
    avoidance.set_road_borders({border});
    const auto result = avoidance.adjust(make_straight_trajectory(0.0), ego_pose_);
    EXPECT_GT(result.num_shifted_points, 0U);
    EXPECT_LT(result.trajectory.points.back().pose.position.y, 0.0);
  }
  {
    params_.propagate_shift = false;
    RoadBorderAvoidance avoidance(params_, vehicle_info_);
    avoidance.set_road_borders({border});
    const auto result = avoidance.adjust(make_straight_trajectory(0.0), ego_pose_);
    EXPECT_GT(result.num_shifted_points, 0U);
    EXPECT_DOUBLE_EQ(result.trajectory.points.back().pose.position.y, 0.0);
  }
}

TEST_F(RoadBorderAvoidanceTest, BisectsAfterThreeLinearSteps)
{
  // Half-width + margin = 1.1 m. Border at y=0.73 needs ~0.37 m of shift, so the first
  // three 0.1 m probes still collide and bisection must land between 0.3 m and 0.4 m.
  params_.shift_step_m = 0.1;
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  avoidance.set_road_borders({make_parallel_border(0.73)});

  const auto result = avoidance.adjust(make_straight_trajectory(0.0), ego_pose_);
  ASSERT_GT(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(point.pose.position.y, -0.3);
    EXPECT_GT(point.pose.position.y, -0.4);
  }
}

TEST_F(RoadBorderAvoidanceTest, IgnoresBordersOutsideSearchRadius)
{
  params_.search_radius_m = 10.0;
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  LineString2d far_border;
  far_border.emplace_back(100.0, 1.0);
  far_border.emplace_back(150.0, 1.0);
  avoidance.set_road_borders({far_border});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_EQ(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
}

TEST_F(RoadBorderAvoidanceTest, IgnoresBordersOnAnotherLevel)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  // Overpass borders crossing the path perpendicularly, 6 m above the ego road.
  std::vector<RoadBorder> borders;
  for (const double x : {9.0, 11.0}) {
    RoadBorder border;
    border.line.emplace_back(x, -20.0);
    border.line.emplace_back(x, 20.0);
    border.z = {6.0, 6.0};
    borders.push_back(border);
  }
  avoidance.set_road_borders(borders);

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_EQ(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
}

TEST_F(RoadBorderAvoidanceTest, FollowsBorderHeightOnSlope)
{
  params_.propagate_shift = false;  // every point must detect the border by itself
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  // Ramp climbing 10 % while the trajectory keeps the ego height (as raw model output does).
  // The left border at y = 1.0 rises 6 m over 60 m but is still on the ego road.
  RoadBorder border;
  border.line = make_parallel_border(1.0);
  border.z = {-1.0, 5.0};
  avoidance.set_road_borders(std::vector<RoadBorder>{border});

  const auto raw = make_straight_trajectory(0.0, 40);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_EQ(result.num_unresolved_points, 0U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(point.pose.position.y, 0.0);
  }
}

TEST_F(RoadBorderAvoidanceTest, IgnoresOverpassChunkOfLongMixedLevelLinestring)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  // Long linestring (> 47.5 m chunk limit) starting beside ego at z = 0, climbing onto a 6 m
  // overpass far ahead, and crossing over x = 10 at z = 6 m. Ground chunks stay accepted while
  // the overpass crossing chunk (whose vertices are all at z = 6 m) is rejected.
  RoadBorder border;
  border.line.emplace_back(-10.0, 1.5);
  border.line.emplace_back(45.0, 1.5);
  border.line.emplace_back(50.0, 1.5);
  border.line.emplace_back(50.0, 20.0);
  border.line.emplace_back(10.0, 20.0);
  border.line.emplace_back(10.0, -20.0);
  border.z = {0.0, 0.0, 6.0, 6.0, 6.0, 6.0};
  avoidance.set_road_borders(std::vector<RoadBorder>{border});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_EQ(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
}

TEST_F(RoadBorderAvoidanceTest, KeepsShortMixedLevelLinestringChunkStartingAtEgoLevel)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  // Short mixed-level linestring (<= 47.5 m, one 20-point chunk): its ego-closest vertex is on the
  // ego level at z = 0, so #3659's chunk-level classification retains the entire chunk.
  RoadBorder border;
  border.line.emplace_back(0.0, 1.5);
  border.line.emplace_back(5.0, 1.5);
  border.line.emplace_back(10.0, 1.5);
  border.line.emplace_back(10.0, -20.0);
  border.z = {0.0, 0.0, 6.0, 6.0};
  avoidance.set_road_borders(std::vector<RoadBorder>{border});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_TRUE(result.modified());
}

TEST_F(RoadBorderAvoidanceTest, RespectsThreeMeterBoundaryNearEgo)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  // #3659 keeps elements within 4.0 + 0.05 * distance in the ego frame, so a crossing border 3 m
  // above ego is still treated as same-level (whereas 6 m in IgnoresBordersOnAnotherLevel is not).
  RoadBorder border;
  border.line.emplace_back(10.0, -20.0);
  border.line.emplace_back(10.0, 20.0);
  border.z = {3.0, 3.0};
  avoidance.set_road_borders(std::vector<RoadBorder>{border});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_TRUE(result.modified());
}

TEST_F(RoadBorderAvoidanceTest, TransformsChunkIntoPitchedEgoFrame)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  // Uphill border starting 30 m ahead at 20 % grade (tan(0.20) ~ 0.2027 -> z ~ 6.08 m at x = 30).
  // Flat-ego check at (0,0,0) rejects it (6.08 > 4.0 + 0.05 * 30 = 5.50), while a nose-up pitched
  // ego (pitch = -0.20 rad about +Y) puts the border at z_ego ~ 0 and shifts away from it.
  constexpr double pitch_rad = -0.20;
  const double slope = std::tan(-pitch_rad);
  RoadBorder border;
  border.line.emplace_back(30.0, 1.0);
  border.line.emplace_back(45.0, 1.0);
  border.z = {30.0 * slope, 45.0 * slope};
  avoidance.set_road_borders(std::vector<RoadBorder>{border});

  const auto raw = make_straight_trajectory(0.0, 45);
  const auto flat_result = avoidance.adjust(raw, ego_pose_);
  EXPECT_EQ(flat_result.num_shifted_points, 0U);
  EXPECT_EQ(flat_result.num_unresolved_points, 0U);

  geometry_msgs::msg::Pose pitched_ego = ego_pose_;
  const Eigen::Quaterniond q_pitch(Eigen::AngleAxisd(pitch_rad, Eigen::Vector3d::UnitY()));
  pitched_ego.orientation.w = q_pitch.w();
  pitched_ego.orientation.x = q_pitch.x();
  pitched_ego.orientation.y = q_pitch.y();
  pitched_ego.orientation.z = q_pitch.z();

  const auto pitched_result = avoidance.adjust(raw, pitched_ego);
  EXPECT_GT(pitched_result.num_shifted_points, 0U);
  EXPECT_EQ(pitched_result.num_unresolved_points, 0U);
  for (size_t i = 30; i < pitched_result.trajectory.points.size(); ++i) {
    EXPECT_LT(pitched_result.trajectory.points[i].pose.position.y, 0.0);
  }
}

TEST_F(RoadBorderAvoidanceTest, FilteringIsInvariantToTrajectoryResampling)
{
  params_.propagate_shift = false;
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  RoadBorder crossing_ground;
  crossing_ground.line.emplace_back(12.0, -20.0);
  crossing_ground.line.emplace_back(12.0, 20.0);
  crossing_ground.z = {0.0, 0.0};

  RoadBorder parallel_high;
  parallel_high.line.emplace_back(4.0, 1.0);
  parallel_high.line.emplace_back(36.0, 1.0);
  parallel_high.z = {6.0, 6.0};
  avoidance.set_road_borders(std::vector<RoadBorder>{crossing_ground, parallel_high});

  const auto make_traj_with_step = [](const double step_m) {
    Trajectory traj;
    traj.header.frame_id = "map";
    for (double x = 0.0; x <= 40.0 + 1e-6; x += step_m) {
      TrajectoryPoint point;
      point.pose.position.x = x;
      point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
      traj.points.push_back(point);
    }
    return traj;
  };

  const auto sparse_result = avoidance.adjust(make_traj_with_step(4.0), ego_pose_);
  const auto dense_result = avoidance.adjust(make_traj_with_step(0.5), ego_pose_);
  // At x = 24, only the 6 m parallel border is near the trajectory; neither sampling avoids it.
  EXPECT_DOUBLE_EQ(sparse_result.trajectory.points.at(6).pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(dense_result.trajectory.points.at(48).pose.position.y, 0.0);
  // Both see the ground crossing border identically.
  EXPECT_GT(sparse_result.num_unresolved_points, 0U);
  EXPECT_GT(dense_result.num_unresolved_points, 0U);
}

TEST_F(RoadBorderAvoidanceTest, FilteringIsInvariantToMapVertexSubdivision)
{
  params_.propagate_shift = false;
  RoadBorderAvoidance coarse_avoidance(params_, vehicle_info_);
  RoadBorderAvoidance dense_avoidance(params_, vehicle_info_);

  RoadBorder coarse;
  coarse.line.emplace_back(0.0, 1.0);
  coarse.line.emplace_back(40.0, 1.0);
  coarse.z = {0.0, 6.0};
  coarse_avoidance.set_road_borders(std::vector<RoadBorder>{coarse});

  RoadBorder dense;
  for (int i = 0; i <= 40; ++i) {
    dense.line.emplace_back(static_cast<double>(i), 1.0);
    dense.z.push_back(6.0 * static_cast<double>(i) / 40.0);
  }
  dense_avoidance.set_road_borders(std::vector<RoadBorder>{dense});

  const auto raw = make_straight_trajectory(0.0, 40);
  const auto coarse_result = coarse_avoidance.adjust(raw, ego_pose_);
  const auto dense_result = dense_avoidance.adjust(raw, ego_pose_);
  ASSERT_EQ(coarse_result.trajectory.points.size(), dense_result.trajectory.points.size());
  EXPECT_EQ(coarse_result.num_shifted_points, dense_result.num_shifted_points);
  for (size_t i = 0; i < raw.points.size(); ++i) {
    EXPECT_NEAR(
      coarse_result.trajectory.points[i].pose.position.y,
      dense_result.trajectory.points[i].pose.position.y, 1e-6);
  }
}

TEST_F(RoadBorderAvoidanceTest, RejectsNonFiniteBorderElevations)
{
  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  const double nan_z = std::numeric_limits<double>::quiet_NaN();

  // Closest vertex at (0, 1) has NaN elevation while far endpoints sit at 0 m -> must still reject.
  RoadBorder mixed_nan;
  mixed_nan.line.emplace_back(-20.0, 1.0);
  mixed_nan.line.emplace_back(0.0, 1.0);
  mixed_nan.line.emplace_back(20.0, 1.0);
  mixed_nan.z = {0.0, nan_z, 0.0};

  RoadBorder all_nan;
  all_nan.line = make_parallel_border(1.0);
  all_nan.z = {nan_z, nan_z};

  avoidance.set_road_borders(std::vector<RoadBorder>{mixed_nan, all_nan});

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_EQ(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
}

TEST_F(RoadBorderAvoidanceTest, SetMapPreservesHeightsAndIgnoresOverpass)
{
  lanelet::LaneletMap lanelet_map;
  lanelet::LineString3d overpass_border(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 10.0, -20.0, 6.0),
                              lanelet::Point3d(lanelet::utils::getId(), 10.0, 20.0, 6.0)});
  overpass_border.setAttribute("type", "road_border");
  lanelet_map.add(overpass_border);

  lanelet::LineString3d ground_border(
    lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), -10.0, 1.0, 0.0),
                              lanelet::Point3d(lanelet::utils::getId(), 50.0, 1.0, 0.0)});
  ground_border.setAttribute("type", "road_border");
  lanelet_map.add(ground_border);

  RoadBorderAvoidance avoidance(params_, vehicle_info_);
  avoidance.set_map(lanelet_map);

  const auto raw = make_straight_trajectory(0.0);
  const auto result = avoidance.adjust(raw, ego_pose_);
  EXPECT_GT(result.num_shifted_points, 0U);
  EXPECT_EQ(result.num_unresolved_points, 0U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_LT(point.pose.position.y, 0.0);
  }
}

}  // namespace autoware::trajectory_modifier::test
