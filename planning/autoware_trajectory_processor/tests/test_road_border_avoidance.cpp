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

#include "autoware/trajectory_processor/time_sequence_raw/road_border_avoidance.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::trajectory_processor::test
{
using autoware::trajectory_processor::time_sequence_raw::RoadBorderAvoidance;
using autoware::trajectory_processor::time_sequence_raw::RoadBorderAvoidanceParams;
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

}  // namespace autoware::trajectory_processor::test
