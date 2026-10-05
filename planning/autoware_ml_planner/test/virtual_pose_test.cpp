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

#include "autoware/ml_planner/utils/virtual_pose.hpp"

#include <Eigen/Dense>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>

#include <vector>

namespace autoware::ml_planner::test
{
namespace
{
Eigen::Matrix4d pose_at_x(const double x)
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 3) = x;
  return pose;
}

utils::VirtualPoseParams time_based_params()
{
  utils::VirtualPoseParams params{};
  params.time_based_enable = true;
  params.time_based_enter_speed_mps = 0.5;
  params.time_based_exit_speed_mps = 0.7;
  return params;
}
}  // namespace

TEST(VirtualPoseTest, PointAtElapsedTimeInterpolatesBetweenVertices)
{
  // Planning start at x = 0, then 0.1 m every 0.1 s: 1 m/s.
  const std::vector<Eigen::Matrix4d> prediction{pose_at_x(0.1), pose_at_x(0.2), pose_at_x(0.3)};
  const std::vector<double> times{0.1, 0.2, 0.3};

  const auto at_start = utils::point_at_elapsed_time(pose_at_x(0.0), prediction, times, 0.05);
  ASSERT_TRUE(at_start.has_value());
  EXPECT_NEAR(at_start->position.x(), 0.05, 1e-9);
  EXPECT_NEAR(at_start->speed_mps, 1.0, 1e-9);

  const auto later = utils::point_at_elapsed_time(pose_at_x(0.0), prediction, times, 0.25);
  ASSERT_TRUE(later.has_value());
  EXPECT_NEAR(later->position.x(), 0.25, 1e-9);

  // Past the end: clamped onto the last pose.
  const auto past_end = utils::point_at_elapsed_time(pose_at_x(0.0), prediction, times, 1.0);
  ASSERT_TRUE(past_end.has_value());
  EXPECT_NEAR(past_end->position.x(), 0.3, 1e-9);

  EXPECT_FALSE(utils::point_at_elapsed_time(pose_at_x(0.0), {}, {}, 0.1).has_value());
}

TEST(VirtualPoseTest, PointAtElapsedTimeStaysOnAStop)
{
  // A stopped trajectory repeats its pose: the point does not move and its speed is zero.
  const std::vector<Eigen::Matrix4d> prediction{pose_at_x(0.0), pose_at_x(0.0)};
  const auto point = utils::point_at_elapsed_time(pose_at_x(0.0), prediction, {0.1, 0.2}, 0.15);
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->position.x(), 0.0, 1e-9);
  EXPECT_NEAR(point->speed_mps, 0.0, 1e-9);
}

TEST(VirtualPoseTest, TimeBasedModeNeedsEngagement)
{
  const auto params = time_based_params();
  EXPECT_FALSE(utils::update_time_based_mode(false, false, 0.0, params));
  EXPECT_FALSE(utils::update_time_based_mode(true, false, 0.0, params));
  EXPECT_TRUE(utils::update_time_based_mode(false, true, 0.0, params));

  auto disabled = params;
  disabled.time_based_enable = false;
  EXPECT_FALSE(utils::update_time_based_mode(true, true, 0.0, disabled));
}

TEST(VirtualPoseTest, TimeBasedModeHysteresisDoesNotChatter)
{
  const auto params = time_based_params();
  // A speed hovering around the enter speed keeps whatever state it is in.
  bool active = true;
  for (const double speed : {0.45, 0.55, 0.48, 0.62, 0.51, 0.69}) {
    active = utils::update_time_based_mode(active, true, speed, params);
    EXPECT_TRUE(active) << speed;
  }
  active = utils::update_time_based_mode(active, true, 0.71, params);
  EXPECT_FALSE(active);
  for (const double speed : {0.69, 0.55, 0.62, 0.51, 0.65}) {
    active = utils::update_time_based_mode(active, true, speed, params);
    EXPECT_FALSE(active) << speed;
  }
  EXPECT_TRUE(utils::update_time_based_mode(active, true, 0.49, params));
}

TEST(VirtualPoseTest, ResetLimitsAreSplitAlongAndAcrossTheVirtualHeading)
{
  // Previous trajectory along x, every 0.5 m from 0 to 10 m.
  std::vector<Eigen::Matrix4d> polyline;
  for (int i = 0; i <= 20; ++i) {
    polyline.push_back(pose_at_x(0.5 * i));
  }
  utils::VirtualPoseParams params{};
  params.enable = true;
  params.max_longitudinal_error_m = 0.5;
  params.max_lateral_error_m = 0.3;
  params.max_yaw_error_deg = 5.0;
  params.max_search_segment_count = 50;
  params.yaw_fit_half_window_m = 1.0;
  params.yaw_fit_min_length_m = 0.2;
  params.history_prefix_count = 0;
  params.reference = "raw";
  const auto vehicle_at = [](const double x, const double y) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.w = 1.0;
    return pose;
  };
  const auto query_at = [](const double x) {
    geometry_msgs::msg::Point point;
    point.x = x;
    return point;
  };

  // Virtual pose 0.4 m ahead of the vehicle (time-based query): within the longitudinal limit.
  EXPECT_FALSE(
    utils::compute_virtual_pose(vehicle_at(5.0, 0.0), query_at(5.4), polyline, 0, params).reset);
  // 0.6 m ahead: beyond it.
  EXPECT_TRUE(
    utils::compute_virtual_pose(vehicle_at(5.0, 0.0), query_at(5.6), polyline, 0, params).reset);
  // 0.25 m beside the trajectory: within the lateral limit; 0.35 m: beyond it.
  EXPECT_FALSE(
    utils::compute_virtual_pose(vehicle_at(5.0, 0.25), query_at(5.0), polyline, 0, params).reset);
  EXPECT_TRUE(
    utils::compute_virtual_pose(vehicle_at(5.0, 0.35), query_at(5.0), polyline, 0, params).reset);
}

}  // namespace autoware::ml_planner::test
