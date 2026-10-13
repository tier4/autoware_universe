// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <Eigen/Dense>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace autoware::ml_planner::test
{
using autoware_planning_msgs::msg::Trajectory;

TEST(PostprocessingUtilsTest, CreateTrajectoryAndMultipleTrajectories)
{
  constexpr auto prediction_shape = OUTPUT_SHAPE;
  auto batch_size = prediction_shape[0];
  auto agent_size = prediction_shape[1];
  auto rows = prediction_shape[2];
  auto cols = prediction_shape[3];
  std::vector<float> data(batch_size * agent_size * rows * cols, 0.0f);
  // Fill with some values for checking
  for (size_t i = 0; i < data.size(); ++i) data[i] = static_cast<float>(i);

  std::vector<int64_t> shape{batch_size, agent_size, rows, cols};
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  rclcpp::Time stamp(123, 0);

  auto expected_points = prediction_shape[2];
  const auto agent_poses = postprocess::parse_predictions(data, transform);
  geometry_msgs::msg::Point base_position;
  auto traj = postprocess::create_ego_trajectory(agent_poses, stamp, base_position, 0);
  ASSERT_EQ(traj.points.size(), expected_points);
}

TEST(PostprocessingUtilsTest, FixStopPointsAfterConfiguredDecelerationDuration)
{
  Trajectory trajectory;
  for (size_t i = 0; i < 7; ++i) {
    auto & point = trajectory.points.emplace_back();
    const double time = 0.5 * static_cast<double>(i);
    point.time_from_start.sec = static_cast<int32_t>(time);
    point.time_from_start.nanosec = static_cast<uint32_t>((time - std::floor(time)) * 1.0e9);
    point.pose.position.x = static_cast<double>(i);
    point.longitudinal_velocity_mps = i >= 5 ? 0.2F : 2.0F;
    point.acceleration_mps2 = i >= 3 ? -1.0F : 0.0F;
  }

  postprocess::StopPointFixingParams params;
  params.velocity_threshold_mps = 0.3;
  params.min_deceleration_duration_sec = 1.0;
  const auto stop_index = postprocess::fix_stop_points(trajectory, params);

  ASSERT_EQ(stop_index, 5U);
  EXPECT_DOUBLE_EQ(trajectory.points[4].pose.position.x, 4.0);
  for (size_t i = 5; i < trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[i].pose.position.x, 5.0);
    EXPECT_FLOAT_EQ(trajectory.points[i].longitudinal_velocity_mps, 0.0F);
    EXPECT_FLOAT_EQ(trajectory.points[i].acceleration_mps2, 0.0F);
  }
}

TEST(PostprocessingUtilsTest, FixStopPointsResetsDecelerationDuration)
{
  Trajectory trajectory;
  for (size_t i = 0; i < 5; ++i) {
    auto & point = trajectory.points.emplace_back();
    point.time_from_start.sec = static_cast<int32_t>(i);
    point.pose.position.x = static_cast<double>(i);
    point.longitudinal_velocity_mps = 0.2F;
    point.acceleration_mps2 = -1.0F;
  }
  trajectory.points[2].acceleration_mps2 = 0.0F;

  postprocess::StopPointFixingParams params;
  params.min_deceleration_duration_sec = 2.0;
  EXPECT_FALSE(postprocess::fix_stop_points(trajectory, params).has_value());
  EXPECT_DOUBLE_EQ(trajectory.points.back().pose.position.x, 4.0);
}

namespace
{
// Ego poses on the model output grid, laid out along +x at the given positions.
std::vector<std::vector<std::vector<Eigen::Matrix4d>>> make_ego_poses_along_x(
  const std::vector<double> & positions_x)
{
  std::vector<Eigen::Matrix4d> ego_poses;
  for (const double x : positions_x) {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose(0, 3) = x;
    ego_poses.push_back(pose);
  }
  return {{ego_poses}};
}
}  // namespace

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryConstantSpeed)
{
  // Points 1 m apart on the 0.1 s grid, the first one 1 m ahead of the ego position: 10 m/s.
  const auto agent_poses = make_ego_poses_along_x({1.0, 2.0, 3.0, 4.0, 5.0});
  geometry_msgs::msg::Point base_position;

  const auto trajectory =
    postprocess::create_ego_trajectory(agent_poses, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 5U);
  for (const auto & point : trajectory.points) {
    EXPECT_NEAR(point.longitudinal_velocity_mps, 10.0F, 1e-3F);
    EXPECT_NEAR(point.acceleration_mps2, 0.0F, 1e-3F);
    // The model predicts poses only.
    EXPECT_FLOAT_EQ(point.heading_rate_rps, 0.0F);
    EXPECT_FLOAT_EQ(point.front_wheel_angle_rad, 0.0F);
  }
}

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryAcceleration)
{
  // Steps of 0.1, 0.2, 0.3 m on the 0.1 s grid: 1, 2, 3 m/s, i.e. 10 m/s^2.
  const auto agent_poses = make_ego_poses_along_x({0.1, 0.3, 0.6});
  geometry_msgs::msg::Point base_position;

  const auto trajectory =
    postprocess::create_ego_trajectory(agent_poses, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 3U);
  EXPECT_NEAR(trajectory.points[0].longitudinal_velocity_mps, 1.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 2.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[2].longitudinal_velocity_mps, 3.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[0].acceleration_mps2, 10.0F, 1e-2F);
  EXPECT_NEAR(trajectory.points[1].acceleration_mps2, 10.0F, 1e-2F);
  // The last point has no successor.
  EXPECT_FLOAT_EQ(trajectory.points[2].acceleration_mps2, 0.0F);
}

TEST(PostprocessingUtilsTest, CreateEgoTrajectoryUsesEgoPositionForTheFirstPoint)
{
  // Same poses, ego already at x = 1: the first point no longer covers any distance.
  const auto agent_poses = make_ego_poses_along_x({1.0, 2.0});
  geometry_msgs::msg::Point base_position;
  base_position.x = 1.0;

  const auto trajectory =
    postprocess::create_ego_trajectory(agent_poses, rclcpp::Time(0), base_position, 0);

  ASSERT_EQ(trajectory.points.size(), 2U);
  EXPECT_NEAR(trajectory.points[0].longitudinal_velocity_mps, 0.0F, 1e-3F);
  EXPECT_NEAR(trajectory.points[1].longitudinal_velocity_mps, 10.0F, 1e-3F);
}

TEST(PostprocessingUtilsTest, SmoothInitialVelocityRemovesSpacingNoise)
{
  // 1.2 m/s for 3 s with +-2 cm spacing noise, the first point 0.24 m ahead of the ego.
  std::vector<double> positions_x;
  for (int i = 0; i < 30; ++i) {
    positions_x.push_back(0.24 + 0.12 * i + ((i % 2 == 0) ? 0.02 : -0.02));
  }
  const auto agent_poses = make_ego_poses_along_x(positions_x);
  geometry_msgs::msg::Point base_position;
  auto trajectory =
    postprocess::create_ego_trajectory(agent_poses, rclcpp::Time(0), base_position, 0);
  const auto raw = trajectory;

  postprocess::VelocitySmoothingParams params;
  params.enable = true;
  params.horizon_sec = 1.5;
  postprocess::smooth_initial_velocity(trajectory, params);

  // Points up to 1.5 s (indices 0..14) follow the fit; the rest keep the finite differences.
  for (size_t i = 0; i < 15; ++i) {
    EXPECT_NEAR(trajectory.points[i].longitudinal_velocity_mps, 1.2F, 0.1F) << i;
    EXPECT_NEAR(trajectory.points[i].acceleration_mps2, 0.0F, 0.2F) << i;
    EXPECT_DOUBLE_EQ(trajectory.points[i].pose.position.x, raw.points[i].pose.position.x);
  }
  for (size_t i = 15; i < trajectory.points.size(); ++i) {
    EXPECT_FLOAT_EQ(
      trajectory.points[i].longitudinal_velocity_mps, raw.points[i].longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(trajectory.points[i].acceleration_mps2, raw.points[i].acceleration_mps2);
  }
}

TEST(PostprocessingUtilsTest, SmoothInitialVelocityFitsConstantAcceleration)
{
  // s = 1.0 t + 0.5 * 2.0 t^2 on the 0.1 s grid.
  std::vector<double> positions_x;
  for (int i = 1; i <= 20; ++i) {
    const double t = 0.1 * i;
    positions_x.push_back(t + t * t);
  }
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);

  postprocess::VelocitySmoothingParams params;
  params.enable = true;
  postprocess::smooth_initial_velocity(trajectory, params);

  for (size_t i = 0; i < 15; ++i) {
    const double t = 0.1 * static_cast<double>(i + 1);
    EXPECT_NEAR(trajectory.points[i].longitudinal_velocity_mps, 1.0 + 2.0 * t, 1e-3) << i;
    EXPECT_NEAR(trajectory.points[i].acceleration_mps2, 2.0, 1e-3) << i;
  }
}

TEST(PostprocessingUtilsTest, SmoothInitialVelocityMovesOffAPlanThatWaitsFirst)
{
  // Standing still for 0.3 s, then 0.5 m/s^2: the leading speeds must not be zero.
  std::vector<double> positions_x;
  for (int i = 1; i <= 20; ++i) {
    const double t = std::max(0.1 * i - 0.3, 0.0);
    positions_x.push_back(0.25 * t * t);
  }
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);

  postprocess::VelocitySmoothingParams params;
  params.enable = true;
  postprocess::smooth_initial_velocity(trajectory, params);

  for (size_t i = 0; i < 15; ++i) {
    EXPECT_GT(trajectory.points[i].longitudinal_velocity_mps, 0.0F) << i;
    EXPECT_GT(trajectory.points[i].acceleration_mps2, 0.0F) << i;
  }
  for (size_t i = 1; i < 15; ++i) {
    EXPECT_GT(
      trajectory.points[i].longitudinal_velocity_mps,
      trajectory.points[i - 1].longitudinal_velocity_mps)
      << i;
  }
}

TEST(PostprocessingUtilsTest, SmoothInitialVelocityKeepsAStandstillAtZero)
{
  const std::vector<double> positions_x(20, 0.0);
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);

  postprocess::VelocitySmoothingParams params;
  params.enable = true;
  postprocess::smooth_initial_velocity(trajectory, params);

  for (size_t i = 0; i < 15; ++i) {
    EXPECT_FLOAT_EQ(trajectory.points[i].longitudinal_velocity_mps, 0.0F) << i;
  }
}

TEST(PostprocessingUtilsTest, SmoothInitialPathStartsAtTheVehicleAndRemovesJitter)
{
  // 1.0 m/s along x, starting 0.1 m to the side of the vehicle, with +-4 cm sideways jitter.
  std::vector<double> positions_x;
  for (int i = 1; i <= 30; ++i) {
    positions_x.push_back(0.1 * i);
  }
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points[i].pose.position.y = 0.1 + ((i % 2 == 0) ? 0.04 : -0.04);
  }
  const auto raw = trajectory;

  geometry_msgs::msg::Pose ego_pose;
  ego_pose.orientation.w = 1.0;
  postprocess::PathSmoothingParams params;
  params.enable = true;
  postprocess::smooth_initial_path(trajectory, ego_pose, 1.0, params);

  // The first point is near the vehicle line, and the jitter is gone: no sideways zigzag.
  EXPECT_LT(std::abs(trajectory.points[0].pose.position.y), 0.05);
  for (size_t i = 1; i + 1 < 15; ++i) {
    const auto & p = trajectory.points;
    const double second_difference =
      p[i + 1].pose.position.y - 2.0 * p[i].pose.position.y + p[i - 1].pose.position.y;
    EXPECT_LT(std::abs(second_difference), 0.01) << i;
    EXPECT_NEAR(p[i].pose.position.x, raw.points[i].pose.position.x, 0.05) << i;
  }
  // Points after horizon + blend (2.0 s) are untouched.
  for (size_t i = 20; i < trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[i].pose.position.y, raw.points[i].pose.position.y) << i;
  }
}

TEST(PostprocessingUtilsTest, SmoothInitialPathKeepsTheVehicleHeadingAtStandstill)
{
  // Vehicle at (10, 5) heading 90 deg, standing; the model output barely moves with jitter.
  std::vector<double> positions_x(30, 0.0);
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points[i].pose.position.x = 10.0 + ((i % 2 == 0) ? 0.03 : -0.03);
    trajectory.points[i].pose.position.y = 5.0 + 0.002 * static_cast<double>(i);
  }
  geometry_msgs::msg::Pose ego_pose;
  ego_pose.position.x = 10.0;
  ego_pose.position.y = 5.0;
  ego_pose.orientation = autoware_utils::create_quaternion_from_yaw(M_PI_2);
  postprocess::PathSmoothingParams params;
  params.enable = true;
  postprocess::smooth_initial_path(trajectory, ego_pose, 0.0, params);

  for (size_t i = 0; i < 15; ++i) {
    const double yaw = autoware_utils::get_rpy(trajectory.points[i].pose.orientation).z;
    EXPECT_NEAR(yaw, M_PI_2, 0.05) << i;
    EXPECT_NEAR(trajectory.points[i].pose.position.x, 10.0, 0.01) << i;
  }
}

TEST(PostprocessingUtilsTest, SmoothPathTailFollowsTheArcAndTakesItsHeading)
{
  // 3 m/s on a circle of radius 10 m about (0, 10), +-3 cm radial wobble, model headings off by
  // +-4 deg.
  constexpr double radius = 10.0;
  constexpr double speed = 3.0;
  std::vector<double> positions_x(60, 0.0);
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);
  const auto arc_angle = [&](const size_t i) {
    return speed * 0.1 * static_cast<double>(i + 1) / radius;
  };
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double wobble = (i % 2 == 0) ? 0.03 : -0.03;
    const double angle = arc_angle(i);
    auto & pose = trajectory.points[i].pose;
    pose.position.x = (radius + wobble) * std::sin(angle);
    pose.position.y = radius - (radius + wobble) * std::cos(angle);
    pose.orientation = autoware_utils::create_quaternion_from_yaw(
      angle + autoware_utils::deg2rad((i % 2 == 0) ? 4.0 : -4.0));
  }
  const auto raw = trajectory;
  postprocess::PathSmoothingParams params;
  params.enable = true;
  postprocess::smooth_path_tail(trajectory, params);

  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & pose = trajectory.points[i].pose;
    if (i < 15) {
      // Up to the horizon (1.5 s) the points are left to smooth_initial_path.
      EXPECT_DOUBLE_EQ(pose.position.y, raw.points[i].pose.position.y) << i;
      continue;
    }
    const double yaw = autoware_utils::get_rpy(pose.orientation).z;
    const double yaw_error = autoware_utils::normalize_radian(yaw - arc_angle(i));
    if (i + 2 >= trajectory.points.size()) {
      // The last two points keep their position and continue the heading before them.
      EXPECT_NEAR(yaw_error, 0.0, autoware_utils::deg2rad(4.0)) << i;
      continue;
    }
    const double distance_from_centre = std::hypot(pose.position.x, pose.position.y - radius);
    EXPECT_NEAR(distance_from_centre, radius, 0.012) << i;
    EXPECT_NEAR(yaw_error, 0.0, autoware_utils::deg2rad(0.5)) << i;
  }
}

TEST(PostprocessingUtilsTest, SmoothPathTailKeepsTheHeadingThroughAStop)
{
  // 2 m/s along y (heading 90 deg) for 2 s, then standing with jitter.
  std::vector<double> positions_x(40, 0.0);
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    auto & pose = trajectory.points[i].pose;
    pose.position.x = (i % 2 == 0) ? 0.002 : -0.002;
    pose.position.y = 0.2 * static_cast<double>(std::min<size_t>(i + 1, 20));
    pose.orientation = autoware_utils::create_quaternion_from_yaw(i < 20 ? M_PI_2 : 0.0);
  }
  postprocess::PathSmoothingParams params;
  params.enable = true;
  postprocess::smooth_path_tail(trajectory, params);

  for (size_t i = 15; i < trajectory.points.size(); ++i) {
    const double yaw = autoware_utils::get_rpy(trajectory.points[i].pose.orientation).z;
    EXPECT_NEAR(yaw, M_PI_2, autoware_utils::deg2rad(2.0)) << i;
  }
}

TEST(PostprocessingUtilsTest, SmoothPathTailDisabledWithZeroWindow)
{
  std::vector<double> positions_x;
  for (int i = 1; i <= 30; ++i) {
    positions_x.push_back(0.1 * i);
  }
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    trajectory.points[i].pose.position.y = (i % 2 == 0) ? 0.04 : -0.04;
  }
  const auto raw = trajectory;
  postprocess::PathSmoothingParams params;
  params.tail_half_window_sec = 0.0;
  postprocess::smooth_path_tail(trajectory, params);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    EXPECT_DOUBLE_EQ(trajectory.points[i].pose.position.y, raw.points[i].pose.position.y) << i;
  }
}

TEST(PostprocessingUtilsTest, LimitCurveSpeedCapsTheCurveAndBrakesBeforeIt)
{
  // 10 m/s: 30 m straight along x, then a left curve of radius 20 m.
  constexpr double speed = 10.0;
  constexpr double radius = 20.0;
  std::vector<double> positions_x(60, 0.0);
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const double s = speed * 0.1 * static_cast<double>(i + 1);
    auto & point = trajectory.points[i];
    if (s <= 30.0) {
      point.pose.position.x = s;
      point.pose.position.y = 0.0;
    } else {
      const double angle = (s - 30.0) / radius;
      point.pose.position.x = 30.0 + radius * std::sin(angle);
      point.pose.position.y = radius * (1.0 - std::cos(angle));
    }
    point.longitudinal_velocity_mps = static_cast<float>(speed);
  }
  postprocess::CurveSpeedLimitParams params;
  params.enable = true;
  postprocess::limit_curve_speed(trajectory, params);

  const double cap = std::sqrt(params.max_lateral_acceleration_mps2 * radius);
  for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    const double s = speed * 0.1 * static_cast<double>(i + 1);
    if (s > 32.0) {
      EXPECT_NEAR(point.longitudinal_velocity_mps, cap, 0.05) << i;
    }
    // Braking toward the cap never exceeds the deceleration limit.
    const auto & next = trajectory.points[i + 1];
    const double ds = std::hypot(
      next.pose.position.x - point.pose.position.x, next.pose.position.y - point.pose.position.y);
    const double v0 = point.longitudinal_velocity_mps;
    const double v1 = next.longitudinal_velocity_mps;
    EXPECT_GE((v1 * v1 - v0 * v0) / (2.0 * ds), -params.max_deceleration_mps2 - 1.0e-3) << i;
  }
  // The car brakes early: at 10 m/s and 1 m/s^2 the speed starts to drop about 40 m before the
  // curve, i.e. from the first point.
  EXPECT_LT(trajectory.points[0].longitudinal_velocity_mps, speed);
  EXPECT_LT(trajectory.points[0].acceleration_mps2, 0.0F);
}

TEST(PostprocessingUtilsTest, LimitCurveSpeedLeavesAStraightAlone)
{
  std::vector<double> positions_x;
  for (int i = 1; i <= 40; ++i) {
    positions_x.push_back(0.8 * i);
  }
  auto trajectory = postprocess::create_ego_trajectory(
    make_ego_poses_along_x(positions_x), rclcpp::Time(0), geometry_msgs::msg::Point{}, 0);
  const auto before = trajectory;
  postprocess::CurveSpeedLimitParams params;
  params.enable = true;
  postprocess::limit_curve_speed(trajectory, params);
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    EXPECT_FLOAT_EQ(
      trajectory.points[i].longitudinal_velocity_mps, before.points[i].longitudinal_velocity_mps)
      << i;
    EXPECT_FLOAT_EQ(trajectory.points[i].acceleration_mps2, before.points[i].acceleration_mps2)
      << i;
  }
}

}  // namespace autoware::ml_planner::test
