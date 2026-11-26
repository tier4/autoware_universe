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

#include "autoware/ml_planner/optimization/reference_resample.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::ml_planner::optimization
{
namespace
{
constexpr double kWheelbase = 2.75;
constexpr double kSpeed = 30.0 / 3.6;
constexpr double kStep = kSpeed * opt_dt_s;

std::array<PoseSample, opt_horizon + 1> straight()
{
  std::array<PoseSample, opt_horizon + 1> poses{};
  for (size_t index = 0; index < poses.size(); ++index) {
    poses[index] = {kStep * static_cast<double>(index), 0.0, 0.0, 0.0, kSpeed};
  }
  return poses;
}

std::array<PoseSample, opt_horizon + 1> circle(const double radius_m, const double sign)
{
  std::array<PoseSample, opt_horizon + 1> poses{};
  const double steering = std::atan(sign * kWheelbase / radius_m);
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  for (size_t index = 0; index < poses.size(); ++index) {
    poses[index] = {x, y, yaw, steering, kSpeed};
    yaw += sign * kStep / radius_m;
    x += kStep * std::cos(yaw);
    y += kStep * std::sin(yaw);
  }
  return poses;
}
}  // namespace

TEST(ReferenceResample, StraightKeepsEgoSpeed)
{
  const auto resampled = resample_pose_sequence(straight(), kSpeed, 0.35, kWheelbase, -4.0, 3.0);
  EXPECT_NEAR(resampled.speed_mps.back(), kSpeed, 0.05);
  EXPECT_NEAR(resampled.curvature_inv_m[40], 0.0, 1.0e-3);
}

TEST(ReferenceResample, LeftAndRightTurnsShareTheSpeedCap)
{
  const auto right =
    resample_pose_sequence(circle(100.0, 1.0), kSpeed, 0.35, kWheelbase, -4.0, 3.0);
  const auto left =
    resample_pose_sequence(circle(100.0, -1.0), kSpeed, 0.35, kWheelbase, -4.0, 3.0);
  EXPECT_NEAR(right.speed_mps.back(), left.speed_mps.back(), 1.0e-6);
  EXPECT_GT(right.curvature_inv_m[40], 0.0);
  EXPECT_LT(left.curvature_inv_m[40], 0.0);
  EXPECT_LT(right.speed_mps.back(), 6.2);
  EXPECT_GT(right.speed_mps.back(), 5.4);
}

TEST(ReferenceResample, CurvatureFollowsSteeringNotYaw)
{
  auto yaw_noise_only = straight();
  for (size_t index = 0; index < yaw_noise_only.size(); ++index) {
    yaw_noise_only[index].yaw = 0.15 * std::sin(0.4 * static_cast<double>(index));
  }
  auto steered = straight();
  const double steering = std::atan(kWheelbase / 100.0);
  for (auto & pose : steered) {
    pose.steering_rad = steering;
    pose.yaw = 0.0;
  }
  const auto from_yaw = resample_pose_sequence(yaw_noise_only, kSpeed, 0.35, kWheelbase, -4.0, 3.0);
  const auto from_steer = resample_pose_sequence(steered, kSpeed, 0.35, kWheelbase, -4.0, 3.0);
  EXPECT_NEAR(from_yaw.speed_mps.back(), kSpeed, 0.05);
  EXPECT_LT(from_steer.speed_mps.back(), 6.2);
  EXPECT_NEAR(from_steer.curvature_inv_m[40], std::tan(steering) / kWheelbase, 1.0e-9);
}

TEST(ReferenceResample, StoppedEgoStillAcceleratesTowardTheModelSpeed)
{
  auto poses = straight();
  for (auto & pose : poses) {
    pose.speed_mps = 8.0;
  }
  const auto resampled = resample_pose_sequence(poses, 0.0, 0.35, kWheelbase, -4.0, 3.0);
  EXPECT_GT(resampled.speed_mps[10], 2.5);
  EXPECT_NEAR(resampled.speed_mps.back(), 8.0, 0.05);
}

TEST(ReferenceResample, AcceleratesTowardTheModelSpeed)
{
  auto poses = straight();
  constexpr double kTarget = 10.0;
  for (auto & pose : poses) {
    pose.speed_mps = kTarget;
  }
  constexpr double kEgo = 2.0;
  constexpr double kMaxAccel = 3.0;
  const auto resampled = resample_pose_sequence(poses, kEgo, 0.35, kWheelbase, -4.0, kMaxAccel);
  EXPECT_NEAR(resampled.speed_mps[10], kEgo + kMaxAccel * 1.0, 0.05);
  EXPECT_NEAR(resampled.speed_mps.back(), kTarget, 0.05);
  EXPECT_GT(resampled.poses.back().x, kEgo * opt_dt_s * static_cast<double>(opt_horizon));
}

TEST(ReferenceResample, CurvatureCapStillLimitsAnAcceleratingProfile)
{
  auto poses = circle(100.0, 1.0);
  for (auto & pose : poses) {
    pose.speed_mps = 30.0;
  }
  const auto resampled = resample_pose_sequence(poses, 2.0, 0.35, kWheelbase, -4.0, 3.0);
  EXPECT_LT(resampled.speed_mps.back(), 6.2);
  EXPECT_GT(resampled.speed_mps.back(), 5.4);
}

}  // namespace autoware::ml_planner::optimization
