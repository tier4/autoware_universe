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

#include "utils/velocity_optimizer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace autoware::safety_planner
{

namespace
{
constexpr double DS = 1.0;
//! The QP bounds are soft; this is what the slack costs leave over them
constexpr double TOLERANCE = 0.05;

void expect_within_limits(
  const VelocityOptimizerResult & result, const std::vector<double> & v_max,
  const VelocityOptimizerParams & params, const std::size_t from = 0)
{
  for (std::size_t i = from; i < v_max.size(); ++i) {
    EXPECT_LE(result.v[i], v_max[i] + TOLERANCE) << "i=" << i;
    EXPECT_GE(result.a[i], params.a_min - TOLERANCE) << "i=" << i;
    EXPECT_LE(result.a[i], params.a_max + TOLERANCE) << "i=" << i;
  }
}

constexpr double TIME_STEP_S = 0.1;
constexpr std::size_t NUM_POINTS = 201;

//! Along x from the origin, with the speed limit v_limit everywhere
PathPointTrajectory make_straight_path(const double length, const double v_limit)
{
  std::vector<PathPointWithLaneId> points;
  for (int i = 0; i <= 10; ++i) {
    auto & p = points.emplace_back();
    p.point.pose.position.x = length * i / 10.0;
    p.point.pose.orientation.w = 1.0;
  }
  auto path = *PathPointTrajectory::Builder{}.build(points);
  path.longitudinal_velocity_mps() = v_limit;
  return path;
}

VelocityPlanningParams make_params()
{
  VelocityPlanningParams params;
  params.resolution_m = 1.0;
  params.max_length_m = 150.0;
  params.lat_accel = 1.0;
  params.steer_rate = std::numeric_limits<double>::infinity();
  params.wheel_base_m = 2.8;
  return params;
}
}  // namespace

TEST(OptimizeVelocity, AcceleratesToTheLimitFromRest)
{
  const std::vector<double> v_max(200, 10.0);
  const VelocityOptimizerParams params;
  const auto result = optimize_velocity(v_max, DS, 0.0, 0.0, params);
  ASSERT_TRUE(result);
  expect_within_limits(*result, v_max, params);
  EXPECT_NEAR(result->v.back(), 10.0, TOLERANCE);
}

TEST(OptimizeVelocity, ChoosesTheInitialAccelerationUnderABoundAhead)
{
  // A start from rest without a0: the first acceleration is the QP's, and a bound on the next
  // grid point (a steer rate limit right ahead) holds it down
  std::vector<double> v_max(50, 10.0);
  v_max[1] = 0.5;
  const VelocityOptimizerParams params;
  const auto result = optimize_velocity(v_max, DS, 0.3, std::nullopt, params);
  ASSERT_TRUE(result);
  expect_within_limits(*result, v_max, params);
  EXPECT_LE(result->a[0], (0.5 * 0.5 - 0.3 * 0.3) / (2.0 * DS) + TOLERANCE);
  EXPECT_GT(result->a[0], 0.0);
}

TEST(OptimizeVelocity, BrakesAtTheNominalDecelerationFromAboveTheLimit)
{
  // The ego is over the limit by a little, as when it touches the speed limit: no braking harder
  // than a_min, and no stop
  const std::vector<double> v_max(100, 9.72);
  const VelocityOptimizerParams params;
  const auto result = optimize_velocity(v_max, DS, 9.9, 0.0, params);
  ASSERT_TRUE(result);
  EXPECT_GE(*std::min_element(result->a.begin(), result->a.end()), params.a_min - TOLERANCE);
  EXPECT_GT(*std::min_element(result->v.begin(), result->v.end()), 9.0);
  expect_within_limits(*result, v_max, params, 20);
}

TEST(OptimizeVelocity, StopsAtTheFirstStopPoint)
{
  // 65 m before a stop at 9.4 m/s needs about 0.7 m/s^2, within the nominal 1.0
  std::vector<double> v_max(66, 9.72);
  v_max[65] = 0.0;
  const VelocityOptimizerParams params;
  const auto result = optimize_velocity(v_max, DS, 9.4, 0.0, params);
  ASSERT_TRUE(result);
  expect_within_limits(*result, v_max, params);
  EXPECT_NEAR(result->v.back(), 0.0, TOLERANCE);
}

TEST(PlanVelocity, StaysUnderTheSpeedLimitAndStopsAtThePathEnd)
{
  const auto path = make_straight_path(60.0, 10.0);
  const auto points = plan_velocity(path, 0.0, {5.0, 0.0}, make_params(), NUM_POINTS, TIME_STEP_S);
  ASSERT_TRUE(points);
  ASSERT_EQ(points->size(), NUM_POINTS);
  for (std::size_t k = 0; k < points->size(); ++k) {
    const auto & point = (*points)[k];
    EXPECT_NEAR(rclcpp::Duration(point.time_from_start).seconds(), k * TIME_STEP_S, 1e-9);
    EXPECT_LE(point.longitudinal_velocity_mps, 10.0 + TOLERANCE) << "k=" << k;
    EXPECT_NEAR(point.pose.position.y, 0.0, 1e-6);
    if (k > 0) {
      EXPECT_GE(point.pose.position.x, (*points)[k - 1].pose.position.x - 1e-6) << "k=" << k;
    }
  }
  EXPECT_NEAR(points->front().longitudinal_velocity_mps, 5.0, TOLERANCE);
  EXPECT_NEAR(points->back().longitudinal_velocity_mps, 0.0, TOLERANCE);
  EXPECT_LE(points->back().pose.position.x, 60.0 + 1e-6);
}

TEST(PlanVelocity, KeepsASpeedLimitShorterThanTheGridSpacing)
{
  // [30.3, 30.7] holds no grid point (resolution_m = 1); the grid points on both sides take it
  auto path = make_straight_path(100.0, 10.0);
  path.longitudinal_velocity_mps().range(30.3, 30.7).set(2.0);
  path.longitudinal_velocity_mps().at(30.7).set(10.0);
  const auto points = plan_velocity(path, 0.0, {5.0, 0.0}, make_params(), NUM_POINTS, TIME_STEP_S);
  ASSERT_TRUE(points);
  bool passed = false;
  for (const auto & point : *points) {
    if (point.pose.position.x >= 30.3 && point.pose.position.x <= 30.7) {
      passed = true;
      EXPECT_LE(point.longitudinal_velocity_mps, 2.0 + TOLERANCE) << "x=" << point.pose.position.x;
    }
  }
  EXPECT_TRUE(passed);
}

TEST(PlanVelocity, StopsBeforeTheStopLine)
{
  auto path = make_straight_path(100.0, 10.0);
  path.set_stopline(40.0);
  const auto points = plan_velocity(path, 0.0, {5.0, 0.0}, make_params(), NUM_POINTS, TIME_STEP_S);
  ASSERT_TRUE(points);
  EXPECT_LE(points->back().pose.position.x, 40.0 + 1e-6);
  EXPECT_NEAR(points->back().longitudinal_velocity_mps, 0.0, TOLERANCE);
}

TEST(PlanVelocity, CapsTheLateralAcceleration)
{
  // Half a circle of radius R: v^2 / R stays within lat_accel
  constexpr double R = 20.0;
  std::vector<PathPointWithLaneId> path_points;
  for (int i = 0; i <= 36; ++i) {
    const double theta = M_PI * i / 36.0;
    auto & p = path_points.emplace_back();
    p.point.pose.position.x = R * std::sin(theta);
    p.point.pose.position.y = R * (1.0 - std::cos(theta));
    p.point.pose.orientation.z = std::sin(theta / 2.0);
    p.point.pose.orientation.w = std::cos(theta / 2.0);
  }
  auto path = *PathPointTrajectory::Builder{}.build(path_points);
  path.longitudinal_velocity_mps() = 10.0;
  const auto params = make_params();
  const auto points = plan_velocity(path, 0.0, {4.0, 0.0}, params, NUM_POINTS, TIME_STEP_S);
  ASSERT_TRUE(points);
  double fastest = 0.0;
  for (const auto & point : *points) {
    EXPECT_LE(point.longitudinal_velocity_mps, std::sqrt(params.lat_accel * R) + 0.1);
    fastest = std::max(fastest, static_cast<double>(point.longitudinal_velocity_mps));
  }
  EXPECT_GT(fastest, std::sqrt(params.lat_accel * R) - 0.3);
}

TEST(PlanVelocity, StaysStoppedWithTheStopWithinTheResolution)
{
  // From standstill (a0 left to the QP) the path up to the stop is laid out at zero speed, so
  // that the trajectory keeps a direction
  auto path = make_straight_path(100.0, 10.0);
  path.set_stopline(10.5);
  const auto points =
    plan_velocity(path, 10.0, {0.3, std::nullopt}, make_params(), NUM_POINTS, TIME_STEP_S);
  ASSERT_TRUE(points);
  for (const auto & point : *points) {
    EXPECT_EQ(point.longitudinal_velocity_mps, 0.0);
  }
  EXPECT_NEAR(points->front().pose.position.x, 10.0, 1e-6);
  EXPECT_NEAR(points->back().pose.position.x, 10.5, 1e-6);
}

TEST(PlanVelocity, StartsFromStandstillWithTheStopBeyondTheResolution)
{
  auto path = make_straight_path(100.0, 10.0);
  path.set_stopline(30.0);
  const auto points =
    plan_velocity(path, 10.0, {0.3, std::nullopt}, make_params(), NUM_POINTS, TIME_STEP_S);
  ASSERT_TRUE(points);
  const auto fastest = std::max_element(
    points->begin(), points->end(), [](const TrajectoryPoint & x, const TrajectoryPoint & y) {
      return x.longitudinal_velocity_mps < y.longitudinal_velocity_mps;
    });
  EXPECT_GT(fastest->longitudinal_velocity_mps, 1.0);
  EXPECT_LE(points->back().pose.position.x, 30.0 + 1e-6);
}

}  // namespace autoware::safety_planner
