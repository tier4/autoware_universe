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
#include <cstddef>
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

}  // namespace autoware::safety_planner
