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

#include "autoware/tensorrt_e2e/prev_plan.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::tensorrt_e2e
{
namespace
{
constexpr int64_t T = 40;
constexpr int64_t STEP_NS = 100000000;

Eigen::Matrix4d pose(const double x, const double y, const double yaw)
{
  Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
  m(0, 0) = std::cos(yaw);
  m(0, 1) = -std::sin(yaw);
  m(1, 0) = std::sin(yaw);
  m(1, 1) = std::cos(yaw);
  m(0, 3) = x;
  m(1, 3) = y;
  return m;
}

//! Straight plan along the ego x axis, 1 m per step, heading 0: point j at x = j + 1.
std::vector<float> straight_plan()
{
  std::vector<float> raw;
  for (int64_t j = 0; j < T; ++j) {
    raw.insert(raw.end(), {static_cast<float>(j + 1), 0.0f, 1.0f, 0.0f});
  }
  return raw;
}
}  // namespace

TEST(PrevPlan, OneStepLaterMapsOntoTheCurrentGridAndDropsTheLastPoint)
{
  // Previous ego at the origin yawed 90 deg, then the ego moved one step (1 m) along the plan.
  const auto prev = pose(5.0, 7.0, M_PI / 2);
  const auto cache = cache_from_output(straight_plan(), T, prev, 1000, 3);
  const auto now = pose(5.0, 8.0, M_PI / 2);  // plan point 0
  const auto out = build_prev_plan_tensor(&cache, now.inverse(), 1000 + STEP_NS, 3, T);
  ASSERT_EQ(out.size(), static_cast<size_t>(T * 5));
  for (int64_t k = 0; k < T - 1; ++k) {
    // Row k is previous point k+1 (x = k + 2), re-expressed relative to previous point 0 (x = 1).
    EXPECT_NEAR(out[k * 5 + 0], k + 1.0, 1e-4);
    EXPECT_NEAR(out[k * 5 + 1], 0.0, 1e-4);
    EXPECT_NEAR(out[k * 5 + 2], 1.0, 1e-5);
    EXPECT_NEAR(out[k * 5 + 3], 0.0, 1e-5);
    EXPECT_EQ(out[k * 5 + 4], 1.0f);
  }
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(out[(T - 1) * 5 + i], 0.0f);
  }
}

TEST(PrevPlan, StalePlanIsInvalid)
{
  const auto cache = cache_from_output(straight_plan(), T, pose(0, 0, 0), 0, 0);
  const auto out = build_prev_plan_tensor(&cache, Eigen::Matrix4d::Identity(), 350000000, 0, T);
  for (int64_t k = 0; k < T; ++k) {
    EXPECT_EQ(out[k * 5 + 4], 0.0f);
  }
  const auto fresh = build_prev_plan_tensor(&cache, Eigen::Matrix4d::Identity(), 250000000, 0, T);
  EXPECT_EQ(fresh[4], 1.0f);
}

TEST(PrevPlan, GenerationChangeOrMissingCacheIsInvalid)
{
  const auto cache = cache_from_output(straight_plan(), T, pose(0, 0, 0), 0, 1);
  for (const auto * c : {&cache, static_cast<const PrevPlanCache *>(nullptr)}) {
    const auto out = build_prev_plan_tensor(c, Eigen::Matrix4d::Identity(), STEP_NS, 2, T);
    for (int64_t k = 0; k < T; ++k) {
      EXPECT_EQ(out[k * 5 + 4], 0.0f);
    }
  }
}

TEST(PrevPlan, FractionalShiftInterpolatesPositionAndNormalisesHeading)
{
  // Heading turns from 0 to 90 deg between the first two points.
  std::vector<float> raw = straight_plan();
  raw[4 * 1 + 2] = 0.0f;
  raw[4 * 1 + 3] = 1.0f;
  const auto cache = cache_from_output(raw, T, Eigen::Matrix4d::Identity(), 0, 0);
  // Half a step later: row 0 is half way between points 0 and 1.
  const auto out = build_prev_plan_tensor(&cache, Eigen::Matrix4d::Identity(), STEP_NS / 2, 0, T);
  EXPECT_NEAR(out[0], 1.5, 1e-5);
  EXPECT_NEAR(out[2], std::sqrt(0.5), 1e-5);
  EXPECT_NEAR(out[3], std::sqrt(0.5), 1e-5);
  EXPECT_EQ(out[4], 1.0f);
}

}  // namespace autoware::tensorrt_e2e
