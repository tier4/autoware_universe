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

#include "autoware/tensorrt_e2e/postprocess/latentdrive_postprocessor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::tensorrt_e2e::latentdrive
{

namespace
{
constexpr double kTimeStep = 0.1;
constexpr size_t kSteps = 40;

/// Straight plan at constant speed `v`: waypoint i at (i + 1) * dt * v.
Plan straight_plan(const double v, const double reach_scale = 1.0)
{
  Plan plan(kSteps);
  for (size_t i = 0; i < kSteps; ++i) {
    plan[i] = {v * kTimeStep * static_cast<double>(i + 1) * reach_scale, 0.0, 0.0};
  }
  return plan;
}
}  // namespace

TEST(LatentDrivePlanSamplingTest, InterpolatesAndExtrapolates)
{
  const Plan plan = straight_plan(10.0);           // 1 m apart
  EXPECT_DOUBLE_EQ(sample_at(plan, -1.0).x, 0.0);  // the ego origin
  EXPECT_DOUBLE_EQ(sample_at(plan, 0.0).x, 1.0);
  EXPECT_DOUBLE_EQ(sample_at(plan, 2.5).x, 3.5);
  EXPECT_DOUBLE_EQ(sample_at(plan, 41.0).x, 42.0);  // extended final segment
  EXPECT_DOUBLE_EQ(sample_at(plan, -3.0).x, 0.0);   // clamped at the origin
}

TEST(LatentDrivePlanSamplingTest, ToLocalRotatesIntoTheOriginFrame)
{
  const Waypoint origin{1.0, 1.0, M_PI / 2.0};
  const Waypoint local = to_local(Waypoint{1.0, 3.0, M_PI / 2.0}, origin);
  EXPECT_NEAR(local.x, 2.0, 1e-9);  // 2 m ahead along the rotated x axis
  EXPECT_NEAR(local.y, 0.0, 1e-9);
  EXPECT_NEAR(local.yaw, 0.0, 1e-9);
}

TEST(LatentDriveCarryForwardTest, PlanFollowedExactlyCarriesOntoItself)
{
  // Ego drove exactly as planned for one tick: the carried plan equals the plan itself.
  const Plan plan = straight_plan(10.0);
  const Waypoint ego_now{1.0, 0.0, 0.0};  // 10 m/s for 0.1 s
  const Plan carried = carry_forward(plan, ego_now, kTimeStep, kTimeStep);
  ASSERT_EQ(carried.size(), plan.size());
  for (size_t i = 0; i < plan.size(); ++i) {
    EXPECT_NEAR(carried[i].x, plan[i].x, 1e-9) << i;
    EXPECT_NEAR(carried[i].y, plan[i].y, 1e-9) << i;
  }
}

TEST(LatentDriveCarryForwardTest, UsesMeasuredMotionNotThePlan)
{
  // The plan said 1 m per tick, the vehicle only moved 0.5 m: the carried plan is 0.5 m
  // further ahead than the plan, not equal to it.
  const Plan plan = straight_plan(10.0);
  const Plan carried = carry_forward(plan, Waypoint{0.5, 0.0, 0.0}, kTimeStep, kTimeStep);
  EXPECT_NEAR(carried[0].x, plan[0].x + 0.5, 1e-9);
  EXPECT_NEAR(carried.back().x, plan.back().x + 0.5, 1e-9);
}

TEST(LatentDrivePlanSmootherTest, FirstPlanPassesThroughThenBlends)
{
  SmoothingParams params;
  params.enable = true;
  params.alpha = 0.5;
  PlanSmoother smoother(params);

  const Plan first = straight_plan(10.0);
  const Plan out1 = smoother.update(first, Waypoint{}, 0.0, kTimeStep);
  EXPECT_DOUBLE_EQ(out1.back().x, first.back().x);

  // Next tick the plan reaches 1 m further (the jitter the filter is for); the ego moved as
  // planned. The carried plan equals `first`, so the blend lands halfway.
  const Plan second = straight_plan(10.0, 1.025);  // 41 m instead of 40 m at the end
  const Plan out2 = smoother.update(second, Waypoint{1.0, 0.0, 0.0}, kTimeStep, kTimeStep);
  EXPECT_NEAR(out2.back().x, 40.5, 1e-9);
}

TEST(LatentDrivePlanSmootherTest, LargeJumpAndTimeGapResetTheState)
{
  SmoothingParams params;
  params.enable = true;
  params.alpha = 0.35;
  params.reset_jump_m = 8.0;
  params.max_gap_seconds = 1.0;
  PlanSmoother smoother(params);

  smoother.update(straight_plan(10.0), Waypoint{}, 0.0, kTimeStep);
  // The plan now stops (end 40 m closer): a decision, passed through untouched.
  const Plan stop(kSteps, Waypoint{});
  const Plan out = smoother.update(stop, Waypoint{1.0, 0.0, 0.0}, kTimeStep, kTimeStep);
  EXPECT_DOUBLE_EQ(out.back().x, 0.0);

  // A bag loop: time steps back, the state is dropped and the plan passes through.
  const Plan again = straight_plan(10.0);
  const Plan out2 = smoother.update(again, Waypoint{}, -59.9, kTimeStep);
  EXPECT_DOUBLE_EQ(out2.back().x, again.back().x);
}

}  // namespace autoware::tensorrt_e2e::latentdrive
