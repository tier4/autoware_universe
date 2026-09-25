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

#include "autoware/mppi_optimizer/mppi_application_status.hpp"

#include <gtest/gtest.h>

namespace autoware::mppi_optimizer
{

TEST(MppiApplicationStatus, AppliesSuccessfulOptimizedTrajectory)
{
  const auto status = makeMppiApplicationStatus(false, false, false, 80U);

  EXPECT_TRUE(status.optimization_succeeded);
  EXPECT_TRUE(status.optimized_trajectory_applied);
  EXPECT_FALSE(status.fallback_applied);
  EXPECT_TRUE(status.output_applied);
}

TEST(MppiApplicationStatus, DoesNotApplySkippedOptimization)
{
  const auto status = makeMppiApplicationStatus(false, false, false, 0U);

  EXPECT_FALSE(status.optimization_succeeded);
  EXPECT_FALSE(status.optimized_trajectory_applied);
  EXPECT_FALSE(status.fallback_applied);
  EXPECT_FALSE(status.output_applied);
}

TEST(MppiApplicationStatus, ReportsShadowResultAsNotApplied)
{
  const auto status = makeMppiApplicationStatus(true, false, false, 80U);

  EXPECT_TRUE(status.optimization_succeeded);
  EXPECT_FALSE(status.optimized_trajectory_applied);
  EXPECT_FALSE(status.fallback_applied);
  EXPECT_FALSE(status.output_applied);
}

TEST(MppiApplicationStatus, DistinguishesAppliedFallbackFromOptimizedResult)
{
  const auto status = makeMppiApplicationStatus(false, true, true, 80U);

  EXPECT_FALSE(status.optimization_succeeded);
  EXPECT_FALSE(status.optimized_trajectory_applied);
  EXPECT_TRUE(status.fallback_applied);
  EXPECT_TRUE(status.output_applied);
}

TEST(MppiApplicationStatus, DoesNotApplyRejectedResultWithoutFallback)
{
  const auto status = makeMppiApplicationStatus(false, true, false, 80U);

  EXPECT_FALSE(status.optimization_succeeded);
  EXPECT_FALSE(status.optimized_trajectory_applied);
  EXPECT_FALSE(status.fallback_applied);
  EXPECT_FALSE(status.output_applied);
}

}  // namespace autoware::mppi_optimizer
