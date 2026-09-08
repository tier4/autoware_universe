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

#include "autoware/mppi_optimizer/curvature_adaptive_steering_filter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{

TEST(CurvatureAdaptiveSteeringFilter, SuppressesStraightNoiseAndSmoothsTurnEntry)
{
  CurvatureAdaptiveSteeringFilter filter({0.1F, 0.5F, 0.02F});
  std::vector<float> commands{0.01F, -0.01F, 0.2F};

  filter.filter(commands, 0.0F);

  EXPECT_LT(std::abs(commands[0]), 0.01F);
  EXPECT_LT(std::abs(commands[1]), 0.01F);
  EXPECT_GT(commands[2], 0.0F);
  EXPECT_LT(commands[2], 0.2F);
}

TEST(CurvatureAdaptiveSteeringFilter, SeedsFromMeasuredSteering)
{
  CurvatureAdaptiveSteeringFilter filter({0.5F, 0.5F, 0.2F});
  std::vector<float> commands{0.0F};

  filter.filter(commands, 0.2F);

  ASSERT_EQ(commands.size(), 1U);
  EXPECT_NEAR(commands.front(), 0.1F, 1.0E-6F);
}

TEST(CurvatureAdaptiveSteeringFilter, SmoothsTheExitFromATurn)
{
  CurvatureAdaptiveSteeringFilter filter({0.1F, 0.5F, 0.02F});
  std::vector<float> commands{0.0F};

  filter.filter(commands, 0.2F);

  EXPECT_FLOAT_EQ(commands.front(), 0.1F);
}

TEST(CurvatureAdaptiveSteeringFilter, TargetSpikeCannotIncreaseItsOwnAlpha)
{
  CurvatureAdaptiveSteeringFilter filter({0.1F, 0.5F, 0.02F});
  std::vector<float> commands{0.3F};

  filter.filter(commands, 0.0F);

  EXPECT_NEAR(commands.front(), 0.03F, 1.0E-6F);
}

TEST(CurvatureAdaptiveSteeringFilter, SmoothsSignReversalsAboveTurnThreshold)
{
  CurvatureAdaptiveSteeringFilter filter({0.1F, 0.5F, 0.02F});
  std::vector<float> commands{-0.03F};

  filter.filter(commands, 0.03F);

  EXPECT_NEAR(commands.front(), 0.0F, 1.0E-6F);
}

TEST(CurvatureAdaptiveSteeringFilter, PreservesShiftedCommandAndFiltersTailFromIt)
{
  CurvatureAdaptiveSteeringFilter filter({0.1F, 0.5F, 0.02F});
  std::vector<float> commands{0.0F, 0.01F};
  filter.filter(commands, 0.0F);
  const float already_filtered = commands[1];
  ASSERT_NEAR(already_filtered, 0.001F, 1.0E-7F);

  commands = {already_filtered, 0.01F};
  filter.filter(commands, 0.0F, true);

  EXPECT_FLOAT_EQ(commands.front(), already_filtered);
  EXPECT_GT(commands[1], already_filtered);
  EXPECT_LT(commands[1], 0.01F);

  // Persist the issued first command, not the filtered tail.
  commands = {already_filtered};
  filter.filter(commands, 0.0F);
  EXPECT_FLOAT_EQ(commands.front(), already_filtered);
}

TEST(CurvatureAdaptiveSteeringFilter, ColdStartDoesNotBypassFilteringOfShiftedCommand)
{
  CurvatureAdaptiveSteeringFilter filter({0.1F, 0.5F, 0.02F});
  std::vector<float> commands{0.03F};

  filter.filter(commands, 0.0F, true);

  EXPECT_NEAR(commands.front(), 0.003F, 1.0E-7F);
}

TEST(CurvatureAdaptiveSteeringFilter, DiscardedCandidateDoesNotAffectFallbackOrResumedFiltering)
{
  CurvatureAdaptiveSteeringFilter filter({0.5F, 0.5F, 0.02F});
  std::vector<float> commands{0.02F};
  filter.filter(commands, 0.0F);

  auto candidate = filter;
  commands = {0.3F};
  candidate.filter(commands, 0.0F);

  // A limited fallback is filtered from the pre-candidate state.
  auto fallback = filter;
  commands = {0.0F};
  fallback.filter(commands, 0.0F);
  EXPECT_NEAR(commands.front(), 0.005F, 1.0E-7F);

  // An unfiltered fallback invalidates history. Even a shifted nominal must then be re-seeded
  // from measurement, rather than trusting the discarded candidate or the old accepted command.
  filter.reset();
  commands = {0.0F};
  filter.filter(commands, -0.02F, true);
  EXPECT_NEAR(commands.front(), -0.01F, 1.0E-7F);
}

TEST(CurvatureAdaptiveSteeringFilter, NonFiniteShiftedCommandCannotBypassSanitization)
{
  CurvatureAdaptiveSteeringFilter filter({0.5F, 0.5F, 0.02F});
  std::vector<float> commands{0.02F};
  filter.filter(commands, 0.0F);
  commands = {std::numeric_limits<float>::infinity()};

  filter.filter(commands, 0.0F, true);

  EXPECT_NEAR(commands.front(), 0.01F, 1.0E-7F);
}

TEST(CurvatureAdaptiveSteeringFilter, PersistsOnlyTheAppliedFirstCommand)
{
  CurvatureAdaptiveSteeringFilter filter({0.5F, 0.5F, 0.2F});
  std::vector<float> first_horizon{1.0F, 1.0F};
  filter.filter(first_horizon, 0.0F);
  ASSERT_EQ(first_horizon.size(), 2U);
  EXPECT_FLOAT_EQ(first_horizon[0], 0.5F);
  EXPECT_FLOAT_EQ(first_horizon[1], 0.75F);

  std::vector<float> next_horizon{1.0F};
  filter.filter(next_horizon, -1.0F);

  EXPECT_FLOAT_EQ(next_horizon.front(), 0.75F);
}

TEST(CurvatureAdaptiveSteeringFilter, ResetUsesTheNextMeasurement)
{
  CurvatureAdaptiveSteeringFilter filter({0.5F, 0.5F, 0.2F});
  std::vector<float> commands{1.0F};
  filter.filter(commands, 0.0F);

  filter.reset();
  commands.front() = 0.0F;
  filter.filter(commands, -0.2F);

  EXPECT_NEAR(commands.front(), -0.1F, 1.0E-6F);
}

TEST(CurvatureAdaptiveSteeringFilter, HoldsPreviousCommandForNonFiniteTargets)
{
  CurvatureAdaptiveSteeringFilter filter({0.5F, 0.5F, 0.2F});
  std::vector<float> commands{std::numeric_limits<float>::quiet_NaN()};

  filter.filter(commands, 0.2F);

  EXPECT_FLOAT_EQ(commands.front(), 0.2F);
}

TEST(CurvatureAdaptiveSteeringFilter, RejectsInvalidParameters)
{
  CurvatureAdaptiveSteeringFilter filter;
  EXPECT_THROW(filter.setParams({0.8F, 0.2F, 0.1F}), std::invalid_argument);
  EXPECT_THROW(filter.setParams({0.1F, 1.0F, 0.0F}), std::invalid_argument);
}

}  // namespace
}  // namespace autoware::mppi_optimizer
