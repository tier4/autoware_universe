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

#include "autoware/mppi_optimizer/trajectory_mppi_optimizer.hpp"

#include <gtest/gtest.h>

namespace autoware::mppi_optimizer::plugin
{
TEST(TrajectoryMppiOptimizer, SecondaryCandidateBypassesInitializationAndPreservesPoints)
{
  TrajectoryMppiOptimizer optimizer;
  autoware::trajectory_modifier::TrajectoryModifierData data;
  data.candidate_index = 1U;
  TrajectoryPoints points(2);
  points[0].pose.position.x = 42.0;
  points[1].longitudinal_velocity_mps = 1.5F;
  const auto original = points;
  // No node/context/parameter listener: touching primary-cycle state would fail this call.
  EXPECT_EQ(
    optimizer.process(points, data),
    autoware::trajectory_modifier::plugin::ProcessingResult::Unchanged);
  EXPECT_EQ(points, original);
  data.candidate_index = 9U;
  EXPECT_EQ(
    optimizer.process(points, data),
    autoware::trajectory_modifier::plugin::ProcessingResult::Unchanged);
  EXPECT_EQ(points, original);
}
}  // namespace autoware::mppi_optimizer::plugin
