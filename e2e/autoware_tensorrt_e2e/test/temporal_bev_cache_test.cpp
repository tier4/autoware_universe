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

#include "autoware/tensorrt_e2e/bev_feature/temporal_bev_cache.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::tensorrt_e2e
{

// The history selection is pure logic (no CUDA); the GPU paths are covered by the on-vehicle
// integration, not by unit tests. Stamps are newest-first, relative seconds.

TEST(TemporalBevCacheTest, SelectsSensorRateHistoryWhenIntervalMatchesSensorPeriod)
{
  // The original ResWorld contract: 0.1 s history on a 10 Hz LiDAR.
  const std::vector<double> stamps{0.0, -0.1, -0.2};
  const auto selection = TemporalBevCache::select_history_slots(stamps, 3, 0.1, 0.02);
  EXPECT_EQ(selection, (std::vector<int64_t>{0, 1, 2}));
}

TEST(TemporalBevCacheTest, SelectsStridedHistoryWhenIntervalExceedsSensorPeriod)
{
  // A 0.2 s contract on a 10 Hz LiDAR must pick every second map — the sensor keeps its own
  // cadence, so intermediate maps sit in the window and must be skipped, not rejected.
  const std::vector<double> stamps{0.0, -0.1, -0.2, -0.3, -0.4};
  const auto selection = TemporalBevCache::select_history_slots(stamps, 3, 0.2, 0.02);
  EXPECT_EQ(selection, (std::vector<int64_t>{0, 2, 4}));
}

TEST(TemporalBevCacheTest, PicksTheClosestStampWithinTolerance)
{
  const std::vector<double> stamps{0.0, -0.115, -0.19};
  const auto selection = TemporalBevCache::select_history_slots(stamps, 3, 0.1, 0.02);
  // -0.115 is 0.015 off the -0.1 target (inside 0.02); -0.19 is 0.01 off the -0.2 target.
  EXPECT_EQ(selection, (std::vector<int64_t>{0, 1, 2}));
}

TEST(TemporalBevCacheTest, ReportsHolesInsteadOfMisassigningNeighbours)
{
  // The t-0.2 map was dropped: both neighbours are 0.1 s off target, far outside tolerance.
  // The step must come back unfilled (-1) so ready() waits for the window to refill.
  const std::vector<double> stamps{0.0, -0.1, -0.3, -0.4};
  const auto selection = TemporalBevCache::select_history_slots(stamps, 3, 0.2, 0.02);
  EXPECT_EQ(selection[0], 0);
  EXPECT_EQ(selection[1], -1);
  EXPECT_EQ(selection[2], 3);
}

TEST(TemporalBevCacheTest, StepZeroAlwaysSelectsTheNewestMap)
{
  const std::vector<double> stamps{0.0};
  const auto selection = TemporalBevCache::select_history_slots(stamps, 3, 0.1, 0.02);
  EXPECT_EQ(selection[0], 0);
  EXPECT_EQ(selection[1], -1);
  EXPECT_EQ(selection[2], -1);
}

TEST(TemporalBevCacheTest, EmptyStampsSelectNothing)
{
  const auto selection = TemporalBevCache::select_history_slots({}, 3, 0.1, 0.02);
  EXPECT_EQ(selection, (std::vector<int64_t>{-1, -1, -1}));
}

}  // namespace autoware::tensorrt_e2e
