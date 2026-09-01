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

namespace autoware::tensorrt_e2e
{

// The interval contract check is pure logic (no CUDA); the GPU paths are covered by the
// on-vehicle integration, not by unit tests.
TEST(TemporalBevCacheTest, IntervalContract)
{
  constexpr double interval = 0.1;
  constexpr double tolerance = 0.02;

  EXPECT_TRUE(TemporalBevCache::is_consecutive(0.1, interval, tolerance));
  EXPECT_TRUE(TemporalBevCache::is_consecutive(0.081, interval, tolerance));
  EXPECT_TRUE(TemporalBevCache::is_consecutive(0.119, interval, tolerance));

  // A dropped frame (0.2 s gap) or a burst (0.05 s) violates the ResWorld temporal contract.
  EXPECT_FALSE(TemporalBevCache::is_consecutive(0.2, interval, tolerance));
  EXPECT_FALSE(TemporalBevCache::is_consecutive(0.05, interval, tolerance));
  EXPECT_FALSE(TemporalBevCache::is_consecutive(0.0, interval, tolerance));
}

}  // namespace autoware::tensorrt_e2e
