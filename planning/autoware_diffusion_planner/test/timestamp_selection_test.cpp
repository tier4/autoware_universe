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

#include "autoware/diffusion_planner/utils/timestamp_selection.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace autoware::diffusion_planner::test
{

using utils::select_timestamp;

constexpr int64_t kReference = 1'000'000'000LL;
constexpr int64_t kDomainTolerance = 10'000LL;

TEST(TimestampSelectionTest, PrefersInDomainReceiveTime)
{
  const auto selection = select_timestamp(
    kReference + 100, kReference - 100, kReference - 200, kReference, kDomainTolerance);
  EXPECT_EQ(selection.time_ns, kReference + 100);
  EXPECT_FALSE(selection.used_header_fallback);
}

TEST(TimestampSelectionTest, UsesInDomainSourceWhenReceiveTimeIsInAnotherDomain)
{
  const auto selection = select_timestamp(
    kReference + 1'000'000'000'000LL, kReference + 200, kReference - 200, kReference,
    kDomainTolerance);
  EXPECT_EQ(selection.time_ns, kReference + 200);
  EXPECT_FALSE(selection.used_header_fallback);
}

TEST(TimestampSelectionTest, FallsBackToHeaderWhenTransportClocksAreUnavailable)
{
  const auto selection = select_timestamp(
    kReference + 1'000'000'000'000LL, kReference + 2'000'000'000'000LL, kReference - 200,
    kReference, kDomainTolerance);
  EXPECT_EQ(selection.time_ns, kReference - 200);
  EXPECT_TRUE(selection.used_header_fallback);
}

TEST(TimestampSelectionTest, KeepsAnOutOfDomainTransportTimestampForStaleRejection)
{
  const auto selection = select_timestamp(
    kReference + 1'000'000'000'000LL, 0, 0, kReference, kDomainTolerance);
  EXPECT_EQ(selection.time_ns, kReference + 1'000'000'000'000LL);
  EXPECT_FALSE(selection.used_header_fallback);
}

TEST(TimestampSelectionTest, RejectsMessagesWithoutAnyTimestamp)
{
  EXPECT_THROW(select_timestamp(0, 0, 0, kReference, kDomainTolerance), std::runtime_error);
}

}  // namespace autoware::diffusion_planner::test
