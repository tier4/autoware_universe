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

#include "autoware/diffusion_planner/preprocessing/fixed_time_grid.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <memory>

namespace autoware::diffusion_planner::test
{
namespace
{

struct TestMessage
{
  int id{0};
  int64_t header_time_ns{0};
};

using Buffer = preprocess::FixedTimeGridBuffer<TestMessage>;

Buffer::MessagePtr make_message(const int id)
{
  return std::make_shared<const TestMessage>(TestMessage{id, 0});
}

}  // namespace

TEST(FixedTimeGridTest, UsesLatestAtOrBeforeEachFixedTick)
{
  Buffer buffer{10'000'000'000LL};
  buffer.add(20, make_message(1));
  buffer.add(110, make_message(2));
  buffer.add(240, make_message(3));

  const auto sample = buffer.sample(300, 4, 100);

  ASSERT_EQ(sample.messages.size(), 4U);
  ASSERT_EQ(sample.selected_time_ns.size(), 4U);
  EXPECT_EQ(sample.messages[0], nullptr);
  EXPECT_EQ(sample.selected_time_ns[0], std::nullopt);
  ASSERT_NE(sample.messages[1], nullptr);
  ASSERT_NE(sample.messages[2], nullptr);
  ASSERT_NE(sample.messages[3], nullptr);
  EXPECT_EQ(sample.messages[1]->id, 1);
  EXPECT_EQ(sample.messages[2]->id, 2);
  EXPECT_EQ(sample.messages[3]->id, 3);
  EXPECT_EQ(sample.selected_time_ns[1], std::optional<int64_t>{20});
  EXPECT_EQ(sample.selected_time_ns[2], std::optional<int64_t>{110});
  EXPECT_EQ(sample.selected_time_ns[3], std::optional<int64_t>{240});
}

TEST(FixedTimeGridTest, NeverSelectsAFutureMessage)
{
  Buffer buffer{10'000'000'000LL};
  buffer.add(101, make_message(1));

  const auto before_message = buffer.sample(100, 1, 100);
  EXPECT_EQ(before_message.messages.front(), nullptr);

  const auto at_message = buffer.sample(101, 1, 100);
  ASSERT_NE(at_message.messages.front(), nullptr);
  EXPECT_EQ(at_message.messages.front()->id, 1);
}

TEST(FixedTimeGridTest, EqualTimestampsUseTheLastMessageInBagOrder)
{
  Buffer buffer{10'000'000'000LL};
  buffer.add(100, make_message(1));
  buffer.add(100, make_message(2));

  const auto sample = buffer.sample(100, 1, 100);

  ASSERT_NE(sample.messages.front(), nullptr);
  EXPECT_EQ(sample.messages.front()->id, 2);
}

TEST(FixedTimeGridTest, SelectionUsesTheSelectionClockNotThePayloadHeader)
{
  Buffer buffer{10'000'000'000LL};
  auto message = std::make_shared<const TestMessage>(TestMessage{1, 10});
  buffer.add(100, message);

  const auto sample = buffer.sample(100, 1, 100);

  ASSERT_NE(sample.messages.front(), nullptr);
  EXPECT_EQ(sample.messages.front()->id, 1);
  EXPECT_EQ(sample.messages.front()->header_time_ns, 10);
  EXPECT_EQ(sample.selected_time_ns.front(), std::optional<int64_t>{100});
}

TEST(FixedTimeGridTest, OutOfOrderArrivalIsSortedBySelectionTime)
{
  Buffer buffer{10'000'000'000LL};
  buffer.add(200, make_message(2));
  buffer.add(100, make_message(1));

  const auto sample = buffer.sample(250, 2, 100);

  ASSERT_NE(sample.messages[0], nullptr);
  ASSERT_NE(sample.messages[1], nullptr);
  EXPECT_EQ(sample.messages[0]->id, 1);
  EXPECT_EQ(sample.messages[1]->id, 2);
}

TEST(FixedTimeGridTest, RetentionKeepsTheBoundaryHoldMessage)
{
  Buffer buffer{300};
  buffer.add(0, make_message(0));
  buffer.add(100, make_message(1));
  buffer.add(200, make_message(2));
  buffer.add(400, make_message(4));

  // The message immediately before the nominal retention boundary must survive so a target
  // exactly at the boundary can still perform ZOH.
  EXPECT_EQ(buffer.latest_at_or_before(100)->id, 1);
  EXPECT_EQ(buffer.latest_at_or_before(250)->id, 2);
  EXPECT_EQ(buffer.latest_at_or_before(400)->id, 4);
}

TEST(FixedTimeGridTest, InvalidArgumentsAreRejected)
{
  EXPECT_THROW(Buffer{-1}, std::invalid_argument);

  Buffer buffer{100};
  EXPECT_THROW((void)buffer.sample(0, 1, 0), std::invalid_argument);
  EXPECT_TRUE(buffer.sample(0, 0, 100).messages.empty());
}

TEST(FixedTimeGridTest, ClearRemovesOldSnapshots)
{
  Buffer buffer{10'000'000'000LL};
  buffer.add(100, make_message(1));
  buffer.clear();

  EXPECT_EQ(buffer.size(), 0U);
  EXPECT_EQ(buffer.latest_at_or_before(100), nullptr);
  EXPECT_EQ(buffer.sample(100, 1, 100).messages.front(), nullptr);
}

}  // namespace autoware::diffusion_planner::test
