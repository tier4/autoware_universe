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

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/fixed_time_grid.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>

namespace autoware::diffusion_planner::test
{
namespace
{

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using Buffer = preprocess::FixedTimeGridBuffer<TrackedObjects>;

TrackedObjects::ConstSharedPtr make_snapshot(const double x, const bool present)
{
  auto snapshot = std::make_shared<TrackedObjects>();
  if (!present) {
    return snapshot;
  }

  TrackedObject object;
  object.object_id.uuid[0] = 1;
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  snapshot->objects.push_back(object);
  return snapshot;
}

}  // namespace

TEST(FixedNeighborHistoryTest, ZeroOrderHoldRepeatsTheSelectedSnapshot)
{
  Buffer buffer{10'000};
  buffer.add(0, make_snapshot(0.0, true));
  buffer.add(250, make_snapshot(3.0, true));

  const auto grid = buffer.sample(300, 4, 100);
  ASSERT_EQ(grid.messages.size(), 4U);

  AgentData agent_data;
  for (const auto & snapshot : grid.messages) {
    ASSERT_NE(snapshot, nullptr);
    agent_data.update_histories(*snapshot);
  }

  const auto histories =
    agent_data.transformed_and_trimmed_histories(Eigen::Matrix4d::Identity(), 1);
  ASSERT_EQ(histories.size(), 1U);
  const auto values = histories.front().as_array();
  ASSERT_EQ(values.size(), static_cast<size_t>(INPUT_T_WITH_CURRENT) * AGENT_STATE_DIM);
  const size_t last_four_start = static_cast<size_t>(INPUT_T_WITH_CURRENT - 4) * AGENT_STATE_DIM;
  EXPECT_FLOAT_EQ(values[last_four_start + 0U * AGENT_STATE_DIM], 0.0F);
  EXPECT_FLOAT_EQ(values[last_four_start + 1U * AGENT_STATE_DIM], 0.0F);
  EXPECT_FLOAT_EQ(values[last_four_start + 2U * AGENT_STATE_DIM], 0.0F);
  EXPECT_FLOAT_EQ(values[last_four_start + 3U * AGENT_STATE_DIM], 3.0F);
}

TEST(FixedNeighborHistoryTest, ADetectionGapErasesTheTrackAndReappearanceRepadsIt)
{
  Buffer buffer{10'000};
  buffer.add(0, make_snapshot(0.0, true));
  buffer.add(100, make_snapshot(1.0, true));
  buffer.add(200, make_snapshot(0.0, false));
  buffer.add(300, make_snapshot(3.0, true));

  const auto grid = buffer.sample(300, 4, 100);
  ASSERT_EQ(grid.messages.size(), 4U);

  AgentData agent_data;
  for (const auto & snapshot : grid.messages) {
    ASSERT_NE(snapshot, nullptr);
    agent_data.update_histories(*snapshot);
  }

  const auto histories =
    agent_data.transformed_and_trimmed_histories(Eigen::Matrix4d::Identity(), 1);
  ASSERT_EQ(histories.size(), 1U);
  const auto values = histories.front().as_array();
  ASSERT_EQ(values.size(), static_cast<size_t>(INPUT_T_WITH_CURRENT) * AGENT_STATE_DIM);
  for (size_t i = 0; i < static_cast<size_t>(INPUT_T_WITH_CURRENT - 1) * AGENT_STATE_DIM; ++i) {
    EXPECT_FLOAT_EQ(values[i], 0.0F);
  }
  EXPECT_FLOAT_EQ(values[(static_cast<size_t>(INPUT_T_WITH_CURRENT) - 1U) * AGENT_STATE_DIM], 3.0F);
}

}  // namespace autoware::diffusion_planner::test
