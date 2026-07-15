// Copyright 2025 TIER IV, Inc.
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

#include <Eigen/Dense>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace autoware::diffusion_planner::test
{

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

class AgentEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a basic tracked object
    tracked_object_.object_id = autoware_utils_uuid::generate_uuid();
    tracked_object_.kinematics.pose_with_covariance.pose.position.x = 1.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.y = 2.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.z = 0.0;
    tracked_object_.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(0.0);

    tracked_object_.kinematics.twist_with_covariance.twist.linear.x = 3.0;
    tracked_object_.kinematics.twist_with_covariance.twist.linear.y = 4.0;

    tracked_object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    tracked_object_.shape.dimensions.x = 5.0;
    tracked_object_.shape.dimensions.y = 2.0;
    tracked_object_.shape.dimensions.z = 1.5;

    tracked_object_.existence_probability = 0.9;

    // Add classification
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 0.9;
    tracked_object_.classification.push_back(classification);
  }

  TrackedObject tracked_object_;
};

TEST_F(AgentEdgeCaseTest, NewlyObservedAgentUsesLeadingZeroPadding)
{
  TrackedObjects objects;
  objects.header.stamp.sec = 1;
  objects.header.stamp.nanosec = 0;
  objects.objects.push_back(tracked_object_);

  AgentData agent_data;
  agent_data.update_histories(objects);

  const auto histories = agent_data.transformed_and_trimmed_histories(
    Eigen::Matrix4d::Identity(), 1);
  ASSERT_EQ(histories.size(), 1U);

  const auto values = histories.front().as_array();
  ASSERT_EQ(values.size(), static_cast<size_t>(INPUT_T_WITH_CURRENT * AGENT_STATE_DIM));
  for (size_t i = 0; i < static_cast<size_t>(INPUT_T_WITH_CURRENT - 1) * AGENT_STATE_DIM; ++i) {
    EXPECT_FLOAT_EQ(values[i], 0.0F) << "padding index " << i;
  }
  EXPECT_FLOAT_EQ(values[(INPUT_T_WITH_CURRENT - 1) * AGENT_STATE_DIM], 1.0F);
  EXPECT_FLOAT_EQ(values[(INPUT_T_WITH_CURRENT - 1) * AGENT_STATE_DIM + 1], 2.0F);
}

TEST_F(AgentEdgeCaseTest, KeepsNonUnknownObjectsForTrainingContract)
{
  TrackedObjects objects;
  objects.header.stamp.sec = 1;
  objects.header.stamp.nanosec = 0;

  auto hazard_object = tracked_object_;
  hazard_object.object_id = autoware_utils_uuid::generate_uuid();
  hazard_object.shape.type = autoware_perception_msgs::msg::Shape::POLYGON;
  hazard_object.classification.front().label =
    autoware_perception_msgs::msg::ObjectClassification::HAZARD;
  objects.objects.push_back(hazard_object);

  auto unknown_object = tracked_object_;
  unknown_object.object_id = autoware_utils_uuid::generate_uuid();
  unknown_object.classification.front().label =
    autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  objects.objects.push_back(unknown_object);

  AgentData agent_data;
  agent_data.update_histories(objects);

  const auto histories = agent_data.transformed_and_trimmed_histories(
    Eigen::Matrix4d::Identity(), 2);
  ASSERT_EQ(histories.size(), 1U);
  EXPECT_EQ(histories.front().get_latest_state().object_id,
            autoware_utils_uuid::to_hex_string(hazard_object.object_id));
}

}  // namespace autoware::diffusion_planner::test
