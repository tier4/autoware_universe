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

// Guards the silent failure mode of widening AGENT_STATE_DIM: leaving as_array()'s initializer list
// short is legal aggregate initialization, so the extra feature would just read back as zero.
TEST_F(AgentEdgeCaseTest, AgentStateArrayWidthMatchesNeighborTensor)
{
  TrackedObjects objects;
  objects.header.stamp.sec = 100;
  objects.objects.push_back(tracked_object_);

  AgentData agent_data;
  agent_data.update_histories(objects);
  const auto histories =
    agent_data.transformed_and_trimmed_histories(Eigen::Matrix4d::Identity(), 10);

  ASSERT_EQ(histories.size(), 1U);
  const auto array = histories.front().get_latest_state().as_array();
  EXPECT_EQ(array.size(), AGENT_STATE_DIM);
  EXPECT_EQ(array.size(), static_cast<size_t>(NEIGHBOR_SHAPE[3]));
  // A CAR sets exactly one one-hot entry, and the UNKNOWN slot stays zero: no classification is
  // mapped to AgentLabel::UNKNOWN, so the model never sees an unknown neighbor on this branch.
  EXPECT_FLOAT_EQ(array[8], 1.0F);
  EXPECT_FLOAT_EQ(array[9], 0.0F);
  EXPECT_FLOAT_EQ(array[10], 0.0F);
  EXPECT_FLOAT_EQ(array[11], 0.0F);
}

}  // namespace autoware::diffusion_planner::test
