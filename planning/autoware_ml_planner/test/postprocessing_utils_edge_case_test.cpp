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

#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/ml_planner/preprocessing/items/agent.hpp"

#include <Eigen/Dense>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/time.hpp>
#include <tf2/utils.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ml_planner::test
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_planning_msgs::msg::Trajectory;

class PostprocessingUtilsEdgeCaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a simple tracked object for testing
    tracked_object_.object_id = autoware_utils_uuid::generate_uuid();
    tracked_object_.kinematics.pose_with_covariance.pose.position.x = 10.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.y = 5.0;
    tracked_object_.kinematics.pose_with_covariance.pose.position.z = 0.0;
    tracked_object_.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(0.0);
    tracked_object_.existence_probability = 0.9;
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
    classification.probability = 1.0;
    tracked_object_.classification.push_back(classification);
  }

  TrackedObject tracked_object_;
};

// Test edge case: Empty agent data (no neighbors)
TEST_F(PostprocessingUtilsEdgeCaseTest, CreatePredictedObjects_EmptyAgentData)
{
  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 0.0f);

  const std::vector<preprocess::SelectedAgent> selected_agents;
  rclcpp::Time stamp(123, 0);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  const auto agent_poses = postprocess::parse_predictions(prediction, transform);
  constexpr int64_t batch_idx = 0;
  auto result =
    postprocess::create_predicted_objects(agent_poses, selected_agents, stamp, batch_idx);

  EXPECT_EQ(result.objects.size(), 0);
  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_EQ(result.header.stamp, stamp);
}

// Test edge case: More agents in prediction than in history
TEST_F(PostprocessingUtilsEdgeCaseTest, CreatePredictedObjects_MorePredictionsThanSelectedAgents)
{
  // Create prediction data for maximum agents
  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 1.0f);

  // Only one current agent is available for the predicted neighbor slots.
  const std::vector<preprocess::SelectedAgent> selected_agents{
    {tracked_object_.object_id.uuid, tracked_object_, Eigen::Matrix4d::Identity()}};
  rclcpp::Time stamp(123, 0);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  const auto agent_poses = postprocess::parse_predictions(prediction, transform);
  constexpr int64_t batch_idx = 0;
  auto result =
    postprocess::create_predicted_objects(agent_poses, selected_agents, stamp, batch_idx);

  EXPECT_EQ(result.objects.size(), 1);
}

}  // namespace autoware::ml_planner::test
