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
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"

#include <Eigen/Dense>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/time.hpp>
#include <tf2/utils.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::test
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
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
  }

  TrackedObject tracked_object_;
};

// Test edge case: Empty agent data (no neighbors)
TEST_F(PostprocessingUtilsEdgeCaseTest, CreatePredictedObjects_EmptyAgentData)
{
  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 0.0f);

  TrackedObjects empty_objects;
  empty_objects.header.stamp = rclcpp::Time(0);

  AgentData agent_data;
  agent_data.update_histories(empty_objects, false);
  rclcpp::Time stamp(123, 0);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  const auto agent_poses = postprocess::parse_predictions(prediction, transform);
  constexpr int64_t batch_idx = 0;
  auto result = postprocess::create_predicted_objects(
    agent_poses,
    agent_data.transformed_and_trimmed_histories(
      Eigen::Matrix4d::Identity(), MAX_NUM_NEIGHBORS, MAX_NUM_NEIGHBORS, MAX_NUM_NEIGHBORS,
      MAX_NUM_NEIGHBORS),
    stamp, batch_idx);

  EXPECT_EQ(result.objects.size(), 0);
  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_EQ(result.header.stamp, stamp);
}

// Test edge case: More agents in prediction than in history
TEST_F(PostprocessingUtilsEdgeCaseTest, CreatePredictedObjects_MorePredictionsThanHistory)
{
  // Create prediction data for maximum agents
  std::vector<float> prediction(
    OUTPUT_SHAPE[0] * OUTPUT_SHAPE[1] * OUTPUT_SHAPE[2] * OUTPUT_SHAPE[3], 1.0f);

  // Create only 2 tracked objects (same ID)
  TrackedObjects objects;
  objects.header.stamp = rclcpp::Time(0);
  objects.objects.push_back(tracked_object_);
  objects.objects.push_back(tracked_object_);

  AgentData agent_data;
  agent_data.update_histories(objects, false);
  rclcpp::Time stamp(123, 0);
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  const auto agent_poses = postprocess::parse_predictions(prediction, transform);
  constexpr int64_t batch_idx = 0;
  auto result = postprocess::create_predicted_objects(
    agent_poses,
    agent_data.transformed_and_trimmed_histories(
      Eigen::Matrix4d::Identity(), MAX_NUM_NEIGHBORS, MAX_NUM_NEIGHBORS, MAX_NUM_NEIGHBORS,
      MAX_NUM_NEIGHBORS),
    stamp, batch_idx);

  EXPECT_EQ(result.objects.size(), 1);
}

// Test: per-class caps in transformed_and_trimmed_histories
TEST_F(PostprocessingUtilsEdgeCaseTest, TransformedAndTrimmedHistories_PerClassCaps)
{
  // Arrange: 10 VEHICLE + 10 PEDESTRIAN + 10 BICYCLE objects around ego, each with a
  // unique id and an increasing distance so the sort order is deterministic.
  auto make_object = [](double x, double y, uint8_t label) {
    TrackedObject obj;
    obj.object_id = autoware_utils_uuid::generate_uuid();
    obj.kinematics.pose_with_covariance.pose.position.x = x;
    obj.kinematics.pose_with_covariance.pose.position.y = y;
    obj.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::create_quaternion_from_yaw(0.0);
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = label;
    classification.probability = 1.0;
    obj.classification.push_back(classification);
    obj.existence_probability = 0.9;
    return obj;
  };

  TrackedObjects objects;
  objects.header.stamp = rclcpp::Time(0);
  for (int i = 0; i < 10; ++i) {
    const double d = static_cast<double>(i + 1);
    objects.objects.push_back(
      make_object(d, 0.0, autoware_perception_msgs::msg::ObjectClassification::CAR));
    objects.objects.push_back(
      make_object(0.0, d, autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN));
    objects.objects.push_back(
      make_object(-d, 0.0, autoware_perception_msgs::msg::ObjectClassification::BICYCLE));
  }

  AgentData agent_data;
  agent_data.update_histories(objects, false);

  const size_t max_num_vehicle = 3;
  const size_t max_num_pedestrian = 4;
  const size_t max_num_bicycle = 2;

  // Act
  const auto histories = agent_data.transformed_and_trimmed_histories(
    Eigen::Matrix4d::Identity(), MAX_NUM_NEIGHBORS, max_num_vehicle, max_num_pedestrian,
    max_num_bicycle);

  // Assert: each class is capped to its limit and the total matches the sum.
  size_t num_vehicle = 0;
  size_t num_pedestrian = 0;
  size_t num_bicycle = 0;
  for (const auto & history : histories) {
    const auto label = history.get_latest_state().label;
    if (label == AgentLabel::VEHICLE) {
      ++num_vehicle;
    } else if (label == AgentLabel::PEDESTRIAN) {
      ++num_pedestrian;
    } else if (label == AgentLabel::BICYCLE) {
      ++num_bicycle;
    }
  }
  EXPECT_EQ(num_vehicle, max_num_vehicle);
  EXPECT_EQ(num_pedestrian, max_num_pedestrian);
  EXPECT_EQ(num_bicycle, max_num_bicycle);
  EXPECT_EQ(histories.size(), max_num_vehicle + max_num_pedestrian + max_num_bicycle);
}

}  // namespace autoware::diffusion_planner::test
