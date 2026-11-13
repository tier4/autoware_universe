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

#include "autoware/mppi_optimizer/tracked_objects_obstacles.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{
geometry_msgs::msg::Pose pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);
  result.orientation = tf2::toMsg(rotation);
  return result;
}

autoware_perception_msgs::msg::PredictedPath path(float confidence, double first_y, double last_y)
{
  autoware_perception_msgs::msg::PredictedPath result;
  result.confidence = confidence;
  result.time_step.nanosec = 200000000U;
  result.path = {pose(1000000.0, first_y), pose(1000002.0, last_y)};
  return result;
}

class PredictedObstacles : public ::testing::Test
{
protected:
  PredictedObjects objects;
  std::vector<float> x, y, yaw, half_length, half_width;
  void SetUp() override
  {
    objects.header.frame_id = "map";
    objects.objects.resize(1);
    auto & object = objects.objects.front();
    object.shape.dimensions.x = 4.0;
    object.shape.dimensions.y = 2.0;
    object.kinematics.initial_pose_with_covariance.pose = pose(1000000.0, 10.0);
  }
  void sample(int steps = 3)
  {
    buildObstacleTrajectoryBuffersFromPredictedObjects(
      objects, 0.1F, steps, x, y, yaw, half_length, half_width, 1000000.0, 0.0);
  }
};

TEST_F(PredictedObstacles, SamplesEveryHighestConfidenceTieAtPostStepTimes)
{
  auto & paths = objects.objects.front().kinematics.predicted_paths;
  paths = {path(0.8F, 10.0, 0.0), path(0.1F, -100.0, -100.0), path(0.8F, -10.0, 0.0)};
  EXPECT_EQ(predictedObjectObstacleCount(objects), 2);
  sample();
  ASSERT_EQ(x.size(), 6U);
  EXPECT_FLOAT_EQ(x[0], 1.0F);  // t=0.1, retaining precision after local translation.
  EXPECT_FLOAT_EQ(x[1], 2.0F);  // t=0.2.
  EXPECT_FLOAT_EQ(x[2], 2.0F);  // Hold endpoint at t=0.3.
  EXPECT_FLOAT_EQ(y[0], 5.0F);
  EXPECT_FLOAT_EQ(y[3], -5.0F);
  EXPECT_NEAR(y[1], 0.0F, 1.0E-5F);
  EXPECT_NEAR(y[4], 0.0F, 1.0E-5F);
  ASSERT_EQ(half_length.size(), 2U);
  EXPECT_FLOAT_EQ(half_length[1], 2.0F);
  EXPECT_FLOAT_EQ(half_width[1], 1.0F);
  std::reverse(paths.begin(), paths.end());
  sample();
  EXPECT_FLOAT_EQ(y[0], -5.0F);
  EXPECT_FLOAT_EQ(y[3], 5.0F);
}

TEST_F(PredictedObstacles, InterpolatesYawAcrossPiOnTheShortArc)
{
  auto hypothesis = path(1.0F, 0.0, 0.0);
  hypothesis.path = {pose(1000000.0, 0.0, 3.0), pose(1000000.0, 0.0, -3.0)};
  objects.objects.front().kinematics.predicted_paths = {hypothesis};
  sample();
  EXPECT_NEAR(std::abs(yaw[0]), M_PI, 1.0E-5);
  EXPECT_NEAR(yaw.back(), -3.0F, 1.0E-5);
}

TEST_F(PredictedObstacles, MissingPathsUseLongitudinalVelocityFallback)
{
  auto & kinematics = objects.objects.front().kinematics;
  kinematics.initial_pose_with_covariance.pose = pose(1000000.0, 10.0, M_PI / 2.0);
  kinematics.initial_twist_with_covariance.twist.linear.x = 2.0;
  sample();
  ASSERT_EQ(x.size(), 3U);
  EXPECT_NEAR(x[0], 0.0F, 1.0E-5);
  EXPECT_NEAR(y[0], 10.2F, 1.0E-5);
  EXPECT_NEAR(y[2], 10.6F, 1.0E-5);
}

TEST_F(PredictedObstacles, CountsHypothesesAndRejectsOverflowWithoutTruncation)
{
  auto & paths = objects.objects.front().kinematics.predicted_paths;
  paths.assign(64, path(1.0F, 0.0, 1.0));
  EXPECT_EQ(predictedObjectObstacleCount(objects), 64);
  sample(1);
  ASSERT_EQ(x.size(), 64U);
  paths.push_back(path(1.0F, 0.0, 2.0));
  EXPECT_THROW(sample(), std::length_error);
  EXPECT_THROW(predictedObjectObstacleCount(objects), std::length_error);
}

TEST_F(PredictedObstacles, RejectsInvalidSelectedPathsAndConfidence)
{
  auto & paths = objects.objects.front().kinematics.predicted_paths;
  paths = {path(1.0F, 0.0, 0.0)};
  paths.front().path.clear();
  EXPECT_THROW(sample(), std::invalid_argument);
  paths = {path(1.0F, 0.0, 0.0)};
  paths.front().time_step.nanosec = 0;
  EXPECT_THROW(sample(), std::invalid_argument);
  paths = {path(1.0F, 0.0, 0.0)};
  paths.front().path.back().position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(sample(), std::invalid_argument);
  paths = {path(std::numeric_limits<float>::quiet_NaN(), 0.0, 0.0)};
  EXPECT_THROW(sample(), std::invalid_argument);
  paths = {path(1.1F, 0.0, 0.0)};
  EXPECT_THROW(sample(), std::invalid_argument);
  // An unused lower-confidence empty path is not substituted for the valid best path.
  paths = {path(1.0F, 0.0, 0.0), path(0.2F, 0.0, 0.0)};
  paths.back().path.clear();
  EXPECT_NO_THROW(sample());
}

TEST_F(PredictedObstacles, AssociatesByUuidAndIncludesCrossingHypothesesInBroadPhase)
{
  TrackedObjects tracked;
  tracked.header = objects.header;
  tracked.objects.resize(2);
  for (std::size_t i = 0; i < tracked.objects.size(); ++i) {
    tracked.objects[i].object_id.uuid[0] = static_cast<uint8_t>(i + 1U);
    tracked.objects[i].kinematics.pose_with_covariance.pose = pose(1000000.0, 10.0);
  }
  auto & prediction = objects.objects.front();
  prediction.object_id = tracked.objects[1].object_id;
  prediction.kinematics.predicted_paths = {path(1.0F, 10.0, 10.0), path(1.0F, 10.0, -10.0)};
  const auto selected = predictionsForTrackedObjects(tracked, objects);
  ASSERT_EQ(selected.objects.size(), 2U);
  EXPECT_TRUE(selected.objects[0].kinematics.predicted_paths.empty());
  EXPECT_EQ(selected.objects[1].kinematics.predicted_paths.size(), 2U);

  autoware_planning_msgs::msg::Trajectory reference;
  reference.points.resize(2);
  reference.points[0].pose = pose(999999.0, 0.0);
  reference.points[1].pose = pose(1000003.0, 0.0);
  TrackedObjects in_range;
  in_range.header = tracked.header;
  std::vector<std::array<std::uint8_t, 16>> reaching_ids;
  includePredictedPathTargets(in_range, tracked, objects, reference, 0.1, &reaching_ids);
  ASSERT_EQ(in_range.objects.size(), 1U);
  EXPECT_EQ(in_range.objects.front().object_id, tracked.objects[1].object_id);
  ASSERT_EQ(reaching_ids.size(), 1U);
  EXPECT_EQ(reaching_ids.front(), tracked.objects[1].object_id.uuid);
  reaching_ids.clear();
  includePredictedPathTargets(in_range, tracked, objects, reference, 0.1, &reaching_ids);
  EXPECT_EQ(in_range.objects.size(), 1U);  // Never duplicate the tracked object for tied paths.
  EXPECT_EQ(reaching_ids.size(), 1U);      // The selector still needs its prediction admission.
  objects.objects.push_back(prediction);
  EXPECT_THROW(predictionsForTrackedObjects(tracked, objects), std::invalid_argument);
  objects.objects.pop_back();
  objects.header.frame_id = "other";
  EXPECT_THROW(predictionsForTrackedObjects(tracked, objects), std::invalid_argument);
}
}  // namespace
}  // namespace autoware::mppi_optimizer
