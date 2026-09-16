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

#include "autoware/ml_planner/preprocessing/items/agent.hpp"

#include <Eigen/Dense>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <vector>

namespace autoware::ml_planner::preprocess::test
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

class AgentRemapTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    object_ = TrackedObject{};
    object_.object_id.uuid[0] = 1;
    object_.kinematics.pose_with_covariance.pose.position.x = 10.0;
    object_.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
    object_.shape.type = Shape::BOUNDING_BOX;
    object_.shape.dimensions.x = 5.0;
    object_.shape.dimensions.y = 2.0;
    object_.shape.dimensions.z = 1.5;
    set_label(ObjectClassification::CAR);
  }

  void set_label(const uint8_t label)
  {
    object_.classification.clear();
    ObjectClassification classification;
    classification.label = label;
    classification.probability = 0.9;
    object_.classification.push_back(classification);
  }

  // Feed the single fixture object through select_current_agents() and return what survived.
  std::vector<SelectedAgent> run(const bool remap)
  {
    messages_.clear();
    TrackedObjects objects;
    objects.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    objects.objects.push_back(object_);
    messages_.push_back(objects);

    return select_current_agents(
      MessageView<TrackedObjects>(messages_), rclcpp::Time(0, 0, RCL_ROS_TIME),
      Eigen::Matrix4d::Identity(), 10, remap);
  }

  TrackedObject object_;
  std::deque<TrackedObjects> messages_;
};

TEST_F(AgentRemapTest, UnknownObjectIsRemappedToPedestrian)
{
  set_label(ObjectClassification::UNKNOWN);

  const auto agents = run(true);

  ASSERT_EQ(agents.size(), 1u);
  EXPECT_EQ(
    agents.front().current_object.classification.front().label, ObjectClassification::PEDESTRIAN);

  const auto labels = create_agent_label(agents, 10);
  EXPECT_FLOAT_EQ(labels(0, AgentLabel::VEHICLE), 0.0F);
  EXPECT_FLOAT_EQ(labels(0, AgentLabel::PEDESTRIAN), 1.0F);
  EXPECT_FLOAT_EQ(labels(0, AgentLabel::BICYCLE), 0.0F);

  // A BOX shape must keep its real extents.
  const auto shapes = create_agent_shape(agents, 10);
  EXPECT_FLOAT_EQ(shapes(0, 0), 2.0F);
  EXPECT_FLOAT_EQ(shapes(0, 1), 5.0F);
}

TEST_F(AgentRemapTest, HazardObjectIsRemappedToPedestrian)
{
  set_label(ObjectClassification::HAZARD);

  const auto agents = run(true);

  ASSERT_EQ(agents.size(), 1u);
  EXPECT_EQ(
    agents.front().current_object.classification.front().label, ObjectClassification::PEDESTRIAN);
}

TEST_F(AgentRemapTest, UnknownPolygonObjectGetsDefaultBox)
{
  set_label(ObjectClassification::UNKNOWN);
  object_.shape.type = Shape::POLYGON;
  object_.shape.footprint.points.emplace_back();

  const auto agents = run(true);

  // Not dropped by the POLYGON filter, because the remap replaced the shape first.
  ASSERT_EQ(agents.size(), 1u);
  const auto & shape = agents.front().current_object.shape;
  EXPECT_EQ(shape.type, Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(shape.dimensions.x, 0.5);
  EXPECT_DOUBLE_EQ(shape.dimensions.y, 0.5);
  EXPECT_TRUE(shape.footprint.points.empty());
}

TEST_F(AgentRemapTest, UnsupportedObjectsAreIgnoredWhenRemapDisabled)
{
  for (const uint8_t label : {ObjectClassification::UNKNOWN, ObjectClassification::HAZARD}) {
    set_label(label);

    // get_model_label() maps both to AgentLabel::IGNORE, so they are dropped entirely.
    EXPECT_TRUE(run(false).empty()) << "label " << static_cast<int>(label) << " was not ignored";
  }
}

TEST_F(AgentRemapTest, SupportedPolygonObjectIsStillSkipped)
{
  // Stays CAR, so the remap must not touch it and the POLYGON filter must still drop it.
  object_.shape.type = Shape::POLYGON;

  EXPECT_TRUE(run(true).empty());
}

TEST_F(AgentRemapTest, EmptyClassificationObjectIsStillIgnored)
{
  // getHighestProbLabel() returns UNKNOWN for an empty vector; there is no label to rewrite, so the
  // object must stay ignored rather than being silently promoted to PEDESTRIAN.
  object_.classification.clear();

  EXPECT_TRUE(run(true).empty());
}

TEST_F(AgentRemapTest, UnrecognizedFutureLabelIsRemapped)
{
  // Stands in for a class added to ObjectClassification later: get_model_label() sends it to
  // IGNORE, so a remap keyed on a fixed list of labels would drop it silently.
  set_label(200);

  const auto agents = run(true);

  ASSERT_EQ(agents.size(), 1u);
  EXPECT_EQ(
    agents.front().current_object.classification.front().label, ObjectClassification::PEDESTRIAN);
}

TEST_F(AgentRemapTest, DeliberatelyIgnoredLabelsAreNotRemapped)
{
  for (const uint8_t label :
       {ObjectClassification::ANIMAL, ObjectClassification::OVER_DRIVABLE,
        ObjectClassification::UNDER_DRIVABLE}) {
    set_label(label);

    EXPECT_TRUE(run(true).empty()) << "label " << static_cast<int>(label) << " was remapped";
  }
}

TEST_F(AgentRemapTest, SupportedObjectIsUnchangedByRemap)
{
  // A CAR must pass through untouched with the flag on.
  const auto agents = run(true);

  ASSERT_EQ(agents.size(), 1u);
  EXPECT_EQ(agents.front().current_object.classification.front().label, ObjectClassification::CAR);

  const auto labels = create_agent_label(agents, 10);
  EXPECT_FLOAT_EQ(labels(0, AgentLabel::VEHICLE), 1.0F);
  EXPECT_FLOAT_EQ(labels(0, AgentLabel::PEDESTRIAN), 0.0F);
}

}  // namespace autoware::ml_planner::preprocess::test
