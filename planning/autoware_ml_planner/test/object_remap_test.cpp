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

#include "autoware/ml_planner/utils/object_remap.hpp"

#include "autoware/ml_planner/preprocessing/items/agent.hpp"

#include <Eigen/Dense>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

namespace autoware::ml_planner::utils::test
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

class ObjectRemapTest : public ::testing::Test
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

  static uint8_t label_of(const TrackedObject & object)
  {
    return object.classification.empty() ? ObjectClassification::UNKNOWN
                                         : object.classification.front().label;
  }

  // Run the fixture object through the preprocessing selection, as the node does after the remap.
  static std::vector<preprocess::SelectedAgent> select(const TrackedObject & object)
  {
    TrackedObjects objects;
    objects.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    objects.objects.push_back(object);
    std::deque<TrackedObjects> messages{objects};

    return preprocess::select_current_agents(
      preprocess::MessageView<TrackedObjects>(messages), rclcpp::Time(0, 0, RCL_ROS_TIME),
      Eigen::Matrix4d::Identity(), 10);
  }

  TrackedObject object_;
};

TEST_F(ObjectRemapTest, UnknownObjectIsRemappedToPedestrian)
{
  set_label(ObjectClassification::UNKNOWN);

  const auto remapped = remap_unsupported_to_pedestrian(object_);

  EXPECT_EQ(label_of(remapped), ObjectClassification::PEDESTRIAN);
  // A BOX shape must keep its real extents.
  EXPECT_EQ(remapped.shape.type, Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(remapped.shape.dimensions.x, 5.0);
  EXPECT_DOUBLE_EQ(remapped.shape.dimensions.y, 2.0);
}

TEST_F(ObjectRemapTest, HazardObjectIsRemappedToPedestrian)
{
  set_label(ObjectClassification::HAZARD);

  EXPECT_EQ(label_of(remap_unsupported_to_pedestrian(object_)), ObjectClassification::PEDESTRIAN);
}

TEST_F(ObjectRemapTest, UnknownPolygonObjectGetsDefaultBox)
{
  set_label(ObjectClassification::UNKNOWN);
  object_.shape.type = Shape::POLYGON;
  object_.shape.footprint.points.emplace_back();

  const auto remapped = remap_unsupported_to_pedestrian(object_);

  EXPECT_EQ(remapped.shape.type, Shape::BOUNDING_BOX);
  EXPECT_DOUBLE_EQ(remapped.shape.dimensions.x, 0.5);
  EXPECT_DOUBLE_EQ(remapped.shape.dimensions.y, 0.5);
  EXPECT_TRUE(remapped.shape.footprint.points.empty());
}

TEST_F(ObjectRemapTest, SupportedObjectIsUntouched)
{
  // Also with a POLYGON shape: the remap must not rescue an object the preprocessing drops for
  // another reason.
  object_.shape.type = Shape::POLYGON;

  const auto remapped = remap_unsupported_to_pedestrian(object_);

  EXPECT_EQ(label_of(remapped), ObjectClassification::CAR);
  EXPECT_EQ(remapped.shape.type, Shape::POLYGON);
}

TEST_F(ObjectRemapTest, EmptyClassificationObjectIsUntouched)
{
  // getHighestProbLabel() returns UNKNOWN for an empty vector; there is no label to rewrite, so
  // the object must stay as it is rather than being silently promoted to PEDESTRIAN.
  object_.classification.clear();

  EXPECT_TRUE(remap_unsupported_to_pedestrian(object_).classification.empty());
}

TEST_F(ObjectRemapTest, UnrecognizedFutureLabelIsRemapped)
{
  // Stands in for a class added to ObjectClassification later: the preprocessing would ignore it,
  // so a remap keyed on a fixed list of labels would drop it silently.
  set_label(200);

  EXPECT_EQ(label_of(remap_unsupported_to_pedestrian(object_)), ObjectClassification::PEDESTRIAN);
}

TEST_F(ObjectRemapTest, DeliberatelyIgnoredLabelsAreNotRemapped)
{
  for (const uint8_t label :
       {ObjectClassification::ANIMAL, ObjectClassification::OVER_DRIVABLE,
        ObjectClassification::UNDER_DRIVABLE}) {
    set_label(label);

    EXPECT_EQ(label_of(remap_unsupported_to_pedestrian(object_)), label)
      << "label " << static_cast<int>(label) << " was remapped";
    EXPECT_FALSE(is_unsupported_obstacle_label(label));
  }
}

TEST_F(ObjectRemapTest, MessageOverloadRemapsEveryObject)
{
  TrackedObjects objects;
  set_label(ObjectClassification::UNKNOWN);
  objects.objects.push_back(object_);
  set_label(ObjectClassification::CAR);
  objects.objects.push_back(object_);

  const auto remapped = remap_unsupported_objects_to_pedestrian(objects);

  ASSERT_EQ(remapped.objects.size(), 2u);
  EXPECT_EQ(label_of(remapped.objects[0]), ObjectClassification::PEDESTRIAN);
  EXPECT_EQ(label_of(remapped.objects[1]), ObjectClassification::CAR);
}

TEST_F(ObjectRemapTest, BatchOverloadKeepsPointersWhenNothingToRemap)
{
  auto supported = std::make_shared<TrackedObjects>();
  supported->objects.push_back(object_);  // CAR
  auto unsupported = std::make_shared<TrackedObjects>();
  set_label(ObjectClassification::UNKNOWN);
  unsupported->objects.push_back(object_);

  const std::vector<std::shared_ptr<const TrackedObjects>> messages{
    supported, nullptr, unsupported};

  const auto remapped = remap_unsupported_objects_to_pedestrian(messages);

  ASSERT_EQ(remapped.size(), 3u);
  EXPECT_EQ(remapped[0], supported);  // passed through, nothing copied
  EXPECT_EQ(remapped[1], nullptr);
  EXPECT_NE(remapped[2], unsupported);
  EXPECT_EQ(label_of(remapped[2]->objects.front()), ObjectClassification::PEDESTRIAN);
  // The input message is never modified in place.
  EXPECT_EQ(label_of(unsupported->objects.front()), ObjectClassification::UNKNOWN);
}

// Guards the label list duplicated in object_remap.cpp against get_model_label() in
// preprocessing/items/agent.cpp: an unsupported object is dropped by the preprocessing, and the
// remapped one must survive it.
TEST_F(ObjectRemapTest, RemappedObjectSurvivesPreprocessingSelection)
{
  set_label(ObjectClassification::UNKNOWN);
  object_.shape.type = Shape::POLYGON;

  EXPECT_TRUE(select(object_).empty());

  const auto agents = select(remap_unsupported_to_pedestrian(object_));

  ASSERT_EQ(agents.size(), 1u);
  const auto labels = preprocess::create_agent_label(agents, 10);
  EXPECT_FLOAT_EQ(labels(0, preprocess::AgentLabel::PEDESTRIAN), 1.0F);
  EXPECT_FLOAT_EQ(labels(0, preprocess::AgentLabel::VEHICLE), 0.0F);
  EXPECT_FLOAT_EQ(labels(0, preprocess::AgentLabel::BICYCLE), 0.0F);
}

TEST_F(ObjectRemapTest, SupportedLabelsAreAllSelectableWithoutRemap)
{
  for (const uint8_t label :
       {ObjectClassification::CAR, ObjectClassification::TRUCK, ObjectClassification::BUS,
        ObjectClassification::MOTORCYCLE, ObjectClassification::TRAILER,
        ObjectClassification::BICYCLE, ObjectClassification::PEDESTRIAN}) {
    set_label(label);

    EXPECT_FALSE(is_unsupported_obstacle_label(label)) << "label " << static_cast<int>(label);
    EXPECT_EQ(select(object_).size(), 1u)
      << "label " << static_cast<int>(label) << " is not selectable, so the supported-label list "
      << "in object_remap.cpp disagrees with get_model_label()";
  }
}

}  // namespace autoware::ml_planner::utils::test
