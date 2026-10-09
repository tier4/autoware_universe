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

#include "autoware/ptv3/ros_utils.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::ptv3
{
namespace test
{
using autoware_perception_msgs::msg::ObjectClassification;

TEST(RosUtilsTest, MapsDetectionClassNames)
{
  EXPECT_EQ(get_classification_type("CAR"), ObjectClassification::CAR);
  EXPECT_EQ(get_classification_type("MOTORBIKE"), ObjectClassification::MOTORCYCLE);
  EXPECT_EQ(get_classification_type("TRAFFIC_CONE"), ObjectClassification::HAZARD);
  EXPECT_EQ(get_classification_type("UNKNOWN_CLASS"), ObjectClassification::UNKNOWN);
}

TEST(RosUtilsTest, ConvertsBox3DToDetectedObject)
{
  constexpr float pi = 3.14159265358979323846F;
  const Box3D box{1, 0.75F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 0.5F * pi, 1.0F, 2.0F};
  autoware_perception_msgs::msg::DetectedObject object;

  box3d_to_detected_object(box, {"CAR", "PEDESTRIAN"}, true, object);

  ASSERT_EQ(object.classification.size(), 1U);
  EXPECT_EQ(object.classification.front().label, ObjectClassification::PEDESTRIAN);
  EXPECT_FLOAT_EQ(object.existence_probability, 0.75F);
  EXPECT_FLOAT_EQ(object.kinematics.pose_with_covariance.pose.position.x, 1.0F);
  EXPECT_FLOAT_EQ(object.shape.dimensions.x, 4.0F);
  EXPECT_TRUE(object.kinematics.has_twist);
  EXPECT_NEAR(object.kinematics.twist_with_covariance.twist.linear.x, 2.0F, 1e-5F);
  EXPECT_NEAR(object.kinematics.twist_with_covariance.twist.linear.y, -1.0F, 1e-5F);
}

TEST(RosUtilsTest, ConvertsUnknownBoxLabelWithoutTwist)
{
  const Box3D box{99, 0.5F, 0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.0F, 1.0F, 1.0F};
  autoware_perception_msgs::msg::DetectedObject object;

  box3d_to_detected_object(box, {"CAR"}, false, object);

  ASSERT_EQ(object.classification.size(), 1U);
  EXPECT_EQ(object.classification.front().label, ObjectClassification::UNKNOWN);
  EXPECT_FALSE(object.kinematics.has_twist);
}

class ClassificationMappingTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static rclcpp::Node::SharedPtr makeNode(const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<rclcpp::Node>("class_mapping_test_node", options);
  }
};

TEST_F(ClassificationMappingTest, ResolvesMappingKeyedByClassName)
{
  const auto node = makeNode(
    {{"segmentation3d.class_mapping.car", "CAR"},
     {"segmentation3d.class_mapping.traffic_cone", "HAZARD"}});

  const auto class_mapping = declare_class_mapping(
    *node, {"car", "traffic_cone"}, rcl_interfaces::msg::ParameterDescriptor{});

  ASSERT_EQ(class_mapping.size(), 2U);
  EXPECT_EQ(class_mapping.at("car"), "CAR");
  EXPECT_EQ(class_mapping.at("traffic_cone"), "HAZARD");
}

TEST_F(ClassificationMappingTest, SkipsMappingEntriesNotInClassNames)
{
  const auto node = makeNode(
    {{"segmentation3d.class_mapping.car", "CAR"},
     {"segmentation3d.class_mapping.vegetation", "VEGETATION"}});

  const auto class_mapping =
    declare_class_mapping(*node, {"car"}, rcl_interfaces::msg::ParameterDescriptor{});

  ASSERT_EQ(class_mapping.size(), 1U);
  EXPECT_EQ(class_mapping.at("car"), "CAR");
  EXPECT_EQ(class_mapping.count("vegetation"), 0U);
}

TEST_F(ClassificationMappingTest, ThrowsWhenClassIsNotMapped)
{
  const auto node = makeNode({{"segmentation3d.class_mapping.car", "CAR"}});

  EXPECT_THROW(
    declare_class_mapping(*node, {"car", "truck"}, rcl_interfaces::msg::ParameterDescriptor{}),
    std::runtime_error);
}

autoware_perception_msgs::msg::DetectedObject makeObject(
  const std::string & class_name, const float yaw, const float width = 2.0F)
{
  const Box3D box{0, 0.9F, 10.0F, 5.0F, 1.0F, 4.0F, width, 1.5F, yaw, 0.0F, 0.0F};
  autoware_perception_msgs::msg::DetectedObject object;
  box3d_to_detected_object(box, {class_name}, false, object);
  return object;
}

constexpr std::array<double, 3> voxel_size{0.12, 0.12, 0.12};

TEST(BboxAdjustmentTest, ShrinksSidesWithoutMovingCenter)
{
  auto object = makeObject("CAR", 0.3F);

  EXPECT_FALSE(adjust_bbox(
    {{ObjectClassification::CAR, {0.0, -0.15, 0.0, 0.0, -0.15, 0.0}}}, voxel_size, object));

  EXPECT_NEAR(object.shape.dimensions.x, 4.0, 1e-6);
  EXPECT_NEAR(object.shape.dimensions.y, 1.7, 1e-6);
  EXPECT_NEAR(object.shape.dimensions.z, 1.5, 1e-6);
  const auto & position = object.kinematics.pose_with_covariance.pose.position;
  EXPECT_NEAR(position.x, 10.0, 1e-6);
  EXPECT_NEAR(position.y, 5.0, 1e-6);
  EXPECT_NEAR(position.z, 1.0, 1e-6);
}

TEST(BboxAdjustmentTest, AsymmetricMarginsMoveCenterInBoxFrame)
{
  constexpr float pi = 3.14159265358979323846F;
  auto object = makeObject("CAR", 0.5F * pi);

  // Extend the front face by 0.5 m and lower the bottom face by 0.2 m.
  EXPECT_FALSE(
    adjust_bbox({{ObjectClassification::CAR, {0.0, 0.0, 0.2, 0.5, 0.0, 0.0}}}, voxel_size, object));

  EXPECT_NEAR(object.shape.dimensions.x, 4.5, 1e-6);
  EXPECT_NEAR(object.shape.dimensions.y, 2.0, 1e-6);
  EXPECT_NEAR(object.shape.dimensions.z, 1.7, 1e-6);
  // The box x axis points along the map y axis at a yaw of pi / 2.
  const auto & position = object.kinematics.pose_with_covariance.pose.position;
  EXPECT_NEAR(position.x, 10.0, 1e-5);
  EXPECT_NEAR(position.y, 5.25, 1e-5);
  EXPECT_NEAR(position.z, 0.9, 1e-5);
}

TEST(BboxAdjustmentTest, LeavesOtherClassesUnchanged)
{
  auto object = makeObject("PEDESTRIAN", 0.0F);

  EXPECT_FALSE(adjust_bbox(
    {{ObjectClassification::CAR, {0.0, -0.15, 0.0, 0.0, -0.15, 0.0}}}, voxel_size, object));

  EXPECT_NEAR(object.shape.dimensions.y, 2.0, 1e-6);
  EXPECT_NEAR(object.kinematics.pose_with_covariance.pose.position.y, 5.0, 1e-6);
}

TEST(BboxAdjustmentTest, TrimsEachFaceBeforeTheCenter)
{
  auto object = makeObject("CAR", 0.0F, 1.0F);

  // The right face may move in by 0.44 m only, half a voxel before the center, while the left face
  // moves in by its full 0.2 m.
  EXPECT_TRUE(adjust_bbox(
    {{ObjectClassification::CAR, {0.0, -1.1, 0.0, 0.0, -0.2, 0.0}}}, voxel_size, object));

  EXPECT_NEAR(object.shape.dimensions.y, 0.36, 1e-6);
  EXPECT_NEAR(object.kinematics.pose_with_covariance.pose.position.y, 5.12, 1e-6);
}

TEST(BboxAdjustmentTest, ShrinksBothFacesToOneVoxelAroundTheCenter)
{
  auto object = makeObject("CAR", 0.0F, 1.0F);

  EXPECT_TRUE(adjust_bbox(
    {{ObjectClassification::CAR, {0.0, -0.6, 0.0, 0.0, -0.6, 0.0}}}, voxel_size, object));

  EXPECT_NEAR(object.shape.dimensions.y, 0.12, 1e-6);
  EXPECT_NEAR(object.kinematics.pose_with_covariance.pose.position.y, 5.0, 1e-6);
}

TEST(BboxAdjustmentTest, DoesNotShrinkDimensionBelowMinimum)
{
  auto object = makeObject("CAR", 0.0F, 0.05F);

  EXPECT_TRUE(adjust_bbox(
    {{ObjectClassification::CAR, {0.0, -0.1, 0.0, 0.2, -0.1, 0.0}}}, voxel_size, object));

  EXPECT_NEAR(object.shape.dimensions.x, 4.2, 1e-6);
  EXPECT_NEAR(object.shape.dimensions.y, 0.05, 1e-6);
  const auto & position = object.kinematics.pose_with_covariance.pose.position;
  EXPECT_NEAR(position.x, 10.1, 1e-6);
  EXPECT_NEAR(position.y, 5.0, 1e-6);
}

class BboxAdjustmentParameterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static rclcpp::Node::SharedPtr makeNode(const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<rclcpp::Node>("bbox_adjustment_test_node", options);
  }

  static std::string param(const std::string & class_name)
  {
    return "detection3d.post_process_params.bbox_adjustment.margins." + class_name;
  }

  /// Margins of every class, zero unless given.
  static std::vector<rclcpp::Parameter> marginOverrides(
    const std::unordered_map<std::string, std::vector<double>> & margins_by_class)
  {
    std::vector<rclcpp::Parameter> overrides;
    for (const auto * class_name :
         {"UNKNOWN", "CAR", "TRUCK", "BUS", "TRAILER", "MOTORCYCLE", "BICYCLE", "PEDESTRIAN",
          "ANIMAL", "HAZARD"}) {
      const auto it = margins_by_class.find(class_name);
      overrides.emplace_back(
        param(class_name), it != margins_by_class.end() ? it->second : std::vector<double>(6, 0.0));
    }
    return overrides;
  }
};

TEST_F(BboxAdjustmentParameterTest, ResolvesMarginsOfEveryClass)
{
  const auto node = makeNode(marginOverrides(
    {{"CAR", {0.0, -0.15, 0.0, 0.0, -0.15, 0.0}}, {"BUS", {0.0, -0.25, 0.0, 0.1, -0.25, 0.0}}}));

  const auto margins_by_label =
    declare_bbox_adjustment(*node, rcl_interfaces::msg::ParameterDescriptor{});

  ASSERT_EQ(margins_by_label.size(), 10U);
  EXPECT_EQ(
    margins_by_label.at(ObjectClassification::CAR),
    (BboxMargins{0.0, -0.15, 0.0, 0.0, -0.15, 0.0}));
  EXPECT_EQ(
    margins_by_label.at(ObjectClassification::BUS),
    (BboxMargins{0.0, -0.25, 0.0, 0.1, -0.25, 0.0}));
  EXPECT_EQ(margins_by_label.at(ObjectClassification::HAZARD), BboxMargins{});
}

TEST_F(BboxAdjustmentParameterTest, ThrowsWhenAClassHasNoMargins)
{
  auto overrides = marginOverrides({});
  overrides.erase(
    std::remove_if(
      overrides.begin(), overrides.end(),
      [](const rclcpp::Parameter & parameter) { return parameter.get_name() == param("HAZARD"); }),
    overrides.end());
  const auto node = makeNode(overrides);

  EXPECT_THROW(
    declare_bbox_adjustment(*node, rcl_interfaces::msg::ParameterDescriptor{}),
    rclcpp::exceptions::UninitializedStaticallyTypedParameterException);
}

TEST_F(BboxAdjustmentParameterTest, ThrowsOnWrongMarginCount)
{
  const auto node = makeNode(marginOverrides({{"CAR", {0.0, -0.15, 0.0}}}));

  EXPECT_THROW(
    declare_bbox_adjustment(*node, rcl_interfaces::msg::ParameterDescriptor{}), std::runtime_error);
}

}  // namespace test
}  // namespace autoware::ptv3
