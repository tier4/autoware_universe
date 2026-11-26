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

#include "autoware/ml_planner/preprocessing/scene_input_builder.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/time.hpp>
#include <t4scene_generation/generator.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
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

  // Exercise the same ROS message ingestion and T4Scene feature conversion as the node.
  static preprocess::SceneInputBuilderResult build_input(const TrackedObject & object)
  {
    const auto config =
      std::filesystem::path{ament_index_cpp::get_package_share_directory("autoware_ml_planner")} /
      "config/t4scene_online.yaml";
    t4scene::ros2::generation::SceneBuilder builder(config);
    constexpr int64_t anchor_ns = 4'000'000'000;

    const auto boundary = [](const double y) {
      return lanelet::LineString3d(
        lanelet::utils::getId(), {lanelet::Point3d(lanelet::utils::getId(), 0.0, y, 0.0),
                                  lanelet::Point3d(lanelet::utils::getId(), 30.0, y, 0.0)});
    };
    auto lane = lanelet::Lanelet(lanelet::utils::getId(), boundary(2.0), boundary(-2.0));
    lane.setAttribute("subtype", "road");
    auto map = std::make_shared<lanelet::LaneletMap>();
    map->add(lane);

    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "map";
    odometry.header.stamp = rclcpp::Time(anchor_ns);
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.orientation.w = 1.0;
    autoware_planning_msgs::msg::LaneletRoute route;
    route.header = odometry.header;
    route.start_pose.orientation.w = 1.0;
    route.goal_pose.orientation.w = 1.0;
    route.goal_pose.position.x = 20.0;
    auto & segment = route.segments.emplace_back();
    segment.preferred_primitive.id = lane.id();
    segment.primitives.push_back(segment.preferred_primitive);
    TrackedObjects objects;
    objects.header = odometry.header;
    objects.objects.push_back(object);
    if (
      !builder.ingest(odometry, anchor_ns) || !builder.ingest(route, anchor_ns) ||
      !builder.ingest(objects, anchor_ns)) {
      return tl::unexpected(std::string("T4Scene rejected a test observation"));
    }

    auto scene = builder.build_window(anchor_ns, {"object-remap"}, anchor_ns);
    constexpr t4scene::Transform3D identity{1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    scene.global_origin_3d = identity;
    scene.map.emplace();
    scene.map->global_origin_3d = identity;
    scene.map->lanelet_map = map;
    scene.ego->shape.emplace();
    scene.ego->shape->base_link_to_front = 3.5;
    scene.ego->shape->length = 4.5;
    scene.ego->shape->width = 2.0;
    scene.ego->shape->wheel_base = 2.8;
    return preprocess::build_scene_input(scene, *preprocess::build_map_context(map));
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

TEST_F(ObjectRemapTest, RemappedObjectSurvivesPreprocessingSelection)
{
  set_label(ObjectClassification::UNKNOWN);
  object_.shape.type = Shape::POLYGON;

  const auto original = build_input(object_);
  ASSERT_TRUE(original.has_value()) << original.error();
  EXPECT_TRUE(original->selected_track_ids.empty());

  const auto remapped = build_input(remap_unsupported_to_pedestrian(object_));
  ASSERT_TRUE(remapped.has_value()) << remapped.error();
  ASSERT_EQ(remapped->selected_track_ids.size(), 1u);
  const auto & labels = remapped->tensors.at("agent_label");
  EXPECT_FLOAT_EQ(labels(0, 0), 0.0F);  // Vehicle
  EXPECT_FLOAT_EQ(labels(0, 1), 1.0F);  // Pedestrian
  EXPECT_FLOAT_EQ(labels(0, 2), 0.0F);  // Bicycle
}

TEST_F(ObjectRemapTest, SupportedLabelsAreAllSelectableWithoutRemap)
{
  for (const uint8_t label :
       {ObjectClassification::CAR, ObjectClassification::TRUCK, ObjectClassification::BUS,
        ObjectClassification::MOTORCYCLE, ObjectClassification::TRAILER,
        ObjectClassification::BICYCLE, ObjectClassification::PEDESTRIAN}) {
    set_label(label);

    EXPECT_FALSE(is_unsupported_obstacle_label(label)) << "label " << static_cast<int>(label);
    const auto result = build_input(object_);
    ASSERT_TRUE(result.has_value()) << result.error();
    EXPECT_EQ(result->selected_track_ids.size(), 1u)
      << "label " << static_cast<int>(label) << " is not selectable, so the supported-label list "
      << "in object_remap.cpp disagrees with T4Scene input preprocessing";
  }
}

}  // namespace autoware::ml_planner::utils::test
