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

#include "autoware/ml_planner/constants.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::ml_planner::utils
{
namespace
{
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

// Must stay in sync with get_model_label() in src/preprocessing/items/agent.cpp, which decides
// what the model can represent. object_remap_test.cpp checks the two agree through the public
// select_current_agents() API.
bool is_supported_label(const uint8_t label)
{
  switch (label) {
    case ObjectClassification::CAR:
    case ObjectClassification::TRUCK:
    case ObjectClassification::BUS:
    case ObjectClassification::MOTORCYCLE:
    case ObjectClassification::TRAILER:
    case ObjectClassification::BICYCLE:
    case ObjectClassification::PEDESTRIAN:
      return true;
    default:
      return false;
  }
}

bool needs_remap(const TrackedObject & object)
{
  if (object.classification.empty()) {
    return false;
  }
  return is_unsupported_obstacle_label(
    autoware::object_recognition_utils::getHighestProbLabel(object.classification));
}

}  // namespace

bool is_unsupported_obstacle_label(const uint8_t label)
{
  switch (label) {
    case ObjectClassification::ANIMAL:
    case ObjectClassification::OVER_DRIVABLE:
    case ObjectClassification::UNDER_DRIVABLE:
      return false;
    default:
      return !is_supported_label(label);
  }
}

TrackedObject remap_unsupported_to_pedestrian(const TrackedObject & object)
{
  // An empty classification also yields UNKNOWN from getHighestProbLabel(), but such an object
  // carries no label to rewrite, so leave it alone rather than silently promoting it.
  if (!needs_remap(object)) {
    return object;
  }

  const uint8_t highest_prob_label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);

  TrackedObject remapped = object;
  for (auto & classification : remapped.classification) {
    if (is_unsupported_obstacle_label(classification.label)) {
      classification.label = ObjectClassification::PEDESTRIAN;
    }
  }

  if (remapped.shape.type != Shape::BOUNDING_BOX) {
    static rclcpp::Clock clock{RCL_ROS_TIME};
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("ml_planner"), clock, constants::LOG_THROTTLE_INTERVAL_MS,
      "Unsupported-class object %s (label=%u) has a non-BOX shape (type=%u). Replacing it with a "
      "0.5 m bounding box.",
      autoware_utils_uuid::to_hex_string(remapped.object_id).c_str(), highest_prob_label,
      remapped.shape.type);
    remapped.shape.type = Shape::BOUNDING_BOX;
    remapped.shape.footprint.points.clear();
    remapped.shape.dimensions.x = 0.5;
    remapped.shape.dimensions.y = 0.5;
    remapped.shape.dimensions.z = 0.5;
  }
  return remapped;
}

TrackedObjects remap_unsupported_objects_to_pedestrian(const TrackedObjects & objects)
{
  TrackedObjects remapped = objects;
  for (auto & object : remapped.objects) {
    object = remap_unsupported_to_pedestrian(object);
  }
  return remapped;
}

std::vector<std::shared_ptr<const TrackedObjects>> remap_unsupported_objects_to_pedestrian(
  const std::vector<std::shared_ptr<const TrackedObjects>> & messages)
{
  std::vector<std::shared_ptr<const TrackedObjects>> result;
  result.reserve(messages.size());
  for (const auto & message : messages) {
    if (!message) {
      result.push_back(message);
      continue;
    }
    const bool any_remap = std::any_of(
      message->objects.begin(), message->objects.end(),
      [](const TrackedObject & object) { return needs_remap(object); });
    if (!any_remap) {
      result.push_back(message);
      continue;
    }
    result.push_back(
      std::make_shared<const TrackedObjects>(remap_unsupported_objects_to_pedestrian(*message)));
  }
  return result;
}

}  // namespace autoware::ml_planner::utils
