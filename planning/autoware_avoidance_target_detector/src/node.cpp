// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/node.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <algorithm>

namespace autoware::avoidance_target_detector
{

/**
 * @brief Construct the avoidance target detector node.
 * @param node_options Node options for component loading.
 */
AvoidanceTargetDetectorNode::AvoidanceTargetDetectorNode(const rclcpp::NodeOptions & node_options)
: Node{"avoidance_target_detector", node_options},
  sub_objects_{create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&AvoidanceTargetDetectorNode::on_objects, this, std::placeholders::_1))},
  pub_avoidance_targets_{create_publisher<PredictedObjects>("~/output/avoidance_targets", 1)}
{
}

/**
 * @brief Callback for incoming predicted objects.
 * @param msg Predicted objects message.
 */
void AvoidanceTargetDetectorNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  const auto current_time = get_clock()->now();

  trajectory_ = sub_trajectory_.take_data();
  const Trajectory trajectory_msg = trajectory_ ? *trajectory_ : Trajectory{};

  for (const auto & object : msg->objects) {
    const auto object_id_str = autoware_utils_uuid::to_hex_string(object.object_id);
    if (object_filters_.find(object_id_str) == object_filters_.end()) {
      object_filters_.emplace(object_id_str, FilterManager(object, current_time));
    }
  }

  for (const auto & object : msg->objects) {
    const auto object_id_str = autoware_utils_uuid::to_hex_string(object.object_id);
    const auto it = object_filters_.find(object_id_str);
    if (it != object_filters_.end()) {
      it->second.observe_and_update_all(current_time, object, trajectory_msg);
    }
  }

  for (auto it = object_filters_.begin(); it != object_filters_.end();) {
    if (it->second.is_stale(current_time)) {
      it = object_filters_.erase(it);
    } else {
      ++it;
    }
  }

  for (auto & [object_id_str, filter_manager] : object_filters_) {
    if (filter_manager.get_debug_log().empty()) {
      continue;
    }
    RCLCPP_INFO(
      get_logger(), "Object ID: %s, Debug Log: %s", object_id_str.c_str(),
      filter_manager.get_debug_log().c_str());
    filter_manager.clear_debug_log();
  }

  PredictedObjects avoidance_targets = *msg;
  avoidance_targets.objects.erase(
    std::remove_if(
      avoidance_targets.objects.begin(), avoidance_targets.objects.end(),
      [&](const PredictedObject & object) {
        const auto it = object_filters_.find(autoware_utils_uuid::to_hex_string(object.object_id));
        return it == object_filters_.end() || !it->second.is_target();
      }),
    avoidance_targets.objects.end());

  pub_avoidance_targets_->publish(avoidance_targets);
}

}  // namespace autoware::avoidance_target_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::avoidance_target_detector::AvoidanceTargetDetectorNode)
