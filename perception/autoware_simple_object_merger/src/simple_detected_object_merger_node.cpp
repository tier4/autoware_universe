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

#include "autoware/simple_object_merger/simple_detected_object_merger_node.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::simple_object_merger
{
using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

SimpleDetectedObjectMergerNode::SimpleDetectedObjectMergerNode(
  const rclcpp::NodeOptions & node_options)
: SimpleObjectMergerBase<DetectedObjects>("simple_object_merger", node_options)
{
  // required by RCLCPP_COMPONENTS_REGISTER_NODE macro.
}

void SimpleDetectedObjectMergerNode::approximateMerger(
  AUTOWARE_MESSAGE_SHARED_PTR(const DetectedObjects) && object_msg0,
  AUTOWARE_MESSAGE_SHARED_PTR(const DetectedObjects) && object_msg1)
{
  // Make mutable copies for transformation
  auto transformed_objects0 = std::make_shared<DetectedObjects>(*object_msg0);
  if (node_param_.new_frame_id != object_msg0->header.frame_id) {
    auto transform0 = transform_listener_->get_transform(
      node_param_.new_frame_id, object_msg0->header.frame_id, object_msg0->header.stamp,
      rclcpp::Duration::from_seconds(0.01));
    if (!transform0) {
      return;
    }
    transformed_objects0 =
      getTransformedObjects(transformed_objects0, node_param_.new_frame_id, transform0);
  }

  auto transformed_objects1 = std::make_shared<DetectedObjects>(*object_msg1);
  if (node_param_.new_frame_id != object_msg1->header.frame_id) {
    auto transform1 = transform_listener_->get_transform(
      node_param_.new_frame_id, object_msg1->header.frame_id, object_msg1->header.stamp,
      rclcpp::Duration::from_seconds(0.01));
    if (!transform1) {
      return;
    }
    transformed_objects1 =
      getTransformedObjects(transformed_objects1, node_param_.new_frame_id, transform1);
  }

  auto output_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_objects_);
  output_msg->header = object_msg0->header;
  output_msg->header.frame_id = node_param_.new_frame_id;
  output_msg->objects.reserve(
    transformed_objects0->objects.size() + transformed_objects1->objects.size());
  output_msg->objects = transformed_objects0->objects;
  output_msg->objects.insert(
    output_msg->objects.end(), std::begin(transformed_objects1->objects),
    std::end(transformed_objects1->objects));

  pub_objects_->publish(std::move(output_msg));
}

void SimpleDetectedObjectMergerNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  auto output_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(pub_objects_);
  output_msg->header = objects_data_.at(0)->header;
  output_msg->header.frame_id = node_param_.new_frame_id;

  constexpr double throttle_interval = 3.0;  // seconds
  const rclcpp::Time now = this->now();

  static std::vector<rclcpp::Time> last_log_times;
  if (last_log_times.size() != input_topic_size_) {
    last_log_times.assign(input_topic_size_, now);
  }

  for (size_t i = 0; i < input_topic_size_; i++) {
    double time_diff = rclcpp::Time(objects_data_.at(i)->header.stamp).seconds() -
                       rclcpp::Time(objects_data_.at(0)->header.stamp).seconds();
    if (std::abs(time_diff) < node_param_.timeout_threshold) {
      DetectedObjects::SharedPtr transformed_objects;
      if (node_param_.new_frame_id == objects_data_.at(i)->header.frame_id) {
        transformed_objects = std::make_shared<DetectedObjects>(*objects_data_.at(i));
      } else {
        auto transform = transform_listener_->get_transform(
          node_param_.new_frame_id, objects_data_.at(i)->header.frame_id,
          objects_data_.at(i)->header.stamp, rclcpp::Duration::from_seconds(0.01));
        if (!transform) {
          continue;
        }
        transformed_objects =
          getTransformedObjects(objects_data_.at(i), node_param_.new_frame_id, transform);
      }
      output_msg->objects.insert(
        output_msg->objects.end(), std::begin(transformed_objects->objects),
        std::end(transformed_objects->objects));
    } else if (shouldLogThrottle(i, now, last_log_times, throttle_interval)) {
      RCLCPP_INFO(
        get_logger(), "Topic of %s is timeout by %f sec", node_param_.topic_names.at(i).c_str(),
        time_diff);
    }
  }

  pub_objects_->publish(std::move(output_msg));
}

}  // namespace autoware::simple_object_merger

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::simple_object_merger::SimpleDetectedObjectMergerNode)
