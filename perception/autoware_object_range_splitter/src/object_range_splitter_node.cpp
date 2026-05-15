// Copyright 2020 TierIV
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

#include "object_range_splitter_node.hpp"

#include <utility>

namespace autoware::object_range_splitter
{
ObjectRangeSplitterNode::ObjectRangeSplitterNode(const rclcpp::NodeOptions & node_options)
: Node("object_range_splitter_node", node_options)
{
  spilt_range_ = declare_parameter<double>("split_range");
  sub_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/object", rclcpp::QoS{1},
    [this](const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_perception_msgs::msg::DetectedObjects) &
             msg) { this->objectCallback(msg); });
  long_range_object_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/long_range_object", rclcpp::QoS{1});
  short_range_object_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/short_range_object", rclcpp::QoS{1});
}

void ObjectRangeSplitterNode::objectCallback(
  AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_perception_msgs::msg::DetectedObjects) input_msg)
{
  // Guard
  if (
    long_range_object_pub_->get_subscription_count() < 1 &&
    short_range_object_pub_->get_subscription_count() < 1) {
    return;
  }
  // build output msg
  auto output_long_range_object_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(long_range_object_pub_);
  auto output_short_range_object_msg = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(short_range_object_pub_);
  output_long_range_object_msg->header = input_msg->header;
  output_short_range_object_msg->header = input_msg->header;

  // split
  for (const auto & object : input_msg->objects) {
    const auto & position = object.kinematics.pose_with_covariance.pose.position;
    const auto object_sq_dist = position.x * position.x + position.y * position.y;
    if (object_sq_dist < spilt_range_ * spilt_range_) {  // short range
      output_short_range_object_msg->objects.push_back(object);
    } else {  // long range
      output_long_range_object_msg->objects.push_back(object);
    }
  }

  // publish output msg
  long_range_object_pub_->publish(std::move(output_long_range_object_msg));
  short_range_object_pub_->publish(std::move(output_short_range_object_msg));
}
}  // namespace autoware::object_range_splitter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::object_range_splitter::ObjectRangeSplitterNode)
