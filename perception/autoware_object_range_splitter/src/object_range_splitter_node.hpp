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

#ifndef OBJECT_RANGE_SPLITTER_NODE_HPP_
#define OBJECT_RANGE_SPLITTER_NODE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <memory>

namespace autoware::object_range_splitter
{
class ObjectRangeSplitterNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit ObjectRangeSplitterNode(const rclcpp::NodeOptions & node_options);

private:
  void objectCallback(
    AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_perception_msgs::msg::DetectedObjects) input_msg);

  AUTOWARE_PUBLISHER_PTR(autoware_perception_msgs::msg::DetectedObjects) long_range_object_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_perception_msgs::msg::DetectedObjects) short_range_object_pub_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_perception_msgs::msg::DetectedObjects) sub_;

  // ROS Parameters
  float spilt_range_;
};

}  // namespace autoware::object_range_splitter

#endif  // OBJECT_RANGE_SPLITTER_NODE_HPP_
