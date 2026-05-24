// Copyright 2022 TIER IV, Inc.
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

#include "autoware/map_tf_generator/vector_map_tf_generator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <memory>
#include <string>

namespace autoware::map_tf_generator
{
class VectorMapTFGeneratorNode : public rclcpp::Node
{
public:
  explicit VectorMapTFGeneratorNode(const rclcpp::NodeOptions & options)
  : Node("vector_map_tf_generator", options),
    map_frame_(declare_parameter<std::string>("map_frame")),
    viewer_frame_(declare_parameter<std::string>("viewer_frame"))
  {
    sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      "vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&VectorMapTFGeneratorNode::on_vector_map, this, std::placeholders::_1));

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

private:
  std::string map_frame_;
  std::string viewer_frame_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  void on_vector_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
  {
    const auto transform =
      create_viewer_transform(*msg, map_frame_, viewer_frame_, this->now());
    static_broadcaster_->sendTransform(transform);

    RCLCPP_INFO_STREAM(
      get_logger(), "broadcast static tf. map_frame:"
                      << map_frame_ << ", viewer_frame:" << viewer_frame_ << ", x:"
                      << transform.transform.translation.x << ", y:"
                      << transform.transform.translation.y << ", z:"
                      << transform.transform.translation.z);
  }
};
}  // namespace autoware::map_tf_generator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::map_tf_generator::VectorMapTFGeneratorNode)
