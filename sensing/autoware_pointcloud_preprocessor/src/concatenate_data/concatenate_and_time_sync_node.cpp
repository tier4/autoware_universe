// Copyright 2024 TIER IV, Inc.
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

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include "autoware/pointcloud_preprocessor/concatenate_data/cloud_collector.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <pcl_ros/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <iomanip>
#include <list>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <>
void PointCloudConcatenateDataSynchronizerComponentTemplated<
  PointCloud2Traits>::initialize_pub_sub()
{
  // Publishers
  concatenated_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "output", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));
  concatenation_info_publisher_ =
    this->create_publisher<autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo>(
      "output_info", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));

  // Transformed Raw PointCloud2 Publisher to publish the transformed pointcloud
  if (params_.publish_synchronized_pointcloud) {
    for (auto & topic : params_.input_topics) {
      std::string new_topic =
        replace_sync_topic_name_postfix(topic, params_.synchronized_pointcloud_postfix);
      auto publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        new_topic, rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));
      topic_to_transformed_cloud_publisher_map_.insert({topic, publisher});
    }
  }

  // Subscribers
  auto sub_cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  for (const std::string & topic : params_.input_topics) {
    std::function<void(const AUTOWARE_MESSAGE_SHARED_PTR(sensor_msgs::msg::PointCloud2) msg)>
      callback = std::bind(
        &PointCloudConcatenateDataSynchronizerComponent::cloud_callback, this,
        std::placeholders::_1, topic);

    AUTOWARE_SUBSCRIPTION_OPTIONS sub_options;
    sub_options.callback_group = sub_cbg;
    auto pointcloud_sub = AUTOWARE_CREATE_SUBSCRIPTION(
      sensor_msgs::msg::PointCloud2, topic,
      rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size), callback, sub_options);

    pointcloud_subs_.push_back(pointcloud_sub);
  }

  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Subscribing to " << params_.input_topics.size() << " user given topics as inputs:");
  for (const auto & input_topic : params_.input_topics) {
    RCLCPP_DEBUG_STREAM(get_logger(), " - " << input_topic);
  }
}

}  // namespace autoware::pointcloud_preprocessor

template class autoware::pointcloud_preprocessor::
  PointCloudConcatenateDataSynchronizerComponentTemplated<
    autoware::pointcloud_preprocessor::PointCloud2Traits>;

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent)
