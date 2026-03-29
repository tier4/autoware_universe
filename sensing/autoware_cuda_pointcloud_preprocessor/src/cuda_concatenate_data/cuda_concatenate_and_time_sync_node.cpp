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

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_concatenate_and_time_sync_node.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_cloud_collector.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_traits.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"

#include <agnocast/agnocast.hpp>

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cuda_runtime.h>

#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

template <>
void PointCloudConcatenateDataSynchronizerComponentTemplated<
  CudaPointCloud2Traits>::initialize_pub_sub()
{
  concatenated_cloud_publisher_ = agnocast::create_publisher<agnocast::cuda::PointCloud2>(
    this, "output/cuda", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));
  concatenation_info_publisher_ =
    this->create_publisher<autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo>(
      "output_info", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));

  // CPU copy publisher for ROS 2 subscribers (rviz, localization, etc.)
  // CUDA messages can't be bridged, so we publish a CPU copy alongside.
  auto cpu_pub = agnocast::create_publisher<sensor_msgs::msg::PointCloud2>(
    this, "output", rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));
  agnocast_pre_publish_callback_ =
    [cpu_pub](const agnocast::cuda::PointCloud2 & cuda_msg) {
      auto ros_msg = cpu_pub->borrow_loaned_message();
      ros_msg->header = cuda_msg.header;
      ros_msg->height = cuda_msg.height;
      ros_msg->width = cuda_msg.width;
      ros_msg->fields = cuda_msg.fields;
      ros_msg->is_bigendian = cuda_msg.is_bigendian;
      ros_msg->point_step = cuda_msg.point_step;
      ros_msg->row_step = cuda_msg.row_step;
      ros_msg->is_dense = cuda_msg.is_dense;
      const size_t data_size = cuda_msg.row_step * cuda_msg.height;
      if (data_size > 0 && cuda_msg.data != nullptr) {
        ros_msg->data.resize(data_size);
        cudaMemcpy(ros_msg->data.data(), cuda_msg.data, data_size, cudaMemcpyDeviceToHost);
      }
      cpu_pub->publish(std::move(ros_msg));
    };

  for (auto & topic : params_.input_topics) {
    std::string new_topic =
      replace_sync_topic_name_postfix(topic, params_.synchronized_pointcloud_postfix);
    auto publisher = agnocast::create_publisher<agnocast::cuda::PointCloud2>(
      this, new_topic, rclcpp::SensorDataQoS().keep_last(params_.maximum_queue_size));
    topic_to_transformed_cloud_publisher_map_.insert({topic, publisher});
  }

  for (const std::string & topic : params_.input_topics) {
    auto pointcloud_sub = agnocast::create_subscription<agnocast::cuda::PointCloud2>(
      this, topic, rclcpp::SensorDataQoS().keep_last(1),
      [this, topic](agnocast::ipc_shared_ptr<const agnocast::cuda::PointCloud2> msg) {
        this->cloud_callback(msg, topic);
      });
    pointcloud_subs_.push_back(pointcloud_sub);
  }

  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Subscribing to " << params_.input_topics.size() << " user given topics as inputs:");

  for (const auto & input_topic : params_.input_topics) {
    RCLCPP_DEBUG_STREAM(get_logger(), " - " << input_topic);
  }
}

namespace
{
/// Transfer metadata and GPU pointer from a unique_ptr to a borrowed agnocast message.
/// Uses std::move for heap-allocated fields (header.frame_id string, fields vector)
/// to avoid copies. After this call, src is in a moved-from state.
void transfer_cuda_pointcloud(
  CudaPointCloud2Traits::UniquePtr & src,
  agnocast::ipc_shared_ptr<agnocast::cuda::PointCloud2> & dst)
{
  dst->header = std::move(src->header);
  dst->height = src->height;
  dst->width = src->width;
  dst->fields = std::move(src->fields);
  dst->is_bigendian = src->is_bigendian;
  dst->point_step = src->point_step;
  dst->row_step = src->row_step;
  dst->is_dense = src->is_dense;
  // Transfer GPU pointer ownership to the agnocast message.
  // Agnocast will cudaFree it on reclaim.
  dst->data = src->data;
  src->data = nullptr;
}
}  // namespace

template <>
void PointCloudConcatenateDataSynchronizerComponentTemplated<
  CudaPointCloud2Traits>::
  publish_clouds(
    ConcatenatedCloudResult<CudaPointCloud2Traits> && concatenated_cloud_result,
    std::shared_ptr<CollectorInfoBase> collector_info)
{
  DiagnosticInfo diagnostic_info;
  bool publish_pointcloud = false;
  bool drop_previous_but_late_pointcloud = false;
  bool is_concatenated_cloud_empty = false;

  if (concatenated_cloud_result.concatenate_cloud_ptr == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Concatenated cloud is a nullptr.");
    return;
  }

  if (
    concatenated_cloud_result.concatenate_cloud_ptr->width *
      concatenated_cloud_result.concatenate_cloud_ptr->height ==
    0) {
    RCLCPP_ERROR(this->get_logger(), "Concatenated cloud is an empty pointcloud.");
    is_concatenated_cloud_empty = true;
  }

  current_concatenate_cloud_timestamp_ =
    rclcpp::Time(concatenated_cloud_result.concatenate_cloud_ptr->header.stamp).seconds();

  if (
    current_concatenate_cloud_timestamp_ < latest_concatenate_cloud_timestamp_ &&
    !params_.publish_previous_but_late_pointcloud) {
    if (
      latest_concatenate_cloud_timestamp_ - current_concatenate_cloud_timestamp_ >
      params_.rosbag_length) {
      publish_pointcloud = true;
    } else {
      drop_previous_but_late_pointcloud = true;
    }
  } else {
    publish_pointcloud = true;
  }

  if (publish_pointcloud) {
    latest_concatenate_cloud_timestamp_ = current_concatenate_cloud_timestamp_;

    // Publish CPU copy for ROS 2 subscribers (before GPU pointer transfer)
    if (agnocast_pre_publish_callback_) {
      agnocast_pre_publish_callback_(*concatenated_cloud_result.concatenate_cloud_ptr);
    }

    // Borrow from agnocast publisher, transfer GPU pointer, publish
    auto msg = concatenated_cloud_publisher_->borrow_loaned_message();
    transfer_cuda_pointcloud(concatenated_cloud_result.concatenate_cloud_ptr, msg);
    concatenated_cloud_publisher_->publish(std::move(msg));

    // Publish transformed raw pointclouds
    if (
      params_.publish_synchronized_pointcloud &&
      concatenated_cloud_result.topic_to_transformed_cloud_map) {
      for (const auto & topic : params_.input_topics) {
        if (
          (*concatenated_cloud_result.topic_to_transformed_cloud_map).find(topic) !=
          (*concatenated_cloud_result.topic_to_transformed_cloud_map).end()) {
          auto nh = (*concatenated_cloud_result.topic_to_transformed_cloud_map).extract(topic);
          auto cloud = std::move(nh.mapped());
          auto topic_msg =
            topic_to_transformed_cloud_publisher_map_[topic]->borrow_loaned_message();
          transfer_cuda_pointcloud(cloud, topic_msg);
          topic_to_transformed_cloud_publisher_map_[topic]->publish(std::move(topic_msg));
        } else {
          RCLCPP_WARN(
            this->get_logger(),
            "transformed_raw_points[%s] is nullptr, skipping pointcloud publish.", topic.c_str());
        }
      }
    }
  }
  {
    auto info_msg = concatenation_info_publisher_->borrow_loaned_message();
    *info_msg = *concatenated_cloud_result.concatenation_info_ptr;  // copy (heaphook allocates in shm)
    concatenation_info_publisher_->publish(std::move(info_msg));
  }

  const double processing_time = stop_watch_ptr_->toc("processing_time", true);
  std::unordered_map<std::string, double> topic_to_pipeline_latency_map;
  double max_pipeline_latency = 0.0;
  const double now_sec = this->get_clock()->now().seconds();

  for (const auto & [topic, stamp] : concatenated_cloud_result.topic_to_original_stamp_map) {
    const double latency = (now_sec - stamp) * 1000.0;
    topic_to_pipeline_latency_map[topic] = latency;
    max_pipeline_latency = std::max(max_pipeline_latency, latency);
  }

  diagnostic_info.publish_pointcloud = publish_pointcloud;
  diagnostic_info.drop_previous_but_late_pointcloud = drop_previous_but_late_pointcloud;
  diagnostic_info.is_concatenated_cloud_empty = is_concatenated_cloud_empty;
  diagnostic_info.collector_info = std::move(collector_info);
  diagnostic_info.topic_to_original_stamp_map =
    concatenated_cloud_result.topic_to_original_stamp_map;
  diagnostic_info.processing_time = processing_time;
  diagnostic_info.pipeline_latency = max_pipeline_latency;
  diagnostic_info.topic_to_pipeline_latency_map = topic_to_pipeline_latency_map;
  check_concat_status(diagnostic_info);

  if (debug_publisher_) {
    const double cyclic_time = stop_watch_ptr_->toc("cyclic_time", true);
    publish_debug_message(processing_time, cyclic_time, topic_to_pipeline_latency_map);
  }
}

}  // namespace autoware::pointcloud_preprocessor

template class autoware::pointcloud_preprocessor::
  PointCloudConcatenateDataSynchronizerComponentTemplated<
    autoware::pointcloud_preprocessor::CudaPointCloud2Traits>;

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::cuda_pointcloud_preprocessor::CudaPointCloudConcatenateDataSynchronizerComponent)
