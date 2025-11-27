// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/lidar_frnet/lidar_frnet_node.hpp"

#include "autoware/lidar_frnet/ros_utils.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <autoware/tensorrt_common/utils.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>

namespace autoware::lidar_frnet
{
LidarFRNetNode::LidarFRNetNode(const rclcpp::NodeOptions & options)
: Node("lidar_frnet", options),
  cloud_seg_layout_(ros_utils::generateSegmentationPointCloudLayout()),
  cloud_viz_layout_(ros_utils::generateVisualizationPointCloudLayout()),
  cloud_filtered_layout_(ros_utils::generateFilteredPointCloudLayout())
{
  auto class_names = declare_parameter<std::vector<std::string>>("class_names");

  auto trt_config = TrtCommonConfig(
    declare_parameter<std::string>("onnx_path"), declare_parameter<std::string>("trt_precision"));

  auto preprocessing_params = utils::PreprocessingParams(
    declare_parameter<double>("fov_up_deg"), declare_parameter<double>("fov_down_deg"),
    declare_parameter<int64_t>("frustum_width"), declare_parameter<uint16_t>("frustum_height"),
    declare_parameter<int64_t>("interpolation_width"),
    declare_parameter<int64_t>("interpolation_height"));

  auto postprocessing_params = utils::PostprocessingParams(
    declare_parameter<double>("score_threshold"), class_names,
    declare_parameter<std::vector<int64_t>>("palette"),
    declare_parameter<std::vector<std::string>>("excluded_class_names"));

  auto model_params = utils::NetworkParams(
    class_names, declare_parameter<std::vector<int64_t>>("num_points"),
    declare_parameter<std::vector<int64_t>>("num_unique_coors"));

  diag_params_ = utils::DiagnosticParams(
    declare_parameter<double>("max_allowed_processing_time_ms"),
    declare_parameter<double>("max_acceptable_consecutive_delay_ms"),
    declare_parameter<double>("validation_callback_interval_ms"));

  // Multi-LiDAR support parameters
  joint_inference_ = declare_parameter<bool>("joint_inference", true);
  input_topics_ = declare_parameter<std::vector<std::string>>("input_topics");

  if (input_topics_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "input_topics parameter is empty. At least one topic is required.");
    throw std::runtime_error("input_topics parameter is empty");
  }

  if (joint_inference_) {
    // TODO(Sugahara): Implement joint inference mode that considers point cloud source info
    RCLCPP_WARN(
      this->get_logger(),
      "Joint inference mode is not yet implemented. Falling back to independent inference mode.");
    joint_inference_ = false;
  }

  published_time_pub_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // Initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
  }

  // Setup diagnostics
  {
    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID(this->get_name());
    diag_updater_->add("processing_time_status", this, &LidarFRNetNode::diagnoseProcessingTime);
    diag_updater_->setPeriod(diag_params_.validation_callback_interval_ms * 1e-3);  // to seconds
  }

  // Create processors for each input topic (independent inference mode)
  if (!joint_inference_) {
    for (const auto & topic_name : input_topics_) {
      RCLCPP_INFO(this->get_logger(), "Setting up processor for topic: %s", topic_name.c_str());

      auto processor = std::make_shared<LidarProcessor>();
      processor->topic_name = topic_name;

      // Create FRNet instance for this topic
      processor->frnet = std::make_unique<LidarFRNet>(
        trt_config, model_params, preprocessing_params, postprocessing_params, get_logger());

      // Create stop watch for this topic
      processor->stop_watch_ptr =
        std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
      processor->stop_watch_ptr->tic("cyclic");
      processor->stop_watch_ptr->tic("processing/total");

      // TODO(Sugahara): Use cuda_blackboard for communication between LiDAR processors to reduce
      // data transfer overhead Create subscriber with topic-specific callback
      processor->cloud_in_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, rclcpp::SensorDataQoS{}.keep_last(1),
        [this, topic_name](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          this->cloudCallback(msg, topic_name);
        });

      // Create publishers with topic-specific names
      std::string sanitized_topic_name = topic_name;
      std::replace(sanitized_topic_name.begin(), sanitized_topic_name.end(), '/', '_');

      processor->cloud_seg_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/output" + sanitized_topic_name + "/segmentation", rclcpp::SensorDataQoS{}.keep_last(1));
      processor->cloud_viz_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/output" + sanitized_topic_name + "/visualization", rclcpp::SensorDataQoS{}.keep_last(1));
      processor->cloud_filtered_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/output" + sanitized_topic_name + "/filtered", rclcpp::SensorDataQoS{}.keep_last(1));

      lidar_processors_[topic_name] = processor;
    }
  }

  if (this->declare_parameter<bool>("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

void LidarFRNetNode::cloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const std::string & topic_name)
{
  auto it = lidar_processors_.find(topic_name);
  if (it == lidar_processors_.end()) {
    RCLCPP_ERROR(this->get_logger(), "Processor not found for topic: %s", topic_name.c_str());
    return;
  }

  auto & processor = it->second;

  if (processor->stop_watch_ptr) {
    processor->stop_watch_ptr->toc("processing/total", true);
  }

  const auto active_comm = utils::ActiveComm(
    processor->cloud_seg_pub->get_subscription_count() +
        processor->cloud_seg_pub->get_intra_process_subscription_count() >
      0,
    processor->cloud_viz_pub->get_subscription_count() +
        processor->cloud_viz_pub->get_intra_process_subscription_count() >
      0,
    processor->cloud_filtered_pub->get_subscription_count() +
        processor->cloud_filtered_pub->get_intra_process_subscription_count() >
      0);

  if (!active_comm) {
    return;
  }

  std::unordered_map<std::string, double> proc_timing;
  auto cloud_seg_msg = ros_utils::getMsgFromLayout(*msg, cloud_seg_layout_);
  auto cloud_viz_msg = ros_utils::getMsgFromLayout(*msg, cloud_viz_layout_);
  auto cloud_filtered_msg = ros_utils::getMsgFromLayout(*msg, cloud_filtered_layout_);

  if (!processor->frnet->process(
        *msg, cloud_seg_msg, cloud_viz_msg, cloud_filtered_msg, active_comm, proc_timing))
    return;

  processor->cloud_seg_pub->publish(cloud_seg_msg);
  processor->cloud_viz_pub->publish(cloud_viz_msg);
  processor->cloud_filtered_pub->publish(cloud_filtered_msg);
  published_time_pub_->publish_if_subscribed(processor->cloud_seg_pub, msg->header.stamp);

  if (debug_publisher_ptr_ && processor->stop_watch_ptr) {
    processor->last_processing_time_ms.emplace(
      processor->stop_watch_ptr->toc("processing/total", true));
    const double cyclic_time_ms = processor->stop_watch_ptr->toc("cyclic", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - msg->header.stamp).nanoseconds()))
        .count();

    // Publish debug info with topic-specific prefix
    std::string sanitized_topic_name = topic_name;
    std::replace(sanitized_topic_name.begin(), sanitized_topic_name.end(), '/', '_');

    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug" + sanitized_topic_name + "/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug" + sanitized_topic_name + "/pipeline_latency_ms", pipeline_latency_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug" + sanitized_topic_name + "/processing_time/total_ms",
      *processor->last_processing_time_ms);
    for (const auto & [proc_topic, time_ms] : proc_timing) {
      debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug" + sanitized_topic_name + "/" + proc_topic, time_ms);
    }
  }
}

void LidarFRNetNode::diagnoseProcessingTime(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const rclcpp::Time timestamp_now = this->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus::_level_type diag_level =
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::stringstream message{"OK"};

  // Aggregate processing times across all processors
  bool has_processed = false;
  double max_processing_time_ms = 0.0;

  for (const auto & [topic_name, processor] : lidar_processors_) {
    if (processor->last_processing_time_ms) {
      has_processed = true;
      max_processing_time_ms =
        std::max(max_processing_time_ms, *processor->last_processing_time_ms);
    }
  }

  // Check if any processor has performed inference
  if (has_processed) {
    // Check if processing time exceeds the limit
    if (max_processing_time_ms > diag_params_.max_allowed_processing_time_ms) {
      stat.add("is_processing_time_ms_in_expected_range", false);

      message.clear();
      message << "Processing time exceeds the acceptable limit of "
              << diag_params_.max_allowed_processing_time_ms << " ms by "
              << (max_processing_time_ms - diag_params_.max_allowed_processing_time_ms) << " ms.";

      // In case the processing starts with a delayed inference
      if (!last_in_time_processing_timestamp_) {
        last_in_time_processing_timestamp_.emplace(timestamp_now);
      }

      diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      stat.add("is_processing_time_ms_in_expected_range", true);
      last_in_time_processing_timestamp_.emplace(timestamp_now);
    }
    stat.add("processing_time_ms", max_processing_time_ms);

    const double delayed_state_duration =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (timestamp_now - *last_in_time_processing_timestamp_).nanoseconds()))
        .count();

    // check consecutive delays
    if (delayed_state_duration > diag_params_.max_acceptable_consecutive_delay_ms) {
      stat.add("is_consecutive_processing_delay_in_range", false);

      message << " Processing delay has consecutively exceeded the acceptable limit continuously.";

      diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else {
      stat.add("is_consecutive_processing_delay_in_range", true);
    }
    stat.add("consecutive_processing_delay_ms", delayed_state_duration);
  } else {
    message << "Waiting for the node to perform inference.";
  }

  stat.summary(diag_level, message.str());
}

}  // namespace autoware::lidar_frnet

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_frnet::LidarFRNetNode)
