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

#pragma once

#include "concatenation_info_manager.hpp"
#include "traits.hpp"

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// ROS includes
#include <agnocast/agnocast.hpp>
#include <agnocast/node/tf2/buffer.hpp>
#include <agnocast/node/tf2/transform_listener.hpp>
#include <pcl_ros/transforms.hpp>

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace autoware::pointcloud_preprocessor
{

template <typename MsgTraits>
struct ConcatenatedCloudResult
{
  typename MsgTraits::UniquePtr concatenate_cloud_ptr{nullptr};
  autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::UniquePtr concatenation_info_ptr;
  std::optional<std::unordered_map<std::string, typename MsgTraits::UniquePtr>>
    topic_to_transformed_cloud_map;
  std::unordered_map<std::string, double> topic_to_original_stamp_map;
};

class CombineCloudHandlerBase
{
public:
  CombineCloudHandlerBase(
    agnocast::Node & node, const std::vector<std::string> & input_topics, std::string output_frame,
    bool is_motion_compensated, bool publish_synchronized_pointcloud,
    bool keep_input_frame_in_synchronized_pointcloud)
  : node_(node),
    input_topics_(input_topics),
    output_frame_(output_frame),
    is_motion_compensated_(is_motion_compensated),
    publish_synchronized_pointcloud_(publish_synchronized_pointcloud),
    keep_input_frame_in_synchronized_pointcloud_(keep_input_frame_in_synchronized_pointcloud),
    tf_buffer_(std::make_shared<agnocast::Buffer>(node.get_clock())),
    tf_listener_(std::make_shared<agnocast::TransformListener>(*tf_buffer_, node)),
    concatenation_info_manager_(
      node.get_parameter("matching_strategy.type").as_string(), input_topics)
  {
  }

  std::optional<Eigen::Matrix4f> getTransformMatrix(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time, const rclcpp::Duration & timeout,
    const rclcpp::Logger & logger)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(target_frame, source_frame, time, timeout);
      Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
      auto & t = tf.transform.translation;
      auto & r = tf.transform.rotation;
      Eigen::Quaternionf q(r.w, r.x, r.y, r.z);
      mat.block<3, 3>(0, 0) = q.toRotationMatrix();
      mat(0, 3) = static_cast<float>(t.x);
      mat(1, 3) = static_cast<float>(t.y);
      mat(2, 3) = static_cast<float>(t.z);
      return mat;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_STREAM_THROTTLE(logger, *node_.get_clock(), 1000, ex.what());
      return std::nullopt;
    }
  }

  bool transformPointcloud(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & cloud_in,
    sensor_msgs::msg::PointCloud2 & cloud_out, const rclcpp::Time & time,
    const rclcpp::Duration & timeout, const rclcpp::Logger & logger)
  {
    if (cloud_in.data.empty() || target_frame == cloud_in.header.frame_id) {
      cloud_out = cloud_in;
      cloud_out.header.frame_id = target_frame;
      return true;
    }
    auto mat = getTransformMatrix(target_frame, cloud_in.header.frame_id, time, timeout, logger);
    if (!mat) return false;
    pcl_ros::transformPointCloud(*mat, cloud_in, cloud_out);
    cloud_out.header.frame_id = target_frame;
    return true;
  }

  void process_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped & twist_msg);

  void process_odometry(const nav_msgs::msg::Odometry & input);

  std::deque<geometry_msgs::msg::TwistStamped> get_twist_queue();

  Eigen::Matrix4f compute_transform_to_adjust_for_old_timestamp(
    const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp);

  virtual void allocate_pointclouds() = 0;

protected:
  agnocast::Node & node_;
  std::vector<std::string> input_topics_;
  std::string output_frame_;
  bool is_motion_compensated_;
  bool publish_synchronized_pointcloud_;
  bool keep_input_frame_in_synchronized_pointcloud_;
  std::shared_ptr<agnocast::Buffer> tf_buffer_;
  std::shared_ptr<agnocast::TransformListener> tf_listener_;
  ConcatenationInfoManager concatenation_info_manager_;

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
};

}  // namespace autoware::pointcloud_preprocessor
