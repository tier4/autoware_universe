// Copyright 2025 Shin-kyoto.
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

#ifndef AUTOWARE_TENSORRT_VAD_VAD_NODE_HPP_
#define AUTOWARE_TENSORRT_VAD_VAD_NODE_HPP_

#include "autoware/tensorrt_vad/vad_model.hpp"
#include "autoware/tensorrt_vad/vad_interface.hpp"
#include "autoware/tensorrt_vad/vad_interface_config.hpp"
#include "autoware/tensorrt_vad/synchronization_strategy.hpp"
#include "ros_vad_logger.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <memory>
#include <vector>
#include <map>
#include <optional>

namespace autoware::tensorrt_vad
{

class VadNode : public rclcpp::Node
{
public:
  explicit VadNode(const rclcpp::NodeOptions & options);

private:
  // Config loading function
  VadConfig load_vad_config();
  std::tuple<
    autoware::tensorrt_common::TrtCommonConfig,
    autoware::tensorrt_common::TrtCommonConfig,
    autoware::tensorrt_common::TrtCommonConfig
  > load_trt_common_configs();
  void initialize_vad_model();
  void create_camera_image_subscribers(const rclcpp::QoS& sensor_qos);
  void create_camera_info_subscribers(const rclcpp::QoS& camera_info_qos);

  // Callback methods
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t camera_id);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, std::size_t camera_id);
  void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg);
  void tf_static_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg);
  void anchor_callback();

  // tf Members
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // Number of cameras (configured via parameter)
  int32_t num_cameras_;

  // Subscribers for images (using image_transport)
  std::vector<image_transport::Subscriber> camera_image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;

  // Subscribers for odometry and acceleration data
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;

  // VAD model
  std::unique_ptr<VadModel<RosVadLogger>> vad_model_ptr_{};

  // VAD interface
  std::unique_ptr<VadInterface> vad_interface_ptr_{};
  VadInterfaceConfig vad_interface_config_;

  // Synchronization strategy
  std::unique_ptr<SynchronizationStrategy> sync_strategy_;
  int32_t front_camera_id_;

  // trajectory_timestep parameter
  double trajectory_timestep_;

  // Publishers
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr candidate_trajectories_publisher_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr predicted_objects_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_points_publisher_;

  // Current frame data accumulation
  VadInputTopicData vad_input_topic_data_current_frame_;
  
  std::optional<VadOutputTopicData> execute_inference(const VadInputTopicData & vad_input_topic_data);
  std::optional<VadOutputTopicData> trigger_inference(VadInputTopicData vad_input_topic_data_current_frame);
  void publish(const VadOutputTopicData & vad_output_topic_data);
};
}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE_TENSORRT_VAD_VAD_NODE_HPP_
