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

#include "autoware/tensorrt_vad/vad_node.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>

namespace autoware::tensorrt_vad
{
std::pair<Eigen::Matrix4f, Eigen::Matrix4f> get_transform_matrix(
  const nav_msgs::msg::Odometry & msg)
{
  // Extract position
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  double z = msg.pose.pose.position.z;

  // Create Eigen quaternion and normalize it
  Eigen::Quaternionf q = std::invoke([&msg]() -> Eigen::Quaternionf {
    double qx = msg.pose.pose.orientation.x;
    double qy = msg.pose.pose.orientation.y;
    double qz = msg.pose.pose.orientation.z;
    double qw = msg.pose.pose.orientation.w;

    // Create Eigen quaternion and normalize it
    Eigen::Quaternionf q(qw, qx, qy, qz);
    return (q.norm() < std::numeric_limits<float>::epsilon()) ? Eigen::Quaternionf::Identity()
                                                              : q.normalized();
  });

  // Rotation matrix (3x3)
  Eigen::Matrix3f R = q.toRotationMatrix();

  // Translation vector
  Eigen::Vector3f t(x, y, z);

  // Base_link → Map (forward)
  Eigen::Matrix4f bl2map = Eigen::Matrix4f::Identity();
  bl2map.block<3, 3>(0, 0) = R;
  bl2map.block<3, 1>(0, 3) = t;

  // Map → Base_link (inverse)
  Eigen::Matrix4f map2bl = Eigen::Matrix4f::Identity();
  map2bl.block<3, 3>(0, 0) = R.transpose();
  map2bl.block<3, 1>(0, 3) = -R.transpose() * t;

  return {bl2map, map2bl};
}

VadNode::VadNode(const rclcpp::NodeOptions & options) 
  : Node("vad_node", options), 
    tf_buffer_(this->get_clock()),
    num_cameras_(declare_parameter<int32_t>("node_params.num_cameras")),
    vad_interface_config_(
      declare_parameter<int32_t>("interface_params.input_image_width"),
      declare_parameter<int32_t>("interface_params.input_image_height"),
      declare_parameter<int32_t>("interface_params.target_image_width"),
      declare_parameter<int32_t>("interface_params.target_image_height"),
      declare_parameter<std::vector<double>>("interface_params.detection_range"),
      declare_parameter<int32_t>("interface_params.bev_h"),
      declare_parameter<int32_t>("interface_params.bev_w"),
      declare_parameter<double>("interface_params.default_patch_angle"),
      declare_parameter<int32_t>("model_params.default_command"),
      declare_parameter<std::vector<double>>("interface_params.default_shift"),
      declare_parameter<std::vector<double>>("interface_params.default_can_bus"),
      declare_parameter<std::vector<double>>("interface_params.image_normalization_param_mean"),
      declare_parameter<std::vector<double>>("interface_params.image_normalization_param_std"),
      declare_parameter<std::vector<double>>("interface_params.vad2base"),
      declare_parameter<std::vector<int64_t>>("interface_params.autoware_to_vad_camera_mapping"),
      declare_parameter<std::vector<std::string>>("model_params.map_classes"),
      declare_parameter<std::vector<double>>("interface_params.map_colors")
    ),
    front_camera_id_(declare_parameter<int32_t>("sync_params.front_camera_id")),
    trajectory_timestep_(declare_parameter<double>("interface_params.trajectory_timestep")),
    vad_input_topic_data_current_frame_(num_cameras_)
{
  // Publishers
  trajectory_publisher_ =
      this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
          "~/output/trajectory", rclcpp::QoS(1));

  candidate_trajectories_publisher_ =
      this->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
          "~/output/trajectories", rclcpp::QoS(1));

  predicted_objects_publisher_ =
      this->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
          "~/output/objects",
          rclcpp::QoS(1));

  map_points_publisher_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "~/output/map",
          rclcpp::QoS(1));

  // Create QoS profiles for sensor data (best effort for compatibility with typical sensor topics)
  auto sensor_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::BestEffort);
  auto camera_info_qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
  auto reliable_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::Reliable);

  // Subscribers for each camera
  create_camera_image_subscribers(sensor_qos);

  // Subscribers for camera info
  create_camera_info_subscribers(camera_info_qos);

  // Odometry subscriber (kinematic state is usually reliable)
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "~/input/kinematic_state", reliable_qos,
      std::bind(&VadNode::odometry_callback, this, std::placeholders::_1));

  // Acceleration subscriber (sensor data typically uses best effort)
  acceleration_sub_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "~/input/acceleration", sensor_qos,
      std::bind(&VadNode::acceleration_callback, this, std::placeholders::_1));

  // TF static subscriber (transient local for persistence)
  auto tf_static_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::Reliable)
                                     .durability(rclcpp::DurabilityPolicy::TransientLocal);
  tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf_static_qos,
      std::bind(&VadNode::tf_static_callback, this, std::placeholders::_1));

  // Initialize synchronization strategy
  double sync_tolerance_ms = declare_parameter<double>("sync_params.sync_tolerance_ms");
  sync_strategy_ = std::make_unique<FrontCriticalSynchronizationStrategy>(front_camera_id_, sync_tolerance_ms);

  // Initialize VAD model on first complete frame
  initialize_vad_model();

  RCLCPP_INFO(this->get_logger(), "VAD Node has been initialized - VAD model will be initialized after first callback");
}

void VadNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t camera_id)
{
  // Validate camera_id
  if (static_cast<int32_t>(camera_id) >= num_cameras_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid camera_id: %zu. Expected 0-%d", camera_id, num_cameras_ - 1);
    return;
  }

  vad_input_topic_data_current_frame_.set_image(camera_id, msg);
    
  // Check if this is the front camera (anchor camera)
  if (static_cast<int32_t>(camera_id) == front_camera_id_) {
    anchor_callback();
  }

  RCLCPP_DEBUG(this->get_logger(), "Received image from camera %zu", camera_id);
}

void VadNode::camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, std::size_t camera_id)
{
  // Validate camera_id
  if (static_cast<int32_t>(camera_id) >= num_cameras_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid camera_id: %zu. Expected 0-%d", camera_id, num_cameras_ - 1);
    return;
  }

  vad_input_topic_data_current_frame_.set_camera_info(camera_id, msg);

  RCLCPP_DEBUG(this->get_logger(), "Received camera info from camera %zu", camera_id);
}

void VadNode::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  vad_input_topic_data_current_frame_.set_kinematic_state(msg);

  RCLCPP_DEBUG(this->get_logger(), "Received odometry data");
}

void VadNode::acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg)
{
  vad_input_topic_data_current_frame_.set_acceleration(msg);

  RCLCPP_DEBUG(this->get_logger(), "Received acceleration data");
}

void VadNode::tf_static_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr msg)
{
  // Register transforms in tf_buffer
  for (const auto &transform : msg->transforms) {
    tf_buffer_.setTransform(transform, "default_authority", true);
  }

  RCLCPP_DEBUG(this->get_logger(), "Received TF static data");
}

void VadNode::anchor_callback()
{
  if (sync_strategy_->is_ready(vad_input_topic_data_current_frame_)) {
    auto vad_output_topic_data = trigger_inference(std::move(vad_input_topic_data_current_frame_));
    if (vad_output_topic_data.has_value()) {
      publish(vad_output_topic_data.value());
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Synchronization strategy indicates data is not ready for inference");
  }
  
  vad_input_topic_data_current_frame_.reset();
}

std::optional<VadOutputTopicData> VadNode::trigger_inference(VadInputTopicData vad_input_topic_data_current_frame)
{
  if (sync_strategy_->is_dropped(vad_input_topic_data_current_frame)) {
    auto filled_data_opt = sync_strategy_->fill_dropped_data(vad_input_topic_data_current_frame);
    if (!filled_data_opt) {
      // Cannot fill dropped data (front camera unavailable)
      return std::nullopt;
    }
    vad_input_topic_data_current_frame = std::move(filled_data_opt.value());
  }

  if (vad_input_topic_data_current_frame.is_complete()) {
    // Execute inference
    auto vad_output_topic_data = execute_inference(vad_input_topic_data_current_frame);
    return vad_output_topic_data;
  } else {
    return std::nullopt;
  }
}

void VadNode::initialize_vad_model()
{
  // load configs
  VadConfig vad_config = load_vad_config();
  auto [backbone_trt_config, head_trt_config, head_no_prev_trt_config] = load_trt_common_configs();

  // Initialize VAD interface and model
  auto tf_buffer_shared = std::shared_ptr<tf2_ros::Buffer>(&tf_buffer_, [](tf2_ros::Buffer*){});
  vad_interface_ptr_ = std::make_unique<VadInterface>(vad_interface_config_, tf_buffer_shared);

  // Create RosVadLogger using the logger
  auto ros_logger = std::make_shared<RosVadLogger>(this->get_logger());
  vad_model_ptr_ = std::make_unique<VadModel<RosVadLogger>>(
    vad_config,
    backbone_trt_config,
    head_trt_config,
    head_no_prev_trt_config,
    ros_logger
  );

  RCLCPP_INFO(this->get_logger(), "VAD model and interface initialized successfully");
}

VadConfig VadNode::load_vad_config()
{
  VadConfig vad_config;
  vad_config.num_cameras = this->declare_parameter<int32_t>("model_params.network_io_params.num_cameras");
  vad_config.bev_h = this->declare_parameter<int32_t>("model_params.network_io_params.bev_h");
  vad_config.bev_w = this->declare_parameter<int32_t>("model_params.network_io_params.bev_w");
  vad_config.bev_feature_dim = this->declare_parameter<int32_t>("model_params.network_io_params.bev_feature_dim");
  vad_config.target_image_width = this->declare_parameter<int32_t>("model_params.network_io_params.target_image_width");
  vad_config.target_image_height = this->declare_parameter<int32_t>("model_params.network_io_params.target_image_height");
  vad_config.downsample_factor = this->declare_parameter<int32_t>("model_params.network_io_params.downsample_factor");
  vad_config.num_decoder_layers = this->declare_parameter<int32_t>("model_params.network_io_params.num_decoder_layers");
  vad_config.prediction_num_queries = this->declare_parameter<int32_t>("model_params.network_io_params.prediction_num_queries");
  vad_config.prediction_num_classes = this->declare_parameter<int32_t>("model_params.network_io_params.prediction_num_classes");
  vad_config.prediction_bbox_pred_dim = this->declare_parameter<int32_t>("model_params.network_io_params.prediction_bbox_pred_dim");
  vad_config.prediction_trajectory_modes = this->declare_parameter<int32_t>("model_params.network_io_params.prediction_trajectory_modes");
  vad_config.prediction_timesteps = this->declare_parameter<int32_t>("model_params.network_io_params.prediction_timesteps");
  vad_config.planning_ego_commands = this->declare_parameter<int32_t>("model_params.network_io_params.planning_ego_commands");
  vad_config.planning_timesteps = this->declare_parameter<int32_t>("model_params.network_io_params.planning_timesteps");
  vad_config.map_num_queries = this->declare_parameter<int32_t>("model_params.network_io_params.map_num_queries");
  vad_config.map_num_class = this->declare_parameter<int32_t>("model_params.network_io_params.map_num_class");
  vad_config.map_points_per_polylines = this->declare_parameter<int32_t>("model_params.network_io_params.map_points_per_polylines");
  
  // Load detection range from interface_params (already declared in VadInterfaceConfig)
  auto detection_range = this->get_parameter("interface_params.detection_range").as_double_array();
  vad_config.detection_range.clear();
  vad_config.detection_range.reserve(detection_range.size());
  for (double val : detection_range) {
    vad_config.detection_range.push_back(static_cast<float>(val));
  }

  auto map_classes = this->get_parameter("model_params.map_classes").as_string_array();
  auto map_thresholds = this->declare_parameter<std::vector<double>>("model_params.map_confidence_thresholds");
  
  if (map_classes.size() != map_thresholds.size()) {
    RCLCPP_ERROR(this->get_logger(), "map_classes and map_confidence_thresholds must have the same size");
  }
  vad_config.map_class_names = map_classes;
  vad_config.map_num_classes = static_cast<int32_t>(map_classes.size());
  
  vad_config.map_confidence_thresholds.clear();
  for (size_t i = 0; i < map_classes.size(); ++i) {
    vad_config.map_confidence_thresholds[map_classes[i]] = static_cast<float>(map_thresholds[i]);
  }
  
  // Load object confidence thresholds by class
  vad_config.object_confidence_thresholds["car"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.car");
  vad_config.object_confidence_thresholds["truck"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.truck");
  vad_config.object_confidence_thresholds["construction_vehicle"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.construction_vehicle");
  vad_config.object_confidence_thresholds["bus"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.bus");
  vad_config.object_confidence_thresholds["trailer"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.trailer");
  vad_config.object_confidence_thresholds["barrier"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.barrier");
  vad_config.object_confidence_thresholds["motorcycle"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.motorcycle");
  vad_config.object_confidence_thresholds["bicycle"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.bicycle");
  vad_config.object_confidence_thresholds["pedestrian"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.pedestrian");
  vad_config.object_confidence_thresholds["traffic_cone"] = this->declare_parameter<float>("model_params.object_confidence_thresholds.traffic_cone");
  
  vad_config.can_bus_dim = this->declare_parameter<int32_t>("model_params.network_io_params.can_bus_dim");
  vad_config.plugins_path = this->declare_parameter<std::string>("model_params.plugins_path");

  // backbone configuration
  NetConfig backbone_config;
  backbone_config.name = this->declare_parameter<std::string>("model_params.nets.backbone.name");

  // head configuration
  NetConfig head_config;
  head_config.name = this->declare_parameter<std::string>("model_params.nets.head.name");

  // head inputs
  std::string input_feature = this->declare_parameter<std::string>("model_params.nets.head.inputs.input_feature");
  std::string net_param = this->declare_parameter<std::string>("model_params.nets.head.inputs.net");
  std::string name_param = this->declare_parameter<std::string>("model_params.nets.head.inputs.name");
  head_config.inputs[input_feature]["net"] = net_param;
  head_config.inputs[input_feature]["name"] = name_param;

  // head_no_prev configuration
  NetConfig head_no_prev_config;
  head_no_prev_config.name = this->declare_parameter<std::string>("model_params.nets.head_no_prev.name");

  // head_no_prev inputs
  std::string input_feature_no_prev = this->declare_parameter<std::string>("model_params.nets.head_no_prev.inputs.input_feature");
  std::string net_param_no_prev = this->declare_parameter<std::string>("model_params.nets.head_no_prev.inputs.net");
  std::string name_param_no_prev = this->declare_parameter<std::string>("model_params.nets.head_no_prev.inputs.name");
  head_no_prev_config.inputs[input_feature_no_prev]["net"] = net_param_no_prev;
  head_no_prev_config.inputs[input_feature_no_prev]["name"] = name_param_no_prev;

  vad_config.nets_config.push_back(backbone_config);
  vad_config.nets_config.push_back(head_config);
  vad_config.nets_config.push_back(head_no_prev_config);
  
  return vad_config;
}

std::tuple<
    autoware::tensorrt_common::TrtCommonConfig,
    autoware::tensorrt_common::TrtCommonConfig,
    autoware::tensorrt_common::TrtCommonConfig
> VadNode::load_trt_common_configs()
{
  std::string backbone_onnx_path = this->declare_parameter<std::string>("model_params.nets.backbone.onnx_path");
  std::string backbone_precision = this->declare_parameter<std::string>("model_params.nets.backbone.precision");
  std::string backbone_engine_path = this->declare_parameter<std::string>("model_params.nets.backbone.engine_path");
  autoware::tensorrt_common::TrtCommonConfig backbone_trt_config(
      backbone_onnx_path, backbone_precision, backbone_engine_path, 5ULL << 30U);

  std::string head_onnx_path = this->declare_parameter<std::string>("model_params.nets.head.onnx_path");
  std::string head_precision = this->declare_parameter<std::string>("model_params.nets.head.precision");
  std::string head_engine_path = this->declare_parameter<std::string>("model_params.nets.head.engine_path");
  autoware::tensorrt_common::TrtCommonConfig head_trt_config(
      head_onnx_path, head_precision, head_engine_path, 5ULL << 30U);

  std::string head_no_prev_onnx_path = this->declare_parameter<std::string>("model_params.nets.head_no_prev.onnx_path");
  std::string head_no_prev_precision = this->declare_parameter<std::string>("model_params.nets.head_no_prev.precision");
  std::string head_no_prev_engine_path = this->declare_parameter<std::string>("model_params.nets.head_no_prev.engine_path");
  autoware::tensorrt_common::TrtCommonConfig head_no_prev_trt_config(
      head_no_prev_onnx_path, head_no_prev_precision, head_no_prev_engine_path, 5ULL << 30U);

  RCLCPP_INFO(this->get_logger(), "TrtCommon configurations loaded (5GB workspace):");
  RCLCPP_INFO(this->get_logger(), "  Backbone - ONNX: %s, Precision: %s", backbone_onnx_path.c_str(), backbone_precision.c_str());
  RCLCPP_INFO(this->get_logger(), "  Head - ONNX: %s, Precision: %s", head_onnx_path.c_str(), head_precision.c_str());
  RCLCPP_INFO(this->get_logger(), "  Head No Prev - ONNX: %s, Precision: %s", head_no_prev_onnx_path.c_str(), head_no_prev_precision.c_str());

  return {backbone_trt_config, head_trt_config, head_no_prev_trt_config};
}

std::optional<VadOutputTopicData> VadNode::execute_inference(const VadInputTopicData & vad_input_topic_data)
{
  if (!vad_interface_ptr_ || !vad_model_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "VAD interface or model not initialized");
    return std::nullopt;
  }

  // Convert to VadInputData through VadInterface
  const auto vad_input = vad_interface_ptr_->convert_input(vad_input_topic_data);

  // Execute inference with VadModel
  const auto vad_output = vad_model_ptr_->infer(vad_input);

  const auto [base2map_transform, map2base_transform] = get_transform_matrix(*vad_input_topic_data.kinematic_state);
  // Convert to ROS types through VadInterface
  if (vad_output.has_value()) {
    const auto vad_output_topic_data = vad_interface_ptr_->convert_output(
      *vad_output, this->now(), trajectory_timestep_, base2map_transform);
    // Return VadOutputTopicData
    return vad_output_topic_data;
  }

  return std::nullopt;
}

void VadNode::publish(const VadOutputTopicData & vad_output_topic_data)
{
  // Publish selected trajectory
  trajectory_publisher_->publish(vad_output_topic_data.trajectory);

  // Publish candidate trajectories
  candidate_trajectories_publisher_->publish(vad_output_topic_data.candidate_trajectories);

  // // Publish predicted objects
  predicted_objects_publisher_->publish(vad_output_topic_data.objects);

  // Publish map points
  map_points_publisher_->publish(vad_output_topic_data.map_points);

  RCLCPP_DEBUG(this->get_logger(), "Published trajectories and predicted objects");
}

void VadNode::create_camera_image_subscribers(const rclcpp::QoS& sensor_qos)
{
  camera_image_subs_.resize(num_cameras_);
  std::vector<bool> use_raw_cameras = this->declare_parameter<std::vector<bool>>("node_params.use_raw");
  auto resolve_topic_name = [this](const std::string & query) {
    return this->get_node_topics_interface()->resolve_topic_name(query);
  };
  for (int32_t i = 0; i < num_cameras_; ++i) {
    const auto transport = use_raw_cameras[i] ? "raw" : "compressed";
    auto callback =
        [this, i](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          this->image_callback(msg, i);
        };

    const auto image_topic = resolve_topic_name("~/input/image" + std::to_string(i));
    camera_image_subs_[i] = image_transport::create_subscription(
        this,
        image_topic,
        callback,
        transport,
        sensor_qos.get_rmw_qos_profile());
  }
}

void VadNode::create_camera_info_subscribers(const rclcpp::QoS& camera_info_qos)
{
  camera_info_subs_.resize(num_cameras_);
  for (int32_t i = 0; i < num_cameras_; ++i) {
    auto callback =
        [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          this->camera_info_callback(msg, i);
        };

    camera_info_subs_[i] =
        this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "~/input/camera_info" + std::to_string(i),
            camera_info_qos, callback);
  }
}

}  // namespace autoware::tensorrt_vad

// Register the component with the ROS2 component system
// NOLINTNEXTLINE(readability-identifier-naming,cppcoreguidelines-avoid-non-const-global-variables)
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_vad::VadNode)
