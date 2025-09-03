#include "autoware/tensorrt_vad/vad_interface.hpp"
#include <opencv2/opencv.hpp>

namespace autoware::tensorrt_vad
{

VadInterface::VadInterface(const VadInterfaceConfig& config, const std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  : config_(config),
    prev_can_bus_(config.default_can_bus),
    vad_base2img_transform_(std::nullopt)
{
  // Initialize coordinate transformer
  coordinate_transformer_ = std::make_unique<vad_interface::CoordinateTransformer>(
    config.vad2base, config.base2vad, tf_buffer);
  
  // Initialize input converters
  input_image_converter_ = std::make_unique<vad_interface::InputImageConverter>(*coordinate_transformer_, config_);
  input_transform_matrix_converter_ = std::make_unique<vad_interface::InputTransformMatrixConverter>(*coordinate_transformer_, config_);
  input_can_bus_converter_ = std::make_unique<vad_interface::InputCanBusConverter>(*coordinate_transformer_, config_);
  input_bev_shift_converter_ = std::make_unique<vad_interface::InputBEVShiftConverter>(*coordinate_transformer_, config_);
  
  // Initialize output converters  
  output_trajectory_converter_ = std::make_unique<vad_interface::OutputTrajectoryConverter>(*coordinate_transformer_, config_);
  output_map_converter_ = std::make_unique<vad_interface::OutputMapConverter>(*coordinate_transformer_, config_);
  output_objects_converter_ = std::make_unique<vad_interface::OutputObjectsConverter>(*coordinate_transformer_, config_);
}

VadInputData VadInterface::convert_input(const VadInputTopicData & vad_input_topic_data)
{
  VadInputData vad_input_data;

  // Calculate scaling factors
  float scale_width = config_.target_image_width / static_cast<float>(config_.input_image_width);
  float scale_height = config_.target_image_height / static_cast<float>(config_.input_image_height);

  // Process lidar2img transformation using converter, with caching
  if (!vad_base2img_transform_.has_value()) {
    vad_base2img_transform_ = input_transform_matrix_converter_->process_lidar2img(
      vad_input_topic_data.camera_infos,
      scale_width, scale_height
    );
  }
  vad_input_data.vad_base2img_ = vad_base2img_transform_.value();
  
  // Process can_bus using converter
  vad_input_data.can_bus_ = input_can_bus_converter_->process_can_bus(
    vad_input_topic_data.kinematic_state,
    vad_input_topic_data.acceleration,
    prev_can_bus_
  );
  
  // Process shift using converter
  vad_input_data.shift_ = input_bev_shift_converter_->process_shift(vad_input_data.can_bus_, prev_can_bus_);
  
  // Process image data using converter
  vad_input_data.camera_images_ = input_image_converter_->process_image(vad_input_topic_data.images);
  
  // Set default command
  vad_input_data.command_ = config_.default_command;
  
  // Update prev_can_bus_ for next iteration
  prev_can_bus_ = vad_input_data.can_bus_;
  
  return vad_input_data;
}

VadOutputTopicData VadInterface::convert_output(
  const VadOutputData & vad_output_data,
  const rclcpp::Time & stamp,
  const double trajectory_timestep,
  const Eigen::Matrix4f & base2map_transform) const
{
  VadOutputTopicData output_topic_data;

  // Convert candidate trajectories using converter
  output_topic_data.candidate_trajectories = output_trajectory_converter_->process_candidate_trajectories(
    vad_output_data.predicted_trajectories_, stamp, trajectory_timestep, base2map_transform);
  
  // Convert trajectory using converter
  output_topic_data.trajectory = output_trajectory_converter_->process_trajectory(
    vad_output_data.predicted_trajectory_, stamp, trajectory_timestep, base2map_transform);

  // Convert map_points using converter
  output_topic_data.map_points = output_map_converter_->process_map_points(vad_output_data.map_polylines_, stamp, base2map_transform);

  // Convert predicted objects using converter
  output_topic_data.objects = output_objects_converter_->process_predicted_objects(vad_output_data.predicted_objects_, stamp, base2map_transform);

  return output_topic_data;
}

} // namespace autoware::tensorrt_vad

namespace autoware::tensorrt_vad
{

// VadInputTopicData class implementation
VadInputTopicData::VadInputTopicData(const int32_t num_cameras)
  : images(num_cameras), camera_infos(num_cameras), num_cameras_(num_cameras)
{
}

bool VadInputTopicData::is_complete() const
{
  if (static_cast<int32_t>(images.size()) != num_cameras_ || 
      static_cast<int32_t>(camera_infos.size()) != num_cameras_) {
    return false;
  }
  
  for (int32_t i = 0; i < num_cameras_; ++i) {
    if (!images[i] || !camera_infos[i]) return false;
  }
  
  return kinematic_state != nullptr && 
         acceleration != nullptr;
}

void VadInputTopicData::reset()
{
  frame_started_ = false;
  
  // Reset all images and camera_infos using clear + resize
  images.clear();
  images.resize(num_cameras_);
  camera_infos.clear();
  camera_infos.resize(num_cameras_);
  
  // Reset other data
  kinematic_state = nullptr;
  acceleration = nullptr;
}

void VadInputTopicData::ensure_frame_started(const rclcpp::Time& msg_stamp)
{
  if (!frame_started_) {
    stamp = msg_stamp;
    frame_started_ = true;
  }
}

void VadInputTopicData::set_image(const std::size_t camera_id, const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  if (camera_id < images.size()) {
    ensure_frame_started(msg->header.stamp);
    images[camera_id] = msg;
  }
}

void VadInputTopicData::set_camera_info(const std::size_t camera_id, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg)
{
  if (camera_id < camera_infos.size()) {
    ensure_frame_started(msg->header.stamp);
    camera_infos[camera_id] = msg;
  }
}

void VadInputTopicData::set_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  ensure_frame_started(msg->header.stamp);
  kinematic_state = msg;
}

void VadInputTopicData::set_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr& msg)
{
  ensure_frame_started(msg->header.stamp);
  acceleration = msg;
}

}  // namespace autoware::tensorrt_vad
