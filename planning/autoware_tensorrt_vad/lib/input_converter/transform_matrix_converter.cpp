#include "autoware/tensorrt_vad/input_converter/transform_matrix_converter.hpp"
#include <rclcpp/rclcpp.hpp>

namespace autoware::tensorrt_vad::vad_interface {

InputTransformMatrixConverter::InputTransformMatrixConverter(
  const CoordinateTransformer& transformer, const VadInterfaceConfig& config)
  : Converter(transformer, config)
{
}

Eigen::Matrix4f InputTransformMatrixConverter::create_viewpad(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) const
{
  Eigen::Matrix3f k_matrix;
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      k_matrix(i, j) = camera_info->k[i * 3 + j];
    }
  }
  
  // Create viewpad
  Eigen::Matrix4f viewpad = Eigen::Matrix4f::Zero();
  viewpad.block<3, 3>(0, 0) = k_matrix;
  viewpad(3, 3) = 1.0f;
  
  return viewpad;
}

Eigen::Matrix4f InputTransformMatrixConverter::apply_scaling(
  const Eigen::Matrix4f& lidar2img, float scale_width, float scale_height) const
{
  Eigen::Matrix4f scale_matrix = Eigen::Matrix4f::Identity();
  scale_matrix(0, 0) = scale_width;
  scale_matrix(1, 1) = scale_height;
  return scale_matrix * lidar2img;
}

std::vector<float> InputTransformMatrixConverter::matrix_to_flat(const Eigen::Matrix4f& matrix) const
{
  std::vector<float> flat(16);
  int32_t k = 0;
  for (int32_t i = 0; i < 4; ++i) {
    for (int32_t j = 0; j < 4; ++j) {
      flat[k++] = matrix(i, j);
    }
  }
  return flat;
}

Lidar2ImgData InputTransformMatrixConverter::process_lidar2img(
  const std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr>& camera_infos,
  float scale_width, float scale_height) const
{
  std::vector<float> frame_lidar2img(16 * 6, 0.0f); // Reserve space for 6 cameras

  // Process each camera
  for (int32_t autoware_camera_id = 0; autoware_camera_id < 6; ++autoware_camera_id) {
    if (!camera_infos[autoware_camera_id]) {
      continue;
    }

    // Use transformer to lookup base2cam transformation
    auto base2cam_opt = coordinate_transformer_.lookup_base2cam(camera_infos[autoware_camera_id]->header.frame_id);
    if (!base2cam_opt) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("autoware_tensorrt_vad"), *rclcpp::Clock::make_shared(), 5000, 
                  "Failed to lookup transformation for camera %d", autoware_camera_id);
      continue;
    }
    Eigen::Matrix4f base2cam = *base2cam_opt;

    Eigen::Matrix4f viewpad = create_viewpad(camera_infos[autoware_camera_id]);
    
    // Get vad2base transformation from config through coordinate transformer
    // The coordinate transformer uses vad2base internally, but we need to access it for lidar2cam calculation
    // Calculate lidar2cam transformation: viewpad * base2cam * vad2base
    Eigen::Matrix4f lidar2cam_rt = base2cam * config_.vad2base;
    Eigen::Matrix4f lidar2img = viewpad * lidar2cam_rt;

    // Apply scaling
    Eigen::Matrix4f lidar2img_scaled = apply_scaling(lidar2img, scale_width, scale_height);

    std::vector<float> lidar2img_flat = matrix_to_flat(lidar2img_scaled);

    // Store result at VAD camera ID position after lidar2img calculation
    int32_t vad_camera_id = config_.autoware_to_vad_camera_mapping.at(autoware_camera_id);
    if (vad_camera_id >= 0 && vad_camera_id < 6) {
      std::copy(lidar2img_flat.begin(), lidar2img_flat.end(),
                frame_lidar2img.begin() + vad_camera_id * 16);
    }
  }

  return frame_lidar2img;
}

} // namespace autoware::tensorrt_vad::vad_interface
