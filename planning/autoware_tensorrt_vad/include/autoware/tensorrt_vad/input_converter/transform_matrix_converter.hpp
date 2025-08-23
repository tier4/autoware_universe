#ifndef AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_TRANSFORM_MATRIX_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_TRANSFORM_MATRIX_CONVERTER_HPP_

#include "../converter.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <vector>
#include <optional>
#include <Eigen/Dense>

namespace autoware::tensorrt_vad::vad_interface {

using Lidar2ImgData = std::vector<float>;

/**
 * @brief InputTransformMatrixConverter handles lidar-to-image transformation matrix calculation
 * 
 * This class processes camera calibration data to create transformation matrices:
 * - TF lookup for base_link to camera frame transformations
 * - Camera intrinsics matrix processing (viewpad creation)
 * - Image scaling application for resized images
 * - Lidar-to-image transformation matrix calculation and flattening
 */
class InputTransformMatrixConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param transformer Reference to coordinate transformer (contains tf_buffer)
   * @param config Reference to configuration containing camera parameters
   */
  InputTransformMatrixConverter(const CoordinateTransformer& transformer, const VadInterfaceConfig& config);

  /**
   * @brief Process camera info messages to generate lidar2img transformation matrices
   * @param camera_infos Vector of ROS CameraInfo messages from multiple cameras
   * @param scale_width Width scaling factor for image resizing
   * @param scale_height Height scaling factor for image resizing
   * @return Lidar2ImgData Flattened transformation matrices for all cameras
   */
  Lidar2ImgData process_lidar2img(
    const std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr>& camera_infos,
    const float scale_width, const float scale_height) const;

private:
  /**
   * @brief Create viewpad matrix from camera intrinsics
   * @param camera_info Camera calibration information
   * @return Eigen::Matrix4f 4x4 viewpad matrix with intrinsics and padding
   */
  Eigen::Matrix4f create_viewpad(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) const;

  /**
   * @brief Apply scaling transformation to lidar2img matrix
   * @param lidar2img Original lidar-to-image transformation matrix
   * @param scale_width Width scaling factor
   * @param scale_height Height scaling factor
   * @return Eigen::Matrix4f Scaled transformation matrix
   */
  Eigen::Matrix4f apply_scaling(const Eigen::Matrix4f& lidar2img, const float scale_width, const float scale_height) const;

  /**
   * @brief Convert 4x4 matrix to flattened vector in row-major order
   * @param matrix Input transformation matrix
   * @return std::vector<float> Flattened matrix as 16-element vector
   */
  std::vector<float> matrix_to_flat(const Eigen::Matrix4f& matrix) const;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_TRANSFORM_MATRIX_CONVERTER_HPP_
