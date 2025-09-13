#ifndef AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_TRANSFORM_MATRIX_CONVERTER_HPP_
#define AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_TRANSFORM_MATRIX_CONVERTER_HPP_

#include "autoware/tensorrt_vad/converter.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <vector>
#include <optional>
#include <Eigen/Dense>

namespace autoware::tensorrt_vad::vad_interface {

using VadBase2ImgData = std::vector<float>;

/**
 * @brief InputTransformMatrixConverter handles vad base_link to image (camera optical link) transformation matrix calculation
 * 
 * This class processes camera calibration data to create transformation matrices:
 * - TF lookup for base_link to camera frame transformations
 * - Camera intrinsics matrix processing (cam2img matrix creation)
 * - Image scaling application for resized images
 * - vad base_link to image (camera optical link) transformation matrix calculation and flattening
 */
class InputTransformMatrixConverter : public Converter {
public:
  /**
   * @brief Constructor
   * @param coordinate_transformer Reference to coordinate transformer (contains tf_buffer)
   * @param config Reference to configuration containing camera parameters
   */
  InputTransformMatrixConverter(const CoordinateTransformer& coordinate_transformer, const VadInterfaceConfig& config);

  /**
   * @brief Process camera info messages to generate vad base_link (coordinate used in VAD inference) to image (camera optical link) transformation matrices
   * @param camera_infos Vector of ROS CameraInfo messages from multiple cameras
   * @return VadBase2ImgData Flattened transformation matrices for all cameras
   */
  VadBase2ImgData process_vad_base2img(
    const std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr>& camera_infos) const;

private:
  /**
   * @brief Create cam2img(4x4 camera_link to camera_optical_link matrix, as known as viewpad) matrix from camera intrinsics
   * @param camera_info Camera calibration information
   * @return Eigen::Matrix4f 4x4 cam2img matrix with intrinsics and padding
   */
  Eigen::Matrix4f create_cam2img(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) const;


  /**
   * @brief Calculate scaling factors from original image (width, height) and target image (width, height)
   * @param camera_info Camera calibration information containing original image (width, height)
   * @return std::pair<float, float> Scale factors for width and height
   */
  std::pair<float, float> calculate_scale(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& camera_info) const;

  /**
   * @brief Apply scaling transformation to vad base_link (coordinate used in VAD inference) to image (camera optical link) transformation matrix
   * @param vad_base2img Original vad base_link to image (camera optical link) transformation matrix
   * @param scale_width Width scaling factor
   * @param scale_height Height scaling factor
   * @return Eigen::Matrix4f Scaled transformation matrix
   */
  Eigen::Matrix4f apply_scaling(const Eigen::Matrix4f& vad_base2img, const float scale_width, const float scale_height) const;

  /**
   * @brief Convert 4x4 matrix to flattened vector in row-major order
   * @param matrix Input transformation matrix
   * @return std::vector<float> Flattened matrix as 16-element vector
   */
  std::vector<float> matrix_to_flat(const Eigen::Matrix4f& matrix) const;
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_INPUT_CONVERTER_TRANSFORM_MATRIX_CONVERTER_HPP_
