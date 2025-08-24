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

#ifndef AUTOWARE_TENSORRT_VAD_VAD_INTERFACE_HPP_
#define AUTOWARE_TENSORRT_VAD_VAD_INTERFACE_HPP_


#include <vector>
#include <unordered_map>
#include <optional>
#include <tuple>
#include <map>
#include <cmath>

#include "vad_interface_config.hpp"
#include "coordinate_transformer.hpp"
#include "input_converter/image_converter.hpp"
#include "input_converter/transform_matrix_converter.hpp"
#include "input_converter/can_bus_converter.hpp"
#include "input_converter/bev_shift_converter.hpp"
#include "output_converter/trajectory_converter.hpp"
#include "output_converter/map_converter.hpp"
#include "output_converter/objects_converter.hpp"
#include "vad_model.hpp" 

namespace autoware::tensorrt_vad
{

// Forward declarations for converter classes
namespace vad_interface {
  class CoordinateTransformer;
  class InputImageConverter;
  class InputTransformMatrixConverter;
  class InputCanBusConverter;
  class InputBEVShiftConverter;
  class OutputTrajectoryConverter;
  class OutputMapConverter;
  class OutputObjectsConverter;
}

/**
 * @class VadInputTopicData
 * @brief Class for managing ROS topic data for VAD input
 */
class VadInputTopicData
{
public:
  // Constructor: Initialize vectors with specified number of cameras
  explicit VadInputTopicData(const int32_t num_cameras);

  // Check if frame is complete
  bool is_complete() const;

  // Reset frame data
  void reset();

  // Setter methods with frame initialization
  void set_image(const std::size_t camera_id, const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void set_camera_info(const std::size_t camera_id, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
  void set_kinematic_state(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void set_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr& msg);

  // Reference timestamp for current frame
  rclcpp::Time stamp;

  // Image data from multiple cameras.
  // Corresponds to ~/input/image0, ~/input/image1, ... remapped in launch file.
  // Vector index corresponds to autoware_camera_id.
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> images;

  // Camera calibration information corresponding to each image above
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> camera_infos;

  // Vehicle kinematic state data (from /localization/kinematic_state etc.)
  nav_msgs::msg::Odometry::ConstSharedPtr kinematic_state;

  // Acceleration data (from /localization/acceleration)
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr acceleration;

private:
  int32_t num_cameras_;
  bool frame_started_ = false;

  /**
   * @brief Ensure frame is started with the specified timestamp if not already started
   * 
   * This function checks the frame state and starts the frame with the specified timestamp
   * only if it has not been started yet. If the frame is already started, it does nothing.
   * 
   * @note This function assumes that proper locking has been acquired by the caller.
   *       Thread safety is the responsibility of the caller.
   * 
   * @param msg_stamp The timestamp to set when starting the frame
   */
  void ensure_frame_started(const rclcpp::Time& msg_stamp);
};

struct VadOutputTopicData
{
  autoware_internal_planning_msgs::msg::CandidateTrajectories candidate_trajectories;
  autoware_planning_msgs::msg::Trajectory trajectory;
  visualization_msgs::msg::MarkerArray map_points;  // Transformed map points in Autoware coordinate system
  autoware_perception_msgs::msg::PredictedObjects objects;
};

// Data structures for return values of each process_* method
using CameraImagesData = std::vector<float>;
using ShiftData = std::vector<float>;
using Lidar2ImgData = std::vector<float>;
using CanBusData = std::vector<float>;

/**
 * @class VadInterface
 * @brief Interface for converting VadInputTopicData (ROS topic data) to VADInputData format, and VadOutputData to VadOutputTopicData (ROS topic data).
 */

class VadInterface {
public:
  explicit VadInterface(const VadInterfaceConfig& config, 
                        const std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  VadInputData convert_input(const VadInputTopicData & vad_input_topic_data);
  VadOutputTopicData convert_output(
    const VadOutputData & vad_output_data, 
    const rclcpp::Time & stamp,
    const double trajectory_timestep,
    const Eigen::Matrix4f & base2map_transform) const;


private:
  // Configuration
  VadInterfaceConfig config_;
  
  // Coordinate transformer for all coordinate system conversions
  std::unique_ptr<vad_interface::CoordinateTransformer> coordinate_transformer_;
  
  // Input converters
  std::unique_ptr<vad_interface::InputImageConverter> input_image_converter_;
  std::unique_ptr<vad_interface::InputTransformMatrixConverter> input_transform_matrix_converter_;
  std::unique_ptr<vad_interface::InputCanBusConverter> input_can_bus_converter_;
  std::unique_ptr<vad_interface::InputBEVShiftConverter> input_bev_shift_converter_;
  
  // Output converters
  std::unique_ptr<vad_interface::OutputTrajectoryConverter> output_trajectory_converter_;
  std::unique_ptr<vad_interface::OutputMapConverter> output_map_converter_;
  std::unique_ptr<vad_interface::OutputObjectsConverter> output_objects_converter_;
  
  // Previous can_bus data for velocity calculation and other processes
  std::vector<float> prev_can_bus_;

  // Cached VAD base_link (coordinate used in VAD output trajectory) to camera transformation matrix
  std::optional<Lidar2ImgData> vad_base2img_transform_;
};

}  // namespace autoware::tensorrt_vad

#endif  // AUTOWARE_TENSORRT_VAD_VAD_INTERFACE_HPP_
