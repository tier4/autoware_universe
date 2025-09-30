// Copyright 2025 TIER IV.
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

#ifndef AUTOWARE_TENSORRT_VAD_COORDINATE_TRANSFORMER_HPP_
#define AUTOWARE_TENSORRT_VAD_COORDINATE_TRANSFORMER_HPP_

#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <string>
#include <tuple>

namespace autoware::tensorrt_vad::vad_interface {

/**
 * @brief CoordinateTransformer handles coordinate system transformations and TF lookups
 * 
 * This class manages:
 * - VAD coordinate system to Autoware coordinate system transformations
 * - TF buffer for dynamic coordinate transformations
 */
class CoordinateTransformer {
public:
  /**
   * @brief Constructor
   * @param tf_buffer Shared pointer to TF buffer for dynamic transformations
   */
  CoordinateTransformer(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);

  /**
   * @brief Lookup transformation from base_link to camera frame
   * @param source_frame Camera frame name (e.g., "camera0", "camera1")
   * @return Optional transformation matrix from base_link to camera
   */
  std::optional<Eigen::Matrix4f> lookup_base2cam(const std::string& source_frame) const;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  ///< TF buffer for dynamic transformations
};

} // namespace autoware::tensorrt_vad::vad_interface

#endif  // AUTOWARE_TENSORRT_VAD_COORDINATE_TRANSFORMER_HPP_
