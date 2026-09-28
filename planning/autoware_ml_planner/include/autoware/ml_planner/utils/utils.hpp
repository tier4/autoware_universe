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

#ifndef AUTOWARE__ML_PLANNER__UTILS__UTILS_HPP_
#define AUTOWARE__ML_PLANNER__UTILS__UTILS_HPP_

#include <Eigen/Dense>
#include <xtensor/xarray.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ml_planner::utils
{

/**
 * @brief Checks if the input map contains valid data.
 *
 * @param input_map An unordered_map with string keys and xtensor values.
 * @return True if the input map is valid, false otherwise.
 */
bool check_input_map(const std::unordered_map<std::string, xt::xarray<float>> & input_map);

/**
 * @brief Converts a geometry_msgs::msg::Pose to a 4x4 transformation matrix.
 *
 * @param pose The pose containing position and orientation information.
 * @return A 4x4 transformation matrix representing the pose.
 */
Eigen::Matrix4d pose_to_matrix4d(const geometry_msgs::msg::Pose & pose);

/**
 * @brief Extracts yaw angle from rotation matrix and converts to cos/sin representation.
 *
 * @param rotation_matrix 3x3 rotation matrix.
 * @return A pair containing cos(yaw) and sin(yaw).
 */
std::pair<float, float> rotation_matrix_to_cos_sin(const Eigen::Matrix3d & rotation_matrix);

/**
 * @brief Computes the inverse of a 4x4 transformation matrix.
 * @note This function assumes that the matrix represents a rigid transformation and uses the
 * properties of Eigen::Isometry3d internally instead of a general 4x4 matrix inversion for better
 * numerical stability and performance.
 * @param mat The transformation matrix to invert.
 * @return A 4x4 transformation matrix representing the inverse.
 */
Eigen::Matrix4d inverse(const Eigen::Matrix4d & mat);

/**
 * @brief Replicate single sample data for batch processing.
 * @param single_data Single sample data.
 * @param batch_size The number of times to replicate the data.
 * @return Tensor with a leading batch dimension.
 */
xt::xarray<float> replicate_for_batch(const xt::xarray<float> & single_data, int batch_size);

}  // namespace autoware::ml_planner::utils
#endif  // AUTOWARE__ML_PLANNER__UTILS__UTILS_HPP_
