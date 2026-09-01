// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_E2E__TYPES_HPP_
#define AUTOWARE__TENSORRT_E2E__TYPES_HPP_

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @brief Element type of an engine IO tensor.
 *
 * Kept independent of nvinfer1::DataType so that input providers do not depend on TensorRT
 * headers.
 */
enum class TensorDataType { kFLOAT32, kBOOL, kINT32 };

/**
 * @brief Static description of one engine IO tensor (name, shape including batch, dtype).
 */
struct TensorSpec
{
  std::string name;
  std::vector<int64_t> shape;
  TensorDataType dtype{TensorDataType::kFLOAT32};

  int64_t num_elements() const;
};

/**
 * @brief A named tensor value exchanged between providers, the engine, and postprocessing.
 *
 * Data lives either on the host (`host_data`) or on the device (`device_data`, float32,
 * contiguous, owned by the producer and valid until the next `collect()` on that producer).
 */
struct Tensor
{
  std::vector<int64_t> shape;
  std::vector<float> host_data;
  const float * device_data{nullptr};

  bool is_device() const { return device_data != nullptr; }
  int64_t num_elements() const;

  static Tensor from_host(std::vector<int64_t> tensor_shape, std::vector<float> data);
  static Tensor from_device(std::vector<int64_t> tensor_shape, const float * data);
};

using TensorMap = std::unordered_map<std::string, Tensor>;

/**
 * @brief Per-tick ego state shared with all input providers and the postprocessor.
 *
 * `ego_to_map` uses the model reference pose (base_link, or the vehicle center when the node
 * runs with `shift_x: true`, mirroring the diffusion planner).
 */
struct EgoFrame
{
  nav_msgs::msg::Odometry odometry;
  /// Odometry with the pose replaced by the model reference pose (differs from `odometry` only
  /// when `shift_x: true`). Used for ego history features, mirroring the diffusion planner.
  nav_msgs::msg::Odometry reference_odometry;
  std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> acceleration;
  Eigen::Matrix4d ego_to_map{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d map_to_ego{Eigen::Matrix4d::Identity()};
  rclcpp::Time stamp;
};

int64_t shape_num_elements(const std::vector<int64_t> & shape);
std::string shape_to_string(const std::vector<int64_t> & shape);

/**
 * @brief Find a tensor spec by name. Returns nullptr when absent.
 */
const TensorSpec * find_spec(const std::vector<TensorSpec> & specs, const std::string & name);

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__TYPES_HPP_
