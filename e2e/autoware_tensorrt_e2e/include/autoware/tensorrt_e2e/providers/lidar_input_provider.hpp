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

#ifndef AUTOWARE__TENSORRT_E2E__PROVIDERS__LIDAR_INPUT_PROVIDER_HPP_
#define AUTOWARE__TENSORRT_E2E__PROVIDERS__LIDAR_INPUT_PROVIDER_HPP_

#include "autoware/tensorrt_e2e/input_provider.hpp"

#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class LidarInputProvider
 * @brief Produces a padded point tensor from the concatenated point cloud (BEVFusion-style).
 *
 * Subscribes to `~/input/pointcloud`. Claimable tensors (names configurable):
 * - `points` `[1, P, D]` (required), D in [3, 5]: x, y, z, intensity, time-lag placeholder (0).
 *   Padded with zeros to P; truncated with a throttled warning when the cloud is larger.
 * - `num_points` `[1, 1]` (or any single-element shape): the valid point count.
 *
 * Voxelization is intentionally not performed here; models are expected to embed it in the
 * engine (see docs/design.md, Non-goals).
 */
class LidarInputProvider : public InputProviderInterface
{
public:
  explicit LidarInputProvider(rclcpp::Node & node);

  std::string name() const override { return "lidar"; }
  std::vector<std::string> claim_inputs(const std::vector<TensorSpec> & engine_inputs) override;
  bool collect(
    const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
    std::string & error) override;

private:
  rclcpp::Node & node_;

  // Deployment parameters
  std::string points_tensor_name_;
  std::string num_points_tensor_name_;
  double max_delay_ms_{200.0};
  double intensity_scale_{1.0};

  // Engine-derived configuration
  std::vector<int64_t> points_shape_;
  std::vector<int64_t> num_points_shape_;
  int64_t max_points_{0};
  int64_t point_dim_{0};
  bool num_points_claimed_{false};

  std::unique_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_;
  std::shared_ptr<const cuda_blackboard::CudaPointCloud2> latest_pointcloud_;
  mutable std::mutex mutex_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__LIDAR_INPUT_PROVIDER_HPP_
