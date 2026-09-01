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

#ifndef AUTOWARE__TENSORRT_E2E__PROVIDERS__CAMERA_INPUT_PROVIDER_HPP_
#define AUTOWARE__TENSORRT_E2E__PROVIDERS__CAMERA_INPUT_PROVIDER_HPP_

#include "autoware/tensorrt_e2e/input_provider.hpp"
#include "autoware/tensorrt_e2e/preprocess/camera_preprocess_kernel.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tf2_ros/buffer.h>

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class CameraInputProvider
 * @brief Produces camera image tensors for 1..N cameras (`camera.num_cameras`).
 *
 * Subscribes to `~/input/camera{i}/image` (and `~/input/camera{i}/camera_info` when the model
 * takes intrinsics). Synchronization is front-critical (camera `camera.anchor_camera_index` is
 * the anchor); non-anchor cameras may optionally be zero-filled when dropped.
 *
 * Claimable tensors (names configurable):
 * - `camera_images` `[1, N, 3, H, W]` (required): normalized RGB float, device-resident.
 *   H and W are taken from the engine.
 * - `camera_intrinsics` `[1, N, 3, 3]`: K matrices rescaled to the model resolution.
 * - `camera2ego` `[1, N, 4, 4]`: camera-to-ego-frame transforms from TF.
 */
class CameraInputProvider : public InputProviderInterface
{
public:
  CameraInputProvider(rclcpp::Node & node, tf2_ros::Buffer & tf_buffer);
  ~CameraInputProvider() override;

  CameraInputProvider(const CameraInputProvider &) = delete;
  CameraInputProvider & operator=(const CameraInputProvider &) = delete;

  std::string name() const override { return "camera"; }
  std::vector<std::string> claim_inputs(const std::vector<TensorSpec> & engine_inputs) override;
  bool collect(
    const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
    std::string & error) override;

private:
  void allocate_gpu_buffers();
  void create_subscriptions();

  /// Snapshot the latest images, verify anchor freshness and per-camera synchronization.
  /// Dropped non-anchor cameras are returned as nullptr when zero-filling is allowed.
  bool take_synchronized_images(
    const rclcpp::Time & now, std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images,
    std::string & error) const;

  bool build_images_tensor(
    const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images, TensorMap & inputs,
    std::string & error);
  bool build_intrinsics_tensor(TensorMap & inputs, std::string & error) const;
  bool build_extrinsics_tensor(
    const std::vector<sensor_msgs::msg::Image::ConstSharedPtr> & images, TensorMap & inputs,
    std::string & error);

  rclcpp::Node & node_;
  tf2_ros::Buffer & tf_buffer_;

  // Deployment parameters
  int64_t num_cameras_{1};
  int64_t input_width_{0};
  int64_t input_height_{0};
  std::string transport_{"compressed"};
  std::string ego_frame_{"base_link"};
  std::string images_tensor_name_;
  std::string intrinsics_tensor_name_;
  std::string extrinsics_tensor_name_;
  double sync_tolerance_ms_{100.0};
  double max_delay_ms_{200.0};
  bool allow_dropped_cameras_{false};
  int64_t anchor_camera_index_{0};

  // Engine-derived configuration
  std::vector<int64_t> images_shape_;
  std::vector<int64_t> intrinsics_shape_;
  std::vector<int64_t> extrinsics_shape_;
  bool intrinsics_claimed_{false};
  bool extrinsics_claimed_{false};
  CameraPreprocessConfig preprocess_config_{};

  // Subscriptions and latched data
  std::vector<image_transport::Subscriber> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> latest_images_;
  std::vector<sensor_msgs::msg::CameraInfo::ConstSharedPtr> latest_camera_infos_;
  mutable std::mutex mutex_;

  // GPU pipeline
  autoware::cuda_utils::CudaUniquePtrHost<uint8_t[]> pinned_input_;
  autoware::cuda_utils::CudaUniquePtr<uint8_t[]> d_input_;
  autoware::cuda_utils::CudaUniquePtr<uint8_t[]> d_resized_;
  autoware::cuda_utils::CudaUniquePtr<float[]> d_output_;
  cudaStream_t stream_{nullptr};

  // Static transforms are looked up once and cached.
  std::vector<std::optional<std::array<float, 16>>> cached_extrinsics_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__CAMERA_INPUT_PROVIDER_HPP_
