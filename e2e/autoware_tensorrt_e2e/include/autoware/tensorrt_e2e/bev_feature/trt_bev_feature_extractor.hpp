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

#ifndef AUTOWARE__TENSORRT_E2E__BEV_FEATURE__TRT_BEV_FEATURE_EXTRACTOR_HPP_
#define AUTOWARE__TENSORRT_E2E__BEV_FEATURE__TRT_BEV_FEATURE_EXTRACTOR_HPP_

#include <autoware/bevfusion/bevfusion_config.hpp>
#include <autoware/bevfusion/preprocess/preprocess_kernel.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cuda_runtime_api.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class TrtBevFeatureExtractor
 * @brief Runs the frozen BEVFusion-L feature extractor: point cloud -> BEV feature map.
 *
 * Implements the ResWorld deployment boundary (`bevfusion` section of the deployment contract):
 * voxelization reuses the production `autoware_bevfusion` preprocessing, sparse convolution
 * runs inside the TensorRT engine (spconv via `autoware_tensorrt_plugins`), and the engine's
 * feature output tensor (default `bev_feature`, `[1, C, H, W]`) is exposed as a device buffer.
 *
 * The engine takes the same lidar-branch inputs as `autoware_bevfusion`
 * (`voxels`, `num_points_per_voxel`, `coors`, dynamic voxel count) and must expose the feature
 * map as an output with static dimensions.
 */
class TrtBevFeatureExtractor
{
public:
  struct Config
  {
    std::string onnx_path;
    std::string plugins_path;
    //! Engine cache path; empty derives it from onnx_path, as TrtCommon does.
    std::string engine_path;
    std::string precision{"fp16"};
    //! TensorRT builder workspace. Shared with the planner engine (`trt_workspace_mib`):
    //! the builder segfaults below a graph's need rather than failing, so both engines
    //! get the same deliberately generous pool.
    size_t max_workspace_size{4ULL << 30U};
    std::string feature_tensor{"bev_feature"};
    int64_t cloud_capacity{2000000};
    // Network description, read from the model's ml_package file with no fallback: a
    // default here would silently voxelize for a different network.
    int64_t max_points_per_voxel{0};
    std::vector<int64_t> voxels_num;       //!< [min, opt, max]
    std::vector<float> point_cloud_range;  //!< [x/y/z min, x/y/z max]
    std::vector<float> voxel_size;         //!< [x, y, z]
    bool use_intensity{false};
  };

  /**
   * @throws std::runtime_error on invalid configuration or engine setup failure.
   */
  TrtBevFeatureExtractor(const Config & config, cudaStream_t stream);

  int64_t channels() const { return channels_; }
  int64_t height() const { return height_; }
  int64_t width() const { return width_; }

  /**
   * @brief Extract the BEV feature map for one point cloud.
   *
   * The cloud is read straight from its device buffer, as `autoware_bevfusion` reads its
   * input; it must use the Autoware `PointXYZIRC` layout (the concatenated cloud format).
   * @return Device pointer to the `[C, H, W]` feature map (owned by the extractor, valid until
   *         the next call), or nullptr with `error` set.
   */
  const float * extract(const cuda_blackboard::CudaPointCloud2 & cloud, std::string & error);

  /// Voxel count of the last extract(), before clamping to the profile maximum.
  int64_t last_num_voxels() const { return last_num_voxels_; }
  /// False when the last cloud produced more voxels than the profile maximum and was clipped.
  bool last_voxels_within_range() const { return last_voxels_within_range_; }

private:
  void init_engine(const Config & config);
  bool validate_cloud_layout(
    const sensor_msgs::msg::PointCloud2 & cloud, std::string & error) const;

  autoware::bevfusion::BEVFusionConfig bevfusion_config_;
  std::string feature_tensor_;
  cudaStream_t stream_;

  std::unique_ptr<autoware::bevfusion::PreprocessCuda> preprocess_;
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> trt_common_;

  // Device buffers (allocated once at maximum size)
  autoware::cuda_utils::CudaUniquePtr<float[]> identity_transform_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> points_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> voxel_features_d_;
  autoware::cuda_utils::CudaUniquePtr<int32_t[]> voxel_coords_d_;
  autoware::cuda_utils::CudaUniquePtr<int32_t[]> num_points_per_voxel_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> feature_d_;

  int64_t channels_{0};
  int64_t height_{0};
  int64_t width_{0};
  int64_t last_num_voxels_{0};
  bool last_voxels_within_range_{true};
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__BEV_FEATURE__TRT_BEV_FEATURE_EXTRACTOR_HPP_
