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

#ifndef AUTOWARE__TENSORRT_E2E__PROVIDERS__BEV_FEATURE_INPUT_PROVIDER_HPP_
#define AUTOWARE__TENSORRT_E2E__PROVIDERS__BEV_FEATURE_INPUT_PROVIDER_HPP_

#include "autoware/tensorrt_e2e/bev_feature/temporal_bev_cache.hpp"
#include "autoware/tensorrt_e2e/bev_feature/trt_bev_feature_extractor.hpp"
#include "autoware/tensorrt_e2e/input_provider.hpp"

#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class BevFeatureInputProvider
 * @brief Produces a temporal LiDAR BEV feature history tensor (ResWorld-style models).
 *
 * Subscribes to `~/input/pointcloud` through `cuda_blackboard`, as `autoware_bevfusion` does:
 * a GPU-resident cloud is negotiated on `~/input/pointcloud/cuda` and a plain PointCloud2
 * publisher is accepted as the fallback. Per new LiDAR frame it runs the
 * frozen BEVFusion-L feature extractor once, caches the feature with its ego pose, and
 * assembles the `[1, K, C, H, W]` current-to-past history (older maps SE(2)-warped into the
 * newest frame's ego frame), exactly as specified by the ResWorld deployment contract.
 *
 * When `bev_feature.contract_path` points at the model's deployment contract JSON, the
 * temporal-cache parameters (frames, interval, BEV half extent, feature tensor name) are read
 * from it, keeping the model artifact the single source of truth.
 *
 * Claimable tensors:
 * - `bev_feature_history` (name configurable) `[1, K, C, H, W]`: device-resident.
 */
class BevFeatureInputProvider : public InputProviderInterface
{
public:
  explicit BevFeatureInputProvider(rclcpp::Node & node);
  ~BevFeatureInputProvider() override;

  BevFeatureInputProvider(const BevFeatureInputProvider &) = delete;
  BevFeatureInputProvider & operator=(const BevFeatureInputProvider &) = delete;

  std::string name() const override { return "bev_feature"; }
  std::vector<std::string> claim_inputs(const std::vector<TensorSpec> & engine_inputs) override;
  bool collect(
    const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
    std::string & error) override;

  /// autoware_bevfusion's `is_num_voxels_within_range`.
  void add_diagnostics(autoware_utils_diagnostics::DiagnosticsInterface & diagnostics) override
  {
    diagnostics.add_key_value(
      "is_num_voxels_within_range", extractor_ ? extractor_->last_voxels_within_range() : true);
  }
  /// The cloud behind the current history.
  std::optional<rclcpp::Time> latest_input_stamp() const override { return last_extracted_stamp_; }

private:
  void load_contract(const std::string & contract_path);

  rclcpp::Node & node_;

  // Deployment parameters (contract JSON values take precedence when provided)
  std::string history_tensor_name_;
  double max_delay_ms_{200.0};
  TemporalBevCache::Config cache_config_;
  TrtBevFeatureExtractor::Config extractor_config_;

  // Engine-derived configuration
  std::vector<int64_t> history_shape_;

  // Pipeline
  std::unique_ptr<TrtBevFeatureExtractor> extractor_;
  std::unique_ptr<TemporalBevCache> cache_;
  cudaStream_t stream_{nullptr};
  std::optional<rclcpp::Time> last_extracted_stamp_;
  const float * history_ptr_{nullptr};

  std::unique_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_;
  std::shared_ptr<const cuda_blackboard::CudaPointCloud2> latest_pointcloud_;
  mutable std::mutex mutex_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__BEV_FEATURE_INPUT_PROVIDER_HPP_
