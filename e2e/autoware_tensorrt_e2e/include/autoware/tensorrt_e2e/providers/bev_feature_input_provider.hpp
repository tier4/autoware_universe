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
#include "autoware/tensorrt_e2e/postprocess/detection_postprocessor.hpp"

#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>

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
 * newest frame's ego frame).
 *
 * Every value that describes the network -- history length and cadence, BEV extent, tensor
 * names, voxelization -- comes from the model's ml_package file, the one configuration the
 * model directory ships.
 *
 * The extractor graph also carries the frozen BEVFusion detection head, so the same
 * engine pass that produces the feature map produces the boxes AWML's own BEVFusion
 * export ships. They are decoded and published on `~/output/detected_objects` with
 * `autoware_bevfusion`'s own postprocessing; nothing in the planner path reads them, so
 * the decode runs in `finish_tick()`, after the trajectory is out.
 *
 * The extractor is launched from the point cloud callback itself, before the node's
 * pass is triggered, on the stream the node bound: by the time the pass collects the
 * context tensors on the CPU the feature map is already being computed, and `collect()`
 * only queues the cache insert and the history assembly behind it. Nothing here waits
 * for the device; the engine's wait for its outputs covers all of it.
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
  void bind_stream(cudaStream_t stream) override { stream_ = stream; }
  bool pace(std::function<void()> on_data) override;
  void finish_tick() override;

  bool collect(
    const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs,
    std::string & error) override;

  /// autoware_bevfusion's `is_num_voxels_within_range`, plus the detection count.
  void add_diagnostics(autoware_utils_diagnostics::DiagnosticsInterface & diagnostics) override
  {
    diagnostics.add_key_value(
      "is_num_voxels_within_range", extractor_ ? extractor_->last_voxels_within_range() : true);
    if (detection_postprocessor_) {
      diagnostics.add_key_value("detected_object_count", last_detected_object_count_);
    }
  }
  /// The cloud behind the current history.
  std::optional<rclcpp::Time> latest_input_stamp() const override { return last_extracted_stamp_; }

private:
  //! Declares the `bev_feature.detection.*` parameters; a no-op when the model is not
  //! configured for the head.
  void declare_detection_params();

  rclcpp::Node & node_;

  // Deployment parameters, from the package defaults and the model's ml_package file
  std::string history_tensor_name_;
  double max_delay_ms_{200.0};
  TemporalBevCache::Config cache_config_;
  TrtBevFeatureExtractor::Config extractor_config_;

  // Engine-derived configuration
  std::vector<int64_t> history_shape_;

  // Pipeline
  std::unique_ptr<TrtBevFeatureExtractor> extractor_;
  std::unique_ptr<TemporalBevCache> cache_;

  // The detection head riding along on the extractor graph. Null when the model does not
  // enable it or the graph does not carry it.
  DetectionPostprocessor::Config detection_config_;
  bool detection_requested_{false};
  std::unique_ptr<DetectionPostprocessor> detection_postprocessor_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_pub_;
  size_t last_detected_object_count_{0};
  //! The node's tick stream once bound, else this provider's own (then destroyed here).
  cudaStream_t stream_{nullptr};
  bool owns_stream_{false};
  //! The extraction queued by the last point cloud callback: the map it produced (or
  //! nullptr with the reason), the stamp of the cloud it came from, and whether the cache
  //! has taken it. Touched only from the callback and the pass it triggers.
  const float * pending_feature_{nullptr};
  std::string pending_error_;
  std::optional<rclcpp::Time> pending_stamp_;
  bool pending_inserted_{false};
  bool pending_detections_published_{false};
  std::optional<rclcpp::Time> last_extracted_stamp_;
  const float * history_ptr_{nullptr};

  std::unique_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    pointcloud_sub_;
  std::shared_ptr<const cuda_blackboard::CudaPointCloud2> latest_pointcloud_;
  //! Set when this provider paces the node; called on every new cloud.
  std::function<void()> on_data_;
  mutable std::mutex mutex_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PROVIDERS__BEV_FEATURE_INPUT_PROVIDER_HPP_
