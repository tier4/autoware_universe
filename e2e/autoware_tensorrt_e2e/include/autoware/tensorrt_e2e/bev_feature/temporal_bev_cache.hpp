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

#ifndef AUTOWARE__TENSORRT_E2E__BEV_FEATURE__TEMPORAL_BEV_CACHE_HPP_
#define AUTOWARE__TENSORRT_E2E__BEV_FEATURE__TEMPORAL_BEV_CACHE_HPP_

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <rclcpp/time.hpp>

#include <cuda_runtime_api.h>

#include <array>
#include <cstdint>
#include <deque>

namespace autoware::tensorrt_e2e
{

/**
 * @class TemporalBevCache
 * @brief Current-to-past cache of BEV feature maps for temporal E2E planners.
 *
 * C++/CUDA implementation of the ResWorld deployment contract
 * (`temporal_cache` in the deployment contract JSON; reference:
 * `deployment/temporal.py::TemporalBEVFeatureCache`):
 *
 * - Holds the last `frames` feature maps with their source ego poses.
 * - `build_history()` assembles `[frames, C, H, W]` ordered current-to-past: slot 0 is the raw
 *   newest map (already in its own ego frame), older slots are SE(2)-warped from their source
 *   ego frame into the newest map's ego frame.
 * - Consecutive frames must be `interval_seconds` apart (within tolerance); a violated gap
 *   resets the cache (re-warmup), which `insert()` reports to the caller.
 * - Warmup either waits for `frames` real maps or duplicates the newest one
 *   (`duplicate_current_on_warmup`, the contract's `duplicate_current_until_ready`).
 */
class TemporalBevCache
{
public:
  struct Config
  {
    int64_t frames{3};
    double interval_seconds{0.1};
    double interval_tolerance_seconds{0.02};
    double bev_half_extent_m{122.4};
    bool duplicate_current_on_warmup{false};
  };

  enum class InsertResult { kConsecutive, kFirst, kGapReset };

  /**
   * @throws std::runtime_error on invalid configuration or CUDA allocation failure.
   */
  TemporalBevCache(
    const Config & config, const int64_t channels, const int64_t height, const int64_t width);

  /**
   * @brief Insert the newest feature map (device `[C, H, W]`) with its source ego pose.
   *
   * The pose is `[x, y, cos(yaw), sin(yaw)]` in the map frame. When the stamp gap to the
   * previous frame violates the interval contract, the cache resets first and reports
   * `kGapReset`. Synchronizes `stream` before returning (the source buffer may be reused by
   * the caller afterwards).
   */
  InsertResult insert(
    const float * d_feature, const std::array<float, 4> & pose, const rclcpp::Time & stamp,
    cudaStream_t stream);

  bool ready() const;
  int64_t cached_frames() const { return static_cast<int64_t>(slots_.size()); }
  int64_t frames() const { return config_.frames; }

  /**
   * @brief Assemble the `[frames, C, H, W]` current-to-past history on the GPU.
   *
   * Requires `ready()`. The returned device buffer is owned by the cache and valid until the
   * next `insert()`/`build_history()` call. Synchronizes `stream` before returning.
   */
  const float * build_history(cudaStream_t stream);

  void reset();

  /// Whether a stamp gap satisfies the interval contract. Exposed for unit testing.
  static bool is_consecutive(
    const double delta_seconds, const double interval_seconds, const double tolerance_seconds);

private:
  struct Slot
  {
    autoware::cuda_utils::CudaUniquePtr<float[]> feature;
    std::array<float, 4> pose{};
    rclcpp::Time stamp;
  };

  Config config_;
  int64_t channels_;
  int64_t height_;
  int64_t width_;
  size_t frame_elements_;

  std::deque<Slot> slots_;  //!< Front = newest.
  autoware::cuda_utils::CudaUniquePtr<float[]> history_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__BEV_FEATURE__TEMPORAL_BEV_CACHE_HPP_
