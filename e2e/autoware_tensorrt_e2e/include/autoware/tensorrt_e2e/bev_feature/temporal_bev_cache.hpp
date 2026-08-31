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
#include <vector>

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
 * - Accepts one feature map per sensor frame at the sensor's own cadence and keeps every map
 *   inside the history window `(frames - 1) * interval_seconds + tolerance`. The contract
 *   interval is a property of the *history*, not of the sensor: a 0.2 s contract on a 10 Hz
 *   LiDAR stores five maps and selects every second one.
 * - `build_history()` assembles `[frames, C, H, W]` ordered current-to-past by selecting, for
 *   each step k, the cached map closest to `newest - k * interval_seconds` (within tolerance).
 *   Slot 0 is the raw newest map (already in its own ego frame); older slots are SE(2)-warped
 *   from their source ego frame into the newest map's ego frame. This matches training, which
 *   anchors at every sensor frame while sampling history at the contract interval
 *   (`center_stride: 1` with `lidar_history_interval_seconds` >= the sensor period).
 * - A dropped sensor frame leaves a hole: `ready()` turns false until the window refills
 *   (self-healing), instead of discarding the whole cache. Only a non-monotonic timestamp
 *   (time jump, bag loop) resets the cache, which `insert()` reports to the caller.
 * - Warmup either waits for a complete history or duplicates the newest map
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
   * The pose is `[x, y, cos(yaw), sin(yaw)]` in the map frame, kept in double: map coordinates
   * are ~1e5 m on T4-style maps, and the SE(2) warp composes pose differences — float storage
   * would quantize a slow-speed inter-frame displacement to centimetres before the warp ever
   * sees it. A stamp at or before the newest cached stamp resets the cache first and reports
   * `kGapReset`. Maps that fall out of the history window are evicted (their device buffers
   * are recycled). Synchronizes `stream` before returning (the source buffer may be reused by
   * the caller afterwards).
   */
  InsertResult insert(
    const float * d_feature, const std::array<double, 4> & pose, const rclcpp::Time & stamp,
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

  /**
   * @brief For each history step k, the index of the stamp closest to
   * `stamps[0] - k * interval_seconds` within tolerance, or -1 when no stamp qualifies.
   *
   * `stamps_newest_first` is ordered newest first; step 0 always selects index 0. Pure logic,
   * exposed for unit testing.
   */
  static std::vector<int64_t> select_history_slots(
    const std::vector<double> & stamps_newest_first, int64_t frames, double interval_seconds,
    double tolerance_seconds);

private:
  struct Slot
  {
    autoware::cuda_utils::CudaUniquePtr<float[]> feature;
    std::array<double, 4> pose{};
    rclcpp::Time stamp;
  };

  std::vector<int64_t> current_selection() const;

  Config config_;
  int64_t channels_;
  int64_t height_;
  int64_t width_;
  size_t frame_elements_;

  std::deque<Slot> slots_;        //!< Front = newest.
  std::vector<Slot> free_slots_;  //!< Evicted slots kept for device-buffer reuse.
  autoware::cuda_utils::CudaUniquePtr<float[]> history_;
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__BEV_FEATURE__TEMPORAL_BEV_CACHE_HPP_
