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

#include "autoware/tensorrt_e2e/pose_discontinuity.hpp"

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
 * C++/CUDA implementation of the temporal-history semantics the model was trained with
 * (`temporal_cache` in the exporter's deployment contract; reference:
 * `deployment/temporal.py::TemporalBEVFeatureCache`):
 *
 * - Accepts one feature map per sensor frame at the sensor's own cadence and keeps every map
 *   inside the history window `(frames - 1) * interval_seconds + tolerance`. The history
 *   interval is a property of the *history*, not of the sensor: a 0.2 s history on a 10 Hz
 *   LiDAR stores five maps and selects every second one.
 * - `build_history()` assembles `[frames, C, H, W]` ordered current-to-past by selecting, for
 *   each step k, the cached map closest to `newest - k * interval_seconds` (within tolerance).
 *   Every slot is SE(2)-warped from its source ego frame into the target ego frame the plan
 *   starts in: the newest map's own frame when planning at the cloud stamp (slot 0 is then
 *   copied, not warped), or the newer planning pose, which warps slot 0 as well
 *   (`planning_time`; training's `points_pose -> center_pose`). This matches training, which
 *   anchors at every sensor frame while sampling history at the configured interval
 *   (`center_stride: 1` with `lidar_history_interval_seconds` >= the sensor period).
 * - A dropped sensor frame is filled the way a T4 window indexes its history: by position
 *   over the recorded scans (`load_history_sweeps`: frame `center - k * stride`), where a
 *   missing scan's slot is the next OLDER one, never a newer one. The training lists drop
 *   every window that spans a timeline hole (devkit `ego_discontinuities`), so the model has
 *   not seen a substitute: it is the least stale real map there is, and replayed plans made
 *   with one score as their neighbours do. A history step whose map was dropped therefore
 *   takes the newest cached map older than its target by at most
 *   `substitute_max_seconds`; a step with no map that close (an outage, not a drop) leaves a
 *   hole, and `ready()` turns false until the window refills (self-healing) instead of
 *   discarding the whole cache. Step 0 is never substituted. Only a non-monotonic timestamp
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
    //! Gap between history frames. 0.2 s is what the shipped models train on; a model
    //! with another cadence says so in its ml_package file.
    double interval_seconds{0.2};
    double interval_tolerance_seconds{0.02};
    //! How much older than its target a map may be to stand in for a dropped one: 0.2 s is
    //! two consecutive 10 Hz scans, 93 % of the scan-drop gaps in the recorded corpus; the
    //! rest are outages.
    double substitute_max_seconds{0.2};
    double bev_half_extent_m{122.4};
    bool duplicate_current_on_warmup{false};
    PoseContinuityLimits pose_limits;
  };

  enum class InsertResult { kConsecutive, kFirst, kGapReset, kPoseReset };

  /**
   * @throws std::runtime_error on invalid configuration or CUDA allocation failure.
   */
  TemporalBevCache(
    const Config & config, const int64_t channels, const int64_t height, const int64_t width);

  /**
   * @brief Insert the newest feature map (device `[C, H, W]`) with its source
   * ego pose.
   *
   * The pose is `[x, y, cos(yaw), sin(yaw)]` in the map frame, kept in double:
   * map coordinates are ~1e5 m on T4-style maps, and the SE(2) warp composes
   * pose differences — float storage would quantize a slow-speed inter-frame
   * displacement to centimetres before the warp ever sees it. A stamp before
   * the newest cached stamp resets the cache first and reports `kGapReset`.
   * Maps that fall out of the history window are evicted (their device buffers
   * are recycled). The copy is queued on `stream` and not waited for: the
   * caller may reuse the source buffer through work queued on the same stream
   * afterwards, and nothing else.
   */
  InsertResult insert(
    const float * d_feature, const std::array<double, 4> & pose, const rclcpp::Time & stamp,
    cudaStream_t stream);

  bool ready() const;
  size_t device_bytes() const {
    return (slots_.size() + free_slots_.size() + config_.frames) *
           frame_elements_ * sizeof(float);
  }
  int64_t cached_frames() const { return static_cast<int64_t>(slots_.size()); }
  //! History steps the current selection fills with an older map for a dropped one.
  int64_t substituted_frames() const;
  int64_t frames() const { return config_.frames; }

  /**
   * @brief Assemble the `[frames, C, H, W]` current-to-past history on the GPU, in the ego
   * frame of `target_pose` (`[x, y, cos(yaw), sin(yaw)]`, map frame, like `insert()`'s).
   *
   * A slot whose pose equals `target_pose` is copied; every other one is warped. Requires
   * `ready()`. The returned device buffer is owned by the cache and valid until the next
   * `insert()`/`build_history()` call. Its contents are complete in stream order: a consumer
   * on `stream` reads the finished history, a host reader synchronizes first.
   */
  const float * build_history(cudaStream_t stream, const std::array<double, 4> & target_pose);

  void reset();

  /**
   * @brief For each history step k, the index of the stamp closest to
   * `target = stamps[0] - k * interval_seconds` within tolerance; failing that (k >= 1), the
   * newest stamp older than the target by at most `substitute_max_seconds` (+ tolerance) and
   * short of the next step's target (- tolerance); -1 when neither exists.
   *
   * `stamps_newest_first` is ordered newest first; step 0 always selects index 0. Pure logic,
   * exposed for unit testing.
   */
  static std::vector<int64_t> select_history_slots(
    const std::vector<double> & stamps_newest_first, int64_t frames, double interval_seconds,
    double tolerance_seconds, double substitute_max_seconds);

  //! How much older than its target a substitute may be: the bound, short of the next step.
  static double substitute_reach_seconds(
    double interval_seconds, double tolerance_seconds, double substitute_max_seconds);

private:
  struct Slot
  {
    autoware::cuda_utils::CudaUniquePtr<float[]> feature;
    std::array<double, 4> pose{};
    rclcpp::Time stamp;
  };

  std::vector<int64_t> current_selection() const;

  bool warmup_complete_{false};
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
