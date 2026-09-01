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

#include "autoware/tensorrt_e2e/bev_feature/temporal_bev_cache.hpp"

#include "autoware/tensorrt_e2e/bev_feature/bev_warp.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::tensorrt_e2e
{

TemporalBevCache::TemporalBevCache(
  const Config & config, const int64_t channels, const int64_t height, const int64_t width)
: config_(config),
  channels_(channels),
  height_(height),
  width_(width),
  frame_elements_(static_cast<size_t>(channels) * height * width)
{
  if (config_.frames < 2) {
    throw std::runtime_error(
      "TemporalBevCache: frames must be at least 2, got " + std::to_string(config_.frames));
  }
  if (config_.interval_seconds <= 0.0) {
    throw std::runtime_error("TemporalBevCache: interval_seconds must be positive");
  }
  if (config_.interval_tolerance_seconds < 0.0) {
    throw std::runtime_error("TemporalBevCache: interval_tolerance_seconds must be >= 0");
  }
  if (config_.bev_half_extent_m <= 0.0 || !std::isfinite(config_.bev_half_extent_m)) {
    throw std::runtime_error("TemporalBevCache: bev_half_extent_m must be finite and positive");
  }
  if (channels_ < 1 || height_ < 1 || width_ < 1) {
    throw std::runtime_error("TemporalBevCache: feature dimensions must be positive");
  }
  history_ =
    autoware::cuda_utils::make_unique<float[]>(static_cast<size_t>(config_.frames) * frame_elements_);
}

bool TemporalBevCache::is_consecutive(
  const double delta_seconds, const double interval_seconds, const double tolerance_seconds)
{
  return std::abs(delta_seconds - interval_seconds) <= tolerance_seconds;
}

TemporalBevCache::InsertResult TemporalBevCache::insert(
  const float * d_feature, const std::array<float, 4> & pose, const rclcpp::Time & stamp,
  cudaStream_t stream)
{
  InsertResult result = InsertResult::kConsecutive;
  if (slots_.empty()) {
    result = InsertResult::kFirst;
  } else {
    const double delta_seconds = (stamp - slots_.front().stamp).seconds();
    if (!is_consecutive(
          delta_seconds, config_.interval_seconds, config_.interval_tolerance_seconds)) {
      reset();
      result = InsertResult::kGapReset;
    }
  }

  Slot slot;
  if (static_cast<int64_t>(slots_.size()) >= config_.frames) {
    // Recycle the oldest device buffer instead of reallocating ~66 MB per frame.
    slot = std::move(slots_.back());
    slots_.pop_back();
  } else {
    slot.feature = autoware::cuda_utils::make_unique<float[]>(frame_elements_);
  }
  slot.pose = pose;
  slot.stamp = stamp;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    slot.feature.get(), d_feature, frame_elements_ * sizeof(float), cudaMemcpyDeviceToDevice,
    stream));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  slots_.push_front(std::move(slot));
  return result;
}

bool TemporalBevCache::ready() const
{
  if (config_.duplicate_current_on_warmup) {
    return !slots_.empty();
  }
  return static_cast<int64_t>(slots_.size()) >= config_.frames;
}

const float * TemporalBevCache::build_history(cudaStream_t stream)
{
  if (!ready()) {
    throw std::runtime_error("TemporalBevCache::build_history called before the cache is ready");
  }

  const Slot & newest = slots_.front();
  for (int64_t frame = 0; frame < config_.frames; ++frame) {
    // During duplicate-current warmup, missing history falls back to the oldest cached map.
    const int64_t slot_index = std::min(frame, static_cast<int64_t>(slots_.size()) - 1);
    const Slot & slot = slots_[slot_index];
    float * destination = history_.get() + static_cast<size_t>(frame) * frame_elements_;

    if (frame == 0 || slot.pose == newest.pose) {
      // The newest map is already in the target ego frame; identical poses need no warp.
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        destination, slot.feature.get(), frame_elements_ * sizeof(float),
        cudaMemcpyDeviceToDevice, stream));
      continue;
    }

    Se2WarpParams params;
    std::copy(newest.pose.begin(), newest.pose.end(), params.current_pose);
    std::copy(slot.pose.begin(), slot.pose.end(), params.source_pose);
    params.half_extent_m = static_cast<float>(config_.bev_half_extent_m);
    params.height = static_cast<int32_t>(height_);
    params.width = static_cast<int32_t>(width_);
    CHECK_CUDA_ERROR(launch_se2_warp_kernel(
      slot.feature.get(), destination, params, static_cast<int32_t>(channels_), stream));
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream));
  return history_.get();
}

void TemporalBevCache::reset()
{
  slots_.clear();
}

}  // namespace autoware::tensorrt_e2e
