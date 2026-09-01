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

#ifndef AUTOWARE__TENSORRT_E2E__BEV_FEATURE__BEV_WARP_HPP_
#define AUTOWARE__TENSORRT_E2E__BEV_FEATURE__BEV_WARP_HPP_

#include <cuda_runtime.h>

#include <cstdint>

#ifdef __CUDACC__
#define TENSORRT_E2E_HOST_DEVICE __host__ __device__
#else
#define TENSORRT_E2E_HOST_DEVICE
#endif

namespace autoware::tensorrt_e2e
{

/**
 * @brief SE(2) warp of a BEV feature map from its source ego frame to the current ego frame.
 *
 * This replicates the ResWorld deployment reference (`deployment/temporal.py`,
 * `warp_bev_feature_to_current`): BEV axes are height = x (forward), width = y (left); both
 * spatial axes span [-half_extent_m, +half_extent_m]; sampling matches
 * `torch.nn.functional.grid_sample(mode="bilinear", padding_mode="zeros",
 * align_corners=False)` with pixel-centre coordinates.
 *
 * Poses are `[x, y, cos(yaw), sin(yaw)]` with unit headings.
 */
struct Se2WarpParams
{
  float current_pose[4];  //!< Pose whose ego frame is the output frame.
  float source_pose[4];   //!< Pose of the frame that produced the source feature.
  float half_extent_m;
  int32_t height;
  int32_t width;
};

/**
 * @brief Source pixel coordinates sampled for one output cell.
 *
 * For output cell (`row`, `col`) this computes
 * `p_source = T_source^{-1}(T_current(p_current))` in pixel-centre coordinates.
 * Callable from host code for unit testing and from the CUDA kernel.
 */
TENSORRT_E2E_HOST_DEVICE inline void se2_warp_source_pixel(
  const Se2WarpParams & params, const int32_t row, const int32_t col, float & source_row,
  float & source_col)
{
  // Physical coordinates of the output pixel centre in the current ego frame.
  const float current_x = ((static_cast<float>(row) + 0.5f) / static_cast<float>(params.height) *
                           2.0f - 1.0f) * params.half_extent_m;
  const float current_y = ((static_cast<float>(col) + 0.5f) / static_cast<float>(params.width) *
                           2.0f - 1.0f) * params.half_extent_m;

  // Current ego frame -> world.
  const float world_x = params.current_pose[0] + params.current_pose[2] * current_x -
                        params.current_pose[3] * current_y;
  const float world_y = params.current_pose[1] + params.current_pose[3] * current_x +
                        params.current_pose[2] * current_y;

  // World -> source ego frame.
  const float delta_x = world_x - params.source_pose[0];
  const float delta_y = world_y - params.source_pose[1];
  const float source_x = params.source_pose[2] * delta_x + params.source_pose[3] * delta_y;
  const float source_y = -params.source_pose[3] * delta_x + params.source_pose[2] * delta_y;

  // Physical -> pixel-centre coordinates (grid_sample align_corners=False unnormalization).
  source_row = (source_x / params.half_extent_m + 1.0f) * static_cast<float>(params.height) *
                 0.5f - 0.5f;
  source_col = (source_y / params.half_extent_m + 1.0f) * static_cast<float>(params.width) *
                 0.5f - 0.5f;
}

/**
 * @brief Warp a `[C, H, W]` feature map on the GPU (bilinear, zero padding).
 */
cudaError_t launch_se2_warp_kernel(
  const float * d_source, float * d_output, const Se2WarpParams & params, const int32_t channels,
  cudaStream_t stream);

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__BEV_FEATURE__BEV_WARP_HPP_
