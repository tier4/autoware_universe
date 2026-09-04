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

#include <array>
#include <cmath>
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
 * The parameters carry the RELATIVE transform `T_source^{-1} . T_current`, composed on the
 * host in double precision by `make_se2_warp_params`. Map-frame poses are ~1e5 m on T4-style
 * maps: composing per cell in float32 loses metre-scale information to cancellation (the
 * reference implementation documents the same failure for reduced precision), while the
 * relative rotation and metre-scale shift are exactly representable working values for the
 * kernel's float32 arithmetic.
 */
struct Se2WarpParams
{
  float cos_rel;   //!< Rotation of `T_source^{-1} . T_current`.
  float sin_rel;
  float shift_x;   //!< Translation of `T_source^{-1} . T_current` (metres, source frame).
  float shift_y;
  float half_extent_m;
  int32_t height;
  int32_t width;
};

/**
 * @brief Compose the warp parameters from map-frame poses in double precision.
 *
 * Poses are `[x, y, cos(yaw), sin(yaw)]`. Headings are re-normalized (odometry provides unit
 * headings up to rounding; normalizing keeps the warp rigid), matching the reference.
 */
inline Se2WarpParams make_se2_warp_params(
  const std::array<double, 4> & current_pose, const std::array<double, 4> & source_pose,
  const double half_extent_m, const int32_t height, const int32_t width)
{
  auto normalized = [](const std::array<double, 4> & pose) {
    const double norm = std::hypot(pose[2], pose[3]);
    return std::array<double, 2>{pose[2] / norm, pose[3] / norm};
  };
  const auto current_heading = normalized(current_pose);
  const auto source_heading = normalized(source_pose);

  const double delta_x = current_pose[0] - source_pose[0];
  const double delta_y = current_pose[1] - source_pose[1];

  Se2WarpParams params;
  // R_source^T . R_current, and R_source^T . (t_current - t_source).
  params.cos_rel = static_cast<float>(
    source_heading[0] * current_heading[0] + source_heading[1] * current_heading[1]);
  params.sin_rel = static_cast<float>(
    source_heading[0] * current_heading[1] - source_heading[1] * current_heading[0]);
  params.shift_x = static_cast<float>(source_heading[0] * delta_x + source_heading[1] * delta_y);
  params.shift_y = static_cast<float>(-source_heading[1] * delta_x + source_heading[0] * delta_y);
  params.half_extent_m = static_cast<float>(half_extent_m);
  params.height = height;
  params.width = width;
  return params;
}

/**
 * @brief Source pixel coordinates sampled for one output cell.
 *
 * For output cell (`row`, `col`) this computes `p_source = T_rel(p_current)` in pixel-centre
 * coordinates, entirely at metre scale. Callable from host code for unit testing and from the
 * CUDA kernel.
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

  // Current-frame cell -> source-frame cell.
  const float source_x = params.cos_rel * current_x - params.sin_rel * current_y + params.shift_x;
  const float source_y = params.sin_rel * current_x + params.cos_rel * current_y + params.shift_y;

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
