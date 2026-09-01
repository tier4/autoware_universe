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

#include "autoware/tensorrt_e2e/bev_feature/bev_warp.hpp"

namespace autoware::tensorrt_e2e
{

namespace
{

__global__ void se2_warp_kernel(
  const float * __restrict__ source, float * __restrict__ output, const Se2WarpParams params,
  const int32_t channels)
{
  const int32_t col = blockIdx.x * blockDim.x + threadIdx.x;
  const int32_t row = blockIdx.y * blockDim.y + threadIdx.y;
  if (row >= params.height || col >= params.width) {
    return;
  }

  float source_row = 0.0f;
  float source_col = 0.0f;
  se2_warp_source_pixel(params, row, col, source_row, source_col);

  // Bilinear taps with zero padding (grid_sample padding_mode="zeros" semantics: out-of-range
  // taps contribute zero, in-range taps keep their bilinear weight).
  const int32_t row0 = static_cast<int32_t>(floorf(source_row));
  const int32_t col0 = static_cast<int32_t>(floorf(source_col));
  const int32_t row1 = row0 + 1;
  const int32_t col1 = col0 + 1;
  const float weight_row1 = source_row - static_cast<float>(row0);
  const float weight_col1 = source_col - static_cast<float>(col0);
  const float weight_row0 = 1.0f - weight_row1;
  const float weight_col0 = 1.0f - weight_col1;

  const bool row0_valid = row0 >= 0 && row0 < params.height;
  const bool row1_valid = row1 >= 0 && row1 < params.height;
  const bool col0_valid = col0 >= 0 && col0 < params.width;
  const bool col1_valid = col1 >= 0 && col1 < params.width;

  const float w00 = (row0_valid && col0_valid) ? weight_row0 * weight_col0 : 0.0f;
  const float w01 = (row0_valid && col1_valid) ? weight_row0 * weight_col1 : 0.0f;
  const float w10 = (row1_valid && col0_valid) ? weight_row1 * weight_col0 : 0.0f;
  const float w11 = (row1_valid && col1_valid) ? weight_row1 * weight_col1 : 0.0f;

  const size_t plane = static_cast<size_t>(params.height) * params.width;
  const size_t idx00 = static_cast<size_t>(max(row0, 0)) * params.width + max(col0, 0);
  const size_t idx01 = static_cast<size_t>(max(row0, 0)) * params.width + min(col1, params.width - 1);
  const size_t idx10 = static_cast<size_t>(min(row1, params.height - 1)) * params.width + max(col0, 0);
  const size_t idx11 =
    static_cast<size_t>(min(row1, params.height - 1)) * params.width + min(col1, params.width - 1);
  const size_t out_idx = static_cast<size_t>(row) * params.width + col;

  for (int32_t channel = 0; channel < channels; ++channel) {
    const float * source_plane = source + static_cast<size_t>(channel) * plane;
    const float value = w00 * source_plane[idx00] + w01 * source_plane[idx01] +
                        w10 * source_plane[idx10] + w11 * source_plane[idx11];
    output[static_cast<size_t>(channel) * plane + out_idx] = value;
  }
}

}  // namespace

cudaError_t launch_se2_warp_kernel(
  const float * d_source, float * d_output, const Se2WarpParams & params, const int32_t channels,
  cudaStream_t stream)
{
  const dim3 block(16, 16);
  const dim3 grid(
    (params.width + block.x - 1) / block.x, (params.height + block.y - 1) / block.y);
  se2_warp_kernel<<<grid, block, 0, stream>>>(d_source, d_output, params, channels);
  return cudaGetLastError();
}

}  // namespace autoware::tensorrt_e2e
