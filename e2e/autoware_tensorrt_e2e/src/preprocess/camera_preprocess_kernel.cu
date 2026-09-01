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

#include "autoware/tensorrt_e2e/preprocess/camera_preprocess_kernel.hpp"

namespace autoware::tensorrt_e2e
{

namespace
{

__global__ void camera_resize_kernel(
  const uint8_t * __restrict__ input, uint8_t * __restrict__ resized, const int32_t input_width,
  const int32_t input_height, const int32_t output_width, const int32_t output_height,
  const int32_t num_cameras, const float scale_x, const float scale_y)
{
  const int32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const int32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  const int32_t camera = blockIdx.z;
  if (x >= output_width || y >= output_height || camera >= num_cameras) {
    return;
  }

  const size_t input_offset = static_cast<size_t>(camera) * input_width * input_height * 3;
  const size_t output_offset = static_cast<size_t>(camera) * output_width * output_height * 3;

  // Bilinear interpolation on the source image.
  const float src_x = (static_cast<float>(x) + 0.5f) * scale_x - 0.5f;
  const float src_y = (static_cast<float>(y) + 0.5f) * scale_y - 0.5f;

  const int32_t x0 = max(0, min(static_cast<int32_t>(floorf(src_x)), input_width - 1));
  const int32_t y0 = max(0, min(static_cast<int32_t>(floorf(src_y)), input_height - 1));
  const int32_t x1 = min(x0 + 1, input_width - 1);
  const int32_t y1 = min(y0 + 1, input_height - 1);

  const float wx = fminf(fmaxf(src_x - static_cast<float>(x0), 0.0f), 1.0f);
  const float wy = fminf(fmaxf(src_y - static_cast<float>(y0), 0.0f), 1.0f);

  for (int32_t c = 0; c < 3; ++c) {
    const float v00 = input[input_offset + (static_cast<size_t>(y0) * input_width + x0) * 3 + c];
    const float v01 = input[input_offset + (static_cast<size_t>(y0) * input_width + x1) * 3 + c];
    const float v10 = input[input_offset + (static_cast<size_t>(y1) * input_width + x0) * 3 + c];
    const float v11 = input[input_offset + (static_cast<size_t>(y1) * input_width + x1) * 3 + c];
    const float value =
      v00 * (1.0f - wx) * (1.0f - wy) + v01 * wx * (1.0f - wy) + v10 * (1.0f - wx) * wy +
      v11 * wx * wy;
    resized[output_offset + (static_cast<size_t>(y) * output_width + x) * 3 + c] =
      static_cast<uint8_t>(fminf(fmaxf(value + 0.5f, 0.0f), 255.0f));
  }
}

__global__ void camera_normalize_kernel(
  const uint8_t * __restrict__ resized, float * __restrict__ output, const int32_t output_width,
  const int32_t output_height, const int32_t num_cameras, const float mean_r, const float mean_g,
  const float mean_b, const float inv_std_r, const float inv_std_g, const float inv_std_b)
{
  const int32_t x = blockIdx.x * blockDim.x + threadIdx.x;
  const int32_t y = blockIdx.y * blockDim.y + threadIdx.y;
  const int32_t camera = blockIdx.z;
  if (x >= output_width || y >= output_height || camera >= num_cameras) {
    return;
  }

  const size_t pixels = static_cast<size_t>(output_width) * output_height;
  const size_t hwc_offset = (static_cast<size_t>(camera) * pixels +
                             static_cast<size_t>(y) * output_width + x) * 3;
  const size_t chw_offset =
    static_cast<size_t>(camera) * 3 * pixels + static_cast<size_t>(y) * output_width + x;

  // Input is BGR; output channel order is RGB.
  const float b = static_cast<float>(resized[hwc_offset + 0]);
  const float g = static_cast<float>(resized[hwc_offset + 1]);
  const float r = static_cast<float>(resized[hwc_offset + 2]);

  output[chw_offset + 0 * pixels] = (r - mean_r) * inv_std_r;
  output[chw_offset + 1 * pixels] = (g - mean_g) * inv_std_g;
  output[chw_offset + 2 * pixels] = (b - mean_b) * inv_std_b;
}

}  // namespace

cudaError_t launch_camera_resize_kernel(
  const uint8_t * d_input, uint8_t * d_resized, const CameraPreprocessConfig & config,
  cudaStream_t stream)
{
  const dim3 block(16, 16);
  const dim3 grid(
    (config.output_width + block.x - 1) / block.x, (config.output_height + block.y - 1) / block.y,
    config.num_cameras);

  const float scale_x =
    static_cast<float>(config.input_width) / static_cast<float>(config.output_width);
  const float scale_y =
    static_cast<float>(config.input_height) / static_cast<float>(config.output_height);

  camera_resize_kernel<<<grid, block, 0, stream>>>(
    d_input, d_resized, config.input_width, config.input_height, config.output_width,
    config.output_height, config.num_cameras, scale_x, scale_y);

  return cudaGetLastError();
}

cudaError_t launch_camera_normalize_kernel(
  const uint8_t * d_resized, float * d_output, const CameraPreprocessConfig & config,
  cudaStream_t stream)
{
  const dim3 block(16, 16);
  const dim3 grid(
    (config.output_width + block.x - 1) / block.x, (config.output_height + block.y - 1) / block.y,
    config.num_cameras);

  camera_normalize_kernel<<<grid, block, 0, stream>>>(
    d_resized, d_output, config.output_width, config.output_height, config.num_cameras,
    config.mean[0], config.mean[1], config.mean[2], config.inverse_std[0], config.inverse_std[1],
    config.inverse_std[2]);

  return cudaGetLastError();
}

}  // namespace autoware::tensorrt_e2e
