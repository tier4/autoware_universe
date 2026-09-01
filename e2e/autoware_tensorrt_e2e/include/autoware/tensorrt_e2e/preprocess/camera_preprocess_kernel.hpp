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

#ifndef AUTOWARE__TENSORRT_E2E__PREPROCESS__CAMERA_PREPROCESS_KERNEL_HPP_
#define AUTOWARE__TENSORRT_E2E__PREPROCESS__CAMERA_PREPROCESS_KERNEL_HPP_

#include <cuda_runtime.h>

#include <cstdint>

namespace autoware::tensorrt_e2e
{

/**
 * @struct CameraPreprocessConfig
 * @brief Parameters for the GPU image preprocessing pipeline.
 *
 * Input images are BGR8 HWC, packed contiguously per camera. The output is normalized RGB
 * float in NCHW layout ([num_cameras, 3, output_height, output_width]).
 */
struct CameraPreprocessConfig
{
  int32_t input_width;
  int32_t input_height;
  int32_t output_width;
  int32_t output_height;
  int32_t num_cameras;
  float mean[3];         //!< Per-channel mean, RGB order.
  float inverse_std[3];  //!< Per-channel 1/std, RGB order.
};

/**
 * @brief Bilinearly resize all camera images (BGR8 HWC, contiguous per camera).
 */
cudaError_t launch_camera_resize_kernel(
  const uint8_t * d_input, uint8_t * d_resized, const CameraPreprocessConfig & config,
  cudaStream_t stream);

/**
 * @brief Convert resized BGR8 HWC images to normalized RGB float NCHW.
 */
cudaError_t launch_camera_normalize_kernel(
  const uint8_t * d_resized, float * d_output, const CameraPreprocessConfig & config,
  cudaStream_t stream);

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PREPROCESS__CAMERA_PREPROCESS_KERNEL_HPP_
