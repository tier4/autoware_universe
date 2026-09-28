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

#include "autoware/tensorrt_e2e/bev_feature/detection_decode.hpp"

namespace autoware::tensorrt_e2e
{

namespace
{

constexpr int THREADS_PER_BLOCK = 256;

// autoware_bevfusion's generateBoxes3D_kernel (autoware_universe main,
// perception/autoware_bevfusion/lib/postprocess/postprocess_kernel.cu), with the same
// float arithmetic, compacting instead of zero-scoring what it drops.
__global__ void decode_detections_kernel(
  const int64_t * __restrict__ label_pred, const float * __restrict__ bbox_pred,
  const float * __restrict__ score, const float * __restrict__ yaw_norm_thresholds,
  const float * __restrict__ squared_upper_limits, const float * __restrict__ score_thresholds,
  const DetectionDecodeGeometry geometry, DecodedBox * __restrict__ boxes,
  int32_t * __restrict__ count)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  const int n = geometry.num_proposals;
  if (i >= n) {
    return;
  }
  const int label = static_cast<int>(label_pred[i]);
  if (label < 0 || label >= geometry.num_classes) {
    return;
  }

  const float yaw_sin = bbox_pred[6 * n + i];
  const float yaw_cos = bbox_pred[7 * n + i];
  const float yaw_norm = sqrtf(yaw_sin * yaw_sin + yaw_cos * yaw_cos);
  const float box_score = yaw_norm >= yaw_norm_thresholds[label] ? score[i] : 0.f;
  if (box_score == 0.f) {
    return;
  }

  const float x =
    bbox_pred[0 * n + i] * geometry.out_size_factor * geometry.voxel_size_x + geometry.min_x_range;
  const float y =
    bbox_pred[1 * n + i] * geometry.out_size_factor * geometry.voxel_size_y + geometry.min_y_range;
  const float squared_distance = x * x + y * y;
  int band = -1;
  for (int b = 0; b < geometry.num_bands; ++b) {
    if (squared_distance < squared_upper_limits[b]) {
      band = b;
      break;
    }
  }
  if (band < 0 || box_score < score_thresholds[band * geometry.num_classes + label]) {
    return;
  }

  DecodedBox box;
  box.label = label;
  box.score = box_score;
  box.x = x;
  box.y = y;
  box.z = bbox_pred[2 * n + i];
  box.length = expf(bbox_pred[3 * n + i]);
  box.width = expf(bbox_pred[4 * n + i]);
  box.height = expf(bbox_pred[5 * n + i]);
  box.yaw = atan2f(yaw_sin, yaw_cos);
  box.vx = bbox_pred[8 * n + i];
  box.vy = bbox_pred[9 * n + i];
  box.proposal = i;
  boxes[atomicAdd(count, 1)] = box;
}

}  // namespace

cudaError_t launch_decode_detections(
  const int64_t * label_pred, const float * bbox_pred, const float * score,
  const float * yaw_norm_thresholds, const float * squared_upper_limits,
  const float * score_thresholds, const DetectionDecodeGeometry & geometry, DecodedBox * boxes,
  int32_t * count, cudaStream_t stream)
{
  if (geometry.num_proposals <= 0) {
    return cudaSuccess;
  }
  const int blocks = (geometry.num_proposals + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
  decode_detections_kernel<<<blocks, THREADS_PER_BLOCK, 0, stream>>>(
    label_pred, bbox_pred, score, yaw_norm_thresholds, squared_upper_limits, score_thresholds,
    geometry, boxes, count);
  return cudaGetLastError();
}

}  // namespace autoware::tensorrt_e2e
