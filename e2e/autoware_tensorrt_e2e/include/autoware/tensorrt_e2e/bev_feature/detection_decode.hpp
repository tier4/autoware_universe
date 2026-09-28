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

#ifndef AUTOWARE__TENSORRT_E2E__BEV_FEATURE__DETECTION_DECODE_HPP_
#define AUTOWARE__TENSORRT_E2E__BEV_FEATURE__DETECTION_DECODE_HPP_

#include <cuda_runtime.h>

#include <cstdint>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @brief One decoded proposal that passed the score thresholds.
 *
 * The fields of `autoware::bevfusion::Box3D`, plus the proposal's index in the head's
 * output: the host restores `autoware_bevfusion`'s order (a stable descending sort over the
 * proposals) from it. Kept separate from Box3D so the kernel needs no autoware_bevfusion
 * headers.
 */
struct DecodedBox
{
  int32_t label;
  float score;
  float x;
  float y;
  float z;
  float width;
  float length;
  float height;
  float yaw;
  float vx;
  float vy;
  int32_t proposal;
};

/// The TransFusion bbox coder's geometry, as `autoware_bevfusion` decodes with it.
struct DetectionDecodeGeometry
{
  int32_t num_proposals;
  int32_t num_classes;
  int32_t num_bands;
  float out_size_factor;
  float voxel_size_x;
  float voxel_size_y;
  float min_x_range;
  float min_y_range;
};

/**
 * @brief The score thresholds in the layout the kernel reads.
 *
 * `autoware_bevfusion`'s `detection_score_thresholds`: `distance_bin_upper_limits` [m],
 * strictly ascending, and one threshold per class per band, distance-major
 * (`thresholds[band * num_classes + label]`). Built as that node builds it: the limits are
 * squared in float, and a threshold outside [0, 1) is read as 0.
 */
struct ScoreThresholdTable
{
  std::vector<float> squared_upper_limits;
  std::vector<float> thresholds;
};

/**
 * @throws std::runtime_error when there is no band, the limits are not positive and
 *         strictly ascending, or the thresholds are not one per class per band.
 */
ScoreThresholdTable make_score_threshold_table(
  const std::vector<double> & distance_bin_upper_limits,
  const std::vector<double> & score_thresholds, size_t num_classes);

/**
 * @brief Decode the head's proposals and cut them, compacting the survivors.
 *
 * One thread per proposal, with `autoware_bevfusion`'s `generateBoxes3D_kernel` arithmetic:
 * the yaw-norm gate, the centre's radial distance picking the first band whose upper limit
 * exceeds it (none: dropped), then the band's threshold for the class. A proposal whose
 * label is outside [0, num_classes) is dropped, where that kernel would read out of bounds.
 * Survivors are appended to `boxes` in no particular order; `count` must be zero on entry.
 */
cudaError_t launch_decode_detections(
  const int64_t * label_pred, const float * bbox_pred, const float * score,
  const float * yaw_norm_thresholds, const float * squared_upper_limits,
  const float * score_thresholds, const DetectionDecodeGeometry & geometry, DecodedBox * boxes,
  int32_t * count, cudaStream_t stream);

/// `autoware_bevfusion`'s order: score descending, ties in proposal order (its radix sort
/// is stable).
void sort_like_bevfusion(std::vector<DecodedBox> & boxes);

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__BEV_FEATURE__DETECTION_DECODE_HPP_
