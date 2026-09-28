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

#pragma once
#include <cmath>

/// @brief rectangle to segment intersection check
__host__ __device__ __forceinline__ bool checkRectSegmentIntersections(
  const float ego_x, const float ego_y, const float ego_yaw, const float front_ext,
  const float back_ext, const float left_ext, const float right_ext, const float margin,
  const float * __restrict__ seg_x0, const float * __restrict__ seg_y0,
  const float * __restrict__ seg_x1, const float * __restrict__ seg_y1, const int num_segments)
{
  if (num_segments <= 0) return false;

  // 1. Calculate local AABB bounds with margin
  const float max_x = front_ext + margin;
  const float min_x = -back_ext - margin;
  const float max_y = left_ext + margin;
  const float min_y = -right_ext - margin;

  // 2. Pre-calculate center and half-extents for SAT math
  const float center_x = (max_x + min_x) * 0.5f;
  const float center_y = (max_y + min_y) * 0.5f;
  const float hx = (max_x - min_x) * 0.5f;
  const float hy = (max_y - min_y) * 0.5f;

  // 3. Compute rotation once per pose
  float s, c;
#ifdef __CUDA_ARCH__
  // Fast hardware intrinsic on GPU
  __sincosf(ego_yaw, &s, &c);
#else
  s = std::sin(ego_yaw);
  c = std::cos(ego_yaw);
#endif

// 4. Iterate over segments
// Unrolling helps the compiler pipeline memory loads and math instructions
#pragma unroll 4
  for (int i = 0; i < num_segments; ++i) {
    // Shift global coordinates to ego-relative origin
    const float dx0 = seg_x0[i] - ego_x;
    const float dy0 = seg_y0[i] - ego_y;
    const float dx1 = seg_x1[i] - ego_x;
    const float dy1 = seg_y1[i] - ego_y;

    // Rotate to align with AABB
    float lx0 = c * dx0 + s * dy0;
    float ly0 = -s * dx0 + c * dy0;
    float lx1 = c * dx1 + s * dy1;
    float ly1 = -s * dx1 + c * dy1;

    // Shift to exact AABB center
    lx0 -= center_x;
    ly0 -= center_y;
    lx1 -= center_x;
    ly1 -= center_y;

    // SAT Axis 1 & 2: AABB vs Segment Bounding Box
    // fminf/fmaxf map to fast single-cycle hardware instructions on GPU
    const float min_sx = fminf(lx0, lx1);
    const float max_sx = fmaxf(lx0, lx1);
    if (min_sx > hx || max_sx < -hx) continue;

    const float min_sy = fminf(ly0, ly1);
    const float max_sy = fmaxf(ly0, ly1);
    if (min_sy > hy || max_sy < -hy) continue;

    // SAT Axis 3: Segment Normal
    const float seg_dx = lx1 - lx0;
    const float seg_dy = ly1 - ly0;

    // R is the projection of AABB half-extents onto the line's normal
    const float R = hx * fabsf(seg_dy) + hy * fabsf(seg_dx);
    // D is the orthogonal distance from the AABB center to the line
    const float D = fabsf(lx0 * ly1 - lx1 * ly0);

    if (D > R) continue;

    // If it passes all 3 SAT tests, we have an intersection!
    return true;
  }
  return false;
}
