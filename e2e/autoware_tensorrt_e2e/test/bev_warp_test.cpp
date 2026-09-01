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

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <utility>

namespace autoware::tensorrt_e2e
{

namespace
{
constexpr int32_t kHeight = 180;
constexpr int32_t kWidth = 180;
constexpr float kHalfExtent = 122.4f;
constexpr float kCellSize = 2.0f * kHalfExtent / kHeight;  // Both axes share the extent/size.

Se2WarpParams make_params(
  const std::array<float, 4> & current_pose, const std::array<float, 4> & source_pose)
{
  Se2WarpParams params;
  std::copy(current_pose.begin(), current_pose.end(), params.current_pose);
  std::copy(source_pose.begin(), source_pose.end(), params.source_pose);
  params.half_extent_m = kHalfExtent;
  params.height = kHeight;
  params.width = kWidth;
  return params;
}

}  // namespace

TEST(BevWarpTest, IdentityPoseSamplesTheSamePixel)
{
  const auto params = make_params({10.0f, -5.0f, 0.6f, 0.8f}, {10.0f, -5.0f, 0.6f, 0.8f});
  const std::array<std::pair<int32_t, int32_t>, 3> cells = {{{0, 0}, {90, 45}, {179, 179}}};
  for (const auto & [row, col] : cells) {
    float source_row = 0.0f;
    float source_col = 0.0f;
    se2_warp_source_pixel(params, row, col, source_row, source_col);
    EXPECT_NEAR(source_row, static_cast<float>(row), 1e-3f);
    EXPECT_NEAR(source_col, static_cast<float>(col), 1e-3f);
  }
}

TEST(BevWarpTest, ForwardTranslationShiftsSamplingForward)
{
  // The vehicle moved one BEV cell forward (+x) between the source and current frames. A cell
  // at the current-frame position must sample one cell further forward in the source map
  // (source row index + 1, since row = x_forward).
  const auto params = make_params({kCellSize, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f});
  float source_row = 0.0f;
  float source_col = 0.0f;
  se2_warp_source_pixel(params, 90, 45, source_row, source_col);
  EXPECT_NEAR(source_row, 91.0f, 1e-3f);
  EXPECT_NEAR(source_col, 45.0f, 1e-3f);
}

TEST(BevWarpTest, LeftTranslationShiftsSamplingLeft)
{
  // One cell to the left (+y): column axis is y_left.
  const auto params = make_params({0.0f, kCellSize, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f});
  float source_row = 0.0f;
  float source_col = 0.0f;
  se2_warp_source_pixel(params, 90, 45, source_row, source_col);
  EXPECT_NEAR(source_row, 90.0f, 1e-3f);
  EXPECT_NEAR(source_col, 46.0f, 1e-3f);
}

TEST(BevWarpTest, NinetyDegreeRotationMapsAxes)
{
  // Current frame rotated +90 deg (heading = +y of the source frame), same origin. The
  // current-frame forward axis must sample the source-frame left axis.
  const auto params = make_params({0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 1.0f, 0.0f});

  // Physical centre cell: current (x=f, y=0) -> world (x=0, y=f) -> source (x=0, y=f).
  const int32_t row = 135;  // Forward of centre by 45.5 cells.
  const int32_t col = 89;   // The physical y at column 89 is -0.5 cells.
  float source_row = 0.0f;
  float source_col = 0.0f;
  se2_warp_source_pixel(params, row, col, source_row, source_col);

  // current physical x = (135.5/180*2-1)*half = +45.5 cells; y = -0.5 cells.
  // world = (+0.5 cells, +45.5 cells) after rotation; source frame equals world here.
  // source row = x/cell - 0.5 + centre = 0.5 + 89.5 = 90 ; source col = 45.5 + 89.5 = 135.
  EXPECT_NEAR(source_row, 90.0f, 1e-2f);
  EXPECT_NEAR(source_col, 135.0f, 1e-2f);
}

}  // namespace autoware::tensorrt_e2e
