// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#include "projection_test_cases.hpp"

#include <gtest/gtest.h>

#include <array>

// This translation unit tests the shared geometry with an ordinary C++17 compiler,
// without constructing a cost/model or requiring CUDA headers, libraries or a device.
#define __host__
#define __device__
#include <mppi/cost_functions/path_tracking_geometry.cuh>
#undef __device__
#undef __host__

namespace
{
using mppi::cost::detail::pathLengthAtProjection;
using mppi::cost::detail::projectPointToPolyline;

TEST(PathTrackingGeometry, GlobalProjectionIsIndependentOfHintAndLegacyStepBudget)
{
  for (const auto & c : autoware::mppi_optimizer::projection_test::cases()) {
    SCOPED_TRACE(c.name);
    const int n = static_cast<int>(c.x.size());
    // Includes every valid hint, no hint and out-of-range hints. Zero and six reproduce
    // the old bounded-search behavior; none may limit global coverage now.
    for (int hint = -2; hint <= n + 1; ++hint) {
      SCOPED_TRACE(hint);
      for (int budget : {-1, 0, 1, 6}) {
        SCOPED_TRACE(budget);
        const auto p =
          projectPointToPolyline(c.query_x, c.query_y, c.x.data(), c.y.data(), n, hint, budget);
        EXPECT_EQ(p.best_i, c.segment);
        EXPECT_NEAR(p.best_t_raw, c.t_raw, 1.0E-5F);
        EXPECT_NEAR(p.lateral_distance, c.lateral, 1.0E-5F);
      }
    }
  }
}

TEST(PathTrackingGeometry, EmptySingletonAndDegeneratePathsRetainTheirConventions)
{
  const auto empty = projectPointToPolyline(2, 3, nullptr, nullptr, 0, 100);
  EXPECT_EQ(empty.best_i, 0);
  EXPECT_FLOAT_EQ(empty.lateral_distance, 0);
  const float x[] = {1, 1, 1};
  const float y[] = {2, 2, 2};
  for (int n : {1, 2, 3}) {
    const auto p = projectPointToPolyline(2, 3, x, y, n, 1);
    EXPECT_EQ(p.best_i, 0);
    EXPECT_FLOAT_EQ(p.best_t_raw, 0);
    EXPECT_FLOAT_EQ(p.lateral_distance, 1);
  }
}

TEST(PathTrackingGeometry, GlobalProjectionDrivesProgressAndEndpointOvershoot)
{
  const float x[] = {0, 10, 10, 0};
  const float y[] = {0, 0, 10, 10};
  const float s[] = {0, 10, 20, 30};
  for (const float * cumulative : std::array<const float *, 2>{s, nullptr}) {
    float progress = 0.0F;
    float remaining = 0.0F;
    float overshoot = 0.0F;
    auto p = projectPointToPolyline(1, 9, x, y, 4, 0);
    pathLengthAtProjection(p, x, y, cumulative, 4, 30, progress, remaining, overshoot);
    EXPECT_FLOAT_EQ(progress, 29);
    EXPECT_FLOAT_EQ(remaining, 1);
    EXPECT_FLOAT_EQ(overshoot, 0);
    p = projectPointToPolyline(-2, 11, x, y, 4, 0);
    pathLengthAtProjection(p, x, y, cumulative, 4, 30, progress, remaining, overshoot);
    EXPECT_FLOAT_EQ(progress, 30);
    EXPECT_FLOAT_EQ(remaining, 0);
    EXPECT_NEAR(overshoot, 2, 1.0E-5F);
    EXPECT_FLOAT_EQ(p.lateral_distance, -1);
  }
}
}  // namespace
