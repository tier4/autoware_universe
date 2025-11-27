// Copyright 2025 TIER IV, Inc.
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

#include "hermite_spline.hpp"
#include "test_utils_plotter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
TEST(HermiteSplineTest, LinearBehavior)
{
  std::vector<double> s = {0.0, 2.0, 4.0};
  std::vector<double> z = {0.0, 4.0, 8.0};  // Constant Slope = 2.0

  HermiteSpline spline;
  spline.build_impl(s, z, HermiteSpline::SplineType::Linear);

  // Value Check
  EXPECT_NEAR(spline.compute(1.0), 2.0, 1e-6);
  EXPECT_NEAR(spline.compute(3.0), 6.0, 1e-6);

  // Derivative Check (Constant Slope)
  EXPECT_NEAR(spline.compute_first_derivative(1.0), 2.0, 1e-6);

  // 2nd Derivative Check (Zero)
  EXPECT_NEAR(spline.compute_second_derivative(1.0), 0.0, 1e-6);

  plot_hermite_spline(spline, s, z, "test_hermite_linear");
}

// -----------------------------------------------------------------------------
// Test 2: Quadratic (Monotone Spline)
// -----------------------------------------------------------------------------
TEST(HermiteSplineTest, QuadraticCurve)
{
  // y = x^2  -> y' = 2x  -> y'' = 2
  std::vector<double> s = {0.0, 1.0, 2.0, 3.0};
  std::vector<double> y = {0.0, 1.0, 4.0, 9.0};

  HermiteSpline spline;
  spline.build_impl(s, y);  // Auto (defaults to Monotone for N<5)

  // Midpoint Value Check (Monotone PCHIP is an approximation)
  // At s=1.5, actual=2.25.
  EXPECT_NEAR(spline.compute(1.5), 2.25, 0.25);

  // Derivative Check at s=2.0 (Should be ~4.0)
  EXPECT_NEAR(spline.compute_first_derivative(2.0), 4.0, 0.5);

  plot_hermite_spline(spline, s, y, "test_hermite_quadratic");
}

// -----------------------------------------------------------------------------
// Test 3: Step Function (Overshoot Check)
// -----------------------------------------------------------------------------
TEST(HermiteSplineTest, MonotoneOvershoot)
{
  // Data: Flat(0) -> Rise -> Flat(1)
  // Standard Cubic Splines often overshoot > 1.0 or < 0.0 here.
  // Monotone splines guarantee output stays within [0.0, 1.0].
  std::vector<double> s = {0.0, 1.0, 2.0, 3.0, 4.0};
  std::vector<double> v = {0.0, 0.0, 1.0, 1.0, 1.0};

  HermiteSpline spline;
  spline.build_impl(s, v, HermiteSpline::SplineType::Monotone);

  // 1. Verify bounds (No Overshoot)
  for (double x = 0.0; x <= 4.0; x += 0.1) {
    double val = spline.compute(x);
    EXPECT_GE(val, -1e-4) << "Undershoot at s=" << x;
    EXPECT_LE(val, 1.0 + 1e-4) << "Overshoot at s=" << x;
  }

  // 2. Verify Monotonicity (Slope >= 0)
  for (double x = 0.0; x <= 4.0; x += 0.1) {
    EXPECT_GE(spline.compute_first_derivative(x), -1e-4) << "Negative slope at s=" << x;
  }

  plot_hermite_spline(spline, s, v, "test_hermite_monotone_step");
}

// -----------------------------------------------------------------------------
// Test 4: Akima Auto-Switching
// -----------------------------------------------------------------------------
TEST(HermiteSplineTest, AkimaZigZag)
{
  // 5 Points -> Should trigger Akima automatically
  // Akima handles "zig-zags" more smoothly/naturally than Monotone.
  std::vector<double> s = {0, 1, 2, 3, 4};
  std::vector<double> y = {0, 1, 0, 1, 0};

  HermiteSpline spline;
  bool success = spline.build_impl(s, y);  // Auto-detect
  ASSERT_TRUE(success);

  // Basic sanity check: interpolated peaks exist
  EXPECT_GT(spline.compute(0.5), 0.0);
  EXPECT_LT(spline.compute(0.5), 1.0);

  plot_hermite_spline(spline, s, y, "test_hermite_akima_zigzag");
}

// -----------------------------------------------------------------------------
// Test 5: Akima Zero-Denominator Edge Case
// -----------------------------------------------------------------------------
// Trigger condition: abs(|m4 - m3| + |m2 - m1|) < epsilon
// This happens when slopes are equal, e.g., a straight line or constant slope data.
TEST(HermiteSplineTest, AkimaZeroDenominator)
{
  // 5 Points to force Akima.
  // Straight line y=x. Slopes (delta) are all 1.0.
  // m1=1, m2=1, m3=1, m4=1.
  // |1-1| + |1-1| = 0.
  std::vector<double> s = {0, 1, 2, 3, 4};
  std::vector<double> y = {0, 1, 2, 3, 4};

  HermiteSpline spline;
  // Force Akima just to be sure, though size 5 would auto-select it.
  bool success = spline.build_impl(s, y, HermiteSpline::SplineType::Akima);
  ASSERT_TRUE(success);

  // The logic `(m2 + m3) / 2.0` should result in `(1+1)/2 = 1.0`.
  // The first derivative should be exactly 1.0 everywhere.
  EXPECT_NEAR(spline.compute(1.5), 1.5, 1e-6);
  EXPECT_NEAR(spline.compute_first_derivative(1.5), 1.0, 1e-6);
  EXPECT_NEAR(spline.compute_second_derivative(1.5), 0.0, 1e-6);

  plot_hermite_spline(spline, s, y, "test_hermite_akima_linear_edge_case");
}

// -----------------------------------------------------------------------------
// Test 6: R-Value Build Overload
// -----------------------------------------------------------------------------
TEST(HermiteSplineTest, BuildWithMoveSemantics)
{
  std::vector<double> s = {0.0, 1.0, 2.0};
  std::vector<double> y = {0.0, 1.0, 4.0};

  HermiteSpline spline;

  // Use std::move to invoke the r-value overload
  bool success = spline.build_impl(s, std::move(y));

  ASSERT_TRUE(success);

  // Verify spline still works correctly
  EXPECT_NEAR(spline.compute(1.0), 1.0, 1e-6);
  EXPECT_NEAR(spline.compute(2.0), 4.0, 1e-6);

  // Verify 'y' vector is in a valid state (likely empty or unspecified but valid)
  // Note: Standard only guarantees "valid but unspecified", but usually it's empty.
  // We just ensure the code didn't crash.
}
}  // namespace autoware::time_to_space_trajectory_converter
