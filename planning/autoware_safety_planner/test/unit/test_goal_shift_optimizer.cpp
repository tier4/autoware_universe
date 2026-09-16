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

#include "utils/reference_path_smoother.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace autoware::safety_planner
{

namespace
{
//! The ramp the goal connection lays down: 0 to l_goal, flat at both ends
std::vector<double> quintic_ramp(const std::size_t n, const double l_goal)
{
  std::vector<double> profile;
  profile.reserve(n + 1);
  for (std::size_t i = 0; i <= n; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(n);
    profile.push_back(l_goal * u * u * u * (10.0 - 15.0 * u + 6.0 * u * u));
  }
  return profile;
}

//! The curvature of the path the offsets describe, as optimize_goal_shift bounds it
double max_curvature_of(
  const std::vector<double> & profile, const std::vector<double> & curvature, const double ds)
{
  double result = 0.0;
  for (std::size_t i = 1; i + 1 < profile.size(); ++i) {
    const double k = curvature[i];
    const double a = 1.0 - k * profile[i];
    const double dl = (profile[i + 1] - profile[i - 1]) / (2.0 * ds);
    const double ddl = (profile[i - 1] - 2.0 * profile[i] + profile[i + 1]) / (ds * ds);
    result = std::max(
      result, std::abs((a * a * k + a * ddl + 2.0 * k * dl * dl) / std::pow(a * a + dl * dl, 1.5)));
  }
  return result;
}
}  // namespace

TEST(OptimizeGoalShift, BringsTheRampWithinTheCurvatureBound)
{
  constexpr double ds = 1.0;
  constexpr double limit = 0.13;
  const auto nominal = quintic_ramp(10, 3.0);
  const std::vector<double> straight(nominal.size(), 0.0);
  ASSERT_GT(max_curvature_of(nominal, straight, ds), limit);

  const std::vector<double> lo(nominal.size(), -10.0);
  const std::vector<double> hi(nominal.size(), 10.0);
  const auto optimized = optimize_goal_shift(nominal, straight, ds, limit, lo, hi);
  ASSERT_TRUE(optimized.has_value());
  EXPECT_LT(max_curvature_of(*optimized, straight, ds), limit + 1.0e-3);
  // The ends are kept, so the goal pose the ramp reaches is unchanged
  const auto n = nominal.size();
  EXPECT_NEAR(optimized->front(), nominal.front(), 1.0e-6);
  EXPECT_NEAR(optimized->back(), nominal.back(), 1.0e-6);
  EXPECT_NEAR((*optimized)[1] - optimized->front(), nominal[1] - nominal.front(), 1.0e-6);
  EXPECT_NEAR(optimized->back() - (*optimized)[n - 2], nominal.back() - nominal[n - 2], 1.0e-6);
}

TEST(OptimizeGoalShift, ReportsAnInfeasibleBound)
{
  constexpr double ds = 1.0;
  const auto nominal = quintic_ramp(10, 3.0);
  const std::vector<double> straight(nominal.size(), 0.0);
  // 3 m of shift over 10 m needs 4 * 3 / 10^2 = 0.12 1/m even at the bang-bang optimum
  const std::vector<double> lo(nominal.size(), -10.0);
  const std::vector<double> hi(nominal.size(), 10.0);
  EXPECT_FALSE(optimize_goal_shift(nominal, straight, ds, 0.05, lo, hi).has_value());
}

}  // namespace autoware::safety_planner
