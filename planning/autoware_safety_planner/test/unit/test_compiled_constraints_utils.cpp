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

#include "trajectory_planner/frenet_sampling_based_planner/compiled_constraints_utils.hpp"

#ifdef SAFETY_PLANNER_UNIT_TEST_PLOT
#include "test_plot_utils.hpp"
#endif

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! Vertices from s0 to s1 (either direction) every step, at a constant l
std::vector<SlPoint> make_leg(const double s0, const double s1, const double step, const double l)
{
  std::vector<SlPoint> leg;
  const auto n = static_cast<int>(std::round(std::abs(s1 - s0) / step));
  for (int i = 0; i <= n; ++i) {
    leg.push_back({s0 + (s1 - s0) * i / n, l});
  }
  return leg;
}

void append(std::vector<SlPoint> & piece, const std::vector<SlPoint> & leg)
{
  piece.insert(piece.end(), leg.begin(), leg.end());
}

//! With SAFETY_PLANNER_UNIT_TEST_PLOT, draws the pieces, the envelope and the rays from the
//! centerline to the envelope (as the lateral_bounds marker does) to test_results/unit/
void plot_envelope(
  [[maybe_unused]] const std::vector<std::vector<SlPoint>> & pieces,
  [[maybe_unused]] const std::vector<SlPoint> & envelope)
{
#ifdef SAFETY_PLANNER_UNIT_TEST_PLOT
  auto plt = autoware::pyplot::import();
  auto [fig, ax] = plt.subplots(Kwargs("figsize"_a = py::make_tuple(10, 5)));
  for (const auto & piece : pieces) {
    std::vector<double> s, l;
    for (const auto & p : piece) {
      s.push_back(p.s);
      l.push_back(p.l);
    }
    ax.plot(Args(s, l), Kwargs("color"_a = "tab:red", "marker"_a = ".", "linewidth"_a = 1.0));
  }
  // Not aligned with the vertices of the test cases, so that what lies between them shows
  for (double s = envelope.front().s; s <= envelope.back().s; s += 0.3) {
    const std::vector<double> ss{s, s};
    const std::vector<double> ll{0.0, interpolate_boundary_l(envelope, s)};
    ax.plot(Args(ss, ll), Kwargs("color"_a = "tab:orange", "linewidth"_a = 0.5));
  }
  std::vector<double> s, l;
  for (const auto & p : envelope) {
    s.push_back(p.s);
    l.push_back(p.l);
  }
  ax.plot(Args(s, l), Kwargs("color"_a = "tab:blue", "linewidth"_a = 2.0, "alpha"_a = 0.6));
  const std::vector<double> centerline_s{envelope.front().s, envelope.back().s};
  ax.plot(Args(centerline_s, std::vector<double>{0.0, 0.0}), Kwargs("color"_a = "k"));
  ax.set_xlabel(Args("s [m]"));
  ax.set_ylabel(Args("l [m]"));
  ax.set_aspect(Args("equal"));
  save_figure(plt, "unit");
#endif
}

//! The envelope interpolated every 0.1 m over [s0, s1] matches expected_l(s)
template <typename F>
void expect_envelope(
  const std::vector<SlPoint> & envelope, const double s0, const double s1, F expected_l)
{
  for (double s = s0; s <= s1 + 1e-9; s += 0.1) {
    EXPECT_NEAR(interpolate_boundary_l(envelope, s), expected_l(s), 1e-6) << "s = " << s;
  }
}

}  // namespace

// A curb that runs along the road, turns around an island and comes back further out (the right
// curb of the picture in the lateral_bounds marker): the returning leg must not show through
TEST(CompiledConstraintsUtils, LateralEnvelopeIgnoresTheLegFoldedBackBehind)
{
  std::vector<SlPoint> piece;
  append(piece, make_leg(0.0, 20.0, 0.5, -2.0));
  append(piece, {{22.0, -4.0}, {20.0, -6.0}});
  // densify leaves the return leg with vertices that do not line up with the outgoing ones
  append(piece, make_leg(19.75, 0.25, 0.5, -3.5));
  const std::vector<std::vector<SlPoint>> pieces{piece};

  const auto envelope = make_lateral_envelope(pieces, Side::RIGHT);
  plot_envelope(pieces, envelope);
  expect_envelope(envelope, 0.0, 20.0, [](double) { return -2.0; });
}

// Two separate pieces of the same boundary overlapping in s (the near and the far side of an
// island): the far one only has vertices at its ends
TEST(CompiledConstraintsUtils, LateralEnvelopeTakesTheNearestPiece)
{
  for (const double sign : {1.0, -1.0}) {
    const std::vector<std::vector<SlPoint>> pieces{
      make_leg(0.0, 30.0, 0.5, sign * 2.0), {{10.0, sign * 8.0}, {20.0, sign * 8.0}}};
    const auto envelope = make_lateral_envelope(pieces, sign > 0.0 ? Side::LEFT : Side::RIGHT);
    if (sign < 0.0) {
      plot_envelope(pieces, envelope);
    }
    expect_envelope(envelope, 0.0, 30.0, [sign](double) { return sign * 2.0; });
  }
}

// Where two pieces cross, the envelope follows whichever is nearer, including the crossing point
TEST(CompiledConstraintsUtils, LateralEnvelopeOfCrossingPieces)
{
  const std::vector<std::vector<SlPoint>> pieces{
    {{0.0, -1.0}, {10.0, -5.0}}, {{0.0, -5.0}, {10.0, -1.0}}};
  const auto envelope = make_lateral_envelope(pieces, Side::RIGHT);
  plot_envelope(pieces, envelope);
  expect_envelope(envelope, 0.0, 10.0, [](double s) { return -1.0 - 0.4 * std::min(s, 10.0 - s); });
}

// A single piece ascending in s is returned as it is
TEST(CompiledConstraintsUtils, LateralEnvelopeOfASinglePiece)
{
  const std::vector<std::vector<SlPoint>> pieces{{{0.0, 1.0}, {10.0, 3.0}, {20.0, 2.0}}};
  const auto envelope = make_lateral_envelope(pieces, Side::LEFT);
  plot_envelope(pieces, envelope);
  expect_envelope(envelope, 0.0, 20.0, [](double s) {
    return s < 10.0 ? 1.0 + 0.2 * s : 3.0 - 0.1 * (s - 10.0);
  });
}

TEST(CompiledConstraintsUtils, InterpolateBoundaryL)
{
  const std::vector<SlPoint> polyline{{0.0, 1.0}, {10.0, 3.0}, {20.0, 3.0}};
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, -5.0), 1.0);  // clamped before the start
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 5.0), 2.0);
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 15.0), 3.0);
  EXPECT_DOUBLE_EQ(interpolate_boundary_l(polyline, 25.0), 3.0);  // clamped after the end
}

TEST(CompiledConstraintsUtils, LateralBoundExtremeL)
{
  LateralBoundEntry bound;
  bound.polyline = {{0.0, 2.0}, {10.0, 1.0}, {20.0, 2.0}};

  double extreme_l = 0.0;
  bound.forbidden_side = Side::LEFT;
  ASSERT_TRUE(lateral_bound_extreme_l(bound, 5.0, 15.0, extreme_l));
  EXPECT_DOUBLE_EQ(extreme_l, 1.0);  // min(l) including the interior vertex

  bound.forbidden_side = Side::RIGHT;
  ASSERT_TRUE(lateral_bound_extreme_l(bound, 5.0, 15.0, extreme_l));
  EXPECT_DOUBLE_EQ(extreme_l, 1.5);  // max(l) is at the interval ends

  EXPECT_FALSE(lateral_bound_extreme_l(bound, 30.0, 40.0, extreme_l));  // no overlap in s
}

TEST(CompiledConstraintsUtils, ViolatesLateralBound)
{
  LateralBoundEntry bound;
  bound.polyline = {{0.0, 2.0}, {20.0, 2.0}};
  bound.forbidden_side = Side::LEFT;

  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{5.0, 10.0, -1.0, 1.9}));
  EXPECT_TRUE(violates_lateral_bound(bound, SlBox{5.0, 10.0, -1.0, 2.1}));
  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{30.0, 35.0, -1.0, 5.0}));  // beyond the polyline

  bound.forbidden_side = Side::RIGHT;
  EXPECT_TRUE(violates_lateral_bound(bound, SlBox{5.0, 10.0, 1.9, 3.0}));
  EXPECT_FALSE(violates_lateral_bound(bound, SlBox{5.0, 10.0, 2.1, 3.0}));
}

TEST(CompiledConstraintsUtils, ViolatesStopBar)
{
  StopBarEntry stop_bar;
  stop_bar.s_stop = 50.0;
  stop_bar.time = TimeWindow{2.0, 4.0};

  EXPECT_TRUE(violates_stop_bar(stop_bar, SlBox{45.0, 50.5, -1.0, 1.0}, 2.5, 3.0));
  EXPECT_FALSE(violates_stop_bar(stop_bar, SlBox{45.0, 49.5, -1.0, 1.0}, 2.5, 3.0));
  EXPECT_FALSE(violates_stop_bar(stop_bar, SlBox{45.0, 50.5, -1.0, 1.0}, 5.0, 6.0));  // inactive
}

}  // namespace autoware::safety_planner
