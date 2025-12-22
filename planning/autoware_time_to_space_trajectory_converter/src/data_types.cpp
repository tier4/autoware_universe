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

#include "data_types.hpp"

#include <algorithm>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
void SplineData::add_point(double new_s, const PlannerPoint & p, double wait_t)
{
  s.push_back(new_s);
  t.push_back(p.t);
  x.push_back(p.pos.x);
  y.push_back(p.pos.y);
  z.push_back(p.pos.z);
  v.push_back(p.v);
  a.push_back(p.a);
  wait_times.push_back(wait_t);
}

void SplineData::reset(size_t capacity)
{
  s.clear();
  x.clear();
  y.clear();
  z.clear();
  v.clear();
  a.clear();
  t.clear();
  wait_times.clear();

  if (s.capacity() < capacity) {
    s.reserve(capacity);
    x.reserve(capacity);
    y.reserve(capacity);
    z.reserve(capacity);
    v.reserve(capacity);
    a.reserve(capacity);
    t.reserve(capacity);
    wait_times.reserve(capacity);
  }
}

void SplineData::set_config(double v_thresh)
{
  th_stop_velocity_mps = v_thresh;
}

SplineSet SplineSet::build(const SplineData & input, bool recompute_accel)
{
  SplineSet splines;
  splines.sx.build_impl(input.s, input.x);
  splines.sy.build_impl(input.s, input.y);
  splines.sz.build_impl(input.s, input.z, HermiteSpline::SplineType::Linear);
  splines.sv.build_impl(input.s, input.v);

  // We only build the acceleration spline if we intend to use the original data
  splines.use_original_accel = !input.a.empty() && !recompute_accel;
  if (splines.use_original_accel) {
    splines.sa.build_impl(input.s, input.a);
  }
  return splines;
}

void SplineData::smooth_acceleration(double resolution)
{
  // Heuristic: Smooth over ~1.5 meters of travel
  auto window_size = static_cast<size_t>(std::ceil(1.5 / resolution));

  // Ensure odd window size for symmetry
  if (window_size % 2 == 0) window_size++;

  // Need at least 3 points to smooth anything
  if (window_size < 3 || a.size() < window_size) return;

  std::vector<double> smoothed = a;
  int radius = static_cast<int>(window_size / 2);

  for (size_t i = 0; i < a.size(); ++i) {
    // Handle edges by clamping window bounds
    size_t start = (i > static_cast<size_t>(radius)) ? i - radius : 0;
    size_t end = std::min(i + radius, a.size() - 1);

    double sum = 0.0;
    for (size_t k = start; k <= end; ++k) {
      sum += a[k];
    }

    // Average
    smoothed[i] = sum / static_cast<double>(end - start + 1);
  }

  a = smoothed;
}

double SplineSet::compute_acceleration(double s, double v, double total_s) const
{
  // CASE A: Use Upstream Acceleration
  if (use_original_accel) {
    return sa.compute(s);
  }

  // CASE B: Recompute (a = v * dv/ds)

  // 1. Normal Motion
  if (v > 0.1) {
    return v * sv.compute_first_derivative(s);
  }

  // 2. Zero-Start / Low Speed Handling
  // We use a look-ahead finite difference to determine the "kick" required.
  const double lookahead_dist = 0.1;
  const double next_s = std::min(s + lookahead_dist, total_s);
  const double dist = next_s - s;

  if (dist > 1e-3) {
    const double v_next = sv.compute(next_s);
    // Kinematic: v^2 = u^2 + 2as  =>  a = (v^2 - u^2) / 2s
    double a = (v_next * v_next) / (2.0 * dist);

    // Preserve sign (if we are reversing)
    if (v_next < 0.0) a = -a;
    return a;
  }

  return 0.0;
}
}  // namespace autoware::time_to_space_trajectory_converter
