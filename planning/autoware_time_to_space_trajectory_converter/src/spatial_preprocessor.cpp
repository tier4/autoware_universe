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

#include "spatial_preprocessor.hpp"

#include <autoware_utils_math/normalization.hpp>

#include <fmt/format.h>

#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{

SpatialPreprocessor::SpatialPreprocessor(const SpatialPreprocessorConfig & config) : config_(config)
{
}

std::optional<std::string> SpatialPreprocessor::process(
  const std::vector<PlannerPoint> & input, SplineData & output,
  const SpatialPreprocessorConfig & config)
{
  output.reset(input.size());
  output.set_config(config.stop_v_thresh);

  if (input.empty()) {
    return "Input trajectory is empty.";
  }

  // Always trust the start point
  output.add_point(0.0, input[0], 0.0);

  ProcessingState state;
  state.accum_s = 0.0;
  state.current_s = 0.0;
  state.last_knot_yaw = input[0].yaw;

  for (size_t i = 1; i < input.size(); ++i) {
    const auto delta_pos = input[i].pos - input[i - 1].pos;
    const double dist = delta_pos.norm2d();
    const double dt = input[i].t - input[i - 1].t;
    const bool is_zero_velocity = std::abs(input[i].v) < config.stop_v_thresh;

    if (is_zero_velocity) {
      if (input[i - 1].v > config.stop_v_thresh) {
        state.accum_s += dist;
      }
      accumulate_wait_time(input[i], dt, output, state);
    } else {
      accumulate_motion(input[i], dist, output, state, config);
    }
  }

  if (state.accum_s > g_geom_eps) {
    add_spline_knot(input.back(), output, state);
  }

  if (output.s.empty()) {
    return "Preprocessing resulted in empty trajectory.";
  }

  return std::nullopt;
}

void SpatialPreprocessor::add_spline_knot(
  const PlannerPoint & pt, SplineData & output, ProcessingState & state)
{
  state.current_s += state.accum_s;
  output.add_point(state.current_s, pt, 0.0);
  state.accum_s = 0.0;
  state.last_knot_yaw = pt.yaw;
}

void SpatialPreprocessor::accumulate_motion(
  const PlannerPoint & pt, double dist, SplineData & output, ProcessingState & state,
  const SpatialPreprocessorConfig & config)
{
  // Buffer tiny movements (creeping) until they are significant enough for the spline.
  state.accum_s += dist;
  const bool is_exceed_dist_thresh = state.accum_s >= config.min_knot_dist;
  const double yaw_diff =
    autoware_utils_math::normalize_radian(std::abs(pt.yaw - state.last_knot_yaw));
  const bool is_exceed_yaw_th = yaw_diff >= config.max_knot_yaw_diff;

  if (is_exceed_dist_thresh || (is_exceed_yaw_th && state.accum_s >= g_geom_eps)) {
    add_spline_knot(pt, output, state);
    return;
  }
}

void SpatialPreprocessor::finalize_motion(
  const PlannerPoint & pt, SplineData & output, ProcessingState & state)
{
  // Only finalize if there is actual motion pending.
  // This logic is crucial to create the exact "Stop Line" point.
  if (state.accum_s > g_geom_eps) {
    // Create the knot point with forced zero dynamics
    PlannerPoint stop_pt = pt;
    stop_pt.v = 0.0;
    stop_pt.a = 0.0;

    add_spline_knot(stop_pt, output, state);
    return;
  }
}

void SpatialPreprocessor::accumulate_wait_time(
  const PlannerPoint & pt, double dt, SplineData & output, ProcessingState & state)
{
  // 1. Finalize Motion
  // We were moving (creeping), but now we stopped. Commit that distance.
  finalize_motion(pt, output, state);

  // 2. Accumulate Wait Time
  // Compress time onto the anchor point instead of creating new geometry.
  if (!output.wait_times.empty()) {
    output.wait_times.back() += dt;
    output.v.back() = 0.0;  // Enforce physical stop consistency
    output.a.back() = 0.0;
  }
}

}  // namespace autoware::time_to_space_trajectory_converter
