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

#ifndef AUTOWARE__TENSORRT_E2E__PREV_PLAN_HPP_
#define AUTOWARE__TENSORRT_E2E__PREV_PLAN_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::tensorrt_e2e
{

//! The graph input that carries the model's own previous plan ("planning momentum").
inline constexpr const char * PREV_PLAN_TENSOR = "prev_plan";

//! A previous plan older than this (seconds) is not fed: the ego has left it.
inline constexpr double PREV_PLAN_MAX_AGE_S = 0.3;

/**
 * @brief The previous tick's RAW planner output (before force-stop and smoothing), held in the
 * map frame so the next tick can re-express it in its own ego frame.
 *
 * Point j is the pose at `stamp_ns + (j + 1) * 0.1 s`; each is `{x, y, cos, sin}` in metres.
 */
struct PrevPlanCache
{
  int64_t stamp_ns{0};  //!< the previous tick's `ego.stamp`
  uint64_t generation{0};  //!< its `ego.localization_generation`
  std::vector<std::array<double, 4>> map_poses;
};

/**
 * @brief Cache the ego (batch 0, agent 0) plan of a raw `[.., T, 4]` output.
 * @param raw the prediction tensor's floats; the first `T * 4` are the ego plan.
 */
inline PrevPlanCache cache_from_output(
  const std::vector<float> & raw, const int64_t num_timesteps, const Eigen::Matrix4d & ego_to_map,
  const int64_t stamp_ns, const uint64_t generation)
{
  PrevPlanCache cache;
  cache.stamp_ns = stamp_ns;
  cache.generation = generation;
  const double ce = ego_to_map(0, 0);
  const double se = ego_to_map(1, 0);
  for (int64_t j = 0; j < num_timesteps; ++j) {
    const double x = raw[4 * j + 0];
    const double y = raw[4 * j + 1];
    const double c = raw[4 * j + 2];
    const double s = raw[4 * j + 3];
    cache.map_poses.push_back(
      {ego_to_map(0, 0) * x + ego_to_map(0, 1) * y + ego_to_map(0, 3),
       ego_to_map(1, 0) * x + ego_to_map(1, 1) * y + ego_to_map(1, 3), ce * c - se * s,
       se * c + ce * s});
  }
  return cache;
}

/**
 * @brief The `[1, T, 5]` (x, y, cos, sin, valid) tensor data for this tick.
 *
 * Row k is the previous plan's pose at `now + (k + 1) * 0.1 s`, linearly interpolated (heading
 * as a re-normalised cos/sin blend) and expressed in the current ego frame. A row the previous
 * plan does not cover, or every row when the plan is older than PREV_PLAN_MAX_AGE_S, from the
 * future, from another localization generation, or of another length, is all zeros.
 */
inline std::vector<float> build_prev_plan_tensor(
  const PrevPlanCache * cache, const Eigen::Matrix4d & map_to_ego, const int64_t now_ns,
  const uint64_t generation, const int64_t num_timesteps)
{
  std::vector<float> out(static_cast<size_t>(num_timesteps) * 5, 0.0f);
  if (
    !cache || cache->generation != generation ||
    static_cast<int64_t>(cache->map_poses.size()) != num_timesteps) {
    return out;
  }
  const double gap_steps = static_cast<double>(now_ns - cache->stamp_ns) * 1e-9 / 0.1;
  if (gap_steps < -1e-6 || gap_steps > PREV_PLAN_MAX_AGE_S / 0.1 + 1e-6) {
    return out;
  }
  const double ce = map_to_ego(0, 0);
  const double se = map_to_ego(1, 0);
  for (int64_t k = 0; k < num_timesteps; ++k) {
    // Previous-plan index of the time now + (k+1)*0.1 s.
    const double pos = gap_steps + static_cast<double>(k);
    if (pos > static_cast<double>(num_timesteps - 1) + 1e-6) {
      break;  // later rows are later still
    }
    const auto lo = static_cast<int64_t>(std::floor(pos + 1e-9));
    const int64_t hi = std::min(lo + 1, num_timesteps - 1);
    const double a = std::min(std::max(pos - static_cast<double>(lo), 0.0), 1.0);
    const auto & p = cache->map_poses[lo];
    const auto & q = cache->map_poses[hi];
    const double x = p[0] + a * (q[0] - p[0]);
    const double y = p[1] + a * (q[1] - p[1]);
    double c = p[2] + a * (q[2] - p[2]);
    double s = p[3] + a * (q[3] - p[3]);
    const double norm = std::hypot(c, s);
    if (norm < 1e-6) {
      continue;
    }
    c /= norm;
    s /= norm;
    float * row = &out[static_cast<size_t>(k) * 5];
    row[0] = static_cast<float>(map_to_ego(0, 0) * x + map_to_ego(0, 1) * y + map_to_ego(0, 3));
    row[1] = static_cast<float>(map_to_ego(1, 0) * x + map_to_ego(1, 1) * y + map_to_ego(1, 3));
    row[2] = static_cast<float>(ce * c - se * s);
    row[3] = static_cast<float>(se * c + ce * s);
    row[4] = 1.0f;
  }
  return out;
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__PREV_PLAN_HPP_
