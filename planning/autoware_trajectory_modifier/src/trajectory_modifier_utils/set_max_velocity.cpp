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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/set_max_velocity.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::utils
{

void set_max_velocity(TrajectoryPoints & input_trajectory_array, const float max_velocity)
{
  if (input_trajectory_array.empty()) {
    return;
  }

  if (input_trajectory_array.size() == 1) {
    input_trajectory_array[0].longitudinal_velocity_mps =
      std::min(input_trajectory_array[0].longitudinal_velocity_mps, max_velocity);
    input_trajectory_array[0].acceleration_mps2 = 0.0f;
    return;
  }

  std::vector<std::pair<size_t, size_t>> modified_segment_indices;
  std::vector<double> original_dts;
  original_dts.reserve(input_trajectory_array.size() - 1);

  auto compute_dt_using_velocity_and_acc =
    [](const TrajectoryPoint & from, const TrajectoryPoint & to) -> double {
    const auto dv = static_cast<double>(to.longitudinal_velocity_mps) -
                    static_cast<double>(from.longitudinal_velocity_mps);
    const auto acc = static_cast<double>(from.acceleration_mps2);
    constexpr double epsilon_acceleration = 1e-6;
    const auto denominator_acc = std::max(std::abs(acc), epsilon_acceleration);
    return std::abs(dv) / denominator_acc;
  };

  for (size_t i = 0; i < input_trajectory_array.size() - 1; ++i) {
    original_dts.push_back(
      compute_dt_using_velocity_and_acc(input_trajectory_array[i], input_trajectory_array[i + 1]));
  }

  size_t segment_start = 0;
  bool in_segment = false;

  for (size_t i = 0; i < input_trajectory_array.size(); ++i) {
    const bool exceeds_max = input_trajectory_array[i].longitudinal_velocity_mps > max_velocity;

    if (exceeds_max && !in_segment) {
      segment_start = i;
      in_segment = true;
    } else if (!exceeds_max && in_segment) {
      modified_segment_indices.emplace_back(segment_start, i - 1);
      in_segment = false;
    }
  }

  if (in_segment) {
    modified_segment_indices.emplace_back(segment_start, input_trajectory_array.size() - 1);
  }

  for (const auto & [start, end] : modified_segment_indices) {
    for (size_t i = start; i <= end; ++i) {
      input_trajectory_array[i].longitudinal_velocity_mps = max_velocity;
    }
    for (size_t i = start; i < end; ++i) {
      input_trajectory_array[i].acceleration_mps2 = 0.0f;
    }
  }

  for (const auto & [start, end] : modified_segment_indices) {
    if (start > 0) {
      const size_t idx_before = start - 1;
      const double dt = original_dts[idx_before];
      const auto v_before =
        static_cast<double>(input_trajectory_array[idx_before].longitudinal_velocity_mps);
      const auto v_start =
        static_cast<double>(input_trajectory_array[start].longitudinal_velocity_mps);
      const double new_acc = (v_start - v_before) / dt;
      input_trajectory_array[idx_before].acceleration_mps2 = static_cast<float>(new_acc);
    }

    if (end < input_trajectory_array.size() - 1) {
      const double dt = original_dts[end];
      const auto v_end = static_cast<double>(input_trajectory_array[end].longitudinal_velocity_mps);
      const auto v_after =
        static_cast<double>(input_trajectory_array[end + 1].longitudinal_velocity_mps);
      const double new_acc = (v_after - v_end) / dt;
      input_trajectory_array[end].acceleration_mps2 = static_cast<float>(new_acc);
    }
  }

  input_trajectory_array.back().acceleration_mps2 = 0.0f;
}

}  // namespace autoware::trajectory_modifier::utils
