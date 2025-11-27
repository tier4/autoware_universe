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

#include "wait_time_accumulator.hpp"

#include <numeric>

namespace autoware::time_to_space_trajectory_converter
{
double WaitTimeAccumulator::collect(double s_limit)
{
  if (data_.wait_times.empty()) return 0.0;

  auto start_it = data_.s.begin() + current_idx_;
  auto limit_it = std::upper_bound(start_it, data_.s.end(), s_limit + g_math_eps);
  int limit_idx = static_cast<int>(std::distance(data_.s.begin(), limit_it));

  double wait_sum = 0.0;

  if (limit_idx > current_idx_) {
    wait_sum = std::accumulate(
      data_.wait_times.begin() + current_idx_, data_.wait_times.begin() + limit_idx, 0.0,
      [](double sum, double val) { return sum + (val > 1e-3 ? val : 0.0); });

    // Advance cursor
    current_idx_ = limit_idx;
  }

  return wait_sum;
}
}  // namespace autoware::time_to_space_trajectory_converter
