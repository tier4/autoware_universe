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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_APPLICATION_STATUS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_APPLICATION_STATUS_HPP_

#include <cstddef>

namespace autoware::mppi_optimizer
{

/** @brief Separates optimizer execution from the output ultimately applied by the plugin. */
struct MppiApplicationStatus
{
  bool optimization_succeeded{false};
  bool optimized_trajectory_applied{false};
  bool fallback_applied{false};
  bool output_applied{false};
};

/** @brief Derives all application decisions from one result, avoiding inconsistent debug state. */
constexpr MppiApplicationStatus makeMppiApplicationStatus(
  const bool shadow_mode, const bool was_rejected, const bool velocity_limit_profile_active,
  const std::size_t optimized_point_count)
{
  const bool optimization_succeeded = !was_rejected && optimized_point_count > 0U;
  const bool optimized_trajectory_applied = !shadow_mode && optimization_succeeded;
  const bool fallback_applied =
    !shadow_mode && was_rejected && velocity_limit_profile_active && optimized_point_count > 0U;
  return {
    optimization_succeeded, optimized_trajectory_applied, fallback_applied,
    optimized_trajectory_applied || fallback_applied};
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_APPLICATION_STATUS_HPP_
