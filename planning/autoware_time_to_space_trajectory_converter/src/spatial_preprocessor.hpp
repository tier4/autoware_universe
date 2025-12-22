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

#ifndef SPATIAL_PREPROCESSOR_HPP_
#define SPATIAL_PREPROCESSOR_HPP_

#include "data_types.hpp"
#include "parameters.hpp"

#include <optional>
#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
/**
 * @brief Logic core for converting raw time-series points into spatially distributed knots.
 *
 * This class handles the specific edge cases of "Creeping" (slow movement that shouldn't be
 * filtered) and "Stopping" (accumulating time duration instead of geometry).
 */
class SpatialPreprocessor
{
public:
  explicit SpatialPreprocessor(
    const SpatialPreprocessorConfig & config = SpatialPreprocessorConfig());

  /**
   * @brief Processes the raw trajectory.
   *
   * This method iterates through the time-based input and compresses it into
   * space-based knots. It buffers small movements until they satisfy the
   * resolution threshold and collapses stops into wait times.
   *
   * @param input Raw points from the upstream planner.
   * @param output The destination data structure (will be reset).
   * @return std::nullopt on success, or an error message string on failure.
   */
  [[nodiscard]] static std::optional<std::string> process(
    const std::vector<PlannerPoint> & input, SplineData & output,
    const SpatialPreprocessorConfig & config);

private:
  SpatialPreprocessorConfig config_;

  /**
   * @brief Internal state accumulator for the processing loop.
   */
  struct ProcessingState
  {
    double current_s = 0.0;      ///< Total arc length committed to the trajectory.
    double accum_s = 0.0;        ///< Buffered distance (creeping) not yet committed.
    double last_knot_yaw = 0.0;  ///< Yaw of the last committed knot point.
  };

  /**
   * @brief Commits a new knot point to the output and resets the accumulator.
   */
  static void add_spline_knot(
    const PlannerPoint & pt, SplineData & output, ProcessingState & state);

  /**
   * @brief Handles the moving state (Fast or Creeping).
   * Buffers distance into the accumulator. Only creates a knot if the
   * resolution threshold is met.
   */
  static void accumulate_motion(
    const PlannerPoint & pt, double dist, SplineData & output, ProcessingState & state,
    const SpatialPreprocessorConfig & config);

  /**
   * @brief Handles the stopped state.
   * 1. Finalizes any pending creep distance to capture the exact stop location.
   * 2. Accumulates the time delta onto the current anchor point's wait_time.
   */
  static void accumulate_wait_time(
    const PlannerPoint & pt, double dt, SplineData & output, ProcessingState & state);

  /**
   * @brief Helper to force-commit any pending creep distance into a real point.
   * Used when transitioning from Move -> Stop.
   */
  static void finalize_motion(
    const PlannerPoint & pt, SplineData & output, ProcessingState & state);
};

}  // namespace autoware::time_to_space_trajectory_converter

#endif  // SPATIAL_PREPROCESSOR_HPP_
