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

#ifndef AUTOWARE__TENSORRT_E2E__START_ACCEL_HPP_
#define AUTOWARE__TENSORRT_E2E__START_ACCEL_HPP_

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::tensorrt_e2e
{

/**
 * A ResWorld head that predicts its rollout's starting acceleration low-passes it, inside the
 * graph, against the start the node's previous planning tick used: the published plan's speed
 * profile otherwise moves from one tick to the next, and the trajectory optimizer, solving
 * time-indexed, turns that into steering in curves (OnePlanner's
 * `projects/resworld/start_accel.py`). The node only carries the state between ticks:
 *
 *   inputs  `start_accel_previous` [1, 1] m/s^2: the `start_accel` the graph returned on the
 *           previous successful run; `start_accel_dt` [1, 1] s: this tick's planning time minus
 *           that run's. Both 0 when there is none (the graph reads dt <= 0 as none).
 *   output  `start_accel` [1, 1] m/s^2: the start this tick's plan used, kept for the next tick.
 */
inline constexpr const char * START_ACCEL_PREVIOUS_TENSOR = "start_accel_previous";
inline constexpr const char * START_ACCEL_DT_TENSOR = "start_accel_dt";
inline constexpr const char * START_ACCEL_OUTPUT_TENSOR = "start_accel";

class StartAccelState
{
public:
  /**
   * @brief The two inputs for a tick planning at `stamp_s`: (previous start, seconds since it).
   *
   * (0, 0) before the first successful run. A clock that went backwards gives dt <= 0, which
   * the graph also reads as none.
   */
  std::pair<float, float> inputs(const double stamp_s) const
  {
    if (!last_) {
      return {0.0f, 0.0f};
    }
    return {last_->first, static_cast<float>(stamp_s - last_->second)};
  }

  /// @brief Keep the start a successful run returned, with the planning time it planned at.
  void record(const float start_accel, const double stamp_s) { last_ = {start_accel, stamp_s}; }

  bool has_value() const { return last_.has_value(); }

private:
  std::optional<std::pair<float, double>> last_;
};

/**
 * @brief Refuse a graph that takes the previous start without returning this one, or the
 * reverse: the node could not carry the state.
 */
inline void check_start_accel_tensors(
  const bool reads_previous, const bool reads_dt, const bool returns_start)
{
  if (reads_previous != reads_dt || reads_previous != returns_start) {
    throw std::runtime_error(
      std::string("A predicted-start graph takes '") + START_ACCEL_PREVIOUS_TENSOR + "' and '" +
      START_ACCEL_DT_TENSOR + "' and returns '" + START_ACCEL_OUTPUT_TENSOR +
      "', all three or none; this one has " + (reads_previous ? "" : "no ") +
      START_ACCEL_PREVIOUS_TENSOR + ", " + (reads_dt ? "" : "no ") + START_ACCEL_DT_TENSOR + ", " +
      (returns_start ? "" : "no ") + START_ACCEL_OUTPUT_TENSOR);
  }
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__START_ACCEL_HPP_
