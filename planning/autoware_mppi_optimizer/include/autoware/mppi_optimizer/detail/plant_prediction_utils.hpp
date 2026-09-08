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

#ifndef AUTOWARE__MPPI_OPTIMIZER__DETAIL__PLANT_PREDICTION_UTILS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__DETAIL__PLANT_PREDICTION_UTILS_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::mppi_optimizer::detail
{

/** Plant IC + delay FIFOs captured at the start of an MPPI cycle. */
struct FirstOrderDubinsMppiPlantSnapshot
{
  bool valid{false};
  builtin_interfaces::msg::Time stamp{};
  float sim_time{0.0F};
  FirstOrderDubinsMppiAppliedPlantState plant;
};

/** Applied control issued at a wall-clock stamp (piecewise-constant replay). */
struct FirstOrderDubinsMppiControlHistoryEntry
{
  builtin_interfaces::msg::Time stamp{};
  FirstOrderDubinsMppiControl control{};
};

[[nodiscard]] double stampToSeconds(const builtin_interfaces::msg::Time & stamp);

[[nodiscard]] double elapsedSeconds(
  const builtin_interfaces::msg::Time & from, const builtin_interfaces::msg::Time & to);

/**
 * Split elapsed time into MPPI dt chunks plus a final remainder (if any).
 * Example: elapsed=0.54, dt=0.1 -> [0.1, 0.1, 0.1, 0.1, 0.1, 0.04].
 */
[[nodiscard]] inline std::vector<float> buildChunkedIntegrationDts(
  const float elapsed_s, const float dt)
{
  constexpr float kTimeEpsilonS = 1.0E-6F;
  std::vector<float> dts;
  if (elapsed_s <= kTimeEpsilonS || dt <= kTimeEpsilonS) {
    return dts;
  }
  const int full_steps = static_cast<int>(std::floor((elapsed_s + kTimeEpsilonS) / dt));
  const float remainder = elapsed_s - static_cast<float>(full_steps) * dt;
  dts.assign(static_cast<size_t>(full_steps), dt);
  if (remainder > kTimeEpsilonS) {
    dts.push_back(remainder);
  }
  return dts;
}

struct PlantPredictionReplayInput
{
  FirstOrderDubinsMppiVehicleParams vehicle{};
  bool enable_input_delay_compensation{true};
  float integration_dt{0.1F};
  FirstOrderDubinsMppiPlantSnapshot anchor{};
  std::vector<FirstOrderDubinsMppiControlHistoryEntry> control_history{};
  builtin_interfaces::msg::Time measurement_stamp{};
  float measured_x{0.0F};
  float measured_y{0.0F};
  float measured_yaw{0.0F};
  float measured_vel{0.0F};
};

/** Integrate the delay-bicycle plant from anchor IC and compare to the measurement. */
[[nodiscard]] FirstOrderDubinsMppiPredictionAccuracy evaluatePlantPredictionAccuracy(
  const PlantPredictionReplayInput & input);

}  // namespace autoware::mppi_optimizer::detail

#endif  // AUTOWARE__MPPI_OPTIMIZER__DETAIL__PLANT_PREDICTION_UTILS_HPP_
