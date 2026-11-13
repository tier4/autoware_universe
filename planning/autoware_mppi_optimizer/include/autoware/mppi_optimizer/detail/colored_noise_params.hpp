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

#ifndef AUTOWARE__MPPI_OPTIMIZER__DETAIL__COLORED_NOISE_PARAMS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__DETAIL__COLORED_NOISE_PARAMS_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"

namespace autoware::mppi_optimizer::detail
{
/** Shared by production setup and sampler regression tests. Historical controls belong in
 * actuator delay queues; anchoring colored noise at t=0 would suppress new-command exploration.
 */
template <class SamplingParams>
void configureColoredNoise(
  SamplingParams & params, const FirstOrderDubinsMppiCostParams & costs,
  const int acceleration_index, const int steering_index)
{
  params.offset_decay_rate = 0.0F;
  params.exponents[acceleration_index] = costs.accel_cmd_noise_exponent;
  params.exponents[steering_index] = costs.steer_cmd_noise_exponent;
}
}  // namespace autoware::mppi_optimizer::detail

#endif  // AUTOWARE__MPPI_OPTIMIZER__DETAIL__COLORED_NOISE_PARAMS_HPP_
