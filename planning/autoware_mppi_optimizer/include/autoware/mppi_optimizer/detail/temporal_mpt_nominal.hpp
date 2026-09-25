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

#ifndef AUTOWARE__MPPI_OPTIMIZER__DETAIL__TEMPORAL_MPT_NOMINAL_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__DETAIL__TEMPORAL_MPT_NOMINAL_HPP_

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"

#include <memory>
#include <optional>
#include <vector>

namespace autoware::mppi_optimizer::detail
{

/**
 * Owns a temporal_mpt::PathTrackingSolver and converts its u-trajectory into MPPI nominal
 * controls (accel_cmd, steer_cmd). Falls back to nullopt when the solve fails.
 *
 * Plant matches FirstOrderDubinsBicycle continuous dynamics (lag + rate limit), without
 * input-delay FIFOs.
 */
class TemporalMptNominalSeeder
{
public:
  TemporalMptNominalSeeder();
  ~TemporalMptNominalSeeder();

  TemporalMptNominalSeeder(const TemporalMptNominalSeeder &) = delete;
  TemporalMptNominalSeeder & operator=(const TemporalMptNominalSeeder &) = delete;
  TemporalMptNominalSeeder(TemporalMptNominalSeeder &&) noexcept;
  TemporalMptNominalSeeder & operator=(TemporalMptNominalSeeder &&) noexcept;

  /**
   * ``L = lf + lr`` with ``lr = rear_axle_to_cg``, ``lf = wheel_base - lr``.
   * Actuator lags / rate limit match MPPI vehicle params.
   */
  void setBicycleParameters(
    float wheel_base_m, float rear_axle_to_cg_m, float accel_time_constant_s,
    float steer_time_constant_s, float max_steer_rate_rad_s);

  /** Drop stored t-MPT warm-start (call after stop / tracking reset). */
  void resetWarmStart();

  /**
   * Optional external NLP guess from a prior control sequence. Consumed on the next ``solve``.
   * Unused by online MPPI: the seeder warm-starts from its own previous solution.
   */
  void setWarmStartControls(
    const std::vector<float> & accel_cmd, const std::vector<float> & steer_cmd);

  [[nodiscard]] std::optional<std::vector<FirstOrderDubinsMppiControl>> solve(
    const Trajectory & reference, const InitialState & ego,
    const FirstOrderDubinsMppiVehicleParams & vehicle_params, int horizon = kMppiHorizon);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::mppi_optimizer::detail

#endif  // AUTOWARE__MPPI_OPTIMIZER__DETAIL__TEMPORAL_MPT_NOMINAL_HPP_
