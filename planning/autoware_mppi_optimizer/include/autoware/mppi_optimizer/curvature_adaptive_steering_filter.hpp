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

#ifndef AUTOWARE__MPPI_OPTIMIZER__CURVATURE_ADAPTIVE_STEERING_FILTER_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__CURVATURE_ADAPTIVE_STEERING_FILTER_HPP_

#include <vector>

namespace autoware::mppi_optimizer
{

/** Parameters for the curvature-adaptive exponential moving average. */
struct CurvatureAdaptiveSteeringFilterParams
{
  /** EMA smoothing factor for a zero steering command. */
  float alpha_straight{0.1F};
  /** EMA smoothing factor at and above turn_angle_rad. */
  float alpha_turn{1.0F};
  /** Absolute steering command [rad] at which alpha reaches alpha_turn. */
  float turn_angle_rad{0.1F};
};

/**
 * Stateful output filter for a predicted steering-command sequence.
 *
 * The sequence is filtered using a local EMA cursor, but only the first filtered command is
 * retained between control cycles. This avoids advancing persistent state to the end of the
 * prediction horizon when only its first command is applied.
 */
class CurvatureAdaptiveSteeringFilter
{
public:
  CurvatureAdaptiveSteeringFilter() = default;
  explicit CurvatureAdaptiveSteeringFilter(const CurvatureAdaptiveSteeringFilterParams & params);

  /** Validate and install parameters. Parameter changes reset the persistent EMA state. */
  void setParams(const CurvatureAdaptiveSteeringFilterParams & params);

  /** Forget the previously applied filtered command. */
  void reset();

  /**
   * Filter steering commands in place.
   *
   * On the first call after reset, measured_steering seeds the EMA. Non-finite measurements use
   * zero and non-finite commands hold the previous filtered command.
   */
  void filter(std::vector<float> & steering_commands, float measured_steering);

private:
  CurvatureAdaptiveSteeringFilterParams params_{};
  float previous_filtered_command_{0.0F};
  bool initialized_{false};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__CURVATURE_ADAPTIVE_STEERING_FILTER_HPP_
