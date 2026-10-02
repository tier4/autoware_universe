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

#ifndef AUTOWARE__TENSORRT_E2E__CURVATURE_BIAS_HPP_
#define AUTOWARE__TENSORRT_E2E__CURVATURE_BIAS_HPP_

#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

namespace autoware::tensorrt_e2e
{

/**
 * The steering sensor's curvature bias, `ego_curvature_bias` `[1, 1]` in 1/m: how far the
 * curvature the EKF yaw rate says (r / v) sits from the one the measured tire angle says
 * (tan(steer) / L), low-passed. A ResWorld kinematic head trained with
 * `resworld_kinematic_initial_steer: complementary` starts its rollout from
 * atan(tan(steer) + L b). The recursion is OnePlanner's
 * `projects/resworld/curvature_bias.py`, which the loader ran over the recorded 10 Hz frames;
 * the package states its constants (`context.curvature_bias.*`):
 *
 *   u = r / v - tan(steer) / L
 *   |v| >= min_speed: b = u at the first such tick, else b += (1 - exp(-dt / tau)) (u - b)
 *   otherwise b is unchanged; b = 0 before the first update
 */
inline constexpr const char * CURVATURE_BIAS_TENSOR = "ego_curvature_bias";

class CurvatureBiasFilter
{
public:
  CurvatureBiasFilter(const double tau_s, const double min_speed_mps)
  : tau_s_(tau_s), min_speed_mps_(min_speed_mps)
  {
    if (!(tau_s > 0.0) || !(min_speed_mps > 0.0)) {
      throw std::runtime_error(
        "context.curvature_bias needs tau_s > 0 and min_speed_mps > 0, got " +
        std::to_string(tau_s) + " and " + std::to_string(min_speed_mps));
    }
  }

  /**
   * @brief One planning tick, on the values this tick's `ego_current_state` carries.
   * @param dt_s seconds since the previous tick; ignored by the first update.
   */
  void update(
    const double speed, const double yaw_rate, const double steering, const double wheel_base,
    const double dt_s)
  {
    if (std::abs(speed) < min_speed_mps_) {
      return;  // r / v is noise near rest: hold
    }
    const double u = yaw_rate / speed - std::tan(steering) / wheel_base;
    if (!bias_) {
      bias_ = u;
      return;
    }
    if (dt_s > 0.0) {
      *bias_ += (1.0 - std::exp(-dt_s / tau_s_)) * (u - *bias_);
    }
  }

  float value() const { return static_cast<float>(bias_.value_or(0.0)); }

private:
  double tau_s_;
  double min_speed_mps_;
  std::optional<double> bias_;
};

/**
 * @brief Refuse a package whose graph and `context.curvature_bias.enabled` disagree.
 * @throws std::runtime_error when the graph reads the bias and the package does not keep
 * one, or the package keeps one the graph never reads.
 */
inline void check_curvature_bias_inputs(const bool enabled, const bool graph_reads_bias)
{
  if (graph_reads_bias && !enabled) {
    throw std::runtime_error(
      "The graph reads '" + std::string(CURVATURE_BIAS_TENSOR) +
      "' (its head starts from the complementary steer) but context.curvature_bias.enabled "
      "is false: regenerate its ml_package file, which states the filter");
  }
  if (!graph_reads_bias && enabled) {
    throw std::runtime_error(
      "context.curvature_bias.enabled is true but the graph has no '" +
      std::string(CURVATURE_BIAS_TENSOR) +
      "' input: the package and the graph come from different exports");
  }
}

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__CURVATURE_BIAS_HPP_
