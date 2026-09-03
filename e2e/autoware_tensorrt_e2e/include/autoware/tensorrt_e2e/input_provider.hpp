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

#ifndef AUTOWARE__TENSORRT_E2E__INPUT_PROVIDER_HPP_
#define AUTOWARE__TENSORRT_E2E__INPUT_PROVIDER_HPP_

#include "autoware/tensorrt_e2e/types.hpp"

#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <rclcpp/time.hpp>

#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @class InputProviderInterface
 * @brief Produces named model input tensors from ROS data.
 *
 * Lifecycle: constructed with the node (declares parameters, may not subscribe yet), then
 * `claim_inputs()` is called once with the engine's input manifest. A provider claims the
 * tensors it will produce, validates their shapes against its configuration, sizes internal
 * buffers, and creates the subscriptions it needs. Afterwards `collect()` is called once per
 * planning tick for the claimed tensors.
 */
class InputProviderInterface
{
public:
  virtual ~InputProviderInterface() = default;

  virtual std::string name() const = 0;

  /**
   * @brief Claim the engine input tensors this provider will produce.
   * @param engine_inputs The engine's input manifest.
   * @return Names of the claimed tensors (possibly empty).
   * @throws std::runtime_error when a claimable tensor's shape contradicts the provider
   *         configuration, or when a tensor this provider is configured to produce is missing.
   */
  virtual std::vector<std::string> claim_inputs(const std::vector<TensorSpec> & engine_inputs) = 0;

  /**
   * @brief Fill the claimed tensors for the current planning tick.
   * @param ego Current ego state (pose transforms, odometry, optional acceleration).
   * @param now Current ROS time, for staleness checks.
   * @param[out] inputs Tensor map to insert the claimed tensors into.
   * @param[out] error Human-readable reason when returning false.
   * @return true when all claimed tensors were produced.
   */
  virtual bool collect(
    const EgoFrame & ego, const rclcpp::Time & now, TensorMap & inputs, std::string & error) = 0;

  /**
   * @brief Add this provider's key-values to the tick's diagnostics, after a successful collect().
   *
   * The node knows nothing about what a provider measures; a sensor provider reports what its
   * reference node reports (e.g. bevfusion's `is_num_voxels_within_range`).
   */
  virtual void add_diagnostics(autoware_utils_diagnostics::DiagnosticsInterface & diagnostics)
  {
    (void)diagnostics;
  }

  /**
   * @brief Stamp of the freshest sensor input behind the last collect(), for latency reporting.
   * @return std::nullopt for providers without a sensor stamp (context tensors).
   */
  virtual std::optional<rclcpp::Time> latest_input_stamp() const { return std::nullopt; }
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__INPUT_PROVIDER_HPP_
