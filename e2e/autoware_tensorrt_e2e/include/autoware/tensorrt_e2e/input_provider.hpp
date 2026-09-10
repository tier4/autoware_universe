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

#include <cuda_runtime_api.h>

#include <functional>
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
   * @brief Whether the provider looks transforms up in the node's TF buffer.
   *
   * The node starts a TF listener only when some provider says so: a listener subscribes
   * to `/tf` and `/tf_static` from a helper node, and a model line without cameras has no
   * use for either. The buffer itself is always there to be passed in; empty, it costs
   * nothing.
   */
  virtual bool uses_tf() const { return false; }

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
   * @brief Hand the provider the stream the tick runs on, before `claim_inputs()`.
   *
   * A provider that produces device-resident tensors submits its GPU work here. The
   * engine consumes those tensors on the same stream, so ordering alone guarantees they
   * are complete when the network reads them, and no provider has to drain the device
   * with a host synchronization in the middle of the tick. A provider with no GPU work
   * ignores it. Providers that were never handed a stream keep creating their own.
   */
  virtual void bind_stream(cudaStream_t stream) { (void)stream; }
  /**
   * @brief Called at the end of every tick, after the trajectory is out -- or after the tick
   *        gave up, whichever came first.
   *
   * Work that is not on the trajectory's path belongs here: a sensor provider that also
   * publishes what its network detected does the decoding after the trajectory, so a
   * consumer of the trajectory never waits for a message it does not read.
   */
  virtual void finish_tick() {}
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

  /**
   * @brief Offer to pace the node: call `on_data` when this provider's own sensor
   *        delivers, and return true.
   *
   * A provider reading the input the model is actually waiting on -- the LiDAR
   * sweep -- accepts, and the node then runs the moment that input lands rather
   * than on the next tick of a timer. A timer makes each frame wait up to a full
   * period before it is used, and that wait is latency the controller pays for an
   * input that had already arrived. autoware_bevfusion, which reads the same
   * cloud, is driven this way and has no timer at all. Providers reading slower
   * context -- a map, a route -- decline, and the node falls back to its timer if
   * none accepts.
   */
  virtual bool pace(std::function<void()> on_data)
  {
    (void)on_data;
    return false;
  }
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__INPUT_PROVIDER_HPP_
