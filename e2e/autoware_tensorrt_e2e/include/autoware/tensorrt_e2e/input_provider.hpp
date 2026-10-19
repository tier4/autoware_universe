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

#include <rclcpp/time.hpp>

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

/**
 * @brief Whether a trigger input (see InputProviderInterface::set_tick_trigger) starts a tick
 * at a planning period of `period` seconds: the input whose stamp lands nearest one period after
 * the input that started the last tick does. Judged on header stamps, which the camera trigger
 * keeps exactly periodic, not on arrival, which decode and transport jitter by tens of ms.
 * @param stamp This input's header stamp [s].
 * @param last_tick_stamp The stamp of the input that started the last tick; nullopt before any.
 * @param input_gap The stamp interval between consecutive inputs [s].
 */
inline bool trigger_input_due(
  const double stamp, const std::optional<double> last_tick_stamp, const double input_gap,
  const double period)
{
  if (!last_tick_stamp || stamp < *last_tick_stamp) {
    return true;  // the first input, or the clock was rewound (a bag restarted)
  }
  return stamp - *last_tick_stamp >= period - 0.5 * input_gap;
}

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
   * @brief Let this provider pace the planning tick: it calls `on_input` with the input's header
   * stamp whenever a new input the tick should run on has arrived (for LatentDrive, every stored
   * camera 0 frame). Called after claim_inputs(); `on_input` may be called from any thread and
   * must not block.
   * @return false when the provider has no such input (the node then keeps its timer).
   */
  virtual bool set_tick_trigger(std::function<void(const rclcpp::Time &)> on_input)
  {
    (void)on_input;
    return false;
  }
};

}  // namespace autoware::tensorrt_e2e

#endif  // AUTOWARE__TENSORRT_E2E__INPUT_PROVIDER_HPP_
