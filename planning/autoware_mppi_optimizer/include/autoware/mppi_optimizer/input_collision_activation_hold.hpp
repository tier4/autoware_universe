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

#ifndef AUTOWARE__MPPI_OPTIMIZER__INPUT_COLLISION_ACTIVATION_HOLD_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__INPUT_COLLISION_ACTIVATION_HOLD_HPP_

#include <cmath>
#include <optional>
#include <stdexcept>

namespace autoware::mppi_optimizer
{

enum class InputCollisionActivationSource { none, condition, hold };

inline const char * to_string(const InputCollisionActivationSource source)
{
  switch (source) {
    case InputCollisionActivationSource::condition:
      return "condition";
    case InputCollisionActivationSource::hold:
      return "hold";
    case InputCollisionActivationSource::none:
    default:
      return "none";
  }
}

/** @brief Refreshes the conditional optimization hold on every positive observation. */
class InputCollisionActivationHold
{
public:
  [[nodiscard]] InputCollisionActivationSource update(
    const bool condition_met, const double now_s, const double min_duration_s)
  {
    if (!std::isfinite(now_s) || !std::isfinite(min_duration_s) || min_duration_s < 0.0) {
      throw std::invalid_argument("MPPI input collision activation time or duration is invalid");
    }

    if (last_update_time_s_ && now_s < *last_update_time_s_) {
      reset();
    }
    last_update_time_s_ = now_s;
    if (condition_met) {
      last_condition_time_s_ = now_s;
      return InputCollisionActivationSource::condition;
    }
    if (last_condition_time_s_ && now_s - *last_condition_time_s_ < min_duration_s) {
      return InputCollisionActivationSource::hold;
    }
    last_condition_time_s_.reset();
    return InputCollisionActivationSource::none;
  }

  void reset()
  {
    last_condition_time_s_.reset();
    last_update_time_s_.reset();
  }

private:
  std::optional<double> last_condition_time_s_;
  std::optional<double> last_update_time_s_;
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__INPUT_COLLISION_ACTIVATION_HOLD_HPP_
