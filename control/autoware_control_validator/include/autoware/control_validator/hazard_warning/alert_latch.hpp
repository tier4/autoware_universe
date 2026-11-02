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

#ifndef AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__ALERT_LATCH_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__ALERT_LATCH_HPP_

#include <rclcpp/time.hpp>

#include <cstdint>
#include <optional>

namespace autoware::control_validator::hazard_warning
{
/**
 * @brief Rising-edge immediate, falling-edge delayed latch.
 *
 * The output is the input `level` while it is 1 or more, and the last such level until `t_min`
 * has elapsed since then (0 afterwards). A clock going backwards resets the latch.
 */
class AlertLatch
{
public:
  uint8_t update(const uint8_t level, const rclcpp::Time & now, const double t_min)
  {
    if (level >= 1) {
      last_on_time_ = now;
      last_level_ = level;
      return level;
    }
    if (last_on_time_.has_value()) {
      const double elapsed =
        static_cast<double>(now.nanoseconds() - last_on_time_->nanoseconds()) * 1e-9;
      if (elapsed >= 0.0 && elapsed <= t_min) {
        return last_level_;
      }
    }
    last_on_time_.reset();
    last_level_ = 0;
    return 0;
  }

private:
  std::optional<rclcpp::Time> last_on_time_;
  uint8_t last_level_{0};
};
}  // namespace autoware::control_validator::hazard_warning

#endif  // AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__ALERT_LATCH_HPP_
