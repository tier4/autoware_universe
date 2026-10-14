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

#ifndef HAZARD__ALERT_LATCH_HPP_
#define HAZARD__ALERT_LATCH_HPP_

#include <rclcpp/time.hpp>

#include <optional>

namespace autoware::control_validator::hazard
{
/**
 * @brief Rising-edge immediate, falling-edge delayed latch.
 *
 * The output stays true until `t_min` has elapsed since the last cycle in which `raw` was true.
 * A clock going backwards resets the latch.
 */
class AlertLatch
{
public:
  bool update(const bool raw, const rclcpp::Time & now, const double t_min)
  {
    if (raw) {
      last_on_time_ = now;
      return true;
    }
    if (last_on_time_.has_value()) {
      const double elapsed =
        static_cast<double>(now.nanoseconds() - last_on_time_->nanoseconds()) * 1e-9;
      if (elapsed >= 0.0 && elapsed <= t_min) {
        return true;
      }
    }
    last_on_time_.reset();
    return false;
  }

  bool is_on() const { return last_on_time_.has_value(); }

private:
  std::optional<rclcpp::Time> last_on_time_;
};
}  // namespace autoware::control_validator::hazard

#endif  // HAZARD__ALERT_LATCH_HPP_
