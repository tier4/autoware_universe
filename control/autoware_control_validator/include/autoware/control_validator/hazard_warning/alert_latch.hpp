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

#include <autoware_internal_control_msgs/msg/hazard_sector.hpp>

#include <cstdint>
#include <optional>

namespace autoware::control_validator::hazard_warning
{
struct SectorState
{
  uint8_t risk_level{autoware_internal_control_msgs::msg::HazardSector::NONE};
  uint8_t sources{0};

  bool is_on() const
  {
    return risk_level > autoware_internal_control_msgs::msg::HazardSector::NONE || sources != 0;
  }
};

/**
 * @brief Rising-edge immediate, falling-edge delayed latch.
 *
 * The output is the input `raw` while it is on, and the last such state until `t_min` has elapsed
 * since then (off afterwards). A clock going backwards resets the latch.
 */
class AlertLatch
{
public:
  SectorState update(const SectorState & raw, const rclcpp::Time & now, const double t_min)
  {
    if (raw.is_on()) {
      last_on_time_ = now;
      last_state_ = raw;
      return raw;
    }
    if (last_on_time_.has_value()) {
      const double elapsed =
        static_cast<double>(now.nanoseconds() - last_on_time_->nanoseconds()) * 1e-9;
      if (elapsed >= 0.0 && elapsed <= t_min) {
        return last_state_;
      }
    }
    last_on_time_.reset();
    last_state_ = SectorState{};
    return SectorState{};
  }

private:
  std::optional<rclcpp::Time> last_on_time_;
  SectorState last_state_;
};
}  // namespace autoware::control_validator::hazard_warning

#endif  // AUTOWARE__CONTROL_VALIDATOR__HAZARD_WARNING__ALERT_LATCH_HPP_
