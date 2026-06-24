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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_ACTION_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_ACTION_HPP_

#include <autoware_trajectory_validator/msg/metric_report.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <cstdint>

namespace autoware::trajectory_validator
{

/** @brief Severity-ordered action the diagnostic system reacts to. */
enum class Action : uint8_t { NONE = 0, COMFORTABLE = 1, MODERATE = 2, EMERGENCY = 3 };

/**
 * @brief Translates a MetricReport.level into an Action.
 * @note This is the ONLY place MetricReport level values (OK/WARN/ERROR) are named.
 * @param metric_level Level value from MetricReport (OK/WARN/ERROR).
 */
inline Action to_action(uint8_t metric_level)
{
  using autoware_trajectory_validator::msg::MetricReport;
  // Interim map (current 3-level scale):
  //   OK, WARN -> NONE  |  ERROR -> MODERATE
  if (metric_level == MetricReport::ERROR) return Action::MODERATE;
  return Action::NONE;
}

/**
 * @brief Maps an Action to its published DiagnosticStatus level.
 * @param action Action to map.
 */
inline int8_t published_level_of(Action action)
{
  switch (action) {
    case Action::NONE:
      return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case Action::COMFORTABLE:
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case Action::MODERATE:
    case Action::EMERGENCY:
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::OK;
}

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_ACTION_HPP_
