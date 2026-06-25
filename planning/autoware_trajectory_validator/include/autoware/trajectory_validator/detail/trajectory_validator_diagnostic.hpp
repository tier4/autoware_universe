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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_DIAGNOSTIC_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_DIAGNOSTIC_HPP_

#include "autoware/trajectory_validator/detail/risk_action.hpp"

#include <autoware_trajectory_validator/msg/validation_report.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::trajectory_validator
{

/**
 * @brief Per-validator binding: maps each actionable Action to the diagnostic status name to
 *        publish.
 *
 * An absent or empty name means no status is published for that (validator, action) pair.
 */
struct FilterStatusBinding
{
  std::map<Action, std::string> name_by_action;
};

/** @brief Maps validator_name to its status name bindings. */
using FilterStatusMap = std::unordered_map<std::string, FilterStatusBinding>;

/**
 * @brief Builds a FilterStatusMap from a bindings string array loaded from
 *        trajectory_validator_diagnostic parameters.
 * @param bindings Each entry encodes one binding as
 *        filter_name:action:diagnostic_name. Entries that are empty or
 *        lack the two required colons are skipped silently.
 */
inline FilterStatusMap make_filter_status_map(const std::vector<std::string> & bindings)
{
  FilterStatusMap m;
  for (const auto & binding : bindings) {
    if (binding.empty()) {
      continue;
    }
    const auto first_colon = binding.find(':');
    if (first_colon == std::string::npos) {
      continue;
    }
    const auto second_colon = binding.find(':', first_colon + 1);
    if (second_colon == std::string::npos) {
      continue;
    }
    const auto filter_name = binding.substr(0, first_colon);
    const auto action_str = binding.substr(first_colon + 1, second_colon - first_colon - 1);
    const auto diagnostic_name = binding.substr(second_colon + 1);
    if (filter_name.empty() || diagnostic_name.empty()) {
      continue;
    }
    m[filter_name].name_by_action[parse_action(action_str)] = diagnostic_name;
  }
  return m;
}

/**
 * @brief Creates one DiagnosticsInterface per distinct non-empty status name found in
 *        filter_status_map, plus one for no_candidate_name if non-empty.
 * @param node ROS 2 node used for publisher creation.
 * @param filter_status_map Mapping from validator_name to its status name bindings.
 * @param no_candidate_name Status name published when no candidate trajectory is available.
 */
std::unordered_map<std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>
build_diagnostic_interface_map(
  rclcpp::Node & node, const FilterStatusMap & filter_status_map,
  const std::string & no_candidate_name);

/**
 * @brief Aggregates ValidationReports into a per-trajectory action and republishes every tracked
 *        DiagnosticStatus every cycle so none go stale.
 *
 * Pure logic class: takes a pre-built DiagnosticsInterface map; does not hold a ROS node.
 */
class TrajectoryValidatorDiagnostic
{
public:
  /**
   * @brief Constructs the diagnostic handler with pre-built DiagnosticsInterface objects.
   * @param filter_status_map Mapping from validator_name to its status name bindings.
   * @param no_candidate_name Status name to raise ERROR on when reports is empty.
   * @param active_filter_names Short validator_name strings (from plugin->get_name()) that are
   *        active (non-shadow). Any validator not in this set is treated as shadow and excluded
   *        from action aggregation. An empty set means all validators are considered active.
   * @param diag_by_name Pre-built DiagnosticsInterface map (use build_diagnostic_interface_map).
   */
  TrajectoryValidatorDiagnostic(
    FilterStatusMap filter_status_map, std::string no_candidate_name,
    const std::unordered_set<std::string> & active_filter_names,
    std::unordered_map<std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>
      diag_by_name);

  /**
   * @brief Aggregates reports into a best-available action and publishes all tracked statuses.
   * @param reports One ValidationReport per candidate trajectory.
   * @param stamp Timestamp forwarded to each DiagnosticsInterface::publish call.
   */
  void update_and_publish(
    const std::vector<autoware_trajectory_validator::msg::ValidationReport> & reports,
    const rclcpp::Time & stamp);

private:
  struct ValidatorDiagnosticActionInfo
  {
    Action action{Action::NONE};
    std::unordered_map<std::string, Action> per_validator;
  };

  /**
   * @brief Aggregates per-validator actions for one candidate trajectory, considering only
   *        active validators (those in active_filter_names_, or all if the set is empty).
   * @param report Validation report for a single candidate trajectory.
   */
  ValidatorDiagnosticActionInfo compute_action_info(
    const autoware_trajectory_validator::msg::ValidationReport & report) const;

  /**
   * @brief Looks up status names for each binding validator on the best trajectory and marks them
   *        active.
   * @param best_info Action info for the best-available trajectory.
   * @param active Output map of status name to published level.
   */
  void collect_active_statuses(
    const ValidatorDiagnosticActionInfo & best_info,
    std::unordered_map<std::string, int8_t> & active) const;

  /**
   * @brief Resets every tracked DiagnosticsInterface, applies active levels, and publishes.
   * @param active Map of status name to published level for this cycle.
   * @param stamp Timestamp forwarded to each publish call.
   */
  void publish_all(
    const std::unordered_map<std::string, int8_t> & active, const rclcpp::Time & stamp);

  FilterStatusMap filter_status_map_;
  std::unordered_set<std::string> active_filter_names_;
  std::string no_candidate_name_;
  std::unordered_map<std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>
    diag_by_name_;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_DIAGNOSTIC_HPP_
