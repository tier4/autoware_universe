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
#include <rclcpp/rclcpp.hpp>

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
 * @brief Default production mapping.
 * @note Key is "uncrossable_boundary_departure_filter" — the short name from
 *       UncrossableBoundaryDepartureFilter::get_name().  shadow_mode_filter_names stores plugin
 *       CLASS names; wrapper wiring must translate them via plugin->get_name() before passing
 *       the shadow set to TrajectoryValidatorDiagnostic.
 */
inline FilterStatusMap make_default_filter_status_map()
{
  FilterStatusMap m;
  m["uncrossable_boundary_departure_filter"].name_by_action[Action::MODERATE] =
    "trajectory_validator_uncrossable_boundary_departure_danger";
  return m;
}

/**
 * @brief Reads ValidationReports, decides a per-trajectory Action, and republishes per-validator
 *        DiagnosticStatus values every cycle so none go stale.
 */
class TrajectoryValidatorDiagnostic
{
public:
  /**
   * @brief Constructs the diagnostic handler and pre-creates one DiagnosticsInterface per
   *        distinct status name found in filter_status_map.
   * @param node ROS 2 node used for publisher creation and logging.
   * @param filter_status_map Mapping from validator_name to its status name bindings.
   * @param no_candidate_name Status name to raise ERROR on when reports is empty.
   * @param shadow_validator_names Short validator_name strings (from plugin->get_name()) to skip
   *        during action aggregation.
   */
  TrajectoryValidatorDiagnostic(
    rclcpp::Node & node, FilterStatusMap filter_status_map, std::string no_candidate_name,
    const std::unordered_set<std::string> & shadow_validator_names = {});

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
   * @brief Aggregates per-validator actions for one candidate trajectory, skipping shadow
   *        validators.
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

  rclcpp::Node * node_ptr_;
  FilterStatusMap filter_status_map_;
  std::unordered_set<std::string> shadow_validator_names_;
  std::string no_candidate_name_;
  std::unordered_map<std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>
    diag_by_name_;
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_DIAGNOSTIC_HPP_
