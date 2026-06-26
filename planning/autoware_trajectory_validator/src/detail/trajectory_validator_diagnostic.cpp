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

#include "autoware/trajectory_validator/detail/trajectory_validator_diagnostic.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{

std::unordered_map<std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>
build_diagnostic_interface_map(
  rclcpp::Node & node, const FilterStatusMap & filter_status_map,
  const std::string & no_candidate_name)
{
  std::unordered_map<std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>
    diag_by_name;
  for (const auto & [validator, binding] : filter_status_map) {
    for (const auto & [action, name] : binding.name_by_action) {
      if (!name.empty() && !diag_by_name.count(name)) {
        diag_by_name.emplace(
          name, std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(&node, name));
      }
    }
  }
  if (!no_candidate_name.empty() && !diag_by_name.count(no_candidate_name)) {
    diag_by_name.emplace(
      no_candidate_name,
      std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(&node, no_candidate_name));
  }
  return diag_by_name;
}

TrajectoryValidatorDiagnostic::TrajectoryValidatorDiagnostic(
  FilterStatusMap filter_status_map, std::string no_candidate_name,
  const std::unordered_set<std::string> & active_filter_names,
  std::unordered_map<std::string, std::unique_ptr<autoware_utils_diagnostics::DiagnosticsInterface>>
    diag_by_name)
: filter_status_map_(std::move(filter_status_map)),
  active_filter_names_(active_filter_names),
  no_candidate_name_(std::move(no_candidate_name)),
  diag_by_name_(std::move(diag_by_name))
{
}

void TrajectoryValidatorDiagnostic::collect_active_statuses(
  const std::unordered_map<std::string, Action> & validator_actions,
  std::unordered_map<std::string, int8_t> & active) const
{
  for (const auto & [name, action] : validator_actions) {
    if (action == Action::NONE) {
      continue;
    }
    const auto map_it = filter_status_map_.find(name);
    if (map_it == filter_status_map_.end()) {
      continue;
    }
    for (const auto & [binding_action, status_name] : map_it->second.name_by_action) {
      if (status_name.empty() || binding_action == Action::NONE || binding_action > action) {
        continue;
      }
      active[status_name] = published_level_of(binding_action);
    }
  }
}

void TrajectoryValidatorDiagnostic::mark_shadow_statuses(
  std::unordered_map<std::string, int8_t> & active) const
{
  if (active_filter_names_.empty()) {
    return;
  }
  for (const auto & [name, binding] : filter_status_map_) {
    if (active_filter_names_.count(name)) {
      continue;
    }
    for (const auto & [action, status_name] : binding.name_by_action) {
      if (!status_name.empty() && !active.count(status_name)) {
        active[status_name] = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      }
    }
  }
}

void TrajectoryValidatorDiagnostic::update_and_publish(
  const std::vector<autoware_trajectory_validator::msg::ValidationReport> & reports,
  const rclcpp::Time & stamp)
{
  std::unordered_map<std::string, int8_t> active;

  if (reports.empty()) {
    if (!no_candidate_name_.empty()) {
      active[no_candidate_name_] = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
  } else {
    std::unordered_map<std::string, Action> validator_actions;
    for (const auto & report : reports) {
      // Within one candidate: take the worst (max) action per validator.
      // A validator may produce multiple metrics for the same candidate; the harshest one governs.
      std::unordered_map<std::string, Action> candidate_action;
      for (const auto & metric : report.metrics) {
        if (!active_filter_names_.empty() && !active_filter_names_.count(metric.validator_name)) {
          continue;
        }
        const Action metric_action = to_action(metric.level);
        auto [it, inserted] = candidate_action.emplace(metric.validator_name, metric_action);
        if (!inserted) {
          it->second = std::max(it->second, metric_action);
        }
      }
      // Across candidates: keep the best (min) action per validator.
      // If a validator passes on any candidate, its action stays NONE and no binding fires —
      // the car can always choose that safer trajectory.
      for (const auto & [name, action] : candidate_action) {
        auto [it, inserted] = validator_actions.emplace(name, action);
        if (!inserted) {
          it->second = std::min(it->second, action);
        }
      }
    }
    collect_active_statuses(validator_actions, active);
  }

  mark_shadow_statuses(active);
  publish_all(active, stamp);
}

void TrajectoryValidatorDiagnostic::publish_all(
  const std::unordered_map<std::string, int8_t> & active, const rclcpp::Time & stamp)
{
  for (auto & [name, diag] : diag_by_name_) {
    diag->clear();
    const auto it = active.find(name);
    if (it != active.end()) {
      diag->update_level_and_message(it->second, name + " triggered");
    }
    diag->publish(stamp);
  }
}

}  // namespace autoware::trajectory_validator
