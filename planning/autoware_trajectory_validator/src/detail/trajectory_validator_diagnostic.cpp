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
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator
{

TrajectoryValidatorDiagnostic::TrajectoryValidatorDiagnostic(
  rclcpp::Node & node, const FilterStatusMap & filter_status_map,
  const std::string & no_candidate_name, const std::set<std::string> & shadow_validator_names)
: node_ptr_(&node),
  filter_status_map_(filter_status_map),
  shadow_validator_names_(shadow_validator_names),
  no_candidate_name_(no_candidate_name)
{
  // Create one DiagnosticsInterface per distinct non-empty status name from the map.
  for (const auto & [validator, binding] : filter_status_map_) {
    for (const auto & [action, name] : binding.name_by_action) {
      if (!name.empty() && diag_by_name_.find(name) == diag_by_name_.end()) {
        diag_by_name_.emplace(
          name,
          std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(node_ptr_, name));
      }
    }
  }
  if (
    !no_candidate_name_.empty() && diag_by_name_.find(no_candidate_name_) == diag_by_name_.end()) {
    diag_by_name_.emplace(
      no_candidate_name_, std::make_unique<autoware_utils_diagnostics::DiagnosticsInterface>(
                            node_ptr_, no_candidate_name_));
  }
}

TrajectoryValidatorDiagnostic::ValidatorDiagnosticActionInfo
TrajectoryValidatorDiagnostic::compute_action_info(
  const autoware_trajectory_validator::msg::ValidationReport & report) const
{
  ValidatorDiagnosticActionInfo info;
  for (const auto & metric : report.metrics) {
    if (shadow_validator_names_.count(metric.validator_name)) {
      continue;
    }
    const Action metric_action = to_action(metric.level);
    auto [it, inserted] = info.per_validator.emplace(metric.validator_name, metric_action);
    if (!inserted) {
      it->second = std::max(it->second, metric_action);
    }
  }
  for (const auto & [vname, vaction] : info.per_validator) {
    info.action = std::max(info.action, vaction);
  }
  return info;
}

void TrajectoryValidatorDiagnostic::collect_active_statuses(
  const ValidatorDiagnosticActionInfo & best_info,
  std::unordered_map<std::string, int8_t> & active) const
{
  for (const auto & [vname, vaction] : best_info.per_validator) {
    if (vaction != best_info.action) {
      continue;  // only binding validators (those at the trajectory's action level)
    }
    const auto map_it = filter_status_map_.find(vname);
    if (map_it == filter_status_map_.end()) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
        "[TrajectoryValidatorDiagnostic] no mapping entry for binding validator '%s'",
        vname.c_str());
      continue;
    }
    const auto name_it = map_it->second.name_by_action.find(vaction);
    if (name_it == map_it->second.name_by_action.end() || name_it->second.empty()) {
      RCLCPP_WARN_THROTTLE(
        node_ptr_->get_logger(), *node_ptr_->get_clock(), 1000,
        "[TrajectoryValidatorDiagnostic] empty status name for binding validator '%s' at "
        "action %d",
        vname.c_str(), static_cast<int>(vaction));
      continue;
    }
    active[name_it->second] = published_level_of(vaction);
  }
}

void TrajectoryValidatorDiagnostic::update_and_publish(
  const std::vector<autoware_trajectory_validator::msg::ValidationReport> & reports,
  const rclcpp::Time & stamp)
{
  std::unordered_map<std::string, int8_t> active;  // status name -> published level

  if (reports.empty()) {
    if (!no_candidate_name_.empty()) {
      active[no_candidate_name_] = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    publish_all(active, stamp);
    return;
  }

  std::vector<ValidatorDiagnosticActionInfo> traj_infos;
  traj_infos.reserve(reports.size());
  for (const auto & report : reports) {
    traj_infos.push_back(compute_action_info(report));
  }

  const auto best = std::min_element(
    traj_infos.begin(), traj_infos.end(),
    [](const ValidatorDiagnosticActionInfo & lhs, const ValidatorDiagnosticActionInfo & rhs) {
      return lhs.action < rhs.action;
    });

  if (best->action != Action::NONE) {
    collect_active_statuses(*best, active);
  }

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
