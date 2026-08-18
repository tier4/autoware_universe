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

#include "autoware/trajectory_validator/detail/trajectory_validator.hpp"

#include "autoware/trajectory_validator/detail/risk_utils.hpp"
#include "autoware/trajectory_validator/detail/uuid_hash.hpp"

#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_trajectory_validator::msg::MetricReport;
using autoware_trajectory_validator::msg::RiskLevel;
using autoware_trajectory_validator::msg::ValidationReport;

TrajectoryValidatorReport TrajectoryValidator::process(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & input_trajectories,
  const std::unordered_set<std::string> & active_filter_names,
  const ValidatorContext & context) const
{
  TrajectoryValidatorReport report;
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");

  const auto uuid_to_generator_info = build_uuid_to_generator_info(input_trajectories);

  for (const auto & candidate_trajectory : input_trajectories.candidate_trajectories) {
    const auto generator_entry =
      resolve_generator_info(candidate_trajectory.generator_id, uuid_to_generator_info);

    const auto validation_results = validate_candidate_trajectory(
      plugins_, generator_entry.hex_generator_id, candidate_trajectory, context);

    for (const auto & [plugin_name, elapsed_ms] : validation_results.processing_time_ms) {
      report.processing_time_ms[plugin_name] += elapsed_ms;
    }

    std::move(
      validation_results.planning_factors.factors.begin(),
      validation_results.planning_factors.factors.end(),
      std::back_inserter(report.planning_factors.factors));

    report.evaluation_tables.push_back(validation_results.table);

    if (validation_results.table.all_acceptable()) {
      report.valid_trajectories.candidate_trajectories.push_back(candidate_trajectory);
    }

    if (validation_results.table.all_feasible()) {
      report.num_feasible_trajectories++;
    }

    report.validation_reports.push_back(build_validation_report(
      candidate_trajectory, generator_entry.info.generator_name.data, active_filter_names,
      std::move(validation_results.combined_metrics)));
  }

  report.valid_trajectories.generator_info =
    get_valid_trajectories_generator_info(report.valid_trajectories, uuid_to_generator_info);

  report.processing_time_ms["Total"] = stop_watch.toc("Total");
  return report;
}

GeneratorInfoMap TrajectoryValidator::build_uuid_to_generator_info(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & input_trajectories) const
{
  GeneratorInfoMap uuid_to_generator_info;
  uuid_to_generator_info.reserve(input_trajectories.generator_info.size());
  for (const auto & info : input_trajectories.generator_info) {
    uuid_to_generator_info[info.generator_id.uuid] =
      GeneratorInfoEntry{info, autoware_utils_uuid::to_hex_string(info.generator_id)};
  }
  return uuid_to_generator_info;
}

GeneratorInfoEntry TrajectoryValidator::resolve_generator_info(
  const unique_identifier_msgs::msg::UUID & generator_id,
  const GeneratorInfoMap & uuid_to_generator_info) const
{
  const auto it = uuid_to_generator_info.find(generator_id.uuid);
  if (it != uuid_to_generator_info.end()) {
    return it->second;
  }

  GeneratorInfoEntry fallback;
  fallback.hex_generator_id = autoware_utils_uuid::to_hex_string(generator_id);
  fallback.info.generator_name.data = "unknown_generator:" + fallback.hex_generator_id;
  return fallback;
}

PluginsValidationResult TrajectoryValidator::validate_candidate_trajectory(
  const std::vector<std::shared_ptr<plugin::ValidatorInterface>> & plugins,
  const std::string & hex_generator_id, const CandidateTrajectory & candidate_trajectory,
  const ValidatorContext & context) const
{
  PluginsValidationResult validation_results;
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;

  for (const auto & plugin : plugins) {
    const auto plugin_name = plugin->get_name();
    stop_watch.tic(plugin_name);
    const auto res = plugin->is_feasible(candidate_trajectory, context);
    auto [evaluation, risk_level] =
      summarize_feasibility(res, plugin_name, plugin->is_shadow_mode());

    if (res) {
      const auto & val = res.value();
      validation_results.combined_metrics.insert(
        validation_results.combined_metrics.end(), val.metrics.begin(), val.metrics.end());
      std::move(
        val.planning_factors.factors.begin(), val.planning_factors.factors.end(),
        std::back_inserter(validation_results.planning_factors.factors));
    }

    validation_results.combined_metrics.push_back(
      autoware_trajectory_validator::build<MetricReport>()
        .validator_name(plugin_name)
        .validator_category(plugin->category())
        .metric_name("trajectory_feasibility")
        .metric_value(evaluation.is_feasible ? 1.0 : 0.0)
        .risk(risk_level));
    validation_results.processing_time_ms[plugin_name] += stop_watch.toc(plugin_name);

    validation_results.table.plugin_evaluations.push_back(std::move(evaluation));
  }

  validation_results.table.generator_id = hex_generator_id;
  return validation_results;
}

std::pair<PluginEvaluation, RiskLevel> TrajectoryValidator::summarize_feasibility(
  const plugin::ValidatorInterface::result_t & res, const std::string & plugin_name,
  bool is_shadow_mode) const
{
  PluginEvaluation evaluation;
  RiskLevel risk_level;

  evaluation.plugin_name = plugin_name;
  evaluation.is_shadow_mode = is_shadow_mode;

  if (!res) {
    evaluation.is_feasible = false;
    evaluation.reason = res.error();
    // NOTE: If the plugin fails unexpectedly, treat it as a DANGER risk level.
    risk_level.level = RiskLevel::DANGER;
    return {evaluation, risk_level};
  }

  const auto & val = res.value();
  evaluation.is_feasible = val.is_feasible;
  if (!evaluation.is_feasible) {
    evaluation.reason = "Found failed metrics";
  }

  risk_level.level = worst_risk_level(val.metrics);
  return {evaluation, risk_level};
}

ValidationReport TrajectoryValidator::build_validation_report(
  const CandidateTrajectory & candidate_trajectory, const std::string & generator_name,
  const std::unordered_set<std::string> & active_filter_names,
  std::vector<MetricReport> combined_metrics) const
{
  // only consider metrics from active filters for final trajectory risk level
  std::vector<MetricReport> active_metrics;
  active_metrics.reserve(combined_metrics.size());
  std::copy_if(
    combined_metrics.begin(), combined_metrics.end(), std::back_inserter(active_metrics),
    [&](const auto & metric) { return active_filter_names.count(metric.validator_name) > 0; });

  RiskLevel risk_level;
  risk_level.level = worst_risk_level(active_metrics);

  return autoware_trajectory_validator::build<ValidationReport>()
    .trajectory_stamp(candidate_trajectory.header.stamp)
    .generator_id(candidate_trajectory.generator_id)
    .generator_name(generator_name)
    .risk(risk_level)
    .metrics(std::move(combined_metrics));
}

std::vector<GeneratorInfo> TrajectoryValidator::get_valid_trajectories_generator_info(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & valid_trajectories,
  const GeneratorInfoMap & uuid_to_generator_info) const
{
  std::vector<GeneratorInfo> generator_info;
  std::unordered_set<std::array<uint8_t, 16>, UuidHash> seen_generator_ids;

  for (const auto & traj : valid_trajectories.candidate_trajectories) {
    const auto & uuid = traj.generator_id.uuid;
    if (!seen_generator_ids.insert(uuid).second) {
      continue;
    }

    const auto it = uuid_to_generator_info.find(uuid);
    if (it != uuid_to_generator_info.end()) {
      generator_info.push_back(it->second.info);
    }
  }

  return generator_info;
}
}  // namespace autoware::trajectory_validator
