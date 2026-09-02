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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_HPP_

#include "autoware/trajectory_validator/detail/trajectory_validator_report.hpp"
#include "autoware/trajectory_validator/detail/uuid_hash.hpp"
#include "autoware/trajectory_validator/detail/validator_context.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"
#include "autoware_trajectory_validator/autoware_trajectory_validator_param.hpp"

#include <autoware_utils_system/stop_watch.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::GeneratorInfo;
using autoware_trajectory_validator::msg::MetricReport;
using autoware_trajectory_validator::msg::RiskLevel;
using autoware_trajectory_validator::msg::ValidationReport;

/** @brief Generator info paired with its hex-string UUID, precomputed once to avoid reconversion.
 */
struct GeneratorInfoEntry
{
  GeneratorInfo info;
  std::string hex_generator_id;
};

using GeneratorInfoMap = std::unordered_map<std::array<uint8_t, 16>, GeneratorInfoEntry, UuidHash>;

/** @brief Per-trajectory output of running every plugin against one candidate trajectory. */
struct PluginsValidationResult
{
  EvaluationTable table;
  std::vector<MetricReport> combined_metrics;
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors;
  std::unordered_map<std::string, double> processing_time_ms;
};

/**
 * @brief Runs a set of validator plugins against each candidate trajectory.
 */
class TrajectoryValidator
{
public:
  /**
   * @brief Constructs the validator with the given plugin set.
   * @param plugins Validator plugins to run against each trajectory.
   */
  explicit TrajectoryValidator(std::vector<std::shared_ptr<plugin::ValidatorInterface>> plugins)
  : plugins_(std::move(plugins))
  {
  }

  /**
   * @brief Forwards updated parameters to all plugins.
   * @param params Latest parameter values.
   */
  void update_parameters(const validator::Params & params) const
  {
    for (const auto & plugin : plugins_) {
      plugin->update_parameters(params);
    }
  }

  /**
   * @brief Evaluates all plugins against every trajectory and returns a validation report.
   * @param input_trajectories Candidate trajectories to validate.
   * @param context Current world state snapshot.
   */
  [[nodiscard]] TrajectoryValidatorReport process(
    const CandidateTrajectories & input_trajectories,
    const std::unordered_set<std::string> & active_filter_names,
    const ValidatorContext & context) const;

private:
  /**
   * @brief Builds a lookup from each trajectory's raw UUID bytes to its generator info.
   * @param input_trajectories Candidate trajectories carrying the generator info to index.
   */
  [[nodiscard]] GeneratorInfoMap build_uuid_to_generator_info(
    const CandidateTrajectories & input_trajectories) const;

  /**
   * @brief Resolves a candidate trajectory's generator info, tolerating a missing lookup entry.
   * @param generator_id Trajectory's generator UUID.
   * @param uuid_to_generator_info Lookup built by build_uuid_to_generator_info.
   */
  [[nodiscard]] GeneratorInfoEntry resolve_generator_info(
    const unique_identifier_msgs::msg::UUID & generator_id,
    const GeneratorInfoMap & uuid_to_generator_info) const;

  /**
   * @brief Runs every plugin against one candidate trajectory and collects their results.
   * @param plugins Validator plugins to run.
   * @param hex_generator_id Trajectory's generator UUID as a hex string.
   * @param candidate_trajectory Trajectory to evaluate.
   * @param context Current world state snapshot.
   */
  [[nodiscard]] PluginsValidationResult validate_candidate_trajectory(
    const std::vector<std::shared_ptr<plugin::ValidatorInterface>> & plugins,
    const std::string & hex_generator_id,
    const autoware_internal_planning_msgs::msg::CandidateTrajectory & candidate_trajectory,
    const ValidatorContext & context) const;

  /**
   * @brief Turns one plugin's feasibility result into a plugin evaluation and risk level.
   * @param res Plugin's feasibility result, or an error string on failure.
   * @param plugin_name Name of the plugin that produced the result.
   * @param is_shadow_mode Whether the plugin is running in shadow mode.
   */
  [[nodiscard]] std::pair<PluginEvaluation, RiskLevel> summarize_feasibility(
    const plugin::ValidatorInterface::result_t & res, const std::string & plugin_name,
    bool is_shadow_mode) const;

  /**
   * @brief Assembles the published validation report for one candidate trajectory.
   * @param candidate_trajectory Trajectory the report is built for.
   * @param generator_name Trajectory's resolved generator name.
   * @param active_filter_names Filters whose metrics count toward the trajectory's risk level.
   * @param combined_metrics Metrics collected from every plugin for this trajectory.
   */
  [[nodiscard]] ValidationReport build_validation_report(
    const autoware_internal_planning_msgs::msg::CandidateTrajectory & candidate_trajectory,
    const std::string & generator_name, const std::unordered_set<std::string> & active_filter_names,
    std::vector<MetricReport> combined_metrics) const;

  /**
   * @brief Backfills generator_info for each trajectory that survived filtering, with at most one
   * entry per generator.
   * @param valid_trajectories Trajectories that passed validation.
   * @param uuid_to_generator_info Lookup built by build_uuid_to_generator_info.
   */
  [[nodiscard]] std::vector<GeneratorInfo> get_valid_trajectories_generator_info(
    const autoware_internal_planning_msgs::msg::CandidateTrajectories & valid_trajectories,
    const GeneratorInfoMap & uuid_to_generator_info) const;

  std::vector<std::shared_ptr<plugin::ValidatorInterface>> plugins_;
};
}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__TRAJECTORY_VALIDATOR_HPP_
