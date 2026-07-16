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

#include "autoware/trajectory_ranker/trajectory_ranker.hpp"

#include "autoware/trajectory_ranker/utils.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectory.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace
{
using autoware_trajectory_validator::msg::RiskLevel;
using std::string;
std::string to_risk_string(const RiskLevel::_level_type risk_level)
{
  static const std::unordered_map<RiskLevel::_level_type, std::string> risk_level_to_string = {
    {RiskLevel::SAFE, "safe"},
    {RiskLevel::LOW_CAUTION, "low_caution"},
    {RiskLevel::HIGH_CAUTION, "high_caution"},
    {RiskLevel::DANGER, "danger"},
    {RiskLevel::FATAL, "fatal"},
  };
  if (risk_level_to_string.count(risk_level) == 0) {
    return "";
  }
  return risk_level_to_string.at(risk_level);
}

std::string get_source_from_generator_name_prefix(const std::string & generator_name_prefix)
{
  static const std::unordered_map<std::string, std::string> generator_name_prefix_to_source = {
    {"DiffusionPlanner_", "diffusion_planner"},
    {"MinimumRuleBasedPlanner", "backup_planner"},
  };
  if (generator_name_prefix_to_source.count(generator_name_prefix) == 0) {
    return "";
  }
  return generator_name_prefix_to_source.at(generator_name_prefix);
}
}  // namespace

namespace autoware::trajectory_ranker
{

tl::expected<ScoredCandidateTrajectories, std::string> TrajectoryRanker::process(
  const CandidateTrajectories & candidate_trajectories, const RankerContext & context)
{
  ScoredTrajectories scored_trajectories;

  // Create map from UUID to generator name
  std::unordered_map<std::string, std::string> generator_id_to_name_map;
  generator_id_to_name_map.reserve(candidate_trajectories.generator_info.size());
  for (const auto & info : candidate_trajectories.generator_info) {
    generator_id_to_name_map[autoware_utils_uuid::to_hex_string(info.generator_id)] =
      info.generator_name.data;
  }

  for (const auto & candidate : candidate_trajectories.candidate_trajectories) {
    scored_trajectories.emplace_back(ScoredTrajectory{candidate});
  }

  evaluate_safety(scored_trajectories, context);
  evaluate_source(scored_trajectories, generator_id_to_name_map);
  evaluate_quality(scored_trajectories, context);
  score_trajectories(scored_trajectories);
  update_trajectory_history(scored_trajectories);

  ScoredCandidateTrajectories output;
  for (const auto & scored_trajectory : scored_trajectories) {
    ScoredCandidateTrajectory scored_candidate;
    scored_candidate.candidate_trajectory = scored_trajectory.candidate_trajectory;
    scored_candidate.score = static_cast<float>(scored_trajectory.score);
    output.scored_candidate_trajectories.push_back(scored_candidate);
  }
  return output;
}

void TrajectoryRanker::evaluate_safety(
  ScoredTrajectories & scored_trajectories, const RankerContext & context) const
{
  if (!params_.safety.enable || context.validation_reports == nullptr) return;

  auto & validation_reports = *context.validation_reports;

  auto get_risk_level =
    [&](const CandidateTrajectory & candidate_trajectory) -> RiskLevel::_level_type {
    auto itr =
      std::find_if(validation_reports.begin(), validation_reports.end(), [&](const auto & report) {
        return candidate_trajectory.generator_id.uuid == report->generator_id.uuid;
      });
    if (itr == validation_reports.end()) {
      return RiskLevel::FATAL;
    }
    return itr->get()->risk.level;
  };

  auto get_safety_penalty = [&](const RiskLevel::_level_type risk_level) -> double {
    const auto risk_string = to_risk_string(risk_level);
    const auto idx =
      std::find(params_.safety.levels.begin(), params_.safety.levels.end(), risk_string);
    if (idx == params_.safety.levels.end()) {
      return 1.0;
    }
    return params_.safety.penalty.at(idx - params_.safety.levels.begin());
  };

  for (auto & scored_trajectory : scored_trajectories) {
    const auto risk_level = get_risk_level(scored_trajectory.candidate_trajectory);
    scored_trajectory.safety_penalty = get_safety_penalty(risk_level);
  }
}

void TrajectoryRanker::evaluate_source(
  ScoredTrajectories & scored_trajectories,
  const std::unordered_map<std::string, std::string> & generator_id_to_name_map) const
{
  if (!params_.source.enable) return;

  auto get_source_penalty = [&](const std::string & source) -> double {
    const auto idx = std::find(params_.source.levels.begin(), params_.source.levels.end(), source);
    if (idx == params_.source.levels.end()) {
      return 1.0;
    }
    return params_.source.penalty.at(idx - params_.source.levels.begin());
  };

  for (auto & scored_trajectory : scored_trajectories) {
    const auto generator_id = scored_trajectory.candidate_trajectory.generator_id;
    const auto generator_name =
      generator_id_to_name_map.at(autoware_utils_uuid::to_hex_string(generator_id));
    const auto source = get_source_from_generator_name_prefix(generator_name);
    scored_trajectory.source_penalty = get_source_penalty(source);
  }
}

void TrajectoryRanker::evaluate_quality(
  ScoredTrajectories & scored_trajectories, const RankerContext & context) const
{
  if (!params_.evaluation.enable) return;

  if (!context.route_handler->isHandlerReady() || context.odometry == nullptr) {
    return;
  }

  const auto preferred_lanes =
    std::make_shared<lanelet::ConstLanelets>(context.route_handler->getPreferredLanelets());

  evaluator_->clear();

  // Create shared pointer to trajectory history for passing to CoreData
  auto trajectory_history_ptr = std::make_shared<std::deque<Trajectory>>(trajectory_history_);

  // Process each candidate trajectory
  for (auto & scored_trajectory : scored_trajectories) {
    const auto & candidate = scored_trajectory.candidate_trajectory;
    auto sampled = utils::sampling(
      candidate.points, context.odometry->pose.pose, params_.evaluation.sampling_number,
      params_.evaluation.sampling_resolution);
    auto sampled_points = std::make_shared<TrajectoryPoints>(std::move(sampled));
    auto original_points = std::make_shared<TrajectoryPoints>(candidate.points);

    auto core_data = std::make_shared<CoreData>(
      original_points, sampled_points, previous_points_, preferred_lanes, candidate.header,
      candidate.generator_id, trajectory_history_ptr, candidate.turn_indicators_command);

    auto quality_score = evaluator_->score(core_data);
    scored_trajectory.quality_penalty = 1.0 - quality_score;
  }
}

void TrajectoryRanker::score_trajectories(ScoredTrajectories & scored_trajectories) const
{
  const auto max_cost = params_.safety.scale + params_.source.scale + params_.evaluation.scale;
  for (auto & scored_trajectory : scored_trajectories) {
    auto cost = params_.safety.scale * scored_trajectory.safety_penalty;
    cost += params_.source.scale * scored_trajectory.source_penalty;
    cost += params_.evaluation.scale * scored_trajectory.quality_penalty;
    cost /= max_cost;
    scored_trajectory.score = 1.0 - cost;
  }
}

void TrajectoryRanker::update_trajectory_history(const ScoredTrajectories & scored_trajectories)
{
  auto best_itr = std::max_element(
    scored_trajectories.begin(), scored_trajectories.end(),
    [](const auto & a, const auto & b) { return a.score < b.score; });
  if (best_itr == scored_trajectories.end()) return;

  Trajectory best_trajectory;
  best_trajectory.header = best_itr->candidate_trajectory.header;
  best_trajectory.points = best_itr->candidate_trajectory.points;
  trajectory_history_.push_back(best_trajectory);

  if (
    trajectory_history_.size() > static_cast<size_t>(params_.evaluation.trajectory_history_size)) {
    trajectory_history_.pop_front();
  }
}

}  // namespace autoware::trajectory_ranker
