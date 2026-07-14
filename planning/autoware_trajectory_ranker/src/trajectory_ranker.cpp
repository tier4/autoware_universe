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

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectory.hpp>

namespace autoware::trajectory_ranker
{

tl::expected<ScoredCandidateTrajectories, std::string> TrajectoryRanker::process(const CandidateTrajectories & candidate_trajectories, const RankerContext & context)
{
  ScoredCandidateTrajectories output;

  if (!context.route_handler->isHandlerReady()) {
    return tl::unexpected("Route handler is not ready");
  }

  if (context.odometry == nullptr) {
    return tl::unexpected("Odometry data is not available");
  }

  const auto preferred_lanes =
    std::make_shared<lanelet::ConstLanelets>(context.route_handler->getPreferredLanelets());

  std::vector<autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory> trajectories;
  trajectories.reserve(candidate_trajectories.candidate_trajectories.size());

  evaluator_->clear();

  // Process each candidate trajectory
  for (const auto & candidate : candidate_trajectories.candidate_trajectories) {
    auto sampled = utils::sampling(
      candidate.points, context.odometry->pose.pose, params_.evaluation.sampling_number, params_.evaluation.sampling_resolution);
    auto sampled_points = std::make_shared<TrajectoryPoints>(std::move(sampled));
    auto original_points = std::make_shared<TrajectoryPoints>(candidate.points);

    // Create shared pointer to trajectory history for passing to CoreData
    auto trajectory_history_ptr = std::make_shared<std::deque<Trajectory>>(trajectory_history_);

    auto core_data = std::make_shared<CoreData>(
      original_points, sampled_points, previous_points_, preferred_lanes,
      candidate.header, candidate.generator_id, trajectory_history_ptr,
      candidate.turn_indicators_command);

    evaluator_->add(core_data);
  }

  const auto best_data = evaluator_->best();
  previous_points_ = best_data == nullptr ? nullptr : best_data->points();

  // Update trajectory history buffer with the best trajectory
  if (best_data != nullptr && best_data->points() != nullptr) {
    // Create Trajectory from best trajectory
    Trajectory best_trajectory;
    best_trajectory.header = best_data->header();
    best_trajectory.points = *best_data->original();

    trajectory_history_.push_back(best_trajectory);

    // Limit buffer size
    if (trajectory_history_.size() > static_cast<size_t>(params_.evaluation.trajectory_history_size)) {
      trajectory_history_.pop_front();
    }
  }

  for (const auto & result : evaluator_->results()) {
    const auto candidate = autoware_internal_planning_msgs::build<
                             autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                             .header(result->header())
                             .generator_id(result->uuid())
                             .points(*result->original())
                             .turn_indicators_command(result->turn_indicators_command());
    const auto scored_trajectory =
      autoware_internal_planning_msgs::build<
        autoware_internal_planning_msgs::msg::ScoredCandidateTrajectory>()
        .candidate_trajectory(candidate)
        .score(result->total());
    trajectories.push_back(scored_trajectory);
  }

  output.scored_candidate_trajectories = trajectories;
  return output;
}

}  // namespace autoware::trajectory_ranker