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

#ifndef AUTOWARE__TRAJECTORY_RANKER__METRICS__TRAJECTORY_RANKER_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__METRICS__TRAJECTORY_RANKER_HPP_

#include "autoware/trajectory_ranker/interface/metrics_interface.hpp"
#include "autoware/trajectory_ranker/evaluation.hpp"

#include <autoware_trajectory_ranker/autoware_trajectory_ranker_param.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/scored_candidate_trajectories.hpp>
#include <autoware_trajectory_validator/msg/validation_report.hpp>

namespace autoware::trajectory_ranker
{
using metrics::MetricInterface;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::ScoredCandidateTrajectories;
using autoware_trajectory_validator::msg::ValidationReport;

struct RankerContext
{
  ValidationReport::ConstSharedPtr validation_report;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry;
  std::shared_ptr<RouteHandler> route_handler;
};

class TrajectoryRanker
{
public:
  /**
   * @brief Constructs the ranker with the given evaluator and parameters.
   * @param evaluator Evaluator to use for ranking trajectories.
   * @param params Parameters for the ranker.
   */
  explicit TrajectoryRanker(const std::shared_ptr<Evaluator> & evaluator, const trajectory_ranker_params::Params & params)
  : evaluator_(evaluator), params_(params)
  {
  }

  tl::expected<ScoredCandidateTrajectories, std::string> process(const CandidateTrajectories & candidate_trajectories, const RankerContext & context);

private:
  std::shared_ptr<Evaluator> evaluator_;

  std::shared_ptr<TrajectoryPoints> previous_points_;
  std::deque<autoware_planning_msgs::msg::Trajectory> trajectory_history_;

  trajectory_ranker_params::Params params_;
};

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__METRICS__TRAJECTORY_RANKER_HPP_