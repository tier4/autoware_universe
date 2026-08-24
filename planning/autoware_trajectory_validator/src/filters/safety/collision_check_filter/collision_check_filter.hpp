// Copyright 2025 TIER IV, Inc.
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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__COLLISION_CHECK_FILTER_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__COLLISION_CHECK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"
#include "parameter.hpp"
#include "reporter.hpp"
#include "stop_tracker.hpp"
#include "trajectory_utils.hpp"
#include "types.hpp"

#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
class CollisionCheckFilter : public plugin::ValidatorInterface
{
public:
  CollisionCheckFilter() : ValidatorInterface("collision_check_filter") {}

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) override;

  void update_parameters(const validator::Params & node_params) final;

private:
  GlobalParams global_params_;
  DracParamMap drac_param_map_;
  RssParamMap rss_param_map_;
  StopTrackers stop_tracker_;

  // Retained across is_feasible() calls so object trajectories are reused within one perception
  // frame; invalidated via update() when the PredictedObjects timestamp changes.
  trajectory::ObjectTrajectoryCache object_trajectory_cache_;

  std::vector<MetricReport> generate_metric_reports(
    const DracArtifact & drac_artifact, const RssArtifact & rss_artifact) const;
};

}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__COLLISION_CHECK_FILTER_HPP_
