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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"

namespace autoware::trajectory_validator::plugin::safety
{
/**
 * @brief PointCloudCollisionCheckFilter class - checks the trajectory against the semantic
 * segmentation point cloud produced by the perception pipeline.
 */
class PointCloudCollisionCheckFilter final : public plugin::ValidatorInterface
{
public:
  PointCloudCollisionCheckFilter();

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;
};
}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_
