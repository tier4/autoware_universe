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

#include "autoware/trajectory_validator/filters/safety/point_cloud_collision_check_filter.hpp"

namespace autoware::trajectory_validator::plugin::safety
{
PointCloudCollisionCheckFilter::PointCloudCollisionCheckFilter()
: ValidatorInterface("point_cloud_collision_check_filter")
{
}

// memo: assume trajectory_selector subscribes "/perception/obstacle_segmentation/pointcloud" or
// "/perception/segmented/pointcloud" that was published from ptv3 node
PointCloudCollisionCheckFilter::result_t PointCloudCollisionCheckFilter::is_feasible(
  [[maybe_unused]] const CandidateTrajectory & candidate_trajectory,
  [[maybe_unused]] const FilterContext & context)
{
  return ValidationResult{};
}

void PointCloudCollisionCheckFilter::update_parameters(
  [[maybe_unused]] const validator::Params & params)
{
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::PointCloudCollisionCheckFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
