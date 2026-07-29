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

#include "point_cloud_collision_check_filter.hpp"

#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace pcc = autoware::trajectory_validator::plugin::safety::point_cloud_collision_check;

bool PointCloudCollisionCheckFilter::is_available_data(
  [[maybe_unused]] const CandidateTrajectory & candidate_trajectory,
  [[maybe_unused]] const FilterContext & context) const
{
  // 中身は後続 PR で移植する。未検証の入力を参照しないよう、現状は常に false を返す。
  return false;
}

void PointCloudCollisionCheckFilter::set_planner_data_param()
{
  // 中身は後続 PR で移植する。
}

void PointCloudCollisionCheckFilter::update_planner_data(
  [[maybe_unused]] const std::vector<TrajectoryPoint> & raw_trajectory_points,
  [[maybe_unused]] const FilterContext & context)
{
  // 中身は後続 PR で移植する。
}

std::vector<pcc::StopObstacle> PointCloudCollisionCheckFilter::calc_obstacle_stop(
  [[maybe_unused]] const std::vector<TrajectoryPoint> & raw_trajectory_points)
{
  // 中身は後続 PR で移植する。
  return {};
}

bool PointCloudCollisionCheckFilter::judge_stop_feasibility(
  [[maybe_unused]] const std::vector<pcc::StopObstacle> & stop_obstacles,
  [[maybe_unused]] const geometry_msgs::msg::Twist & twist) const
{
  // It is not currently implemented. always return true.
  bool is_feasible = true;
  return is_feasible;
}

PointCloudCollisionCheckFilter::result_t PointCloudCollisionCheckFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  // memo: assume trajectory_selector subscribes "/perception/obstacle_segmentation/pointcloud" or
  // "/perception/segmented/pointcloud" that was published from ptv3 node

  if (!is_available_data(candidate_trajectory, context)) {
    return ValidationResult{};
  }

  update_planner_data(candidate_trajectory.points, context);

  const auto stop_obstacles = calc_obstacle_stop(candidate_trajectory.points);

  ValidationResult result{};
  result.is_feasible = judge_stop_feasibility(stop_obstacles, context.odometry->twist.twist);

  return result;
}

void PointCloudCollisionCheckFilter::update_parameters(const validator::Params & params)
{
  params_ = pcc::Params{params};
  set_planner_data_param();
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::PointCloudCollisionCheckFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
