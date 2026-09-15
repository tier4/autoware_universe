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

#include "autoware/ml_planner/preprocessing/items/goal_pose.hpp"

#include "autoware/ml_planner/utils/utils.hpp"

namespace autoware::ml_planner::preprocess
{
xt::xarray<float> create_goal_pose(
  const geometry_msgs::msg::Pose & goal_pose, const Eigen::Matrix4d & map_to_ego_transform)
{
  const Eigen::Matrix4d goal_pose_ego = map_to_ego_transform * utils::pose_to_matrix4d(goal_pose);
  const auto [cos_yaw, sin_yaw] =
    utils::rotation_matrix_to_cos_sin(goal_pose_ego.block<3, 3>(0, 0));
  return {
    static_cast<float>(goal_pose_ego(0, 3)), static_cast<float>(goal_pose_ego(1, 3)), cos_yaw,
    sin_yaw};
}
}  // namespace autoware::ml_planner::preprocess
