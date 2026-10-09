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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__GOAL_POSE_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__GOAL_POSE_HPP_

#include <Eigen/Core>
#include <xtensor/xarray.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace autoware::ml_planner::preprocess
{
xt::xarray<float> create_goal_pose(
  const geometry_msgs::msg::Pose & goal_pose, const Eigen::Matrix4d & map_to_ego_transform);
}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__GOAL_POSE_HPP_
