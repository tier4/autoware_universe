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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__EGO_HISTORY_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__EGO_HISTORY_HPP_

#include "autoware/ml_planner/preprocessing/message_view.hpp"

#include <Eigen/Core>
#include <rclcpp/time.hpp>
#include <xtensor/xarray.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <cstddef>
#include <deque>

namespace autoware::ml_planner::preprocess
{

/**
 * @brief Creates ego history data, including the current state, from odometry messages.
 *
 * Generates the trajectory by interpolating poses on a fixed time grid (0.1 s)
 * backwards from the reference time.
 *
 * @param[in] odom_msgs Chronological deque of odometry messages
 * @param[in] num_timesteps Number of timesteps to process
 * @param[in] map_to_ego_transform Transformation matrix from map to ego frame
 * @param[in] reference_time Time of the newest grid sample
 * @return Tensor with shape [num_timesteps, 6] containing
 * [x, y, cos_yaw, sin_yaw, velocity, yaw_rate]
 */
xt::xarray<float> create_ego_history(
  const MessageView<nav_msgs::msg::Odometry> & odom_msgs, size_t num_timesteps,
  const Eigen::Matrix4d & map_to_ego_transform, const rclcpp::Time & reference_time);

}  // namespace autoware::ml_planner::preprocess

#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__EGO_HISTORY_HPP_
