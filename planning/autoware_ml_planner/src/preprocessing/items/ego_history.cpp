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

#include "autoware/ml_planner/preprocessing/items/ego_history.hpp"

#include "autoware/ml_planner/constants.hpp"
#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/utils/utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <xtensor/xbuilder.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>

namespace autoware::ml_planner::preprocess
{

xt::xarray<float> create_ego_history(
  const MessageView<nav_msgs::msg::Odometry> & odom_msgs, size_t num_timesteps,
  const Eigen::Matrix4d & map_to_ego_transform, const rclcpp::Time & reference_time)
{
  constexpr size_t features_per_timestep = EGO_HISTORY_DIM;
  xt::xarray<float> ego_agent_past = xt::zeros<float>({num_timesteps, features_per_timestep});

  for (size_t t = 0; t < num_timesteps; ++t) {
    ego_agent_past(t, EGO_AGENT_PAST_IDX_COS) = 1.0f;
  }

  if (odom_msgs.empty()) {
    return ego_agent_past;
  }

  const auto store_state = [&](
                             size_t timestep_idx, const geometry_msgs::msg::Pose & pose,
                             const double velocity, const double yaw_rate) {
    const Eigen::Matrix4d pose_map_4x4 = utils::pose_to_matrix4d(pose);
    const Eigen::Matrix4d pose_ego_4x4 = map_to_ego_transform * pose_map_4x4;
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(pose_ego_4x4.block<3, 3>(0, 0));
    ego_agent_past(timestep_idx, EGO_AGENT_PAST_IDX_X) = pose_ego_4x4(0, 3);
    ego_agent_past(timestep_idx, EGO_AGENT_PAST_IDX_Y) = pose_ego_4x4(1, 3);
    ego_agent_past(timestep_idx, EGO_AGENT_PAST_IDX_COS) = cos_yaw;
    ego_agent_past(timestep_idx, EGO_AGENT_PAST_IDX_SIN) = sin_yaw;
    ego_agent_past(timestep_idx, EGO_AGENT_PAST_IDX_VELOCITY) = static_cast<float>(velocity);
    ego_agent_past(timestep_idx, EGO_AGENT_PAST_IDX_YAW_RATE) =
      std::abs(velocity) < constants::MOVING_VELOCITY_THRESHOLD_MPS
        ? 0.0f
        : std::clamp(static_cast<float>(yaw_rate), -0.95f, 0.95f);
  };

  const auto stamp_to_sec = [](const builtin_interfaces::msg::Time & stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  };

  const double ref_sec = reference_time.seconds();
  constexpr double dt = constants::PREDICTION_TIME_STEP_S;
  const double first_sec = stamp_to_sec(odom_msgs.front().header.stamp);
  const double last_sec = stamp_to_sec(odom_msgs.back().header.stamp);
  size_t search_start = 0;

  for (size_t t = 0; t < num_timesteps; ++t) {
    const double target_sec = ref_sec - static_cast<double>(num_timesteps - 1 - t) * dt;

    geometry_msgs::msg::Pose interpolated_pose;
    double interpolated_velocity = 0.0;
    double interpolated_yaw_rate = 0.0;
    if (target_sec <= first_sec) {
      interpolated_pose = odom_msgs.front().pose.pose;
      interpolated_velocity = odom_msgs.front().twist.twist.linear.x;
      interpolated_yaw_rate = odom_msgs.front().twist.twist.angular.z;
    } else if (target_sec >= last_sec) {
      interpolated_pose = odom_msgs.back().pose.pose;
      interpolated_velocity = odom_msgs.back().twist.twist.linear.x;
      interpolated_yaw_rate = odom_msgs.back().twist.twist.angular.z;
    } else {
      for (; search_start + 1 < odom_msgs.size(); ++search_start) {
        const double next_sec = stamp_to_sec(odom_msgs[search_start + 1].header.stamp);
        if (target_sec <= next_sec) {
          break;
        }
      }

      const double t0 = stamp_to_sec(odom_msgs[search_start].header.stamp);
      const double t1 = stamp_to_sec(odom_msgs[search_start + 1].header.stamp);
      const double ratio = (t1 > t0) ? (target_sec - t0) / (t1 - t0) : 0.0;
      interpolated_pose = autoware_utils_geometry::calc_interpolated_pose(
        odom_msgs[search_start].pose.pose, odom_msgs[search_start + 1].pose.pose, ratio, false);
      const auto & before_twist = odom_msgs[search_start].twist.twist;
      const auto & after_twist = odom_msgs[search_start + 1].twist.twist;
      interpolated_velocity =
        before_twist.linear.x + ratio * (after_twist.linear.x - before_twist.linear.x);
      interpolated_yaw_rate =
        before_twist.angular.z + ratio * (after_twist.angular.z - before_twist.angular.z);
    }

    store_state(t, interpolated_pose, interpolated_velocity, interpolated_yaw_rate);
  }

  return ego_agent_past;
}

}  // namespace autoware::ml_planner::preprocess
