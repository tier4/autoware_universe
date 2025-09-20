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

#include "autoware/diffusion_planner/utils/utils.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::utils
{

namespace
{
Eigen::Matrix3d quaternion_to_matrix(const geometry_msgs::msg::Quaternion & q_msg)
{
  return Eigen::Quaterniond(q_msg.w, q_msg.x, q_msg.y, q_msg.z).toRotationMatrix();
}
}  // namespace

std::pair<Eigen::Matrix4d, Eigen::Matrix4d> get_transform_matrix(
  const nav_msgs::msg::Odometry & msg)
{
  // Extract position
  double x = msg.pose.pose.position.x;
  double y = msg.pose.pose.position.y;
  double z = msg.pose.pose.position.z;

  // Rotation matrix (3x3)
  Eigen::Matrix3d R = quaternion_to_matrix(msg.pose.pose.orientation);

  // Translation vector
  Eigen::Vector3d t(x, y, z);

  // Base_link → Map (forward)
  Eigen::Matrix4d bl2map = Eigen::Matrix4d::Identity();
  bl2map.block<3, 3>(0, 0) = R;
  bl2map.block<3, 1>(0, 3) = t;

  // Map → Base_link (inverse)
  Eigen::Matrix4d map2bl = Eigen::Matrix4d::Identity();
  map2bl.block<3, 3>(0, 0) = R.transpose();
  map2bl.block<3, 1>(0, 3) = -R.transpose() * t;

  return {bl2map, map2bl};
}

std::vector<float> create_float_data(const std::vector<int64_t> & shape, float fill)
{
  size_t total_size = 1;
  for (auto dim : shape) {
    // Check for overflow before multiplication
    if (dim > 0 && total_size > std::numeric_limits<size_t>::max() / static_cast<size_t>(dim)) {
      throw std::overflow_error("Shape dimensions would cause size_t overflow");
    }
    total_size *= static_cast<size_t>(dim);
  }
  std::vector<float> data(total_size, fill);
  return data;
}

bool check_input_map(const std::unordered_map<std::string, std::vector<float>> & input_map)
{
  for (const auto & tup : input_map) {
    if (std::any_of(tup.second.begin(), tup.second.end(), [](const auto & v) {
          return !std::isfinite(v) || std::isnan(v);
        })) {
      std::cerr << "key " << tup.first << " contains invalid values\n";
      return false;
    }
  }
  return true;
}

Eigen::Matrix4d pose_to_matrix4f(const geometry_msgs::msg::Pose & pose)
{
  // Extract position
  double x = pose.position.x;
  double y = pose.position.y;
  double z = pose.position.z;

  // Rotation matrix (3x3)
  Eigen::Matrix3d R = quaternion_to_matrix(pose.orientation);

  // Translation vector
  Eigen::Vector3d t(x, y, z);

  // Create 4x4 transformation matrix
  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
  pose_matrix.block<3, 3>(0, 0) = R;
  pose_matrix.block<3, 1>(0, 3) = t;

  return pose_matrix;
}

std::pair<float, float> rotation_matrix_to_cos_sin(const Eigen::Matrix3d & rotation_matrix)
{
  // Extract yaw angle from rotation matrix and convert to cos/sin
  // Using atan2 to get the yaw angle from the rotation matrix
  const float yaw = std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
  return {std::cos(yaw), std::sin(yaw)};
}

geometry_msgs::msg::Pose shift_x(const geometry_msgs::msg::Pose & pose, const double shift_length)
{
  // Rotation matrix (3x3)
  Eigen::Matrix3d R = quaternion_to_matrix(pose.orientation);

  // Shift along the x-axis in the local frame
  Eigen::Vector3d shift_local(shift_length, 0.0, 0.0);

  // Transform shift to the global frame
  Eigen::Vector3d shift_global = R * shift_local;

  // Create new pose
  geometry_msgs::msg::Pose shifted_pose = pose;
  shifted_pose.position.x += shift_global.x();
  shifted_pose.position.y += shift_global.y();
  shifted_pose.position.z += shift_global.z();

  return shifted_pose;
}

}  // namespace autoware::diffusion_planner::utils
