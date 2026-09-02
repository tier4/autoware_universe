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

#include "trajectory_conversion.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace autoware::safety_planner
{

TrajectoryPoint to_trajectory_point(
  const RoughPlanPoint & rough_point, const double z, const double wheel_base_m)
{
  TrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.0, rough_point.t));
  point.pose.position.x = rough_point.pose.position.x();
  point.pose.position.y = rough_point.pose.position.y();
  point.pose.position.z = z;
  point.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(rough_point.pose.yaw);
  point.longitudinal_velocity_mps = static_cast<float>(rough_point.v);
  point.lateral_velocity_mps = 0.0F;
  point.acceleration_mps2 = static_cast<float>(rough_point.a);
  point.heading_rate_rps = static_cast<float>(rough_point.v * rough_point.kappa);
  point.front_wheel_angle_rad = static_cast<float>(std::atan(rough_point.kappa * wheel_base_m));
  point.rear_wheel_angle_rad = 0.0F;
  return point;
}

TrajectoryPoint to_trajectory_point(
  const OptimizedTrajectoryPoint & optimized_point, const double z, const double wheel_base_m)
{
  TrajectoryPoint point;
  point.time_from_start = rclcpp::Duration::from_seconds(std::max(0.0, optimized_point.t));
  point.pose.position.x = optimized_point.pose.position.x();
  point.pose.position.y = optimized_point.pose.position.y();
  point.pose.position.z = z;
  point.pose.orientation =
    autoware_utils_geometry::create_quaternion_from_yaw(optimized_point.pose.yaw);
  point.longitudinal_velocity_mps = static_cast<float>(optimized_point.v);
  point.lateral_velocity_mps = 0.0F;
  point.acceleration_mps2 = static_cast<float>(optimized_point.a);
  point.heading_rate_rps = static_cast<float>(optimized_point.v * optimized_point.kappa);
  point.front_wheel_angle_rad = static_cast<float>(std::atan(optimized_point.kappa * wheel_base_m));
  point.rear_wheel_angle_rad = 0.0F;
  return point;
}

Trajectory set_engage_speed(const Trajectory & trajectory, const double engage_velocity_mps)
{
  //! [m] この距離より短い軌道は「発進する周期ではない」とみなして触らない。
  //! goal 目前の減速区間と停止 plan (全点が同じ位置) を発進させないための足切り
  constexpr double MIN_ENGAGE_DIST_M = 0.5;

  Trajectory result = trajectory;
  if (result.points.empty() || !(engage_velocity_mps > 0.0)) {
    return result;
  }

  // 進行距離は軌道の点列そのものから測る。時間パラメタライズの出力は停止区間の点が
  // ほぼ同じ位置に重なるので、停止 plan はここで 0 近くになって足切りされる
  double length = 0.0;
  for (std::size_t k = 0; k + 1 < result.points.size(); ++k) {
    length += autoware_utils_geometry::calc_distance2d(
      result.points[k].pose.position, result.points[k + 1].pose.position);
  }
  if (length <= MIN_ENGAGE_DIST_M) {
    return result;
  }

  for (auto & point : result.points) {
    if (point.longitudinal_velocity_mps >= static_cast<float>(engage_velocity_mps)) {
      break;  // 巡航速度まで乗った点から先はそのまま (終端の減速・停止に触れない)
    }
    point.longitudinal_velocity_mps = static_cast<float>(engage_velocity_mps);
  }
  return result;
}

}  // namespace autoware::safety_planner
