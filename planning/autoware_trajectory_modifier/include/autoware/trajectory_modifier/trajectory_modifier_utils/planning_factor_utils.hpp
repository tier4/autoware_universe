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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__PLANNING_FACTOR_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__PLANNING_FACTOR_UTILS_HPP_

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <vector>

namespace autoware::trajectory_modifier::utils
{

/**
 * @brief Thresholds for stop / slowdown detection on a pose-sequence trajectory.
 *
 * Ported from `autoware_diffusion_planner` so any learning-based planner that
 * outputs a sequence of poses (diffusion, tensorrt e2e, etc.) can be labeled
 * through trajectory_modifier instead of inside the model node.
 */
struct PlanningFactorDetectionConfig
{
  double stop_velocity_threshold{0.1};         // [m/s]
  double stop_keep_duration_threshold{1.0};    // [s]
  double slowdown_accel_threshold{-0.3};       // [m/s^2]
};

struct DetectedStopFactor
{
  geometry_msgs::msg::Pose ego_pose;
  geometry_msgs::msg::Pose stop_pose;
};

struct DetectedSlowdownFactor
{
  geometry_msgs::msg::Pose ego_pose;
  geometry_msgs::msg::Pose start_pose;
  geometry_msgs::msg::Pose end_pose;
  double start_velocity;
  double end_velocity;
};

struct PlanningFactorDetectionResult
{
  std::optional<DetectedStopFactor> stop;
  std::optional<DetectedSlowdownFactor> slowdown;
};

/**
 * @brief Inspect trajectory kinematics and report the first valid stop and slowdown.
 *
 * Stop: first point at or below `stop_velocity_threshold` that stays at/below
 * that speed for at least `stop_keep_duration_threshold` (no resume in between).
 * Slowdown: first contiguous run of points whose `acceleration_mps2` is below
 * `slowdown_accel_threshold`; if the run never ends, the last point is the end.
 */
PlanningFactorDetectionResult detect_planning_factors(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const PlanningFactorDetectionConfig & config);

}  // namespace autoware::trajectory_modifier::utils

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_UTILS__PLANNING_FACTOR_UTILS_HPP_
