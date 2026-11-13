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

#ifndef AUTOWARE__MPPI_OPTIMIZER__TRACKED_OBJECTS_OBSTACLES_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__TRACKED_OBJECTS_OBSTACLES_HPP_

#include "autoware/mppi_optimizer/predicted_objects_obstacles.hpp"

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{

using autoware_perception_msgs::msg::TrackedObjects;

inline const autoware_perception_msgs::msg::PredictedObject * findObjectPrediction(
  const autoware_perception_msgs::msg::TrackedObject & object, const PredictedObjects & predictions)
{
  const autoware_perception_msgs::msg::PredictedObject * match = nullptr;
  for (const auto & prediction : predictions.objects) {
    if (prediction.object_id != object.object_id) continue;
    if (match) throw std::invalid_argument("MPPI received duplicate predicted object UUIDs");
    match = &prediction;
  }
  return match;
}

/** Associate predictions by UUID, retaining the tracked-object selection policy. */
inline PredictedObjects predictionsForTrackedObjects(
  const TrackedObjects & tracked, const PredictedObjects & predictions)
{
  if (!predictions.objects.empty() && predictions.header.frame_id != tracked.header.frame_id) {
    throw std::invalid_argument("MPPI tracked and predicted objects use different frames");
  }
  PredictedObjects selected;
  selected.header = tracked.header;
  selected.objects.reserve(tracked.objects.size());
  for (const auto & object : tracked.objects) {
    const auto * match = findObjectPrediction(object, predictions);
    if (match) {
      selected.objects.push_back(*match);
    } else {
      autoware_perception_msgs::msg::PredictedObject fallback;
      fallback.object_id = object.object_id;
      fallback.shape = object.shape;
      fallback.kinematics.initial_pose_with_covariance = object.kinematics.pose_with_covariance;
      fallback.kinematics.initial_twist_with_covariance = object.kinematics.twist_with_covariance;
      selected.objects.push_back(std::move(fallback));
    }
  }
  return selected;
}

/** Conservative broad phase: retain a hypothesis whose swept AABB reaches the reference.
 * The route-aware selector still decides whether the object is owned by this module.
 */
inline bool predictedPathsReachTrajectory(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double margin)
{
  float length, width;
  obstacleFootprintFromShape(object.shape, length, width);
  const double padding = margin + 0.5 * std::hypot(length, width);
  for (const auto * path : selectPredictedPaths(object)) {
    if (path->path.empty()) {
      throw std::invalid_argument("MPPI highest-confidence predicted path is empty");
    }
    for (std::size_t i = 0; i < path->path.size(); ++i) {
      validateObstaclePose(path->path[i]);
      const auto & a = path->path[i == 0U ? 0U : i - 1U].position;
      const auto & b = path->path[i].position;
      for (std::size_t j = 0; j < trajectory.points.size(); ++j) {
        const auto & c = trajectory.points[j == 0U ? 0U : j - 1U].pose.position;
        const auto & d = trajectory.points[j].pose.position;
        if (
          std::max(a.x, b.x) + padding >= std::min(c.x, d.x) &&
          std::min(a.x, b.x) - padding <= std::max(c.x, d.x) &&
          std::max(a.y, b.y) + padding >= std::min(c.y, d.y) &&
          std::min(a.y, b.y) - padding <= std::max(c.y, d.y)) {
          return true;
        }
      }
    }
  }
  return false;
}

inline void includePredictedPathTargets(
  TrackedObjects & in_range, const TrackedObjects & all_tracked,
  const PredictedObjects & predictions, const autoware_planning_msgs::msg::Trajectory & trajectory,
  const double margin, std::vector<std::array<std::uint8_t, 16>> * reaching_ids = nullptr)
{
  if (!predictions.objects.empty() && predictions.header.frame_id != all_tracked.header.frame_id) {
    throw std::invalid_argument("MPPI tracked and predicted objects use different frames");
  }
  for (const auto & object : all_tracked.objects) {
    const bool already_selected = std::any_of(
      in_range.objects.begin(), in_range.objects.end(),
      [&object](const auto & candidate) { return candidate.object_id == object.object_id; });
    const auto * prediction = findObjectPrediction(object, predictions);
    if (prediction && predictedPathsReachTrajectory(*prediction, trajectory, margin)) {
      if (reaching_ids) reaching_ids->push_back(object.object_id.uuid);
      if (!already_selected) in_range.objects.push_back(object);
    }
  }
}

/**
 * @brief Extrapolate perception tracked objects with constant longitudinal velocity.
 *
 * Twist is treated as object-frame longitudinal speed (linear.x), matching common Autoware usage.
 * Buffer slot t is evaluated at time_offset + (t + 1) * dt, matching ego state x[t + 1].
 * Overflow is rejected: omitted geometry must never escape the final validator.
 */
inline void buildObstacleTrajectoryBuffersFromTrackedObjects(
  const TrackedObjects & objects, const float dt, const int num_timesteps, std::vector<float> & x,
  std::vector<float> & y, std::vector<float> & yaw, std::vector<float> & half_length,
  std::vector<float> & half_width, const float time_offset = 0.0F, const double origin_x = 0.0,
  const double origin_y = 0.0)
{
  if (objects.objects.size() > kMaxMppiObstacles) {
    throw std::length_error("Tracked objects exceed MPPI obstacle capacity");
  }
  const size_t obstacle_count = objects.objects.size();
  const int nt = std::max(1, num_timesteps);

  x.assign(obstacle_count * static_cast<size_t>(nt), -1.0E4F);
  y.assign(obstacle_count * static_cast<size_t>(nt), -1.0E4F);
  yaw.assign(obstacle_count * static_cast<size_t>(nt), 0.0F);
  half_length.assign(obstacle_count, 0.0F);
  half_width.assign(obstacle_count, 0.0F);

  for (size_t obstacle_idx = 0; obstacle_idx < obstacle_count; ++obstacle_idx) {
    const auto & object = objects.objects[obstacle_idx];
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const float x0 = static_cast<float>(pose.position.x - origin_x);
    const float y0 = static_cast<float>(pose.position.y - origin_y);
    const float object_yaw = static_cast<float>(tf2::getYaw(pose.orientation));
    const float speed = static_cast<float>(object.kinematics.twist_with_covariance.twist.linear.x);

    float length = kDefaultObstacleLength;
    float width = kDefaultObstacleWidth;
    obstacleFootprintFromShape(object.shape, length, width);
    half_length[obstacle_idx] = length * 0.5F;
    half_width[obstacle_idx] = width * 0.5F;

    const float vx = speed * std::cos(object_yaw);
    const float vy = speed * std::sin(object_yaw);

    for (int timestep = 0; timestep < nt; ++timestep) {
      const size_t buffer_idx =
        obstacle_idx * static_cast<size_t>(nt) + static_cast<size_t>(timestep);
      const float relative_time = time_offset + static_cast<float>(timestep + 1) * dt;
      x[buffer_idx] = x0 + vx * relative_time;
      y[buffer_idx] = y0 + vy * relative_time;
      yaw[buffer_idx] = object_yaw;
    }
  }
}

inline int trackedObjectObstacleCount(const TrackedObjects & objects)
{
  if (objects.objects.size() > kMaxMppiObstacles) {
    throw std::length_error("Tracked objects exceed MPPI obstacle capacity");
  }
  return static_cast<int>(objects.objects.size());
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__TRACKED_OBJECTS_OBSTACLES_HPP_
