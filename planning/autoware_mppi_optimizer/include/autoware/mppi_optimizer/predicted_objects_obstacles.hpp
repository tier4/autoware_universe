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

#ifndef AUTOWARE__MPPI_OPTIMIZER__PREDICTED_OBJECTS_OBSTACLES_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__PREDICTED_OBJECTS_OBSTACLES_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace autoware::mppi_optimizer
{

using autoware_perception_msgs::msg::PredictedObjects;

constexpr size_t kMaxMppiObstacles = 64U;
constexpr float kDefaultObstacleLength = 0.55F * 1.5F;
constexpr float kDefaultObstacleWidth = 0.28F * 1.5F;

inline float durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<float>(duration.sec) + static_cast<float>(duration.nanosec) * 1.0e-9F;
}

inline std::vector<const autoware_perception_msgs::msg::PredictedPath *> selectPredictedPaths(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  std::vector<const autoware_perception_msgs::msg::PredictedPath *> selected;
  float highest_confidence = -std::numeric_limits<float>::infinity();
  for (const auto & path : object.kinematics.predicted_paths) {
    if (!std::isfinite(path.confidence) || path.confidence < 0.0F || path.confidence > 1.0F) {
      throw std::invalid_argument("MPPI predicted-path confidence must be in [0, 1]");
    }
    if (path.confidence > highest_confidence) {
      selected.clear();
      highest_confidence = path.confidence;
    }
    if (path.confidence == highest_confidence) selected.push_back(&path);
  }
  return selected;
}

inline void obstacleFootprintFromShape(
  const autoware_perception_msgs::msg::Shape & shape, float & length, float & width)
{
  if (!std::isfinite(shape.dimensions.x) || !std::isfinite(shape.dimensions.y)) {
    throw std::invalid_argument("MPPI object dimensions must be finite");
  }
  length =
    shape.dimensions.x > 1.0e-3 ? static_cast<float>(shape.dimensions.x) : kDefaultObstacleLength;
  width =
    shape.dimensions.y > 1.0e-3 ? static_cast<float>(shape.dimensions.y) : kDefaultObstacleWidth;
}

/** Count hypotheses before allocation; capacity applies to paths, not only physical objects. */
inline int predictedObjectObstacleCount(const PredictedObjects & objects)
{
  std::size_t count = 0U;
  for (const auto & object : objects.objects) {
    count += std::max(std::size_t{1}, selectPredictedPaths(object).size());
    if (count > kMaxMppiObstacles) {
      throw std::length_error("Predicted object hypotheses exceed MPPI obstacle capacity");
    }
  }
  return static_cast<int>(count);
}

inline void validateObstaclePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & q = pose.orientation;
  const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (
    !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(norm) ||
    norm < 1.0e-12) {
    throw std::invalid_argument("MPPI predicted object pose is invalid");
  }
}

/**
 * @brief Sample every highest-confidence hypothesis into obstacle-major trajectory buffers.
 *
 * Slot i represents (i + 1) * dt, matching the ego post-step state. Path poses start at t=0;
 * interpolate translation and shortest-arc yaw, and hold the endpoint beyond the supplied path.
 * Ties consume separate slots and are never truncated. Objects without any paths use the
 * existing constant-longitudinal-velocity fallback. Message-age policy remains unchanged.
 */
inline void buildObstacleTrajectoryBuffersFromPredictedObjects(
  const PredictedObjects & objects, const float dt, const int num_timesteps, std::vector<float> & x,
  std::vector<float> & y, std::vector<float> & yaw, std::vector<float> & half_length,
  std::vector<float> & half_width, const double origin_x = 0.0, const double origin_y = 0.0)
{
  if (!std::isfinite(dt) || dt <= 0.0F || num_timesteps <= 0) {
    throw std::invalid_argument("MPPI obstacle sampling requires positive dt and horizon");
  }
  const size_t obstacle_count = static_cast<size_t>(predictedObjectObstacleCount(objects));
  const int nt = num_timesteps;

  x.assign(obstacle_count * static_cast<size_t>(nt), -1.0E4F);
  y.assign(obstacle_count * static_cast<size_t>(nt), -1.0E4F);
  yaw.assign(obstacle_count * static_cast<size_t>(nt), 0.0F);
  half_length.assign(obstacle_count, 0.0F);
  half_width.assign(obstacle_count, 0.0F);

  size_t obstacle_idx = 0U;
  for (const auto & object : objects.objects) {
    float length = kDefaultObstacleLength;
    float width = kDefaultObstacleWidth;
    obstacleFootprintFromShape(object.shape, length, width);
    auto selected = selectPredictedPaths(object);
    if (selected.empty()) selected.push_back(nullptr);
    for (const auto * path : selected) {
      half_length[obstacle_idx] = length * 0.5F;
      half_width[obstacle_idx] = width * 0.5F;
      const auto & initial = object.kinematics.initial_pose_with_covariance.pose;
      const double speed = object.kinematics.initial_twist_with_covariance.twist.linear.x;
      double path_dt = 0.0;
      if (path) {
        path_dt = durationSeconds(path->time_step);
        if (
          path->path.empty() || !std::isfinite(path_dt) || path_dt <= 0.0 ||
          path->time_step.nanosec >= 1000000000U) {
          throw std::invalid_argument("MPPI highest-confidence predicted path is empty or invalid");
        }
        for (const auto & pose : path->path) validateObstaclePose(pose);
      } else {
        validateObstaclePose(initial);
        if (!std::isfinite(speed)) throw std::invalid_argument("MPPI object velocity is invalid");
      }
      for (int timestep = 0; timestep < nt; ++timestep) {
        const size_t index = obstacle_idx * static_cast<size_t>(nt) + timestep;
        const double time = static_cast<double>(timestep + 1) * dt;
        if (path) {
          const double sample =
            std::min(time / path_dt, static_cast<double>(path->path.size() - 1));
          const size_t lower = static_cast<size_t>(sample);
          const size_t upper = std::min(lower + 1U, path->path.size() - 1U);
          const double ratio = sample - static_cast<double>(lower);
          const auto & a = path->path[lower];
          const auto & b = path->path[upper];
          x[index] =
            static_cast<float>(a.position.x - origin_x + ratio * (b.position.x - a.position.x));
          y[index] =
            static_cast<float>(a.position.y - origin_y + ratio * (b.position.y - a.position.y));
          const double yaw_a = tf2::getYaw(a.orientation);
          const double delta = std::remainder(tf2::getYaw(b.orientation) - yaw_a, 2.0 * M_PI);
          yaw[index] = static_cast<float>(yaw_a + ratio * delta);
        } else {
          const double angle = tf2::getYaw(initial.orientation);
          x[index] =
            static_cast<float>(initial.position.x - origin_x + speed * std::cos(angle) * time);
          y[index] =
            static_cast<float>(initial.position.y - origin_y + speed * std::sin(angle) * time);
          yaw[index] = static_cast<float>(angle);
        }
      }
      ++obstacle_idx;
    }
  }
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__PREDICTED_OBJECTS_OBSTACLES_HPP_
