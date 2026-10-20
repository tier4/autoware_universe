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

#ifndef HAZARD__COLLISION_TARGET_HPP_
#define HAZARD__COLLISION_TARGET_HPP_

#include "hazard/collision_sweep.hpp"

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::control_validator::hazard
{
struct EgoSweep
{
  EgoSweep(
    TimeTrajectory times_in, PoseTrajectory poses_in, const FootprintTrajectory & footprints,
    std::vector<bool> stopped_in, bool forward_in);

  TimeTrajectory times;
  PoseTrajectory poses;
  std::vector<Polygon2d> polygons;
  std::vector<bool> stopped;
  TrajectoryData trajectory;
  Box2d envelope;
  bool forward;

  const Polygon2d & now() const { return polygons.front(); }
};

class ObjectMotion
{
public:
  ObjectMotion(
    TrajectoryData trajectory, bool is_static, const geometry_msgs::msg::Pose & initial_pose);

  const TrajectoryData & trajectory() const { return trajectory_; }
  geometry_msgs::msg::Pose pose_at(double t) const;
  bool is_static() const { return is_static_; }

private:
  TrajectoryData trajectory_;
  bool is_static_;
  geometry_msgs::msg::Pose initial_pose_;
};

class BrakedSweeps
{
public:
  static constexpr size_t kNumLevels = 3;

  BrakedSweeps(
    const CollisionSweep & sweep, const autoware_planning_msgs::msg::Trajectory & predicted,
    const nav_msgs::msg::Odometry & odom, const SweepLimits & limits);

  const EgoSweep & at(size_t level) const;

private:
  const CollisionSweep & sweep_;
  const autoware_planning_msgs::msg::Trajectory & predicted_;
  const nav_msgs::msg::Odometry & odom_;
  const SweepLimits & limits_;
  mutable std::array<std::optional<EgoSweep>, kNumLevels> sweeps_;
};

class CollisionTarget
{
public:
  virtual ~CollisionTarget() = default;
  virtual void evaluate(
    const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const = 0;
};

class ObjectTarget : public CollisionTarget
{
public:
  ObjectTarget(
    SweepCandidate candidate, std::shared_ptr<const ObjectMotion> motion, double time_resolution);
  void evaluate(
    const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const override;

private:
  SweepCandidate candidate_;
  std::shared_ptr<const ObjectMotion> motion_;
  double time_resolution_;
};

class PointClusterTarget : public CollisionTarget
{
public:
  PointClusterTarget(
    std::vector<Point2d> points, std::shared_ptr<const ObjectMotion> motion,
    const builtin_interfaces::msg::Time & source_stamp);
  void evaluate(
    const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const override;

private:
  void evaluate_static(
    const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const;
  void evaluate_attached(
    const EgoSweep & ego, BrakedSweeps & braked, std::vector<SweepCandidate> & out) const;

  std::vector<Point2d> points_;
  std::shared_ptr<const ObjectMotion> motion_;
  builtin_interfaces::msg::Time source_stamp_;
};
}  // namespace autoware::control_validator::hazard

#endif  // HAZARD__COLLISION_TARGET_HPP_
