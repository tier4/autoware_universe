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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <any>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator::plugin
{
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::MultiPoint2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

using TimeTrajectory = std::vector<double>;
using TravelDistanceTrajectory = std::vector<double>;
using PoseTrajectory = std::vector<geometry_msgs::msg::Pose>;
using FootprintTrajectory = std::vector<Polygon2d>;
using StepPolygonTrajectory = std::vector<Polygon2d>;

static constexpr double TIME_RESOLUTION = 0.1;  // 100ms intervals

class TrajectoryData
{
private:
  std::string id_;
  TimeTrajectory times_;
  TravelDistanceTrajectory distances_;
  PoseTrajectory poses_;
  FootprintTrajectory footprints_;

public:
  TrajectoryData(
    std::string id, TimeTrajectory times, TravelDistanceTrajectory distances, PoseTrajectory poses,
    FootprintTrajectory footprints)
  : id_(std::move(id)),
    times_(std::move(times)),
    distances_(std::move(distances)),
    poses_(std::move(poses)),
    footprints_(std::move(footprints))
  {
    assert(!times_.empty() && "Trajectory must not be empty");
    assert(times_.size() == distances_.size() && "Trajectory sizes mismatch (times vs distances)");
    assert(times_.size() == poses_.size() && "Trajectory sizes mismatch (times vs poses)");
    assert(
      times_.size() == footprints_.size() && "Trajectory sizes mismatch (times vs footprints)");
  }

  TrajectoryData() = delete;

  const std::string & getId() const { return id_; }
  const TimeTrajectory & getTimes() const { return times_; }
  const TravelDistanceTrajectory & getDistances() const { return distances_; }
  const PoseTrajectory & getPoses() const { return poses_; }
  const FootprintTrajectory & getFootprints() const { return footprints_; }

  size_t size() const { return times_.size(); }

  FootprintTrajectory getFootprintsInTimeRange(double start_time, double end_time) const
  {
    if (start_time > end_time || footprints_.empty()) {
      return FootprintTrajectory{};
    }

    const size_t start_index = getIndex(start_time);
    const size_t end_index = getIndex(end_time);

    if (start_index > end_index) {
      return FootprintTrajectory{};
    }

    const auto start_iter = footprints_.begin() + start_index;
    const auto end_iter = footprints_.begin() + std::min(end_index + 1, footprints_.size());

    return FootprintTrajectory(start_iter, end_iter);
  }

private:
  size_t getIndex(const double t) const
  {
    const auto it = std::lower_bound(times_.begin(), times_.end(), t);

    if (it == times_.begin()) return 0;
    if (it == times_.end()) return times_.size() - 1;

    const auto prev_it = it - 1;
    const auto closest_it = (std::abs(*it - t) < std::abs(*prev_it - t)) ? it : prev_it;
    return std::distance(times_.begin(), closest_it);
  }
};

template <class T>
PoseTrajectory compute_pose_trajectory(
  const T & traj_points, const TravelDistanceTrajectory & distance_trajectory)
{
  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());
  for (const auto & distance : distance_trajectory) {
    const auto pose = autoware::motion_utils::calcInterpolatedPose(traj_points, distance);
    pose_trajectory.push_back(pose);
  }
  return pose_trajectory;
}

class CollisionCheckFilter : public ValidatorInterface
{
public:
  CollisionCheckFilter() : ValidatorInterface("CollisionCheckFilter") {}

  tl::expected<void, std::string> is_feasible(
    const TrajectoryPoints & traj_points, const FilterContext & context) override;

  void set_parameters(rclcpp::Node & node) final;

  void update_parameters(const std::vector<rclcpp::Parameter> & parameters) final;

private:
  struct PetCollisionParams
  {
    double ego_braking_delay{0.0};
    double ego_assumed_acceleration{0.0};  // used for code test, not used in actual collision check
    double collision_time_threshold{1.0};  // time threshold for PET collision check
  } pet_collision_params_;
};

}  // namespace autoware::trajectory_validator::plugin

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__COLLISION_CHECK_FILTER_HPP_
