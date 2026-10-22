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

#include "autoware/control_validator/collision_validator/sweep_types.hpp"

#include "autoware/control_validator/collision_validator/collision_evaluator.hpp"

#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autoware::control_validator::hazard
{
EgoSweep::EgoSweep(
  TimeTrajectory times_in, PoseTrajectory poses_in, const FootprintTrajectory & footprints,
  std::vector<bool> stopped_in, const bool forward_in)
: times(std::move(times_in)),
  poses(std::move(poses_in)),
  stopped(std::move(stopped_in)),
  trajectory(times, poses, footprints),
  forward(forward_in)
{
  polygons.reserve(times.size());
  for (size_t k = 0; k < times.size(); ++k) {
    polygons.push_back(
      std::visit(
        [k](const auto & footprint) { return geometry::to_polygon2d(footprint.at(k)); },
        footprints));
  }
}

ObjectMotion::ObjectMotion(
  TrajectoryData trajectory, const bool is_static, const geometry_msgs::msg::Pose & initial_pose)
: trajectory_(std::move(trajectory)), is_static_(is_static), initial_pose_(initial_pose)
{
}

geometry_msgs::msg::Pose ObjectMotion::pose_at(const double t) const
{
  if (is_static_) {
    return initial_pose_;
  }
  const auto & times = trajectory_.getTimes();
  const auto & poses = trajectory_.getPoses();
  if (times.size() == 1) {
    return poses.front();
  }
  if (t <= times.front()) {
    const double span = times[1] - times[0];
    if (span <= 0.0) {
      return poses.front();
    }
    const double r = (t - times[0]) / span;
    auto pose = poses[0];
    pose.position.x += r * (poses[1].position.x - poses[0].position.x);
    pose.position.y += r * (poses[1].position.y - poses[0].position.y);
    pose.position.z += r * (poses[1].position.z - poses[0].position.z);
    return pose;
  }
  if (t >= times.back()) {
    return poses.back();
  }
  const auto upper = static_cast<size_t>(
    std::distance(times.begin(), std::upper_bound(times.begin(), times.end(), t)));
  const size_t lower = upper - 1;
  const double span = times[upper] - times[lower];
  const double r = span > 0.0 ? (t - times[lower]) / span : 0.0;
  const auto & p0 = poses[lower];
  const auto & p1 = poses[upper];
  geometry_msgs::msg::Pose pose;
  pose.position.x = p0.position.x + r * (p1.position.x - p0.position.x);
  pose.position.y = p0.position.y + r * (p1.position.y - p0.position.y);
  pose.position.z = p0.position.z + r * (p1.position.z - p0.position.z);
  const double yaw0 = autoware_utils_geometry::get_rpy(p0).z;
  const double yaw1 = autoware_utils_geometry::get_rpy(p1).z;
  const double diff = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw0 + r * diff);
  return pose;
}
}  // namespace autoware::control_validator::hazard
