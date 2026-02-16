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

#include "path_shift_to_ego.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

Trajectory shift_trajectory_to_ego(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const TrajectoryShiftParams & params, const double delta_arc_length)
{
  if (trajectory.points.size() < 2) {
    return trajectory;
  }

  // Calculate lateral offset of ego from the trajectory
  const double lateral_offset =
    autoware::motion_utils::calcLateralOffset(trajectory.points, ego_pose.position);

  // If the offset is small enough, no shift needed
  if (std::abs(lateral_offset) < params.minimum_shift_length) {
    return trajectory;
  }

  // TODO: 速度依存で決める
  // Scale shift distance based on lateral offset so that cross-lane shifts get enough room
  const double shift_distance = std::max(
    params.minimum_shift_distance,
    std::abs(lateral_offset) * params.shift_length_to_distance_ratio);

  // Find the nearest index on the trajectory for ego position
  const size_t ego_nearest_idx =
    autoware::motion_utils::findNearestIndex(trajectory.points, ego_pose.position);

  // Walk forward along the trajectory to find the skip end index
  double accumulated_length = 0.0;
  size_t skip_idx = ego_nearest_idx;
  for (size_t i = ego_nearest_idx; i + 1 < trajectory.points.size(); ++i) {
    accumulated_length += autoware_utils::calc_distance2d(
      trajectory.points.at(i).pose.position, trajectory.points.at(i + 1).pose.position);
    skip_idx = i + 1;
    if (accumulated_length >= shift_distance) {
      break;
    }
  }

  // If not enough room to shift (skip end is at trajectory end), skip
  if (skip_idx >= trajectory.points.size() - 1) {
    return trajectory;
  }

  // Build new waypoints: ego + guide point + remaining trajectory from skip_idx onward
  const double ref_velocity = trajectory.points.at(ego_nearest_idx).longitudinal_velocity_mps;
  const double guide_distance = std::max(2.0, shift_distance * 0.15);
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);

  TrajectoryPoint ego_point;
  ego_point.pose = ego_pose;
  ego_point.longitudinal_velocity_mps = ref_velocity;

  // Guide point along ego's heading to constrain the spline's initial tangent
  TrajectoryPoint guide_point;
  guide_point.pose.position.x = ego_pose.position.x + guide_distance * std::cos(ego_yaw);
  guide_point.pose.position.y = ego_pose.position.y + guide_distance * std::sin(ego_yaw);
  guide_point.pose.position.z = ego_pose.position.z;
  guide_point.pose.orientation = ego_pose.orientation;
  guide_point.longitudinal_velocity_mps = ref_velocity;

  std::vector<TrajectoryPoint> new_points;
  new_points.reserve(trajectory.points.size() - skip_idx + 2);
  new_points.push_back(ego_point);
  new_points.push_back(guide_point);
  new_points.insert(
    new_points.end(), trajectory.points.begin() + skip_idx, trajectory.points.end());

  // Build smooth trajectory using pretty_build (CubicSpline interpolation)
  const auto smooth_traj =
    autoware::experimental::trajectory::pretty_build<TrajectoryPoint>(new_points);
  if (!smooth_traj) {
    // fallback: return the raw stitched points
    Trajectory output;
    output.header = trajectory.header;
    output.points = new_points;
    return output;
  }

  // Resample at uniform intervals
  const double total_length = smooth_traj->length();
  std::vector<TrajectoryPoint> resampled_points;
  for (double s = 0.0; s < total_length; s += delta_arc_length) {
    resampled_points.push_back(smooth_traj->compute(s));
  }
  // Ensure the last point is included
  const double last_sampled =
    resampled_points.empty() ? -1.0 : (resampled_points.size() - 1) * delta_arc_length;
  if (total_length - last_sampled > 1e-6) {
    resampled_points.push_back(smooth_traj->compute(total_length));
  }

  Trajectory output;
  output.header = trajectory.header;
  output.points = resampled_points;
  return output;
}

}  // namespace autoware::minimum_rule_based_planner
