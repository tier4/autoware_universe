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
    if (accumulated_length >= params.minimum_shift_distance) {
      break;
    }
  }

  // If not enough room to shift (skip end is at trajectory end), skip
  if (skip_idx >= trajectory.points.size() - 1) {
    return trajectory;
  }

  // Build new waypoints: ego + guide point + remaining trajectory from skip_idx onward
  std::vector<TrajectoryPoint> new_points;
  new_points.reserve(trajectory.points.size() - skip_idx + 2);

  const double ref_velocity = trajectory.points.at(ego_nearest_idx).longitudinal_velocity_mps;

  // Create ego waypoint with ego's position and orientation
  TrajectoryPoint ego_point;
  ego_point.pose = ego_pose;
  ego_point.longitudinal_velocity_mps = ref_velocity;
  new_points.push_back(ego_point);

  // Insert a guide point along ego's heading direction to constrain the spline's
  // initial tangent so that the trajectory starts in the ego's yaw direction.
  constexpr double guide_distance = 2.0;  // [m]
  const double ego_yaw = tf2::getYaw(ego_pose.orientation);
  TrajectoryPoint guide_point;
  guide_point.pose.position.x = ego_pose.position.x + guide_distance * std::cos(ego_yaw);
  guide_point.pose.position.y = ego_pose.position.y + guide_distance * std::sin(ego_yaw);
  guide_point.pose.position.z = ego_pose.position.z;
  guide_point.pose.orientation = ego_pose.orientation;
  guide_point.longitudinal_velocity_mps = ref_velocity;
  new_points.push_back(guide_point);

  // Add remaining trajectory points from skip_idx onward
  for (size_t i = skip_idx; i < trajectory.points.size(); ++i) {
    new_points.push_back(trajectory.points.at(i));
  }

  (void)delta_arc_length;
  // // Build smooth trajectory using pretty_build
  // // CubicSpline for position (x,y), SphericalLinear for orientation
  // // This handles both lateral offset AND yaw deviation naturally
  // const auto smooth_traj =
  //   autoware::experimental::trajectory::pretty_build<TrajectoryPoint>(new_points);
  // if (!smooth_traj) {
  //   return trajectory;  // fallback to original trajectory
  // }

  // // Resample at uniform intervals
  // const double total_length = smooth_traj->length();
  // std::vector<double> ss;
  // for (double s = 0.0; s < total_length; s += delta_arc_length) {
  //   ss.push_back(s);
  // }
  // // Ensure we include the last point
  // if (ss.empty() || ss.back() < total_length - 1e-6) {
  //   ss.push_back(total_length);
  // }

  // const auto resampled_points = smooth_traj->compute(ss);

  // Build output trajectory
  Trajectory output;
  output.header = trajectory.header;
  // output.points = resampled_points;
  output.points = new_points;

  return output;
}

}  // namespace autoware::minimum_rule_based_planner
