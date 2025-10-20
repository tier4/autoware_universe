// Copyright 2023 TIER IV, Inc.
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

#include "autoware/path_smoother/utils/trajectory_utils.hpp"

#include "autoware/motion_utils/resample/resample.hpp"
#include "autoware/motion_utils/trajectory/conversion.hpp"
#include "autoware/path_smoother/utils/geometry_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <stack>
#include <vector>

namespace autoware::path_smoother
{
namespace trajectory_utils
{
Path create_path(Path path_msg, const std::vector<TrajectoryPoint> & traj_points)
{
  path_msg.points.clear();
  PathPoint pp;
  for (const auto & p : traj_points) {
    pp.pose = p.pose;
    pp.longitudinal_velocity_mps = p.longitudinal_velocity_mps;
    pp.lateral_velocity_mps = p.lateral_velocity_mps;
    pp.heading_rate_rps = p.heading_rate_rps;
    pp.is_final = true;
    path_msg.points.push_back(pp);
  }
  return path_msg;
}

std::vector<TrajectoryPoint> resampleTrajectoryPoints(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  constexpr bool enable_resampling_stop_point = true;

  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(
    traj, interval, false, true, true, enable_resampling_stop_point);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

// NOTE: stop point will not be resampled
std::vector<TrajectoryPoint> resampleTrajectoryPointsWithoutStopPoint(
  const std::vector<TrajectoryPoint> & traj_points, const double interval)
{
  constexpr bool enable_resampling_stop_point = false;

  const auto traj = autoware::motion_utils::convertToTrajectory(traj_points);
  const auto resampled_traj = autoware::motion_utils::resampleTrajectory(
    traj, interval, false, true, true, enable_resampling_stop_point);
  return autoware::motion_utils::convertToTrajectoryPointArray(resampled_traj);
}

void insertStopPoint(
  std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & input_stop_pose,
  const size_t stop_seg_idx)
{
  const double offset_to_segment = autoware::motion_utils::calcLongitudinalOffsetToSegment(
    traj_points, stop_seg_idx, input_stop_pose.position);

  const auto traj_spline = autoware::interpolation::SplineInterpolationPoints2d(traj_points);
  const auto stop_pose = traj_spline.getSplineInterpolatedPose(stop_seg_idx, offset_to_segment);

  if (geometry_utils::isSamePoint(traj_points.at(stop_seg_idx), stop_pose)) {
    traj_points.at(stop_seg_idx).longitudinal_velocity_mps = 0.0;
    return;
  }
  if (geometry_utils::isSamePoint(traj_points.at(stop_seg_idx + 1), stop_pose)) {
    traj_points.at(stop_seg_idx + 1).longitudinal_velocity_mps = 0.0;
    return;
  }

  TrajectoryPoint additional_traj_point;
  additional_traj_point.pose = stop_pose;
  additional_traj_point.longitudinal_velocity_mps = 0.0;

  traj_points.insert(traj_points.begin() + stop_seg_idx + 1, additional_traj_point);
}

void apply_input_velocity(
  std::vector<TrajectoryPoint> & output_trajectory,
  const std::vector<TrajectoryPoint> & input_trajectory, const geometry_msgs::msg::Pose & ego_pose,
  const EgoNearestParam & ego_nearest_param)
{
  // crop forward for faster calculation
  const double output_traj_length = autoware::motion_utils::calcArcLength(output_trajectory);
  constexpr double margin_traj_length = 10.0;
  const auto forward_cropped_input_traj_points = [&]() {
    const size_t ego_seg_idx =
      trajectory_utils::findEgoSegmentIndex(input_trajectory, ego_pose, ego_nearest_param);
    return autoware::motion_utils::cropForwardPoints(
      input_trajectory, ego_pose.position, ego_seg_idx, output_traj_length + margin_traj_length);
  }();
  // update velocity
  int64_t input_traj_start_idx = 0;
  for (auto & output_point : output_trajectory) {
    // crop backward for efficient calculation
    const auto cropped_input_traj_points = std::vector<TrajectoryPoint>{
      forward_cropped_input_traj_points.begin() + input_traj_start_idx,
      forward_cropped_input_traj_points.end()};

    const size_t nearest_seg_idx = trajectory_utils::findEgoSegmentIndex(
      cropped_input_traj_points, output_point.pose, ego_nearest_param);
    input_traj_start_idx += static_cast<int64_t>(nearest_seg_idx);

    // calculate velocity with zero order hold
    const double velocity1 = cropped_input_traj_points[nearest_seg_idx].longitudinal_velocity_mps;
    const double velocity2 =
      nearest_seg_idx + 1 >= cropped_input_traj_points.size()
        ? velocity1
        : cropped_input_traj_points[nearest_seg_idx + 1].longitudinal_velocity_mps;
    output_point.longitudinal_velocity_mps = static_cast<float>(std::min(velocity1, velocity2));
  }

  // insert stop point explicitly
  const auto stop_idx =
    autoware::motion_utils::searchZeroVelocityIndex(forward_cropped_input_traj_points);
  if (stop_idx) {
    const auto & input_stop_pose = forward_cropped_input_traj_points.at(stop_idx.value()).pose;
    // NOTE: autoware::motion_utils::findNearestSegmentIndex is used instead of
    // trajectory_utils::findEgoSegmentIndex
    //       for the case where input_traj_points is much longer than output_traj_points, and the
    //       former has a stop point but the latter will not have.
    const auto stop_seg_idx = autoware::motion_utils::findNearestSegmentIndex(
      output_trajectory, input_stop_pose, ego_nearest_param.dist_threshold,
      ego_nearest_param.yaw_threshold);

    // calculate and insert stop pose on output trajectory
    const bool is_stop_point_inside_trajectory = [&]() {
      if (!stop_seg_idx) {
        return false;
      }
      if (output_trajectory.size() > 1 && *stop_seg_idx == output_trajectory.size() - 2) {
        const double signed_projected_length_to_segment =
          autoware::motion_utils::calcLongitudinalOffsetToSegment(
            output_trajectory, *stop_seg_idx, input_stop_pose.position);
        const double segment_length = autoware::motion_utils::calcSignedArcLength(
          output_trajectory, *stop_seg_idx, *stop_seg_idx + 1);
        if (segment_length < signed_projected_length_to_segment) {
          // NOTE: input_stop_pose is outside output_traj_points.
          return false;
        }
      }
      return true;
    }();
    if (is_stop_point_inside_trajectory) {
      trajectory_utils::insertStopPoint(output_trajectory, input_stop_pose, *stop_seg_idx);
    }
  }
}
}  // namespace trajectory_utils
}  // namespace autoware::path_smoother
