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

#include "autoware/boundary_departure_checker/detail/footprints_generator.hpp"

#include <angles/angles.h>

#include <vector>

namespace autoware::boundary_departure_checker::footprints
{
using autoware_utils_geometry::pose2transform;
using autoware_utils_geometry::transform_vector;

FootprintMargin calc_margin_from_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto cov_in_map = covariance.covariance;
  Eigen::Matrix2d cov_xy_map;
  cov_xy_map << cov_in_map[0 * 6 + 0], cov_in_map[0 * 6 + 1], cov_in_map[1 * 6 + 0],
    cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // rotate inverse to transform from map frame to vehicle frame
  Eigen::Matrix2d r_map2vehicle;
  r_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);

  // transform covariance matrix to vehicle frame
  const Eigen::Matrix2d cov_xy_vehicle = r_map2vehicle * cov_xy_map * r_map2vehicle.transpose();

  return FootprintMargin{cov_xy_vehicle(0, 0) * scale, cov_xy_vehicle(1, 1) * scale};
}

FootprintAlignment count_points_within_distance(
  const std::vector<TrajectoryPoint> & trajectory_points, const double dist_m)
{
  if (dist_m <= 0.0 || trajectory_points.empty()) {
    return {dist_m, 0UL};
  }

  double accumulated_dist = 0.0;
  for (size_t i = 1; i < trajectory_points.size(); ++i) {
    accumulated_dist +=
      autoware_utils_geometry::calc_distance2d(trajectory_points[i - 1], trajectory_points[i]);

    if (accumulated_dist >= dist_m) return {dist_m, i};
  }

  return {dist_m, trajectory_points.size()};
}

std::vector<TrajectoryPoint> align_to_ego_pose(
  const std::vector<TrajectoryPoint> & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const FootprintAlignment expected_alignment)
{
  auto aligned_points = trajectory_points;
  if (
    aligned_points.empty() || expected_alignment.align_dist_m <= 0.0 ||
    expected_alignment.align_count == 0) {
    return aligned_points;
  }

  const auto & start_pose = trajectory_points.front().pose;
  const auto start_yaw = tf2::getYaw(start_pose.orientation);
  const auto yaw_correction_rad =
    angles::shortest_angular_distance(start_yaw, tf2::getYaw(ego_pose.orientation));
  const auto x_correction_m = ego_pose.position.x - start_pose.position.x;
  const auto y_correction_m = ego_pose.position.y - start_pose.position.y;

  auto arc_length_m = 0.0;
  for (size_t i = 0; i < expected_alignment.align_count; ++i) {
    if (i > 0) {
      arc_length_m +=
        autoware_utils_geometry::calc_distance2d(trajectory_points[i - 1], trajectory_points[i]);
    }

    const auto weight = 1.0 - arc_length_m / expected_alignment.align_dist_m;
    auto & pose = aligned_points[i].pose;
    pose.position.x += weight * x_correction_m;
    pose.position.y += weight * y_correction_m;
    pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
      tf2::getYaw(trajectory_points[i].pose.orientation) + weight * yaw_correction_rad);
  }

  return aligned_points;
}

// clang-format off
/**
 * footprints::generate:
 *
 *          [V]--[V]        [V]--[V]
 *           | Ego |         | Ego |
 *     >>>  [V]--[V]  --->  [V]--[V]  --->  ...
 *        (Pose 0)        (Pose 1)        (Trajectory)
 *
 * Generates full vehicle footprints at each pose along the trajectory,
 * accounting for vehicle dimensions (overhangs, wheelbase).
 */
// clang-format on
Footprints generate(
  const std::vector<TrajectoryPoint> & trajectory_points,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::PoseWithCovariance & covariance)
{
  const auto margin = calc_margin_from_covariance(covariance, 0.0);
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat_m, margin.lon_m);

  Footprints footprints{};
  footprints.reserve(trajectory_points.size());
  for (const auto & pt : trajectory_points) {
    footprints.push_back(transform_vector(local_vehicle_footprint, pose2transform(pt.pose)));
  }
  return footprints;
}

// clang-format off
/**
 * get_sides_from_footprints:
 *
 *          V (Front Left)  Left Side Segment   V (Rear Left)
 *           +---------------------------------+
 *           |                                 |
 *   Forward |               Ego               |
 *     >>>   |             Vehicle             |
 *           |                                 |
 *           +---------------------------------+
 *          V (Front Right) Right Side Segment  V (Rear Right)
 *
 * Extracts the longitudinal side segments (Left/Right) from a
 * 4-point (or more) vehicle footprint.
 */
// clang-format on
std::vector<Side<autoware_utils_geometry::Segment2d>> get_sides_from_footprints(
  const Footprints & footprints)
{
  std::vector<Side<autoware_utils_geometry::Segment2d>> footprints_sides;
  footprints_sides.reserve(footprints.size());
  for (const auto & footprint : footprints) {
    const auto & right_front = footprint[vehicle_info_utils::VehicleInfo::FrontRightIndex];
    const auto & right_back = footprint[vehicle_info_utils::VehicleInfo::RearRightIndex];

    const auto & left_front = footprint[vehicle_info_utils::VehicleInfo::FrontLeftIndex];
    const auto & left_back = footprint[vehicle_info_utils::VehicleInfo::RearLeftIndex];

    Side<autoware_utils_geometry::Segment2d> side;
    side.right = {
      autoware_utils_geometry::Point2d(right_front.x(), right_front.y()),
      autoware_utils_geometry::Point2d(right_back.x(), right_back.y())};
    side.left = {
      autoware_utils_geometry::Point2d(left_front.x(), left_front.y()),
      autoware_utils_geometry::Point2d(left_back.x(), left_back.y())};

    footprints_sides.push_back(side);
  }
  return footprints_sides;
}

}  // namespace autoware::boundary_departure_checker::footprints
