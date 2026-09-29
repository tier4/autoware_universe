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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__FOOTPRINTS_GENERATOR_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__FOOTPRINTS_GENERATOR_HPP_

#include <autoware/boundary_departure_checker/detail/side_struct.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::boundary_departure_checker::footprints
{
using Footprint = autoware_utils_geometry::LinearRing2d;
using Footprints = std::vector<Footprint>;
using autoware_planning_msgs::msg::TrajectoryPoint;

/**
 * @brief Margin for the footprint.
 */
struct FootprintMargin
{
  double lat_m{1000.0};  ///< lateral margin [m]
  double lon_m{1000.0};  ///< longitudinal margin [m]
};

/**
 * @brief Extent of the ego pose alignment along a trajectory.
 */
struct FootprintAlignment
{
  double align_dist_m{0.0};  ///< arc length over which the alignment applies [m]
  size_t align_count{0};     ///< number of leading points that the alignment covers
};

/**
 * @brief Give the alignment extent for the leading trajectory points within an arc length.
 * @param[in] trajectory_points points along the trajectory
 * @param[in] dist_m arc length from the trajectory start [m]
 * @return alignment arc length and the number of leading points it covers
 */
FootprintAlignment count_points_within_distance(
  const std::vector<TrajectoryPoint> & trajectory_points, const double dist_m);

/**
 * @brief Correct the trajectory poses near the ego vehicle to start at the measured ego pose.
 * @param[in] trajectory_points points along the trajectory
 * @param[in] ego_pose measured ego pose
 * @param[in] expected_alignment extent of the alignment from the trajectory start
 * @return trajectory points with the poses near the ego vehicle corrected
 */
std::vector<TrajectoryPoint> align_to_ego_pose(
  const std::vector<TrajectoryPoint> & trajectory_points, const geometry_msgs::msg::Pose & ego_pose,
  const FootprintAlignment expected_alignment);

/**
 * @brief Generate vehicle footprints along a trajectory.
 * @param[in] trajectory_points points along the trajectory
 * @param[in] vehicle_info information about the vehicle
 * @param[in] covariance pose covariance to consider for margin
 * @return list of generated footprints
 */
Footprints generate(
  const std::vector<TrajectoryPoint> & trajectory_points,
  const vehicle_info_utils::VehicleInfo & vehicle_info,
  const geometry_msgs::msg::PoseWithCovariance & covariance);

/**
 * @brief Extract left and right side segments from footprints.
 * @param[in] footprints list of vehicle footprints
 * @return list of side segments for each footprint
 */
std::vector<Side<autoware_utils_geometry::Segment2d>> get_sides_from_footprints(
  const Footprints & footprints);

/**
 * @brief Calculate margin based on pose covariance.
 * @param[in] covariance pose covariance
 * @param[in] scale scale factor for the covariance
 * @return calculated footprint margin
 */
FootprintMargin calc_margin_from_covariance(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale = 0.0);

}  // namespace autoware::boundary_departure_checker::footprints

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__DETAIL__FOOTPRINTS_GENERATOR_HPP_
