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

#include "utils.hpp"

#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::utils
{
param::FootprintMargin calc_footprint_margin(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const double scale)
{
  const auto cov_in_map = covariance.covariance;
  Eigen::Matrix2d cov_xy_map;
  cov_xy_map << cov_in_map[0 * 6 + 0], cov_in_map[0 * 6 + 1], cov_in_map[1 * 6 + 0],
    cov_in_map[1 * 6 + 1];

  const double yaw_vehicle = tf2::getYaw(covariance.pose.orientation);

  // To get a position in a transformed coordinate, rotate the inverse direction
  Eigen::Matrix2d r_map2vehicle;
  r_map2vehicle << std::cos(-yaw_vehicle), -std::sin(-yaw_vehicle), std::sin(-yaw_vehicle),
    std::cos(-yaw_vehicle);
  // Rotate covariance E((X, Y)^t*(X, Y)) = E(R*(x,y)*(x,y)^t*R^t)
  // when Rotate point (X, Y)^t= R*(x, y)^t.
  const Eigen::Matrix2d cov_xy_vehicle = r_map2vehicle * cov_xy_map * r_map2vehicle.transpose();

  // The longitudinal/lateral length is represented
  // in cov_xy_vehicle(0,0), cov_xy_vehicle(1,1) respectively.
  return param::FootprintMargin{cov_xy_vehicle(0, 0) * scale, cov_xy_vehicle(1, 1) * scale};
}

std::optional<std::vector<std::pair<LinearRing2d, Pose>>> create_vehicle_footprints(
  const geometry_msgs::msg::PoseWithCovariance & covariance, const TrajectoryPoints & trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const double footprint_margin_scale)
{
  if (trajectory.empty()) {
    return std::nullopt;
  }
  // Calculate longitudinal and lateral margin based on covariance
  const auto margin = calc_footprint_margin(covariance, footprint_margin_scale);

  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(margin.lat, margin.lon);

  std::vector<std::pair<LinearRing2d, Pose>> vehicle_footprints;
  vehicle_footprints.reserve(trajectory.size());
  std::transform(
    trajectory.begin(), trajectory.end(), std::back_inserter(vehicle_footprints),
    [&](const auto & p) -> std::pair<LinearRing2d, Pose> {
      using autoware_utils::transform_vector;
      using autoware_utils::pose2transform;
      return {transform_vector(local_vehicle_footprint, pose2transform(p.pose)), p.pose};
    });

  return vehicle_footprints;
}

EgoSides get_ego_sides_from_footprints(
  const std::vector<std::pair<LinearRing2d, Pose>> & footprints_with_pose)
{
  if (footprints_with_pose.empty()) {
    return {};
  }

  EgoSides footprints_sides;
  footprints_sides.reserve(footprints_with_pose.size());
  const auto create_footprint_side = [](const LinearRing2d & fp) {
    EgoSide side;
    side.right = {Point2d(fp.at(1).x(), fp.at(1).y()), Point2d(fp.at(3).x(), fp.at(3).y())};
    side.left = {Point2d(fp.at(6).x(), fp.at(6).y()), Point2d(fp.at(4).x(), fp.at(4).y())};
    return side;
  };

  {
    const auto & [fp, pose] = footprints_with_pose.front();
    EgoSide ego_side = create_footprint_side(fp);
    footprints_sides.push_back(ego_side);
  }

  for (size_t i = 1; i < footprints_with_pose.size(); ++i) {
    const auto & [fp, pose] = footprints_with_pose.at(i);
    const auto & [prev_fp, prev_pose] = footprints_with_pose.at(i - 1);
    EgoSide ego_side = create_footprint_side(fp);
    ego_side.dist_from_start +=
      footprints_sides.back().dist_from_start + autoware_utils::calc_distance2d(prev_pose, pose);
    ego_side.pose = pose;
    footprints_sides.push_back(ego_side);
  }

  return footprints_sides;
}

param::DepartureStatusesIdx check_departure_status(
  const SideToBoundPojections & side_to_bound_projections, const param::NodeParam & param)
{
  param::DepartureStatusesIdx stats;
  for (const auto & left : side_to_bound_projections.left) {
    const auto & [projection, departed_segment, idx_from_orig] = left;
    const auto & [p_orig, p_proj, dist] = projection;
    if (std::abs(dist) < param.stop_before_departure.th_dist_to_boundary_m.left) {
      stats.left.emplace_back(param::DepartureStatus::CRITICAL_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_before_departure.th_dist_to_boundary_m.left) {
      stats.left.emplace_back(param::DepartureStatus::APPROACHING_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_near_boundary.th_dist_to_boundary_m.left) {
      stats.left.emplace_back(param::DepartureStatus::NEAR_BOUNDARY, idx_from_orig);
    }
  }

  for (const auto & right : side_to_bound_projections.right) {
    const auto & [projection, departed_segment, idx_from_orig] = right;
    const auto & [p_orig, p_proj, dist] = projection;
    if (std::abs(dist) < param.stop_before_departure.th_dist_to_boundary_m.right) {
      stats.right.emplace_back(param::DepartureStatus::CRITICAL_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_before_departure.th_dist_to_boundary_m.right) {
      stats.right.emplace_back(param::DepartureStatus::APPROACHING_DEPARTURE, idx_from_orig);
    }

    if (std::abs(dist) < param.slow_down_near_boundary.th_dist_to_boundary_m.right) {
      stats.right.emplace_back(param::DepartureStatus::NEAR_BOUNDARY, idx_from_orig);
    }
  }

  return stats;
}

double calc_braking_distance(
  const double abs_velocity, const double max_deceleration, const double delay_time,
  const double dist_error)
{
  return (abs_velocity * abs_velocity) / (2.0 * max_deceleration) + delay_time * abs_velocity +
         dist_error;
}

std::vector<std::pair<size_t, size_t>> get_traj_indices_candidates(
  const std::vector<param::DepartureStatusIdx> & departure_stats, const EgoSides & ego_sides,
  const double ego_length)
{
  if (departure_stats.empty()) {
    return {};
  }
  std::vector<std::pair<size_t, size_t>> slow_down_candidate_idx;
  size_t p1 = 0;
  size_t p2 = 0;
  for (; p2 + 1 < departure_stats.size(); ++p2) {
    const auto [stat1, idx1] = departure_stats.at(p2);
    const auto [stat2, idx2] = departure_stats.at(p2 + 1);
    if (idx1 >= ego_sides.size() || idx2 >= ego_sides.size()) {
      break;
    }
    const auto diff =
      std::abs(ego_sides.at(idx2).dist_from_start - ego_sides.at(idx1).dist_from_start);
    // have to compare between two arrays
    const auto cond1 = diff < ego_length;
    const auto cond2 = p2 < departure_stats.size() - 1;
    if (cond1 && cond2) {
      ++p2;
      continue;
    }
    slow_down_candidate_idx.emplace_back(
      departure_stats.at(p1).second, departure_stats.at(p2).second);
    p1 = p2;
  }

  if (p1 <= p2 && p2 != 0UL) {
    slow_down_candidate_idx.emplace_back(
      departure_stats.at(p1).second, departure_stats.at(p2 - 1).second);
  }
  return slow_down_candidate_idx;
}
}  // namespace autoware::motion_velocity_planner::utils
