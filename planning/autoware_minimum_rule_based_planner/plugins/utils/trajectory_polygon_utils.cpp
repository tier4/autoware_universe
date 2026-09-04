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

#include "trajectory_polygon_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin::trajectory_polygon_utils
{
namespace bg = boost::geometry;
using autoware_utils_geometry::Point2d;

namespace
{
Point2d msg_to_2d(const geometry_msgs::msg::Point & point)
{
  return Point2d{point.x, point.y};
}

TrajectoryPoint extend_trajectory_point(
  const double extend_distance, const TrajectoryPoint & goal_point, const bool is_driving_forward)
{
  TrajectoryPoint extended_trajectory_point;
  extended_trajectory_point.pose = autoware_utils_geometry::calc_offset_pose(
    goal_point.pose, extend_distance * (is_driving_forward ? 1.0 : -1.0), 0.0, 0.0);
  extended_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extended_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extended_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extended_trajectory_point;
}

std::vector<TrajectoryPoint> get_extended_trajectory_points(
  const std::vector<TrajectoryPoint> & input_points, const double extend_distance,
  const double step_length)
{
  auto output_points = input_points;
  const auto is_driving_forward_opt =
    autoware::motion_utils::isDrivingForwardWithTwist(input_points);
  const bool is_driving_forward = is_driving_forward_opt ? *is_driving_forward_opt : true;

  constexpr double min_step_length = 0.1;
  if (extend_distance < min_step_length) {
    return output_points;
  }

  const auto goal_point = input_points.back();
  for (double extend_sum = step_length; extend_sum < extend_distance - step_length;
       extend_sum += step_length) {
    output_points.push_back(extend_trajectory_point(extend_sum, goal_point, is_driving_forward));
  }
  output_points.push_back(extend_trajectory_point(extend_distance, goal_point, is_driving_forward));

  return output_points;
}

std::vector<geometry_msgs::msg::Pose> calculate_error_poses(
  const std::vector<TrajectoryPoint> & traj_points,
  const geometry_msgs::msg::Pose & current_ego_pose, const double time_to_convergence)
{
  std::vector<geometry_msgs::msg::Pose> error_poses;
  error_poses.reserve(traj_points.size());

  const size_t nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    autoware_utils_geometry::inverse_transform_pose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed{0.0};

  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (time_elapsed >= time_to_convergence) {
      break;
    }

    const double rem_ratio = (time_to_convergence - time_elapsed) / time_to_convergence;
    geometry_msgs::msg::Pose indexed_pose_err;
    indexed_pose_err.set__orientation(
      autoware_utils_geometry::create_quaternion_from_yaw(current_ego_yaw_error * rem_ratio));
    indexed_pose_err.set__position(
      autoware_utils_geometry::create_point(0.0, current_ego_lat_error * rem_ratio, 0.0));
    error_poses.push_back(
      autoware_utils_geometry::transform_pose(indexed_pose_err, traj_points.at(i).pose));

    if (traj_points.at(i).longitudinal_velocity_mps != 0.0 && i < traj_points.size() - 1) {
      time_elapsed += autoware_utils_geometry::calc_distance2d(
                        traj_points.at(i).pose.position, traj_points.at(i + 1).pose.position) /
                      std::abs(traj_points.at(i).longitudinal_velocity_mps);
    } else {
      time_elapsed = std::numeric_limits<double>::max();
    }
  }
  return error_poses;
}

Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose, const VehicleInfo & vehicle_info, const double left_margin,
  const double right_margin)
{
  using autoware_utils_geometry::calc_offset_pose;
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  const auto point0 =
    calc_offset_pose(pose, vehicle_info.max_longitudinal_offset_m, half_width + left_margin, 0.0)
      .position;
  const auto point1 =
    calc_offset_pose(pose, vehicle_info.max_longitudinal_offset_m, -half_width - right_margin, 0.0)
      .position;
  const auto point2 =
    calc_offset_pose(pose, -vehicle_info.rear_overhang_m, -half_width - right_margin, 0.0).position;
  const auto point3 =
    calc_offset_pose(pose, -vehicle_info.rear_overhang_m, half_width + left_margin, 0.0).position;

  Polygon2d polygon;
  bg::append(polygon, msg_to_2d(point0));
  bg::append(polygon, msg_to_2d(point1));
  bg::append(polygon, msg_to_2d(point2));
  bg::append(polygon, msg_to_2d(point3));
  bg::append(polygon, msg_to_2d(point0));

  bg::correct(polygon);
  return polygon;
}

// 前外輪のオフトラッキング量(前外輪と後外輪の旋回半径の差)。
// 左旋回で負、右旋回で正の符号を持つ(= 外側の符号)
std::vector<double> calc_front_outer_wheel_off_tracking(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info)
{
  auto curvature_vec = autoware::motion_utils::calcCurvature(traj_points);
  const auto is_driving_forward_opt = autoware::motion_utils::isDrivingForward(traj_points);
  if (is_driving_forward_opt && !*is_driving_forward_opt) {
    std::transform(curvature_vec.begin(), curvature_vec.end(), curvature_vec.begin(), [](double c) {
      return -c;
    });
  }

  std::vector<double> front_outer_wheel_off_track(curvature_vec.size(), 0.0);
  std::transform(
    curvature_vec.begin(), curvature_vec.end(), front_outer_wheel_off_track.begin(),
    [&vehicle_info](const double base_link_curvature) {
      // 後軸中心の曲率から後外輪の軌跡の曲率を求める
      const double base_link_outer_wheel_curvature =
        base_link_curvature /
        (1.0 + std::abs(base_link_curvature) * vehicle_info.vehicle_width_m / 2.0);
      // 前外輪のオフトラッキング量。絶対値は
      // std::hypot(radius_front_outer_wheel, wheel_base) - radius_front_outer_wheel と等価
      return -1.0 * vehicle_info.wheel_base_m *
             std::tan(0.5 * std::atan(base_link_outer_wheel_curvature * vehicle_info.wheel_base_m));
    });

  return front_outer_wheel_off_track;
}

std::vector<TrajectoryPoint> decimate_trajectory_points_from_ego(
  const autoware::experimental::trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const geometry_msgs::msg::Pose & current_pose, const double decimate_trajectory_step_length,
  const double goal_extended_trajectory_length)
{
  // planner の出力軌道は ego 近傍から始まり自己交差しないため、
  // 元実装の soft constraint 付き近傍探索(dist/yaw 閾値)は使わず幾何的な closest で足りる
  const double ego_s = autoware::experimental::trajectory::closest(trajectory, current_pose);

  // sample the trajectory from ego to the goal with the given step length
  // (keep the goal point, but avoid a duplicated point when the length is a multiple of the step)
  constexpr double epsilon = 1e-3;
  std::vector<TrajectoryPoint> decimated_traj_points_from_ego;
  double s = ego_s;
  for (; s < trajectory.length() - epsilon; s += decimate_trajectory_step_length) {
    decimated_traj_points_from_ego.push_back(trajectory.compute(s));
  }
  decimated_traj_points_from_ego.push_back(trajectory.compute(trajectory.length()));

  // extend trajectory
  const auto extended_traj_points_from_ego = get_extended_trajectory_points(
    decimated_traj_points_from_ego, goal_extended_trajectory_length,
    decimate_trajectory_step_length);
  if (extended_traj_points_from_ego.size() < 2) {
    return trajectory.restore();
  }
  return extended_traj_points_from_ego;
}

std::vector<Polygon2d> create_one_step_polygons_impl(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double additional_front_outer_wheel_off_track_scale)
{
  using autoware_utils_geometry::calc_offset_pose;
  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double half_width = vehicle_info.vehicle_width_m / 2.0;

  const auto error_poses =
    enable_to_consider_current_pose
      ? calculate_error_poses(traj_points, current_ego_pose, time_to_convergence)
      : std::vector<geometry_msgs::msg::Pose>{};
  const auto front_outer_wheel_off_tracks =
    additional_front_outer_wheel_off_track_scale > 0.0
      ? calc_front_outer_wheel_off_tracking(traj_points, vehicle_info)
      : std::vector<double>(traj_points.size(), 0.0);

  std::vector<Polygon2d> output_polygons;
  Polygon2d tmp_polys{};
  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};
    if (i < error_poses.size()) {
      current_poses.push_back(error_poses.at(i));
    }

    const double left_margin = lat_margin + additional_front_outer_wheel_off_track_scale *
                                              std::max(front_outer_wheel_off_tracks.at(i), 0.0);
    const double right_margin = lat_margin + additional_front_outer_wheel_off_track_scale *
                                               std::max(-front_outer_wheel_off_tracks.at(i), 0.0);

    Polygon2d idx_poly{};
    for (const auto & pose : current_poses) {
      if (i == 0 && traj_points.at(i).longitudinal_velocity_mps > 1e-3) {
        const auto point0 =
          calc_offset_pose(pose, front_length, half_width + left_margin, 0.0).position;
        const auto point1 =
          calc_offset_pose(pose, front_length, -half_width - right_margin, 0.0).position;
        const auto point2 = calc_offset_pose(pose, -rear_length, -half_width, 0.0).position;
        const auto point3 = calc_offset_pose(pose, -rear_length, half_width, 0.0).position;

        bg::append(idx_poly, msg_to_2d(point0));
        bg::append(idx_poly, msg_to_2d(point1));
        bg::append(idx_poly, msg_to_2d(point2));
        bg::append(idx_poly, msg_to_2d(point3));
        bg::append(idx_poly, msg_to_2d(point0));
      } else {
        bg::append(
          idx_poly, create_pose_footprint(pose, vehicle_info, left_margin, right_margin).outer());
      }
    }

    bg::append(tmp_polys, idx_poly.outer());
    Polygon2d hull_polygon;
    bg::convex_hull(tmp_polys, hull_polygon);
    bg::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);
    tmp_polys = std::move(idx_poly);
  }
  return output_polygons;
}
}  // namespace

std::vector<Polygon2d> create_one_step_polygons(
  const autoware::experimental::trajectory::Trajectory<TrajectoryPoint> & trajectory,
  const VehicleInfo & vehicle_info, const geometry_msgs::msg::Pose & current_ego_pose,
  const double lat_margin, const bool enable_to_consider_current_pose,
  const double time_to_convergence, const double decimate_trajectory_step_length,
  const double goal_extended_trajectory_length,
  const double additional_front_outer_wheel_off_track_scale)
{
  const auto decimated_traj_points = decimate_trajectory_points_from_ego(
    trajectory, current_ego_pose, decimate_trajectory_step_length, goal_extended_trajectory_length);
  return create_one_step_polygons_impl(
    decimated_traj_points, vehicle_info, current_ego_pose, lat_margin,
    enable_to_consider_current_pose, time_to_convergence,
    additional_front_outer_wheel_off_track_scale);
}

}  // namespace autoware::minimum_rule_based_planner::plugin::trajectory_polygon_utils
