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

#include "autoware/motion_velocity_planner_common_universe/polygon_utils.hpp"

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware_utils/geometry/boost_polygon_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::polygon_utils
{
namespace
{
PointWithStamp calc_nearest_collision_point(
  const size_t first_within_idx, const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points, const bool is_driving_forward)
{
  const size_t prev_idx = first_within_idx == 0 ? first_within_idx : first_within_idx - 1;
  const size_t next_idx = first_within_idx == 0 ? first_within_idx + 1 : first_within_idx;

  std::vector<geometry_msgs::msg::Pose> segment_points{
    decimated_traj_points.at(prev_idx).pose, decimated_traj_points.at(next_idx).pose};
  if (!is_driving_forward) {
    std::reverse(segment_points.begin(), segment_points.end());
  }

  std::vector<double> dist_vec;
  for (const auto & collision_point : collision_points) {
    const double dist = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      segment_points, 0, collision_point.point);
    dist_vec.push_back(dist);
  }

  const size_t min_idx = std::min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
  return collision_points.at(min_idx);
}

// NOTE: max_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::optional<std::pair<size_t, std::vector<PointWithStamp>>> get_collision_index(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Pose & object_pose, const rclcpp::Time & object_time,
  const Shape & object_shape, const double max_dist = std::numeric_limits<double>::max())
{
  const auto obj_polygon = autoware_utils::to_polygon2d(object_pose, object_shape);
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double approximated_dist =
      autoware_utils::calc_distance2d(traj_points.at(i).pose, object_pose);
    if (approximated_dist > max_dist) {
      continue;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);

    std::vector<PointWithStamp> collision_geom_points;
    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;

        for (const auto & collision_point : collision_polygon.outer()) {
          PointWithStamp collision_geom_point;
          collision_geom_point.stamp = object_time;
          collision_geom_point.point.x = collision_point.x();
          collision_geom_point.point.y = collision_point.y();
          collision_geom_point.point.z = traj_points.at(i).pose.position.z;
          collision_geom_points.push_back(collision_geom_point);
        }
      }
    }

    if (has_collision) {
      const auto collision_info =
        std::pair<size_t, std::vector<PointWithStamp>>{i, collision_geom_points};
      return collision_info;
    }
  }

  return std::nullopt;
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
    autoware_utils::inverse_transform_pose(current_ego_pose, nearest_pose);
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
      autoware_utils::create_quaternion_from_yaw(current_ego_yaw_error * rem_ratio));
    indexed_pose_err.set__position(
      autoware_utils::create_point(0.0, current_ego_lat_error * rem_ratio, 0.0));
    error_poses.push_back(
      autoware_utils::transform_pose(indexed_pose_err, traj_points.at(i).pose));

    if (traj_points.at(i).longitudinal_velocity_mps != 0.0 && i < traj_points.size() - 1) {
      time_elapsed += autoware_utils::calc_distance2d(
                        traj_points.at(i).pose.position, traj_points.at(i + 1).pose.position) /
                      std::abs(traj_points.at(i).longitudinal_velocity_mps);
    } else {
      time_elapsed = std::numeric_limits<double>::max();
    }
  }
  return error_poses;
}

Polygon2d create_pose_footprint(
  const geometry_msgs::msg::Pose & pose, const VehicleInfo & vehicle_info, const double lat_margin)
{
  return autoware_utils::to_footprint(
    pose, vehicle_info.max_longitudinal_offset_m, vehicle_info.rear_overhang_m,
    vehicle_info.vehicle_width_m + lat_margin * 2.0);
};

/**
 * @brief Combine multiple polygons safely and efficiently.
 * @details Features include cascaded union, vertex reduction (simplify), and loop iteration limit.
 * @param polygons The list of polygons to combine.
 * @param simplification_tolerance The tolerance for vertex reduction. Disabled if 0 or less.
 * @param iteration_limit The maximum number of loop iterations. A safety mechanism for
 * non-convergence.
 * @return The combined single polygon. Returns std::nullopt if failed.
 */

std::optional<Polygon2d> cascaded_union(
  const std::vector<Polygon2d> & polygons, size_t iteration_limit = 0)
{
  if (iteration_limit == 0) {
    iteration_limit = polygons.size() * 5;
  }

  if (polygons.empty()) {
    return Polygon2d();
  }
  if (polygons.size() == 1) {
    return polygons.front();
  }

  std::vector<Polygon2d> work_list(polygons.rbegin(), polygons.rend());
  size_t iteration_count = 0;
  while (work_list.size() > 1 && iteration_count < iteration_limit) {
    iteration_count++;

    auto pop_valid_polygon = [&work_list]() -> Polygon2d {
      std::string reason;
      while (!work_list.empty()) {
        auto poly = std::move(work_list.back());
        work_list.pop_back();

        if (bg::is_valid(poly, reason)) {
          return poly;
        }
        std::stringstream ss;
        ss << "pop polygon is invalid: " << reason << ", detail points: " << bg::wkt(poly);
        RCLCPP_DEBUG(rclcpp::get_logger("polygon_utils"), "%s", ss.str().c_str());
      }
      return Polygon2d{};
    };

    Polygon2d poly_a = pop_valid_polygon();
    Polygon2d poly_b = pop_valid_polygon();

    std::vector<Polygon2d> temp_results;
    bg::union_(poly_a, poly_b, temp_results);

    work_list.insert(
      work_list.end(), std::make_move_iterator(temp_results.rbegin()),
      std::make_move_iterator(temp_results.rend()));
  }

  if (work_list.size() > 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("polygon_utils"), "cascaded union did not converge within %ld iterations.",
      iteration_limit);
    return std::nullopt;
  }

  if (work_list.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("polygon_utils"),
      "cascaded union result is empty even the input polygons are not empty");
    return std::nullopt;
  }

  std::string reason;
  if (!bg::is_valid(work_list.front(), reason)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("polygon_utils"), "cascaded union result is invalid. reason: %s",
      reason.c_str());
    return std::nullopt;
  }

  return std::move(work_list.front());
}

Polygon2d create_one_step_polygon(
  geometry_msgs::msg::Pose src_pose, geometry_msgs::msg::Pose dst_pose,
  const VehicleInfo & vehicle_info, const double lat_margin, const double required_accuracy)
{
  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double vehicle_width = vehicle_info.vehicle_width_m;

  auto pose_diff = autoware_utils::inverse_transform_pose(dst_pose, src_pose);
  if (std::abs(pose_diff.position.x) < required_accuracy) {
    return create_pose_footprint(dst_pose, vehicle_info, lat_margin);
  }
  if (pose_diff.position.x < 0.0) {
    RCLCPP_WARN(
      rclcpp::get_logger("polygon_utils"), "swap src and dst. pose_diff.position.x: %f",
      pose_diff.position.x);
    std::swap(src_pose, dst_pose);
    pose_diff = autoware_utils::inverse_transform_pose(dst_pose, src_pose);
  }

  const double step_position_dist = std::hypot(pose_diff.position.x, pose_diff.position.y);
  const double step_orientation_diff = tf2::getYaw(pose_diff.orientation);
  const int step_rotation_sign = (step_orientation_diff > 0.0) ? 1 : -1;

  // approximate the curvature esitimation error:
  const double front_compensated_length = front_length + step_position_dist * 0.5;
  const double rear_compensated_length = rear_length + step_position_dist * 0.5;
  const double inner_predicted_accuracy = (front_compensated_length * rear_compensated_length) /
                                          (front_compensated_length + rear_compensated_length) *
                                          std::abs(step_orientation_diff);
  const bool inner_precise_necessary = inner_predicted_accuracy > required_accuracy * 0.5;
  const bool outer_precise_necessary =
    step_position_dist * std::abs(step_orientation_diff) * 0.125 > required_accuracy * 0.5;

  const auto get_transformed_point =
    [](const geometry_msgs::msg::Pose & pose, double x, double y) -> Point2d {
    autoware_utils::Point3d point_3d{x, y, 0};
    const auto transformed = autoware_utils::transform_point(point_3d, pose);
    return autoware_utils::Point2d{transformed.x(), transformed.y()};
  };

  MultiPoint2d points{};
  bg::append(points, create_pose_footprint(src_pose, vehicle_info, lat_margin).outer());
  bg::append(points, create_pose_footprint(dst_pose, vehicle_info, lat_margin).outer());

  // append outer points
  if (outer_precise_necessary) {
    const auto intermidiate_point_by_arc_tangent =
      [](const Point2d & src, const Point2d & dst, const double angle) {
        const Eigen::Vector2d src_vec(src.x(), src.y());
        const Eigen::Vector2d dst_vec(dst.x(), dst.y());

        const Eigen::Vector2d displacement_vec = dst_vec - src_vec;
        const Eigen::Vector2d arc_intermidiate_point =
          src_vec + Eigen::Rotation2Dd(-angle / 2.0) * displacement_vec / (2.0 * cos(-angle / 2.0));

        return Point2d(arc_intermidiate_point.x(), arc_intermidiate_point.y());
      };

    // append front bumper points
    const auto front_bumper_src = get_transformed_point(
      src_pose, front_length, -step_rotation_sign * (vehicle_width * 0.5 + lat_margin));
    const auto front_bumper_dst = get_transformed_point(
      dst_pose, front_length, -step_rotation_sign * (vehicle_width * 0.5 + lat_margin));
    const auto front_bumper_intermidiate_point =
      intermidiate_point_by_arc_tangent(front_bumper_src, front_bumper_dst, step_orientation_diff);
    bg::append(points, front_bumper_intermidiate_point);

    // append rear bumper points
    const auto rear_bumper_src = get_transformed_point(
      src_pose, -rear_length, -step_rotation_sign * (vehicle_width * 0.5 + lat_margin));
    const auto rear_bumper_dst = get_transformed_point(
      dst_pose, -rear_length, -step_rotation_sign * (vehicle_width * 0.5 + lat_margin));
    const auto rear_bumper_intermidiate_point =
      intermidiate_point_by_arc_tangent(rear_bumper_src, rear_bumper_dst, step_orientation_diff);
    bg::append(points, rear_bumper_intermidiate_point);
  }

  std::stringstream points_ss;
  points_ss << "points: " << bg::wkt(points);

  // make convex hull polygon
  Polygon2d output_poly{};
  bg::convex_hull(points, output_poly);
  bg::correct(output_poly);

  std::stringstream convex_hull_ss;
  convex_hull_ss << "convex hull: " << bg::wkt(output_poly);

  // insert base link points to cut the convex hull polygon
  if (inner_precise_necessary) {
    // align the direction to insert the points by the constant direction
    if ((step_rotation_sign > 0) != (autoware_utils::is_clockwise(output_poly))) {
      bg::reverse(output_poly.outer());
    }

    // find index to be inserted
    const auto insert_base_point = get_transformed_point(
      src_pose, -rear_length, step_rotation_sign * (vehicle_width * 0.5 + lat_margin));
    const auto validation_point = get_transformed_point(
      dst_pose, front_length, step_rotation_sign * (vehicle_width * 0.5 + lat_margin));
    auto insert_itr = std::find_if(
      output_poly.outer().begin(), output_poly.outer().end(),
      [&insert_base_point](const Point2d & point) {
        return std::hypot(point.x() - insert_base_point.x(), point.y() - insert_base_point.y()) <
               1e-6;
      });

    if (insert_itr >= std::prev(output_poly.outer().end())) {
      RCLCPP_WARN(rclcpp::get_logger("polygon_utils"), "src back point is not found");
      bg::correct(output_poly);
      return output_poly;
    }

    if (
      std::hypot(
        std::next(insert_itr)->x() - validation_point.x(),
        std::next(insert_itr)->y() - validation_point.y()) > 1e-6) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("polygon_utils"),
        "src back point and dst front point are not connected");
      bg::correct(output_poly);
      return output_poly;
    }

    std::stringstream before_insert_ss;
    before_insert_ss << "before insert size: " << output_poly.outer().size() << " "
                     << bg::wkt(output_poly);

    // insert base link points
    const auto src_base_link_point =
      get_transformed_point(src_pose, 0.0, step_rotation_sign * (vehicle_width * 0.5 + lat_margin));

    insert_itr = output_poly.outer().insert(std::next(insert_itr), src_base_link_point);

    const auto dst_base_link_point =
      get_transformed_point(dst_pose, 0.0, step_rotation_sign * (vehicle_width * 0.5 + lat_margin));
    insert_itr = output_poly.outer().insert(std::next(insert_itr), dst_base_link_point);

    std::stringstream after_insert_ss;
    after_insert_ss << "after insert size: " << output_poly.outer().size() << " "
                    << bg::wkt(output_poly);

    if (std::abs(step_orientation_diff) > 0.1) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("polygon_utils"), "before insert: %s", before_insert_ss.str().c_str());
      RCLCPP_DEBUG(
        rclcpp::get_logger("polygon_utils"), "after insert: %s", after_insert_ss.str().c_str());
    }

    bg::correct(output_poly);
  }

  // check valid
  std::string reason;
  if (!bg::is_valid(output_poly, reason)) {
    std::stringstream ss;
    ss << "__FUNCTION__: " << __FUNCTION__ << ", output_polygon is invalid. reason: " << reason
       << ", detail points: " << bg::wkt(output_poly);
    RCLCPP_WARN(rclcpp::get_logger("polygon_utils"), "%s", ss.str().c_str());
    return create_pose_footprint(dst_pose, vehicle_info, lat_margin);
  }

  return output_poly;
}

}  // namespace

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Pose obj_pose, const rclcpp::Time obj_stamp, const Shape & obj_shape,
  const double dist_to_bumper)
{
  const auto collision_info =
    get_collision_index(traj_points, traj_polygons, obj_pose, obj_stamp, obj_shape);
  if (!collision_info) {
    return std::nullopt;
  }

  const auto bumper_pose = autoware_utils::calc_offset_pose(
    traj_points.at(collision_info->first).pose, dist_to_bumper, 0.0, 0.0);

  std::optional<double> max_collision_length = std::nullopt;
  std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
  for (const auto & poly_vertex : collision_info->second) {
    const double dist_from_bumper =
      std::abs(autoware_utils::inverse_transform_point(poly_vertex.point, bumper_pose).x);

    if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
      max_collision_length = dist_from_bumper;
      max_collision_point = poly_vertex.point;
    }
  }
  return std::make_pair(
    *max_collision_point,
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, collision_info->first) -
      *max_collision_length);
}

std::optional<std::pair<geometry_msgs::msg::Point, double>> get_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const size_t collision_idx,
  const std::vector<PointWithStamp> & collision_points, const double dist_to_bumper)
{
  std::pair<size_t, std::vector<PointWithStamp>> collision_info;
  collision_info.first = collision_idx;
  collision_info.second = collision_points;

  const auto bumper_pose = autoware_utils::calc_offset_pose(
    traj_points.at(collision_info.first).pose, dist_to_bumper, 0.0, 0.0);

  std::optional<double> max_collision_length = std::nullopt;
  std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
  for (const auto & poly_vertex : collision_info.second) {
    const double dist_from_bumper =
      std::abs(autoware_utils::inverse_transform_point(poly_vertex.point, bumper_pose).x);

    if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
      max_collision_length = dist_from_bumper;
      max_collision_point = poly_vertex.point;
    }
  }
  if (!max_collision_point.has_value() || !max_collision_length.has_value()) return std::nullopt;
  return std::make_pair(
    *max_collision_point,
    autoware::motion_utils::calcSignedArcLength(traj_points, 0, collision_info.first) -
      *max_collision_length);
}

// NOTE: max_lat_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::vector<PointWithStamp> get_collision_points(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_lat_dist,
  const double max_prediction_time_for_collision_check)
{
  std::vector<PointWithStamp> collision_points;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    if (
      max_prediction_time_for_collision_check <
      rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(i)) {
      break;
    }

    const auto object_time =
      rclcpp::Time(obstacle_stamp) + rclcpp::Duration(predicted_path.time_step) * i;
    // Ignore past position
    if ((object_time - current_time).seconds() < 0.0) {
      continue;
    }

    const auto collision_info = get_collision_index(
      traj_points, traj_polygons, predicted_path.path.at(i), object_time, shape, max_lat_dist);
    if (collision_info) {
      const auto nearest_collision_point = calc_nearest_collision_point(
        collision_info->first, collision_info->second, traj_points, is_driving_forward);
      collision_points.push_back(nearest_collision_point);
      collision_index.push_back(collision_info->first);
    }
  }

  return collision_points;
}

std::vector<Polygon2d> create_one_step_polygons_precise(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  [[maybe_unused]] const double decimate_trajectory_step_length, const double required_accuracy)
{
  const auto start_time = std::chrono::high_resolution_clock::now();

  // set up nominal_pose
  std::vector<geometry_msgs::msg::Pose> nominal_poses;
  nominal_poses.reserve(traj_points.size());
  for (const auto & traj_point : traj_points) {
    nominal_poses.push_back(traj_point.pose);
  }

  // generate error_poses
  const auto error_poses =
    enable_to_consider_current_pose
      ? calculate_error_poses(traj_points, current_ego_pose, time_to_convergence)
      : std::vector<geometry_msgs::msg::Pose>{};

  std::vector<Polygon2d> output_polygons;
  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<Polygon2d> index_polygons;
    if (i == 0) {
      index_polygons.push_back(
        create_pose_footprint(nominal_poses.at(i), vehicle_info, lat_margin));
      if (i < error_poses.size()) {
        index_polygons.push_back(
          create_pose_footprint(error_poses.at(i), vehicle_info, lat_margin));
      }
      // }
    } else {
      index_polygons.push_back(create_one_step_polygon(
        nominal_poses.at(i - 1), nominal_poses.at(i), vehicle_info, lat_margin, required_accuracy));
      if (i < error_poses.size()) {
        index_polygons.push_back(create_one_step_polygon(
          error_poses.at(i - 1), error_poses.at(i), vehicle_info, lat_margin, required_accuracy));
      }
    }

    Polygon2d simplified_poly{};
    if (index_polygons.size() == 0) {
      RCLCPP_WARN(rclcpp::get_logger("polygon_utils"), "index_polygons is empty. fatal error.");
      output_polygons.push_back(
        create_pose_footprint(traj_points.at(i).pose, vehicle_info, lat_margin));
      continue;
    }
    if (index_polygons.size() > 1) {
      if (bg::intersects(index_polygons.front(), index_polygons.back())) {
        const auto union_poly = cascaded_union(index_polygons);
        if (union_poly) {
          bg::simplify(*union_poly, simplified_poly, required_accuracy);
          output_polygons.push_back(std::move(simplified_poly));
          continue;
        }
      }
      RCLCPP_WARN(
        rclcpp::get_logger("polygon_utils"),
        "failed to union the error pose polygon. use the nominal pose one_step polygon");
    }
    bg::simplify(index_polygons.front(), simplified_poly, required_accuracy);
    output_polygons.push_back(std::move(simplified_poly));
  }

  const auto end_time = std::chrono::high_resolution_clock::now();
  const auto duration =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  RCLCPP_DEBUG(
    rclcpp::get_logger("polygon_utils"), "create_one_step_polygons_precise took %ld us",
    duration.count());

  return output_polygons;
}

std::vector<Polygon2d> create_one_step_polygons(
  const std::vector<TrajectoryPoint> & traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const double lat_margin,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length)
{
  const double front_length = vehicle_info.max_longitudinal_offset_m;
  const double rear_length = vehicle_info.rear_overhang_m;
  const double vehicle_width = vehicle_info.vehicle_width_m;

  const size_t nearest_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_ego_pose.position);
  const auto nearest_pose = traj_points.at(nearest_idx).pose;
  const auto current_ego_pose_error =
    autoware_utils::inverse_transform_pose(current_ego_pose, nearest_pose);
  const double current_ego_lat_error = current_ego_pose_error.position.y;
  const double current_ego_yaw_error = tf2::getYaw(current_ego_pose_error.orientation);
  double time_elapsed{0.0};

  std::vector<Polygon2d> output_polygons;
  Polygon2d tmp_polys{};
  for (size_t i = 0; i < traj_points.size(); ++i) {
    std::vector<geometry_msgs::msg::Pose> current_poses = {traj_points.at(i).pose};

    // estimate the future ego pose with assuming that the pose error against the reference path
    // will decrease to zero by the time_to_convergence
    if (enable_to_consider_current_pose && time_elapsed < time_to_convergence) {
      const double rem_ratio = (time_to_convergence - time_elapsed) / time_to_convergence;
      geometry_msgs::msg::Pose indexed_pose_err;
      indexed_pose_err.set__orientation(
        autoware_utils::create_quaternion_from_yaw(current_ego_yaw_error * rem_ratio));
      indexed_pose_err.set__position(
        autoware_utils::create_point(0.0, current_ego_lat_error * rem_ratio, 0.0));
      current_poses.push_back(
        autoware_utils::transform_pose(indexed_pose_err, traj_points.at(i).pose));
      if (traj_points.at(i).longitudinal_velocity_mps != 0.0) {
        time_elapsed +=
          decimate_trajectory_step_length / std::abs(traj_points.at(i).longitudinal_velocity_mps);
      } else {
        time_elapsed = std::numeric_limits<double>::max();
      }
    }

    Polygon2d idx_poly{};
    for (const auto & pose : current_poses) {
      if (i == 0 && traj_points.at(i).longitudinal_velocity_mps > 1e-3) {
        boost::geometry::append(
          idx_poly,
          autoware_utils::to_footprint(pose, front_length, rear_length, vehicle_width).outer());
        boost::geometry::append(
          idx_poly,
          autoware_utils::from_msg(autoware_utils::calc_offset_pose(
                                     pose, front_length, vehicle_width * 0.5 + lat_margin, 0.0)
                                     .position)
            .to_2d());
        boost::geometry::append(
          idx_poly,
          autoware_utils::from_msg(autoware_utils::calc_offset_pose(
                                     pose, front_length, -vehicle_width * 0.5 - lat_margin, 0.0)
                                     .position)
            .to_2d());
      } else {
        boost::geometry::append(
          idx_poly, autoware_utils::to_footprint(
                      pose, front_length, rear_length, vehicle_width + lat_margin * 2.0)
                      .outer());
      }
    }

    boost::geometry::append(tmp_polys, idx_poly.outer());
    Polygon2d hull_polygon;
    boost::geometry::convex_hull(tmp_polys, hull_polygon);
    boost::geometry::correct(hull_polygon);

    output_polygons.push_back(hull_polygon);
    tmp_polys = std::move(idx_poly);
  }
  return output_polygons;
}
}  // namespace autoware::motion_velocity_planner::polygon_utils
