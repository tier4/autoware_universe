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

#include "obstacle_stop.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <optional>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
namespace
{
namespace mvp = autoware::motion_velocity_planner;

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:129-135
double calc_x_offset_to_bumper(const bool is_driving_forward, const VehicleInfo & vehicle_info)
{
  if (is_driving_forward) {
    return vehicle_info.max_longitudinal_offset_m;
  }
  return vehicle_info.min_longitudinal_offset_m;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:157-180
double calc_braking_dist_along_trajectory(
  [[maybe_unused]] const StopObstacleClassification::Type label, const double lon_vel,
  const RSSParam & rss_params)
{
  const double error_considered_vel = std::max(lon_vel + rss_params.velocity_offset, 0.0);
  return error_considered_vel * error_considered_vel * 0.5 / -rss_params.pointcloud_deceleration;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:183-205
PolygonParam create_polygon_param(
  const ObstacleFilteringParam::TrimTrajectoryParam & trim_trajectory_param,
  const std::optional<double> ego_braking_distance,
  const ObstacleFilteringParam::LateralMarginParam & lateral_margin_param,
  const std::optional<double> object_velocity)
{
  PolygonParam p;
  if (!trim_trajectory_param.enable_trimming || !ego_braking_distance.has_value()) {
    p.trimming_length = std::nullopt;
  } else {
    p.trimming_length =
      trim_trajectory_param.min_trajectory_length +
      trim_trajectory_param.braking_distance_scale_factor * ego_braking_distance.value();
  }
  p.lateral_margin = lateral_margin_param.nominal_margin +
                     (object_velocity > lateral_margin_param.is_moving_threshold_velocity
                        ? lateral_margin_param.additional_is_moving_margin
                        : lateral_margin_param.additional_is_stop_margin);
  p.off_track_scale = lateral_margin_param.additional_wheel_off_track_scale;
  return p;
}

}  // namespace

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:222-228
// （init のうちパラメータ構築部）
void ObstacleStop::update_parameters(const Params & params)
{
  common_param_ = params.common;
  stop_planning_param_ = params.stop_planning;
  obstacle_filtering_params_ = {
    {StopObstacleClassification::Type::POINTCLOUD, params.obstacle_filtering}};
  pointcloud_segmentation_param_ = params.pointcloud_segmentation;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:280-341（plan）
std::vector<StopObstacle> ObstacleStop::calc_obstacle_stop(
  const std::vector<TrajectoryPoint> & raw_trajectory_points,
  const std::shared_ptr<const PlannerData> planner_data)
{
  // autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const double x_offset_to_bumper =
    calc_x_offset_to_bumper(planner_data->is_driving_forward, planner_data->vehicle_info_);
  trajectory_polygon_for_inside_map_.clear();

  const auto decimated_traj_points = mvp::utils::decimate_trajectory_points_from_ego(
    raw_trajectory_points, planner_data->current_odometry.pose.pose,
    planner_data->ego_nearest_dist_threshold, planner_data->ego_nearest_yaw_threshold,
    planner_data->trajectory_polygon_collision_check.decimate_trajectory_step_length,
    stop_planning_param_.stop_margin);

  auto stop_obstacles_for_point_cloud = filter_stop_obstacle_for_point_cloud(
    planner_data->current_odometry, raw_trajectory_points, decimated_traj_points,
    planner_data->no_ground_pointcloud, planner_data->vehicle_info_, x_offset_to_bumper,
    planner_data->trajectory_polygon_collision_check);

  return stop_obstacles_for_point_cloud;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:344-353
std::optional<double> ObstacleStop::calc_ego_forwarding_braking_distance(
  const std::vector<TrajectoryPoint> & traj_points, const nav_msgs::msg::Odometry & odometry) const
{
  if (traj_points.empty() || autoware::motion_utils::isDrivingForward(traj_points) != true) {
    return std::nullopt;
  }
  return autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    odometry.twist.twist.linear.x, 0.0, common_param_.max_accel, common_param_.min_accel,
    common_param_.max_jerk, common_param_.min_jerk);
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:356-431
std::optional<CollisionPointWithDist> ObstacleStop::get_nearest_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const Pointcloud & point_cloud, const double x_offset_to_bumper,
  const VehicleInfo & vehicle_info) const
{
  // autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  
  if (traj_points.size() != traj_polygons.size()) {
    RCLCPP_ERROR(
      logger_, "The size of trajectory points and polygons do not match: %zu vs %zu",
      traj_points.size(), traj_polygons.size());
    return std::nullopt;
  }

  if (point_cloud.pointcloud.empty()) {
    return std::nullopt;
  }

  const auto & clusters = point_cloud.get_cluster_indices();
  const auto & pointcloud_ptr = point_cloud.get_filtered_pointcloud_ptr();

  const auto & height_margin = pointcloud_segmentation_param_.height_margin;
  std::vector<geometry_msgs::msg::Point> collision_geom_points{};
  for (size_t traj_index = 0; traj_index < traj_points.size(); ++traj_index) {
    const double rough_dist_th = boost::geometry::perimeter(traj_polygons.at(traj_index)) * 0.5;
    const double traj_height = traj_points.at(traj_index).pose.position.z;

    for (const auto & cluster : clusters) {
      for (const auto & point_index : cluster.indices) {
        const auto obstacle_point = mvp::utils::to_geometry_point(pointcloud_ptr->at(point_index));
        if (
          obstacle_point.z - traj_height < -height_margin.margin_from_bottom ||
          obstacle_point.z - traj_height >
            vehicle_info.max_height_offset_m + height_margin.margin_from_top) {
          continue;
        }
        const double dist_from_base_link =
          autoware_utils_geometry::calc_distance2d(traj_points.at(traj_index).pose, obstacle_point);
        if (dist_from_base_link > rough_dist_th) {
          continue;
        }
        autoware_utils_geometry::Point2d obstacle_point_2d{obstacle_point.x, obstacle_point.y};
        if (boost::geometry::within(obstacle_point_2d, traj_polygons.at(traj_index))) {
          collision_geom_points.push_back(obstacle_point);
        }
      }
    }
    if (collision_geom_points.empty()) {
      continue;
    }

    const auto bumper_pose = autoware_utils_geometry::calc_offset_pose(
      traj_points.at(traj_index).pose, x_offset_to_bumper, 0.0, 0.0);
    std::optional<double> max_collision_length = std::nullopt;
    std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
    for (const auto & point : collision_geom_points) {
      const double dist_from_bumper =
        std::abs(autoware_utils_geometry::inverse_transform_point(point, bumper_pose).x);

      if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
        max_collision_length = dist_from_bumper;
        max_collision_point = point;
      }
    }
    return CollisionPointWithDist{
      *max_collision_point,
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_index) -
        *max_collision_length};
  }
  return std::nullopt;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:520-578
void ObstacleStop::upsert_pointcloud_stop_candidates(
  const CollisionPointWithDist & nearest_collision_point,
  const std::vector<TrajectoryPoint> & traj_points, rclcpp::Time latest_point_cloud_time)
{
  // autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & vel_params = pointcloud_segmentation_param_.velocity_estimation;
  const auto & assoc_params = pointcloud_segmentation_param_.time_series_association;

  for (auto stop_candidate = pointcloud_stop_candidates.rbegin();
       stop_candidate != pointcloud_stop_candidates.rend(); ++stop_candidate) {
    const double time_since_latest_collision =
      (latest_point_cloud_time - stop_candidate->latest_collision_pointcloud_time).seconds();
    if (time_since_latest_collision < 0.05) {
      return;
    }
    const double longitudinal_displacement = autoware::motion_utils::calcSignedArcLength(
      traj_points, stop_candidate->latest_collision_point.point, nearest_collision_point.point);

    if (
      time_since_latest_collision < assoc_params.max_time_diff &&
      -assoc_params.position_diff + assoc_params.min_velocity * time_since_latest_collision <
        longitudinal_displacement &&
      longitudinal_displacement <
        assoc_params.position_diff + assoc_params.max_velocity * time_since_latest_collision) {
      const double clamped_vel = std::clamp(
        longitudinal_displacement / time_since_latest_collision, vel_params.min_clamp_velocity,
        vel_params.max_clamp_velocity);
      if (!stop_candidate->vel_lpf.getValue().has_value()) {
        auto & vel_vec = stop_candidate->initial_velocities;
        vel_vec.push_back(clamped_vel);
        if (vel_vec.size() >= vel_params.required_velocity_count) {
          stop_candidate->vel_lpf.reset(
            std::accumulate(vel_vec.begin(), vel_vec.end(), 0.0) / vel_vec.size());
        }
      } else {
        stop_candidate->vel_lpf.filter(clamped_vel);
      }
      stop_candidate->latest_collision_point = nearest_collision_point;
      stop_candidate->latest_collision_pointcloud_time = latest_point_cloud_time;

      std::sort(
        pointcloud_stop_candidates.begin(), pointcloud_stop_candidates.end(),
        [](const PointcloudStopCandidate & a, const PointcloudStopCandidate & b) {
          return a.latest_collision_pointcloud_time < b.latest_collision_pointcloud_time;
        });

      return;
    }
  }
  PointcloudStopCandidate new_stop_candidate;
  new_stop_candidate.latest_collision_point = nearest_collision_point;
  new_stop_candidate.latest_collision_pointcloud_time = latest_point_cloud_time;
  new_stop_candidate.vel_lpf.setGain(vel_params.lpf_gain);
  pointcloud_stop_candidates.push_back(new_stop_candidate);
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:582-686
std::vector<StopObstacle> ObstacleStop::filter_stop_obstacle_for_point_cloud(
  const nav_msgs::msg::Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points, const Pointcloud & point_cloud,
  const VehicleInfo & vehicle_info, const double x_offset_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  // autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & filtering_param =
    obstacle_filtering_params_.at(StopObstacleClassification::Type::POINTCLOUD);

  if (!filtering_param.check_inside) {
    return std::vector<StopObstacle>{};
  }

  if (
    point_cloud.preprocess_params_.filter_by_trajectory_polygon.lateral_margin <
    filtering_param.lateral_margin.nominal_margin) {
    RCLCPP_WARN_ONCE(
      logger_,
      "pointcloud preprocessing lateral margin in motion_velocity_planner_node (%f) is smaller "
      "than obstacle_stop_module param (%f)",
      point_cloud.preprocess_params_.filter_by_trajectory_polygon.lateral_margin,
      filtering_param.lateral_margin.nominal_margin);
  }

  const auto & tp = trajectory_polygon_collision_check;
  const auto polygon_param = create_polygon_param(
    filtering_param.trim_trajectory, calc_ego_forwarding_braking_distance(traj_points, odometry),
    filtering_param.lateral_margin, std::nullopt);
  const auto detection_polygon_with_lat_margin = get_trajectory_polygon(
    decimated_traj_points, vehicle_info, odometry.pose.pose, polygon_param,
    tp.enable_to_consider_current_pose, tp.time_to_convergence, tp.decimate_trajectory_step_length);

  const auto nearest_collision_point = get_nearest_collision_point(
    detection_polygon_with_lat_margin.traj_points, detection_polygon_with_lat_margin.polygons,
    point_cloud, x_offset_to_bumper, vehicle_info);

  const auto latest_point_cloud_time =
    rclcpp::Time(point_cloud.pointcloud.header.stamp * static_cast<uint32_t>(1e3), RCL_ROS_TIME);
  if (nearest_collision_point) {
    upsert_pointcloud_stop_candidates(
      nearest_collision_point.value(), traj_points, latest_point_cloud_time);
  }

  while (
    !pointcloud_stop_candidates.empty() &&
    (latest_point_cloud_time - pointcloud_stop_candidates.front().latest_collision_pointcloud_time)
        .seconds() > filtering_param.stop_obstacle_hold_time_threshold) {
    pointcloud_stop_candidates.pop_front();
  }

  const rclcpp::Time now_stamp{odometry.header.stamp};
  std::vector<StopObstacle> stop_obstacles;
  for (const auto & stop_candidate : pointcloud_stop_candidates) {
    if (!stop_candidate.vel_lpf.getValue().has_value()) {
      continue;
    }

    const double time_delay =
      (now_stamp - stop_candidate.latest_collision_pointcloud_time).seconds();
    const double time_compensated_dist_to_collide =
      stop_candidate.latest_collision_point.dist_to_collide +
      *stop_candidate.vel_lpf.getValue() * time_delay;

    const bool use_estimated_velocity =
      pointcloud_segmentation_param_.velocity_estimation.use_estimated_velocity;
    if (
      !use_estimated_velocity ||
      *stop_candidate.vel_lpf.getValue() <
        stop_planning_param_.obstacle_velocity_threshold_enter_fixed_stop) {
      stop_obstacles.emplace_back(
        stop_candidate.latest_collision_pointcloud_time,
        StopObstacleClassification{StopObstacleClassification::Type::POINTCLOUD},
        stop_candidate.vel_lpf.getValue().value(), stop_candidate.latest_collision_point.point,
        time_compensated_dist_to_collide, polygon_param);
    } else if (stop_planning_param_.rss_params.use_rss_stop) {
      const auto braking_dist = calc_braking_dist_along_trajectory(
        StopObstacleClassification::Type::POINTCLOUD, *stop_candidate.vel_lpf.getValue(),
        stop_planning_param_.rss_params);
      stop_obstacles.emplace_back(
        stop_candidate.latest_collision_pointcloud_time,
        StopObstacleClassification{StopObstacleClassification::Type::POINTCLOUD},
        stop_candidate.vel_lpf.getValue().value(), stop_candidate.latest_collision_point.point,
        time_compensated_dist_to_collide, polygon_param, braking_dist);
      RCLCPP_DEBUG(
        logger_,
        "|_PC_| total_dist: %2.5f, raw_dist: %2.5f, time_compensated dist: %2.5f, "
        "braking_dist: %2.5f",
        (time_compensated_dist_to_collide + braking_dist),
        (stop_candidate.latest_collision_point.dist_to_collide), time_compensated_dist_to_collide,
        braking_dist);
    }
  }

  return stop_obstacles;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:1368-1391
DetectionPolygon ObstacleStop::get_trajectory_polygon(
  const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const PolygonParam & polygon_param,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length) const
{
  if (trajectory_polygon_for_inside_map_.count(polygon_param) == 0) {
    auto cropped_traj_points =
      polygon_param.trimming_length.has_value()
        ? autoware::motion_utils::cropForwardPoints(
            decimated_traj_points, decimated_traj_points.front().pose.position, 0,
            polygon_param.trimming_length.value())
        : decimated_traj_points;

    auto traj_polys = mvp::polygon_utils::create_one_step_polygons(
      cropped_traj_points, vehicle_info, current_ego_pose, polygon_param.lateral_margin,
      enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length,
      polygon_param.off_track_scale);
    trajectory_polygon_for_inside_map_.emplace(
      polygon_param, DetectionPolygon{std::move(cropped_traj_points), std::move(traj_polys)});
  }
  return trajectory_polygon_for_inside_map_.at(polygon_param);
}

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
