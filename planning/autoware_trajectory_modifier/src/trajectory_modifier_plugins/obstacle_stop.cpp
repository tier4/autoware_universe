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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/obstacle_stop.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <range/v3/view.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using utils::obstacle_stop::cluster_pointcloud;
using utils::obstacle_stop::filter_objects_by_type;
using utils::obstacle_stop::filter_objects_by_velocity;
using utils::obstacle_stop::filter_pointcloud_by_range;
using utils::obstacle_stop::filter_pointcloud_by_voxel_grid;
using utils::obstacle_stop::get_nearest_object_collision;
using utils::obstacle_stop::get_nearest_pcd_collision;
using utils::obstacle_stop::get_trajectory_polygon;
using utils::obstacle_stop::PointCloud;
using utils::obstacle_stop::PointCloud2;

void ObstacleStop::on_initialize(const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "obstacle_stop");

  params_ = params.obstacle_stop;
  enabled_ = params.use_obstacle_stop;
}

bool ObstacleStop::is_trajectory_modification_required(const TrajectoryPoints & traj_points)
{
  const auto trajectory_polygon = get_trajectory_polygon(
    traj_points, data_->current_odometry->pose.pose, data_->vehicle_info, params_.lateral_margin_m);
  const auto collision_point_pcd = check_pointcloud(traj_points, trajectory_polygon);
  const auto collision_point_objects = check_predicted_objects(traj_points, trajectory_polygon);
  if (!collision_point_pcd && !collision_point_objects) return false;

  if (!collision_point_pcd)
    nearest_collision_point_ = collision_point_objects;
  else if (!collision_point_objects)
    nearest_collision_point_ = collision_point_pcd;
  else
    nearest_collision_point_ = collision_point_pcd->arc_length < collision_point_objects->arc_length
                                 ? collision_point_pcd
                                 : collision_point_objects;

  return nearest_collision_point_ != std::nullopt;
}

void ObstacleStop::modify_trajectory(TrajectoryPoints & traj_points)
{
  if (!enabled_ || !is_trajectory_modification_required(traj_points)) return;

  if (!nearest_collision_point_) return;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[TM ObstacleStop] Detected collision point at arc length %f m",
    nearest_collision_point_->arc_length);

  set_stop_point(traj_points);
}

void ObstacleStop::set_stop_point(TrajectoryPoints & traj_points)
{
  const auto stop_margin = params_.stop_margin_m + data_->vehicle_info.max_longitudinal_offset_m;
  const auto target_stop_point_arc_length =
    std::max(nearest_collision_point_->arc_length - stop_margin, 0.0);

  RCLCPP_INFO_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[TM ObstacleStop] Target stop point arc length %f m", target_stop_point_arc_length);

  constexpr double stop_velocity_threshold = 0.01;

  if (params_.duplicate_check_threshold_m > std::numeric_limits<double>::epsilon()) {
    auto checked_distance = 0.0;
    for (const auto & [p_1, p_2] : traj_points | ranges::views::sliding(2)) {
      checked_distance += autoware_utils::calc_distance2d(p_1->pose.position, p_2->pose.position);
      if (p_2->longitudinal_velocity_mps > stop_velocity_threshold) continue;
      if (
        abs(target_stop_point_arc_length - checked_distance) <
        params_.duplicate_check_threshold_m) {
        RCLCPP_WARN_THROTTLE(
          get_node_ptr()->get_logger(), *get_clock(), 500,
          "[TM ObstacleStop] Duplicate stop point detected near target stop point, skip.");
        return;
      }
      if (checked_distance > target_stop_point_arc_length + params_.duplicate_check_threshold_m)
        break;
    }
  }

  auto stop_point =
    motion_utils::calcLongitudinalOffsetPoint(traj_points, 0.0, target_stop_point_arc_length);
  if (!stop_point) return;
  auto stop_point_index = motion_utils::findNearestSegmentIndex(traj_points, stop_point.value());

  for (auto i = stop_point_index + 1; i < traj_points.size(); ++i) {
    traj_points.at(i).longitudinal_velocity_mps = 0.0;
    traj_points.at(i).acceleration_mps2 = 0.0;
    traj_points.at(i).pose = traj_points.at(stop_point_index).pose;
  }

  const auto & stop_pose = traj_points.at(stop_point_index).pose;
  const auto & ego_pose = data_->current_odometry->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 500,
    "[TM ObstacleStop] Inserted stop point at arc length %f m", target_stop_point_arc_length);
}

std::optional<CollisionPoint> ObstacleStop::check_predicted_objects(
  const TrajectoryPoints & traj_points, const MultiPolygon2d & trajectory_polygon)
{
  if (!data_->predicted_objects || data_->predicted_objects->objects.empty()) return std::nullopt;
  auto predicted_objects = *data_->predicted_objects;

  filter_objects_by_type(predicted_objects, params_.objects.object_types);
  filter_objects_by_velocity(predicted_objects, params_.objects.max_velocity_th);

  return get_nearest_object_collision(traj_points, trajectory_polygon, predicted_objects);
}

std::optional<CollisionPoint> ObstacleStop::check_pointcloud(
  const TrajectoryPoints & traj_points, const MultiPolygon2d & trajectory_polygon)
{
  const auto & pointcloud = data_->obstacle_pointcloud;
  if (pointcloud->data.empty()) return std::nullopt;

  PointCloud::Ptr filtered_pointcloud(new PointCloud);
  pcl::fromROSMsg(*pointcloud, *filtered_pointcloud);

  const auto & p = params_.pointcloud;

  const auto max_height = data_->vehicle_info.vehicle_height_m + p.height_buffer;
  filter_pointcloud_by_range(filtered_pointcloud, p.detection_range, p.min_height, max_height);

  {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = data_->tf_buffer.lookupTransform(
        "map", pointcloud->header.frame_id, pointcloud->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(get_node_ptr()->get_logger(), "no transform found for pointcloud: %s", e.what());
    }

    Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
    autoware_utils::transform_pointcloud(*filtered_pointcloud, *filtered_pointcloud, isometry);
  }

  const auto & p_voxel = p.voxel_grid_filter;
  filter_pointcloud_by_voxel_grid(
    filtered_pointcloud, p_voxel.x, p_voxel.y, p_voxel.z, p_voxel.min_size);

  const auto & p_cluster = p.clustering;
  const auto height_offset = data_->current_odometry->pose.pose.position.z;
  PointCloud::Ptr clustered_points(new PointCloud);
  cluster_pointcloud(
    filtered_pointcloud, clustered_points, p_cluster.min_size, p_cluster.max_size,
    p_cluster.tolerance, p_cluster.min_height, height_offset);

  return get_nearest_pcd_collision(traj_points, trajectory_polygon, clustered_points);
}

// TODO(Quda): move logic to minimum rule based planner
// void ObstacleStop::update_collision_points_buffer(
//   const TrajectoryPoints & traj_points, const std::optional<CollisionPoint> & collision_point)
// {
//   constexpr double close_distance_threshold = 0.5;

//   const auto now = get_clock()->now();

//   std::optional<double> closest_distance_diff = std::nullopt;
//   collision_points_buffer_.erase(
//     std::remove_if(
//       collision_points_buffer_.begin(), collision_points_buffer_.end(),
//       [&](CollisionPoint & cp) {
//         if (cp.is_active && (now - cp.start_time).seconds() > params_.off_time_buffer) return
//         true; if (!collision_point && !cp.is_active) return true;

//         cp.arc_length = motion_utils::calcSignedArcLength(traj_points, 0LU, cp.point);
//         const auto distance_diff = collision_point ? collision_point->arc_length - cp.arc_length
//                                                    : std::numeric_limits<double>::max();
//         const bool is_close = abs(distance_diff) < close_distance_threshold;

//         if (!cp.is_active && !is_close) return true;

//         if (!cp.is_active && (now - cp.start_time).seconds() > params_.on_time_buffer) {
//           cp.is_active = true;
//           cp.start_time = now;
//         }

//         if (
//           is_close &&
//           (!closest_distance_diff || abs(distance_diff) < abs(*closest_distance_diff))) {
//           closest_distance_diff = distance_diff;
//         }

//         return false;
//       }),
//     collision_points_buffer_.end());

//   if (!collision_point) return;

//   constexpr double eps = 1e-3;
//   if (collision_points_buffer_.empty() || !closest_distance_diff) {
//     collision_points_buffer_.emplace_back(*collision_point, now, params_.on_time_buffer < eps);
//     return;
//   }

//   auto nearest_collision_point_it = std::find_if(
//     collision_points_buffer_.begin(), collision_points_buffer_.end(),
//     [&](const CollisionPoint & cp) {
//       return abs(cp.arc_length - collision_point->arc_length) < abs(*closest_distance_diff) +
//       eps;
//     });

//   if (nearest_collision_point_it == collision_points_buffer_.end()) return;

//   if (*closest_distance_diff < 0.0) {
//     nearest_collision_point_it->point = collision_point->point;
//     nearest_collision_point_it->arc_length = collision_point->arc_length;
//   }

//   if (nearest_collision_point_it->is_active) {
//     nearest_collision_point_it->start_time = now;
//   }
// }

// TODO(Quda): move logic to minimum rule based planner
// std::optional<CollisionPoint> ObstacleStop::get_nearest_collision_point() const
// {
//   std::optional<CollisionPoint> nearest_collision_point = std::nullopt;
//   if (collision_points_buffer_.empty()) return std::nullopt;

//   auto minimum_arc_length = std::numeric_limits<double>::max();
//   for (const auto & cp : collision_points_buffer_) {
//     if (cp.is_active > minimum_arc_length || !cp.is_active) continue;
//     nearest_collision_point = cp;
//     minimum_arc_length = cp.arc_length;
//   }
//   return nearest_collision_point;
// }

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
