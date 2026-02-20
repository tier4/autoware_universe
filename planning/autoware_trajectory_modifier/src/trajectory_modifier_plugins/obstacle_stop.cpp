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
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin
{
using utils::obstacle_stop::cluster_pointcloud;
using utils::obstacle_stop::filter_pointcloud_by_range;
using utils::obstacle_stop::filter_pointcloud_by_voxel_grid;
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

bool ObstacleStop::is_trajectory_modification_required(const TrajectoryPoints & traj_points) const
{
  (void)traj_points;
  return false;
}

void ObstacleStop::modify_trajectory(TrajectoryPoints & traj_points)
{
  if (!enabled_ || !is_trajectory_modification_required(traj_points)) return;

  // TODO: Implement the obstacle stop logic
  return;
}

bool ObstacleStop::check_obstacles()
{
  return false;
}

bool ObstacleStop::check_predicted_objects()
{
  return false;
}

bool ObstacleStop::check_pointcloud()
{
  const auto & pointcloud = data_->obstacle_pointcloud;
  if (pointcloud->data.empty()) return false;

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

  return false;
}

}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::ObstacleStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
