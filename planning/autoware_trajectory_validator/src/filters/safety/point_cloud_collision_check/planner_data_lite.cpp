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

#include "planner_data_lite.hpp"

#include <Eigen/Geometry>
#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils_pcl/transforms.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
namespace
{
rclcpp::Logger get_logger()
{
  return rclcpp::get_logger("PointCloudCollisionCheckFilter");
}

// motion_velocity_planner_common/planner_data.cpp:88-132
pcl::PointCloud<pcl::PointXYZ>::Ptr crop_by_monolithic_trajectory_polygon(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const PointcloudPreprocessParams::FilterByTrajectoryPolygon & filter_by_trajectory_param,
  const std::vector<autoware_utils_geometry::Polygon2d> & traj_polygons,
  const std::vector<TrajectoryPoint> & decimated_trajectory,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  pcl::CropBox<pcl::PointXYZ> crop_filter;
  crop_filter.setInputCloud(input_pointcloud_ptr);

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();
  for (const auto & poly : traj_polygons) {
    for (const auto & point : poly.outer()) {
      x_min = std::min(x_min, point[0]);
      x_max = std::max(x_max, point[0]);
      y_min = std::min(y_min, point[1]);
      y_max = std::max(y_max, point[1]);
    }
  }
  auto lowest_traj_height = std::numeric_limits<double>::max();
  auto highest_traj_height = std::numeric_limits<double>::lowest();
  for (const auto & trajectory_point : decimated_trajectory) {
    lowest_traj_height = std::min(lowest_traj_height, trajectory_point.pose.position.z);
    highest_traj_height = std::max(highest_traj_height, trajectory_point.pose.position.z);
  }
  crop_filter.setMin(
    Eigen::Vector4f(
      x_min, y_min, lowest_traj_height - filter_by_trajectory_param.height_margin, 1.0f));
  crop_filter.setMax(
    Eigen::Vector4f(
      x_max, y_max,
      highest_traj_height + vehicle_info.vehicle_height_m +
        filter_by_trajectory_param.height_margin,
      1.0f));

  auto ret_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  crop_filter.filter(*ret_pointcloud_ptr);
  return ret_pointcloud_ptr;
}

// motion_velocity_planner_common/planner_data.cpp:190-202
pcl::PointCloud<pcl::PointXYZ>::Ptr downsample_by_voxel_grid(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const PointcloudPreprocessParams::DownsampleByVoxelGrid & downsample_params)
{
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input_pointcloud_ptr);
  filter.setLeafSize(
    downsample_params.voxel_size_x, downsample_params.voxel_size_y, downsample_params.voxel_size_z);
  auto ret_pointcloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  filter.filter(*ret_pointcloud_ptr);
  return ret_pointcloud_ptr;
}

// motion_velocity_planner_common/planner_data.cpp:205-222
std::vector<pcl::PointIndices> make_cluster_indices(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const PointcloudPreprocessParams::EuclideanClustering & clustering_params)
{
  std::vector<pcl::PointIndices> ret_clusters{};
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(input_pointcloud_ptr);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(clustering_params.cluster_tolerance);
  ec.setMinClusterSize(clustering_params.min_cluster_size);
  ec.setMaxClusterSize(clustering_params.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_pointcloud_ptr);
  ec.extract(ret_clusters);
  return ret_clusters;
}

// motion_velocity_planner_common/planner_data.cpp:225-236
std::vector<pcl::PointIndices> make_individual_cluster_indices(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr)
{
  std::vector<pcl::PointIndices> ret_clusters{};
  ret_clusters.resize(input_pointcloud_ptr->size());
  for (size_t i = 0; i < input_pointcloud_ptr->size(); ++i) {
    ret_clusters[i].indices.emplace_back(i);
  }
  return ret_clusters;
}

}  // namespace

// motion_velocity_planner/node.cpp:229-259
std::optional<pcl::PointCloud<pcl::PointXYZ>> process_no_ground_pointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return std::nullopt;
  }

  pcl::PointCloud<pcl::PointXYZ> pc_input;
  pcl::fromROSMsg(*msg, pc_input);

  const Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ> pc_transformed;
  if (!pc_input.empty()) autoware_utils_pcl::transform_pointcloud(pc_input, pc_transformed, affine);

  pc_transformed.header = pc_input.header;
  pc_transformed.header.frame_id = "map";

  return pc_transformed;
}

// motion_velocity_planner_common/planner_data.cpp:279-286
// 差分: velocity_smoother_ の実体の代わりに VelocitySmoother（min_decel/min_jerk のみ）を使う。
std::optional<double> PlannerData::calculate_min_deceleration_distance(
  const double target_velocity) const
{
  return autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    std::abs(current_odometry.twist.twist.linear.x), target_velocity,
    current_acceleration.accel.accel.linear.x, velocity_smoother_.getMinDecel(),
    std::abs(velocity_smoother_.getMinJerk()), velocity_smoother_.getMinJerk());
}

// motion_velocity_planner_common/planner_data.cpp:464-534
// 差分: enable_* フラグと multi_polygon 経路を削除し、常に monolithic crop のみ行う。
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>>
PlannerData::Pointcloud::filter_and_cluster_point_clouds(
  const std::vector<TrajectoryPoint> & raw_trajectory,
  const nav_msgs::msg::Odometry & current_odometry, double min_deceleration_distance,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret_pointcloud_ptr = pointcloud.makeShared();
  std::vector<pcl::PointIndices> ret_clusters{};

  const auto & filter_by_trajectory_param = preprocess_params_.filter_by_trajectory_polygon;
  const auto & traj_poly_param = trajectory_polygon_collision_check;
  if (!raw_trajectory.empty()) {
    const auto decimated_trajectory =
      autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
        raw_trajectory, current_odometry.pose.pose, ego_nearest_dist_threshold,
        ego_nearest_yaw_threshold, traj_poly_param.decimate_trajectory_step_length,
        traj_poly_param.goal_extended_trajectory_length);

    const double trajectory_trim_length =
      filter_by_trajectory_param.min_trajectory_length +
      min_deceleration_distance * filter_by_trajectory_param.braking_distance_scale_factor;
    const auto & trimmed_trajectory =
      autoware::motion_utils::isDrivingForward(raw_trajectory) == true
        ? autoware::motion_utils::cropForwardPoints(
            decimated_trajectory, decimated_trajectory.front().pose.position, 0,
            trajectory_trim_length)
        : decimated_trajectory;

    const auto traj_polygons =
      autoware::motion_velocity_planner::polygon_utils::create_one_step_polygons(
        trimmed_trajectory, vehicle_info, current_odometry.pose.pose,
        filter_by_trajectory_param.lateral_margin, traj_poly_param.enable_to_consider_current_pose,
        traj_poly_param.time_to_convergence, traj_poly_param.decimate_trajectory_step_length);

    if (!ret_pointcloud_ptr->empty()) {
      const auto input_pointcloud_ptr = ret_pointcloud_ptr;
      ret_pointcloud_ptr = crop_by_monolithic_trajectory_polygon(
        input_pointcloud_ptr, filter_by_trajectory_param, traj_polygons, decimated_trajectory,
        vehicle_info);
    }
  }

  const auto & downsample_params = preprocess_params_.downsample_by_voxel_grid;
  if (downsample_params.enable_downsample && !ret_pointcloud_ptr->empty()) {
    const auto input_pointcloud_ptr = ret_pointcloud_ptr;
    ret_pointcloud_ptr = downsample_by_voxel_grid(input_pointcloud_ptr, downsample_params);
  }

  const auto & clustering_param = preprocess_params_.euclidean_clustering;
  if (clustering_param.enable_clustering && !ret_pointcloud_ptr->empty()) {
    ret_clusters = make_cluster_indices(ret_pointcloud_ptr, clustering_param);
  } else {
    ret_clusters = make_individual_cluster_indices(ret_pointcloud_ptr);
  }
  return std::make_pair(ret_pointcloud_ptr, ret_clusters);
}

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
