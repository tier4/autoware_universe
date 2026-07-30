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

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
namespace
{
pcl::PointCloud<pcl::PointXYZ>::Ptr crop_by_monolithic_trajectory_polygon(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_pointcloud_ptr,
  const PointcloudPreprocessParams::FilterByTrajectoryPolygon & filter_by_trajectory_param,
  const std::vector<Polygon2d> & traj_polygons,
  const std::vector<TrajectoryPoint> & decimated_trajectory, const VehicleInfo & vehicle_info)
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

// [plugin 固有] class_id フィールド（UINT8、ptv3 出力）が在るときのみ
// excluded_class_ids の点を除外する。無ければ全点を通す。
pcl::PointCloud<pcl::PointXYZ> filter_pointcloud_by_class_id(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::vector<std::int64_t> & excluded_class_ids)
{
  pcl::PointCloud<pcl::PointXYZ> out;
  out.header = pcl_conversions::toPCL(cloud.header);
  const size_t num_points = static_cast<size_t>(cloud.width) * cloud.height;
  if (num_points == 0) {
    return out;
  }

  const bool has_class_id = std::any_of(
    cloud.fields.begin(), cloud.fields.end(), [](const auto & field) {
      return field.name == "class_id" &&
             field.datatype == sensor_msgs::msg::PointField::UINT8;
    });
  const std::unordered_set<std::int64_t> excluded(
    excluded_class_ids.begin(), excluded_class_ids.end());

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
  std::optional<sensor_msgs::PointCloud2ConstIterator<std::uint8_t>> it_class;
  if (has_class_id && !excluded.empty()) {
    it_class.emplace(cloud, "class_id");
  }

  out.points.reserve(num_points);
  for (size_t i = 0; i < num_points; ++i, ++it_x, ++it_y, ++it_z) {
    if (it_class) {
      const auto class_id = static_cast<std::int64_t>(**it_class);
      ++(*it_class);
      if (excluded.count(class_id) > 0) {
        continue;
      }
    }
    if (!std::isfinite(*it_x) || !std::isfinite(*it_y) || !std::isfinite(*it_z)) {
      continue;
    }
    out.points.emplace_back(*it_x, *it_y, *it_z);
  }
  out.width = out.points.size();
  out.height = 1;
  out.is_dense = false;
  return out;
}

// motion_velocity_planner/node.cpp:250-258（process_no_ground_pointcloud の変換部）
pcl::PointCloud<pcl::PointXYZ> transform_pointcloud_to_map_frame(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const geometry_msgs::msg::Pose & base_link_to_map)
{
  const auto & p = base_link_to_map.position;
  const auto & q = base_link_to_map.orientation;
  const Eigen::Affine3d base_link_to_map_affine =
    Eigen::Translation3d(p.x, p.y, p.z) * Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
  const Eigen::Affine3f affine = base_link_to_map_affine.cast<float>();

  pcl::PointCloud<pcl::PointXYZ> pc_transformed;
  if (!cloud.empty()) {
    autoware_utils_pcl::transform_pointcloud(cloud, pc_transformed, affine);
  }

  pc_transformed.header = cloud.header;
  pc_transformed.header.frame_id = "map";

  return pc_transformed;
}

std::optional<double> PlannerData::calculate_min_deceleration_distance(
  const double target_velocity) const
{
  return autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    std::abs(current_odometry.twist.twist.linear.x), target_velocity,
    current_acceleration.accel.accel.linear.x, min_accel, std::abs(min_jerk), min_jerk);
}

std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>>
PlannerData::Pointcloud::filter_and_cluster_point_clouds(
  const std::vector<TrajectoryPoint> & raw_trajectory,
  const nav_msgs::msg::Odometry & current_odometry, double min_deceleration_distance,
  const VehicleInfo & vehicle_info,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr ret_pointcloud_ptr = pointcloud.makeShared();
  std::vector<pcl::PointIndices> ret_clusters{};

  const auto & filter_by_trajectory_param = preprocess_params_.filter_by_trajectory_polygon;
  const auto & traj_poly_param = trajectory_polygon_collision_check;
  if (!raw_trajectory.empty() && !ret_pointcloud_ptr->empty()) {
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

    const auto input_pointcloud_ptr = ret_pointcloud_ptr;
    ret_pointcloud_ptr = crop_by_monolithic_trajectory_polygon(
      input_pointcloud_ptr, filter_by_trajectory_param, traj_polygons, decimated_trajectory,
      vehicle_info);
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
