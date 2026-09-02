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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__PLANNER_DATA_LITE_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__PLANNER_DATA_LITE_HPP_

#include "types.hpp"

#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PointIndices.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using VehicleInfo = autoware::vehicle_info_utils::VehicleInfo;

struct TrajectoryPolygonCollisionCheck
{
  double decimate_trajectory_step_length{};
  double goal_extended_trajectory_length{};
  bool enable_to_consider_current_pose{};
  double time_to_convergence{};
};

/// velocity_smoother_ の代替。smoother 本体は持たず、
/// calculate_min_deceleration_distance が読む min_decel / min_jerk だけ持つ。
struct VelocitySmoother
{
  double min_decel{};
  double min_jerk{};

  double getMinDecel() const { return min_decel; }
  double getMinJerk() const { return min_jerk; }
};

struct PointcloudPreprocessParams
{
  PointcloudPreprocessParams() = default;
  explicit PointcloudPreprocessParams(const validator::Params::PointCloudCollisionCheck & p)
  {
    const auto & pp = p.pointcloud_preprocessing;
    filter_by_trajectory_polygon.min_trajectory_length =
      pp.filter_by_trajectory_polygon.min_trajectory_length;
    filter_by_trajectory_polygon.braking_distance_scale_factor =
      pp.filter_by_trajectory_polygon.braking_distance_scale_factor;
    filter_by_trajectory_polygon.lateral_margin = pp.filter_by_trajectory_polygon.lateral_margin;
    filter_by_trajectory_polygon.height_margin = pp.filter_by_trajectory_polygon.height_margin;

    downsample_by_voxel_grid.enable_downsample = pp.downsample_by_voxel_grid.enable_downsample;
    downsample_by_voxel_grid.voxel_size_x = pp.downsample_by_voxel_grid.voxel_size_x;
    downsample_by_voxel_grid.voxel_size_y = pp.downsample_by_voxel_grid.voxel_size_y;
    downsample_by_voxel_grid.voxel_size_z = pp.downsample_by_voxel_grid.voxel_size_z;

    euclidean_clustering.enable_clustering = pp.euclidean_clustering.enable_clustering;
    euclidean_clustering.cluster_tolerance = pp.euclidean_clustering.cluster_tolerance;
    euclidean_clustering.min_cluster_size =
      static_cast<int>(pp.euclidean_clustering.min_cluster_size);
    euclidean_clustering.max_cluster_size =
      static_cast<int>(pp.euclidean_clustering.max_cluster_size);
  }

  struct FilterByTrajectoryPolygon
  {
    double min_trajectory_length{};
    double braking_distance_scale_factor{};
    double lateral_margin{};
    double height_margin{};
  } filter_by_trajectory_polygon;
  struct DownsampleByVoxelGrid
  {
    bool enable_downsample{false};
    double voxel_size_x{};
    double voxel_size_y{};
    double voxel_size_z{};
  } downsample_by_voxel_grid;
  struct EuclideanClustering
  {
    bool enable_clustering{false};
    double cluster_tolerance{};
    int min_cluster_size{};
    int max_cluster_size{};
  } euclidean_clustering;
};

/// @brief ptv3 PointCloud2 を名前ベースで読み、入力 frame のまま pcl 点群にする。
/// class_id（UINT8）フィールドが在るときのみ excluded_class_ids の点を除外する。
pcl::PointCloud<pcl::PointXYZ> filter_pointcloud_by_class_id(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::vector<std::int64_t> & excluded_class_ids);

/// @brief 点群を map 系へ変換する。移植元は TF を引くが、plugin は TF buffer を持たないため
/// odometry.pose（base_link→map）を affine として使う。
pcl::PointCloud<pcl::PointXYZ> transform_pointcloud_to_map_frame(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const geometry_msgs::msg::Pose & base_link_to_map);

struct PlannerData
{
public:
  PlannerData() = default;
  PlannerData(const PlannerData &) = delete;
  PlannerData & operator=(const PlannerData &) = delete;
  PlannerData(PlannerData &&) = default;
  PlannerData & operator=(PlannerData &&) = default;

  class Pointcloud
  {
  public:
    void preprocess_pointcloud(
      pcl::PointCloud<pcl::PointXYZ> && arg_pointcloud,
      const std::vector<TrajectoryPoint> & raw_trajectory,
      const nav_msgs::msg::Odometry & current_odometry, double min_deceleration_distance,
      const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
      const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check,
      const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
    {
      pointcloud = arg_pointcloud;
      const auto preprocessed_result = filter_and_cluster_point_clouds(
        raw_trajectory, current_odometry, min_deceleration_distance, vehicle_info,
        trajectory_polygon_collision_check, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
      filtered_pointcloud_ptr = preprocessed_result.first;
      cluster_indices = preprocessed_result.second;
    }

    pcl::PointCloud<pcl::PointXYZ> pointcloud;

    const pcl::PointCloud<pcl::PointXYZ>::Ptr get_filtered_pointcloud_ptr() const
    {
      if (!filtered_pointcloud_ptr) {
        throw std::runtime_error(
          "Filtered pointcloud pointer is not set. Please call preprocess_pointcloud() first.");
      }
      return filtered_pointcloud_ptr.value();
    }
    const std::vector<pcl::PointIndices> get_cluster_indices() const
    {
      if (!cluster_indices) {
        throw std::runtime_error(
          "Cluster indices are not set. Please call preprocess_pointcloud() first.");
      }
      return cluster_indices.value();
    }
    pcl::PointCloud<pcl::PointXYZ> extract_clustered_points() const
    {
      const auto & clusters = get_cluster_indices();
      const auto & source_cloud_ptr = get_filtered_pointcloud_ptr();

      std::vector<int> combined_indices;
      size_t total_points = 0;
      for (const auto & cluster : clusters) {
        total_points += cluster.indices.size();
      }
      combined_indices.reserve(total_points);

      for (const auto & cluster : clusters) {
        combined_indices.insert(
          combined_indices.end(), cluster.indices.begin(), cluster.indices.end());
      }

      pcl::PointCloud<pcl::PointXYZ> extracted_cloud;
      pcl::copyPointCloud(*source_cloud_ptr, combined_indices, extracted_cloud);

      return extracted_cloud;
    }

    PointcloudPreprocessParams preprocess_params_;

  private:
    std::optional<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered_pointcloud_ptr;
    std::optional<std::vector<pcl::PointIndices>> cluster_indices;

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, std::vector<pcl::PointIndices>>
    filter_and_cluster_point_clouds(
      const std::vector<TrajectoryPoint> & raw_trajectory,
      const nav_msgs::msg::Odometry & current_odometry, double min_deceleration_distance,
      const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
      const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check,
      const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold);
  };

  nav_msgs::msg::Odometry current_odometry{};
  geometry_msgs::msg::AccelWithCovarianceStamped current_acceleration{};
  Pointcloud no_ground_pointcloud{};
  VehicleInfo vehicle_info_{};
  bool is_driving_forward{true};
  double ego_nearest_dist_threshold{};
  double ego_nearest_yaw_threshold{};
  TrajectoryPolygonCollisionCheck trajectory_polygon_collision_check{};
  VelocitySmoother velocity_smoother_{};

  // [plugin 固有] class_id フィールドを持つ入力点群から除外するクラス。
  std::vector<std::int64_t> excluded_class_ids{};

  std::optional<double> calculate_min_deceleration_distance(double target_velocity) const;
};

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__PLANNER_DATA_LITE_HPP_
