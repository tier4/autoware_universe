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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINTCLOUD_PREPROCESSING_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINTCLOUD_PREPROCESSING_HPP_

#include "types.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using VehicleInfo = autoware::vehicle_info_utils::VehicleInfo;

// 点群前処理パラメータ（core PointcloudPreprocessParams から Node 結合を外した版・決定7）。
struct PointcloudPreprocessParams
{
  struct FilterByTrajectoryPolygon
  {
    bool enable_monolithic_crop_box{false};
    bool enable_multi_polygon_filtering{false};
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

// map 変換・クラスタリング済みの前処理結果（core PlannerData::Pointcloud の Node 非結合版・決定7）。
struct PreprocessedPointcloud
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud_ptr;
  std::vector<pcl::PointIndices> cluster_indices;
};

/// @brief ptv3 PointCloud2 を名前ベースで読み、base_link→map 変換して pcl 点群に変換する（決定6・9）。
/// class_id フィールドが在るときのみ excluded_class_ids に含まれる点を除外する（無ければ素通し・決定4）。
pcl::PointCloud<pcl::PointXYZ> convert_pointcloud_to_map_frame(
  const sensor_msgs::msg::PointCloud2 & cloud, const geometry_msgs::msg::Pose & base_link_to_map,
  const std::vector<std::int64_t> & excluded_class_ids);

/// @brief 検出ポリゴンで絞り込み→voxel downsample→euclidean clustering（core filter_and_cluster 相当）。
/// traj_polygons / decimated_trajectory は呼び出し側で 1 回だけ生成した decimate 済み軌道から作る（決定3）。
PreprocessedPointcloud filter_and_cluster_point_clouds(
  const pcl::PointCloud<pcl::PointXYZ> & map_pointcloud,
  const std::vector<Polygon2d> & traj_polygons,
  const std::vector<TrajectoryPoint> & decimated_trajectory, const VehicleInfo & vehicle_info,
  const PointcloudPreprocessParams & params);

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINTCLOUD_PREPROCESSING_HPP_
