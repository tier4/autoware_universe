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

#include "hazard/pointcloud_preprocessor.hpp"

#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <boost/geometry.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::control_validator::hazard
{
PointCloudPreprocessor::PointCloudPreprocessor(
  const PointCloudParam & param, const vehicle_info_utils::VehicleInfo & vehicle_info)
: param_(param), vehicle_info_(vehicle_info)
{
}

void PointCloudPreprocessor::update_parameters(const PointCloudParam & param)
{
  param_ = param;
}

bool PointCloudPreprocessor::process(
  const sensor_msgs::msg::PointCloud2 & cloud, const geometry_msgs::msg::Pose & ego_at_cloud,
  const double v_ego, const double horizon_s, const Polygon2d & ego_footprint_base_link)
{
  const rclcpp::Time stamp(cloud.header.stamp);
  if (last_stamp_.has_value() && *last_stamp_ == stamp) {
    return false;
  }
  if (cloud.header.frame_id != kExpectedFrame) {
    return false;
  }
  if (cloud.width * cloud.height == 0) {
    return false;
  }

  // Ego-footprint exclusion and ROI crop, reading the fields by name so that any point layout is
  // accepted. The height bounds come from vehicle_info (base_link is on the ground plane).
  const double x_min = -param_.roi_rear;
  const double x_max = std::max(v_ego, 0.0) * horizon_s + param_.roi_front_margin;
  const double z_low = z_min();
  const double z_high = z_max();
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  roi_cloud->reserve(cloud.width * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    if (x < x_min || x > x_max) continue;
    if (std::abs(y) > param_.roi_lateral) continue;
    if (z < z_low || z >= z_high) continue;
    if (boost::geometry::within(Point2d{x, y}, ego_footprint_base_link)) continue;
    roi_cloud->push_back(pcl::PointXYZ(x, y, z));
  }
  last_roi_point_count_ = roi_cloud->size();

  // Uniform subsampling to bound the clustering time.
  if (param_.max_points > 0 && roi_cloud->size() > param_.max_points) {
    const size_t stride =
      (roi_cloud->size() + param_.max_points - 1) / param_.max_points;  // ceil(n / max_points)
    pcl::PointCloud<pcl::PointXYZ>::Ptr subsampled(new pcl::PointCloud<pcl::PointXYZ>);
    subsampled->reserve(roi_cloud->size() / stride + 1);
    for (size_t i = 0; i < roi_cloud->size(); i += stride) {
      subsampled->push_back((*roi_cloud)[i]);
    }
    roi_cloud = subsampled;
  }

  std::vector<PointCluster> clusters;
  if (!roi_cloud->empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (param_.voxel_size > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(roi_cloud);
      const auto leaf = static_cast<float>(param_.voxel_size);
      voxel_grid.setLeafSize(leaf, leaf, leaf);
      voxel_grid.filter(*voxel_cloud);
    } else {
      voxel_cloud = roi_cloud;
    }

    std::vector<pcl::PointIndices> cluster_indices;
    if (!voxel_cloud->empty()) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(voxel_cloud);
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
      extraction.setClusterTolerance(param_.cluster_tolerance);
      extraction.setMinClusterSize(static_cast<int>(std::max<size_t>(param_.min_cluster_size, 1)));
      extraction.setMaxClusterSize(static_cast<int>(voxel_cloud->size()));
      extraction.setSearchMethod(tree);
      extraction.setInputCloud(voxel_cloud);
      extraction.extract(cluster_indices);
    }

    // Points of each cluster (base_link) -> map through the ego pose at the cloud stamp. Points
    // that belong to no cluster are dropped.
    const double yaw = autoware_utils_geometry::get_rpy(ego_at_cloud).z;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    clusters.reserve(cluster_indices.size());
    for (const auto & indices : cluster_indices) {
      if (indices.indices.empty()) {
        continue;
      }
      PointCluster cluster;
      cluster.points.reserve(indices.indices.size());
      double sum_x = 0.0;
      double sum_y = 0.0;
      for (const auto index : indices.indices) {
        const auto & p = (*voxel_cloud)[index];
        const double mx = ego_at_cloud.position.x + cos_yaw * p.x - sin_yaw * p.y;
        const double my = ego_at_cloud.position.y + sin_yaw * p.x + cos_yaw * p.y;
        cluster.points.emplace_back(mx, my);
        sum_x += mx;
        sum_y += my;
      }
      const auto n = static_cast<double>(cluster.points.size());
      cluster.centroid = Point2d{sum_x / n, sum_y / n};
      clusters.push_back(std::move(cluster));
    }
  }

  clusters_ = std::move(clusters);
  last_stamp_ = stamp;
  return true;
}

std::vector<PointCluster> remove_clusters_inside_objects(
  const std::vector<PointCluster> & clusters,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const std::vector<uint8_t> & enabled_labels, const builtin_interfaces::msg::Time & cloud_stamp)
{
  const double dt = (rclcpp::Time(cloud_stamp) - rclcpp::Time(objects.header.stamp)).seconds();
  std::vector<Polygon2d> object_polygons;
  object_polygons.reserve(objects.objects.size());
  for (const auto & object : objects.objects) {
    uint8_t label = 0;
    float best_probability = -1.0F;
    for (const auto & classification : object.classification) {
      if (classification.probability > best_probability) {
        best_probability = classification.probability;
        label = classification.label;
      }
    }
    const bool enabled =
      std::find(enabled_labels.begin(), enabled_labels.end(), label) != enabled_labels.end();
    if (!enabled) {
      continue;
    }
    // Move the object to the pointcloud time with its twist (object frame), ignoring the yaw rate.
    auto pose = object.kinematics.initial_pose_with_covariance.pose;
    const auto & twist = object.kinematics.initial_twist_with_covariance.twist.linear;
    const double yaw = autoware_utils_geometry::get_rpy(pose).z;
    pose.position.x += (std::cos(yaw) * twist.x - std::sin(yaw) * twist.y) * dt;
    pose.position.y += (std::sin(yaw) * twist.x + std::cos(yaw) * twist.y) * dt;
    object_polygons.push_back(autoware_utils_geometry::to_polygon2d(pose, object.shape));
  }

  std::vector<PointCluster> kept;
  kept.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    const bool duplicates_object = std::any_of(
      object_polygons.begin(), object_polygons.end(),
      [&](const auto & polygon) { return boost::geometry::within(cluster.centroid, polygon); });
    if (!duplicates_object) {
      kept.push_back(cluster);
    }
  }
  return kept;
}
}  // namespace autoware::control_validator::hazard
