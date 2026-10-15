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

#include "hazard/collision_target.hpp"

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
#include <limits>
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
  const auto raw_cloud = roi_cloud;

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
    std::vector<size_t> voxel_counts;
    if (param_.voxel_size > 0.0) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(roi_cloud);
      const auto leaf = static_cast<float>(param_.voxel_size);
      voxel_grid.setLeafSize(leaf, leaf, leaf);
      voxel_grid.setSaveLeafLayout(true);
      voxel_grid.filter(*voxel_cloud);
      voxel_counts.assign(voxel_cloud->size(), 0);
      const Eigen::Vector3i min_b = voxel_grid.getMinBoxCoordinates();
      const Eigen::Vector3i max_b = voxel_grid.getMaxBoxCoordinates();
      for (const auto & p : raw_cloud->points) {
        const Eigen::Vector3i ijk = voxel_grid.getGridCoordinates(p.x, p.y, p.z);
        if ((ijk.array() < min_b.array()).any() || (ijk.array() > max_b.array()).any()) {
          continue;
        }
        const int index = voxel_grid.getCentroidIndexAt(ijk);
        if (index >= 0 && static_cast<size_t>(index) < voxel_counts.size()) {
          ++voxel_counts[static_cast<size_t>(index)];
        }
      }
    } else {
      voxel_cloud = roi_cloud;
      voxel_counts.assign(voxel_cloud->size(), 1);
    }

    std::vector<pcl::PointIndices> cluster_indices;
    if (!voxel_cloud->empty()) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(voxel_cloud);
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
      extraction.setClusterTolerance(param_.cluster_tolerance);
      extraction.setMinClusterSize(1);
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
      size_t raw_point_count = 0;
      for (const auto index : indices.indices) {
        raw_point_count += voxel_counts[static_cast<size_t>(index)];
      }
      if (raw_point_count < param_.min_cluster_size) {
        continue;
      }
      PointCluster cluster;
      cluster.raw_point_count = raw_point_count;
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

std::vector<std::optional<size_t>> attach_clusters_to_objects(
  const std::vector<PointCluster> & clusters,
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const std::vector<std::shared_ptr<const ObjectMotion>> & motions,
  const std::vector<uint8_t> & enabled_labels, const double t_cloud, const ObjectMargin & margin)
{
  using autoware_perception_msgs::msg::Shape;
  struct ObjectArea
  {
    size_t index;
    Polygon2d polygon;
    Point2d origin;
  };
  std::vector<ObjectArea> areas;
  areas.reserve(objects.objects.size());
  for (size_t i = 0; i < objects.objects.size(); ++i) {
    const auto & object = objects.objects[i];
    if (i >= motions.size() || !motions[i]) {
      continue;
    }
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
    const auto pose = motions[i]->pose_at(t_cloud);
    const double yaw = autoware_utils_geometry::get_rpy(pose).z;

    const auto & shape = object.shape;
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;
    if (shape.type == Shape::BOUNDING_BOX) {
      x_max = 0.5 * shape.dimensions.x;
      x_min = -x_max;
      y_max = 0.5 * shape.dimensions.y;
      y_min = -y_max;
    } else if (shape.type == Shape::CYLINDER) {
      x_max = y_max = 0.5 * shape.dimensions.x;
      x_min = y_min = -x_max;
    } else {
      if (shape.footprint.points.empty()) {
        continue;
      }
      x_min = y_min = std::numeric_limits<double>::infinity();
      x_max = y_max = -std::numeric_limits<double>::infinity();
      for (const auto & p : shape.footprint.points) {
        x_min = std::min(x_min, static_cast<double>(p.x));
        x_max = std::max(x_max, static_cast<double>(p.x));
        y_min = std::min(y_min, static_cast<double>(p.y));
        y_max = std::max(y_max, static_cast<double>(p.y));
      }
    }
    x_max += margin.front;
    x_min -= margin.rear;
    y_max += margin.left;
    y_min -= margin.right;

    Polygon2d polygon;
    for (const auto & [lx, ly] :
         {std::pair{x_max, y_max}, std::pair{x_min, y_max}, std::pair{x_min, y_min},
          std::pair{x_max, y_min}}) {
      polygon.outer().emplace_back(
        pose.position.x + std::cos(yaw) * lx - std::sin(yaw) * ly,
        pose.position.y + std::sin(yaw) * lx + std::cos(yaw) * ly);
    }
    boost::geometry::correct(polygon);
    areas.push_back(ObjectArea{i, std::move(polygon), Point2d{pose.position.x, pose.position.y}});
  }

  std::vector<std::optional<size_t>> attached_to;
  attached_to.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    std::optional<size_t> best;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto & area : areas) {
      if (!boost::geometry::within(cluster.centroid, area.polygon)) {
        continue;
      }
      const double distance = boost::geometry::distance(cluster.centroid, area.origin);
      if (distance < best_distance) {
        best_distance = distance;
        best = area.index;
      }
    }
    attached_to.push_back(best);
  }
  return attached_to;
}
}  // namespace autoware::control_validator::hazard
