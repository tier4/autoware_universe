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

#include "autoware/trajectory_modifier/trajectory_modifier_utils/obstacle_stop_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/transform/transforms.hpp>

#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>

namespace autoware::trajectory_modifier::utils::obstacle_stop
{

void filter_pointcloud_by_range(
  PointCloud::Ptr & pointcloud, const double detection_range, const double min_height,
  const double max_height)
{
  if (pointcloud->empty()) return;

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(pointcloud);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(-1.0 * detection_range, detection_range);
    filter.filter(*pointcloud);
  }

  if (pointcloud->empty()) return;

  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(pointcloud);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(min_height, max_height);
    filter.filter(*pointcloud);
  }
}

void filter_pointcloud_by_voxel_grid(
  PointCloud::Ptr & pointcloud, const double voxel_size_x, const double voxel_size_y,
  const double voxel_size_z, const int min_size)
{
  if (pointcloud->empty()) return;

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pointcloud);
  filter.setLeafSize(voxel_size_x, voxel_size_y, voxel_size_z);
  filter.setMinimumPointsNumberPerVoxel(min_size);
  filter.filter(*pointcloud);
}

void cluster_pointcloud(
  PointCloud::Ptr & input, PointCloud::Ptr & output, const size_t min_size, const size_t max_size,
  const double tolerance, const double min_height, const double height_offset)
{
  if (input->empty()) return;

  const std::vector<pcl::PointIndices> cluster_indices = std::invoke([&]() {
    std::vector<pcl::PointIndices> cluster_idx;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(tolerance);
    ec.setMinClusterSize(min_size);
    ec.setMaxClusterSize(max_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input);
    ec.extract(cluster_idx);
    return cluster_idx;
  });

  auto above_height_threshold = [&](const double z) {
    const auto rel_height = z - height_offset;
    return rel_height > min_height;
  };

  for (const auto & indices : cluster_indices) {
    PointCloud::Ptr cluster(new PointCloud);
    bool cluster_above_height_threshold{false};
    for (const auto & index : indices.indices) {
      const auto & point = (*input)[index];

      cluster_above_height_threshold |= above_height_threshold(point.z);
      cluster->push_back(point);
    }
    if (!cluster_above_height_threshold) continue;

    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setDimension(2);
    hull.setInputCloud(cluster);
    PointCloud::Ptr surface_hull(new PointCloud);
    hull.reconstruct(*surface_hull);
    for (const auto & point : *surface_hull) {
      output->push_back(point);
    }
  }
}

void filter_objects_by_type(
  PredictedObjects & objects, const std::vector<std::string> & object_type_strings)
{
  const std::vector<ObjectType> object_types = std::invoke([&]() {
    std::vector<ObjectType> object_types;
    object_types.reserve(object_type_strings.size());
    for (const auto & object_type_string : object_type_strings) {
      object_types.push_back(string_to_object_type.at(object_type_string));
    }
    return object_types;
  });

  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const auto & object) {
        return std::find(
                 object_types.begin(), object_types.end(),
                 classification_to_object_type.at(object.classification.front().label)) ==
               object_types.end();
      }),
    objects.objects.end());
}

void filter_objects_by_velocity(PredictedObjects & objects, const double max_velocity)
{
  objects.objects.erase(
    std::remove_if(
      objects.objects.begin(), objects.objects.end(),
      [&](const auto & object) {
        return object.kinematics.initial_twist_with_covariance.twist.linear.x > max_velocity;
      }),
    objects.objects.end());
}

}  // namespace autoware::trajectory_modifier::utils::obstacle_stop
