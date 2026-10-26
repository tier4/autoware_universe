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

#include "autoware/control_validator/collision_validator/pointcloud_preprocessor.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/logging.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <boost/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::control_validator::collision_validator
{
namespace
{
using autoware_utils_geometry::Point3d;

struct Voxel
{
  Point3d centroid;
  size_t raw_point_count;
};

std::vector<Voxel> voxelize(
  const std::vector<Point3d> & cropped_pointcloud, const double voxel_size)
{
  constexpr int64_t kGridOffset = int64_t{1} << 20;
  constexpr uint64_t kAxisMask = (uint64_t{1} << 21) - 1;
  struct Accumulator
  {
    double sum_x{0.0};
    double sum_y{0.0};
    double sum_z{0.0};
    size_t count{0};
  };

  const double inverse_voxel_size = 1.0 / voxel_size;
  const auto grid_key = [&](const double coordinate) {
    return static_cast<uint64_t>(
             static_cast<int64_t>(std::floor(coordinate * inverse_voxel_size)) + kGridOffset) &
           kAxisMask;
  };

  std::unordered_map<uint64_t, size_t> index_of_key;
  index_of_key.reserve(cropped_pointcloud.size());
  std::vector<Accumulator> accumulators;
  for (const auto & p : cropped_pointcloud) {
    const uint64_t key = grid_key(p.x()) | (grid_key(p.y()) << 21) | (grid_key(p.z()) << 42);
    const auto [it, inserted] = index_of_key.try_emplace(key, accumulators.size());
    if (inserted) {
      accumulators.emplace_back();
    }
    auto & accumulator = accumulators[it->second];
    accumulator.sum_x += p.x();
    accumulator.sum_y += p.y();
    accumulator.sum_z += p.z();
    ++accumulator.count;
  }

  std::vector<Voxel> voxels;
  voxels.reserve(accumulators.size());
  for (const auto & accumulator : accumulators) {
    const auto n = static_cast<double>(accumulator.count);
    voxels.push_back(
      Voxel{
        Point3d{accumulator.sum_x / n, accumulator.sum_y / n, accumulator.sum_z / n},
        accumulator.count});
  }
  return voxels;
}

Polygon2d ego_footprint_base_link(const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const double front = vehicle_info.max_longitudinal_offset_m;
  const double rear = vehicle_info.min_longitudinal_offset_m;
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  Polygon2d footprint;
  footprint.outer() = {
    {front, half_width}, {front, -half_width}, {rear, -half_width}, {rear, half_width}};
  footprint.outer().push_back(footprint.outer().front());
  boost::geometry::correct(footprint);
  return footprint;
}
}  // namespace

PointCloudPreprocessor::PointCloudPreprocessor(
  const rclcpp::Logger & logger, const PointCloudParam & param,
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper)
: logger_(logger), time_keeper_(std::move(time_keeper)), param_(param)
{
}

void PointCloudPreprocessor::update_parameters(const PointCloudParam & param)
{
  param_ = param;
}

double PointCloudPreprocessor::z_min(const vehicle_info_utils::VehicleInfo & vehicle_info) const
{
  return vehicle_info.min_height_offset_m + param_.z_margin_low;
}

double PointCloudPreprocessor::z_max(const vehicle_info_utils::VehicleInfo & vehicle_info) const
{
  return vehicle_info.max_height_offset_m + param_.z_margin_high;
}

const std::optional<PointCloudClusters> & PointCloudPreprocessor::process(
  const sensor_msgs::msg::PointCloud2 & pointcloud,
  const std::optional<geometry_msgs::msg::Pose> & ego_pose_at_pointcloud_stamp, const double v_ego,
  const double horizon_s, const vehicle_info_utils::VehicleInfo & vehicle_info)
{
  const rclcpp::Time stamp(pointcloud.header.stamp);
  if (result_ && result_->stamp == stamp) {
    return result_;
  }
  if (pointcloud.header.frame_id != kExpectedFrame) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000,
      "hazard warning: pointcloud frame_id is '%s', expected '%s'. Frame dropped.",
      pointcloud.header.frame_id.c_str(), kExpectedFrame);
    return result_;
  }
  if (!ego_pose_at_pointcloud_stamp) {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 5000,
      "hazard warning: no odometry close to the pointcloud stamp. Frame dropped.");
    return result_;
  }
  if (pointcloud.width * pointcloud.height == 0) {
    return result_;
  }

  // Ego-footprint exclusion and ROI crop, reading the fields by name so that any point layout is
  // accepted. The height bounds come from vehicle_info (base_link is on the ground plane).
  const double x_min = -param_.roi_rear;
  const double x_max = std::max(v_ego, 0.0) * horizon_s + param_.roi_front_margin;
  const double z_low = z_min(vehicle_info);
  const double z_high = z_max(vehicle_info);
  const auto footprint = ego_footprint_base_link(vehicle_info);
  std::vector<Point3d> cropped_pointcloud;
  cropped_pointcloud.reserve(pointcloud.width * pointcloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> it_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(pointcloud, "z");
  time_keeper_->start_track("crop_and_voxelize");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    if (x < x_min || x > x_max) continue;
    if (std::abs(y) > param_.roi_lateral) continue;
    if (z < z_low || z >= z_high) continue;
    if (boost::geometry::within(Point2d{x, y}, footprint)) continue;
    cropped_pointcloud.emplace_back(x, y, z);
  }

  auto voxels = voxelize(cropped_pointcloud, param_.voxel_size);

  // Uniform subsampling of the voxels to bound the clustering time.
  if (voxels.size() > param_.max_voxels) {
    const size_t stride = (voxels.size() + param_.max_voxels - 1) / param_.max_voxels;
    std::vector<Voxel> subsampled;
    subsampled.reserve(voxels.size() / stride + 1);
    for (size_t i = 0; i < voxels.size(); i += stride) {
      subsampled.push_back(voxels[i]);
    }
    voxels = std::move(subsampled);
  }
  time_keeper_->end_track("crop_and_voxelize");

  time_keeper_->start_track("clustering");
  std::vector<PointCluster> clusters;
  if (!voxels.empty()) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_pointcloud->reserve(voxels.size());
    for (const auto & voxel : voxels) {
      voxel_pointcloud->push_back(
        pcl::PointXYZ(
          static_cast<float>(voxel.centroid.x()), static_cast<float>(voxel.centroid.y()),
          static_cast<float>(voxel.centroid.z())));
    }

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(voxel_pointcloud);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(param_.cluster_tolerance);
    extraction.setMinClusterSize(1);
    extraction.setMaxClusterSize(static_cast<int>(voxel_pointcloud->size()));
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(voxel_pointcloud);
    extraction.extract(cluster_indices);

    // Points of each cluster (base_link) -> map through the ego pose at the pointcloud stamp.
    // Points that belong to no cluster are dropped.
    const auto & ego_pose = *ego_pose_at_pointcloud_stamp;
    const double yaw = autoware_utils_geometry::get_rpy(ego_pose).z;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    clusters.reserve(cluster_indices.size());
    for (const auto & indices : cluster_indices) {
      size_t raw_point_count = 0;
      for (const auto index : indices.indices) {
        raw_point_count += voxels[static_cast<size_t>(index)].raw_point_count;
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
        const auto & p = voxels[static_cast<size_t>(index)].centroid;
        const double map_x = ego_pose.position.x + cos_yaw * p.x() - sin_yaw * p.y();
        const double map_y = ego_pose.position.y + sin_yaw * p.x() + cos_yaw * p.y();
        cluster.points.emplace_back(map_x, map_y);
        sum_x += map_x;
        sum_y += map_y;
      }
      const auto n = static_cast<double>(cluster.points.size());
      cluster.centroid = Point2d{sum_x / n, sum_y / n};
      clusters.push_back(std::move(cluster));
    }
  }
  time_keeper_->end_track("clustering");

  result_ = PointCloudClusters{std::move(clusters), stamp};
  return result_;
}
}  // namespace autoware::control_validator::collision_validator
