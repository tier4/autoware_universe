// Copyright 2021 TIER IV, Inc.
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

#include "autoware/lidar_centerpoint/preprocess/pointcloud_densification.hpp"

#include <pcl_ros/transforms.hpp>

#include <boost/optional.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <utility>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

namespace autoware::lidar_centerpoint
{
PointCloudDensification::PointCloudDensification(const DensificationParam & param) : param_(param)
{
}

bool PointCloudDensification::enqueuePointCloud(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & pointcloud_msg_ptr,
  managed_transform_buffer::ManagedTransformBuffer & managed_tf_buffer)
{
  const auto header = pointcloud_msg_ptr->header;

  if (param_.pointcloud_cache_size() > 1) {
    auto transform_world2current_opt = managed_tf_buffer.getTransform<Eigen::Matrix4f>(
      header.frame_id, param_.world_frame_id(), header.stamp, rclcpp::Duration::from_seconds(0.5),
      rclcpp::get_logger("lidar_centerpoint"));
    if (!transform_world2current_opt) return false;
    auto affine_world2current = Eigen::Affine3f(transform_world2current_opt->matrix());

    enqueue(pointcloud_msg_ptr, affine_world2current);
  } else {
    enqueue(pointcloud_msg_ptr, Eigen::Affine3f::Identity());
  }

  dequeue();

  return true;
}

void PointCloudDensification::enqueue(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  const Eigen::Affine3f & affine_world2current)
{
  affine_world2current_ = affine_world2current;
  current_timestamp_ = rclcpp::Time(msg_ptr->header.stamp).seconds();
  PointCloudWithTransform pointcloud = {msg_ptr, affine_world2current.inverse()};
  pointcloud_cache_.push_front(pointcloud);
}

void PointCloudDensification::dequeue()
{
  if (pointcloud_cache_.size() > param_.pointcloud_cache_size()) {
    pointcloud_cache_.pop_back();
  }
}

}  // namespace autoware::lidar_centerpoint
