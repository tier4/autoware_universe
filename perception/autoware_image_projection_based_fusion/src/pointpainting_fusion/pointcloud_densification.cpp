// Copyright 2024 TIER IV, Inc.
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

#include "autoware/image_projection_based_fusion/pointpainting_fusion/pointcloud_densification.hpp"

#include <pcl_ros/transforms.hpp>

#include <boost/optional.hpp>

#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <string>
#include <utility>

namespace autoware::image_projection_based_fusion
{
PointCloudDensification::PointCloudDensification(
  const autoware::lidar_centerpoint::DensificationParam & param)
: param_(param)
{
}

bool PointCloudDensification::enqueuePointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg,
  managed_transform_buffer::ManagedTransformBuffer & managed_tf_buffer)
{
  const auto header = pointcloud_msg.header;

  if (param_.pointcloud_cache_size() > 1) {
    auto transform_world2current = managed_tf_buffer.getTransform<Eigen::Matrix4f>(
      header.frame_id, param_.world_frame_id(), header.stamp, rclcpp::Duration::from_seconds(0.01),
      rclcpp::get_logger("image_projection_based_fusion"));
    if (!transform_world2current) {
      return false;
    }
    auto affine_world2current = Eigen::Affine3f(transform_world2current->matrix());

    enqueue(pointcloud_msg, affine_world2current);
  } else {
    enqueue(pointcloud_msg, Eigen::Affine3f::Identity());
  }

  dequeue();

  return true;
}

void PointCloudDensification::enqueue(
  const sensor_msgs::msg::PointCloud2 & msg, const Eigen::Affine3f & affine_world2current)
{
  affine_world2current_ = affine_world2current;
  current_timestamp_ = rclcpp::Time(msg.header.stamp).seconds();
  PointCloudWithTransform pointcloud = {msg, affine_world2current.inverse()};
  pointcloud_cache_.push_front(pointcloud);
}

void PointCloudDensification::dequeue()
{
  if (pointcloud_cache_.size() > param_.pointcloud_cache_size()) {
    pointcloud_cache_.pop_back();
  }
}

}  // namespace autoware::image_projection_based_fusion
