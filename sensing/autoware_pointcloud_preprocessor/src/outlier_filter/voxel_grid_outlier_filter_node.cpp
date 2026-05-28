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

#include "autoware/pointcloud_preprocessor/outlier_filter/voxel_grid_outlier_filter_node.hpp"

#include <cmath>
#include <cstddef>
#include <functional>
#include <tuple>
#include <unordered_map>
#include <vector>


namespace autoware::pointcloud_preprocessor
{
VoxelGridOutlierFilterComponent::VoxelGridOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("VoxelGridOutlierFilter", options)
{
  // set initial parameters
  {
    voxel_size_x_ = declare_parameter<double>("voxel_size_x");
    voxel_size_y_ = declare_parameter<double>("voxel_size_y");
    voxel_size_z_ = declare_parameter<double>("voxel_size_z");
    voxel_points_threshold_ = declare_parameter<int>("voxel_points_threshold");
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&VoxelGridOutlierFilterComponent::param_callback, this, _1));
}
void VoxelGridOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  std::scoped_lock lock(mutex_);
  if (indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);

  pcl_output->points.reserve(pcl_input->points.size());

  if (
    voxel_size_x_ <= 0.0 || voxel_size_y_ <= 0.0 || voxel_size_z_ <= 0.0 ||
    voxel_points_threshold_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid voxel_grid_outlier_filter parameters. "
      "voxel_size=(%f,%f,%f), voxel_points_threshold=%d",
      voxel_size_x_, voxel_size_y_, voxel_size_z_, voxel_points_threshold_);
    pcl::toROSMsg(*pcl_output, output);
    output.header = input->header;
    return;
  }

  using VoxelKey = std::tuple<int, int, int>;

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      const auto h1 = std::hash<int>{}(std::get<0>(key));
      const auto h2 = std::hash<int>{}(std::get<1>(key));
      const auto h3 = std::hash<int>{}(std::get<2>(key));
      // Hash combine. This avoids depending on PCL's leaf layout storage.
      return h1 ^ (h2 << 1U) ^ (h3 << 2U);
    }
  };

  const auto get_voxel_key = [this](const pcl::PointXYZ & point) -> VoxelKey {
    return VoxelKey{
      static_cast<int>(std::floor(point.x / voxel_size_x_)),
      static_cast<int>(std::floor(point.y / voxel_size_y_)),
      static_cast<int>(std::floor(point.z / voxel_size_z_))};
  };

  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> voxel_point_counts;
  voxel_point_counts.reserve(pcl_input->points.size());

  std::size_t finite_points = 0;
  for (const auto & point : pcl_input->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    ++finite_points;
    ++voxel_point_counts[get_voxel_key(point)];
  }

  std::size_t kept = 0;
  std::size_t rejected = 0;
  const auto threshold = static_cast<std::size_t>(voxel_points_threshold_);

  for (const auto & point : pcl_input->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      ++rejected;
      continue;
    }

    const auto count_it = voxel_point_counts.find(get_voxel_key(point));
    if (count_it != voxel_point_counts.end() && count_it->second >= threshold) {
      pcl_output->points.push_back(point);
      ++kept;
    } else {
      ++rejected;
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "voxel_grid_outlier_filter: input=%zu finite=%zu voxels=%zu kept=%zu rejected=%zu "
    "leaf=(%f,%f,%f) threshold=%d",
    pcl_input->points.size(), finite_points, voxel_point_counts.size(), kept, rejected,
    voxel_size_x_, voxel_size_y_, voxel_size_z_, voxel_points_threshold_);

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult VoxelGridOutlierFilterComponent::param_callback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "voxel_size_x", voxel_size_x_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_x_);
  }
  if (get_param(p, "voxel_size_y", voxel_size_y_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_y_);
  }
  if (get_param(p, "voxel_size_z", voxel_size_z_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %f.", voxel_size_z_);
  }
  if (get_param(p, "voxel_points_threshold", voxel_points_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance threshold to: %d.", voxel_points_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::VoxelGridOutlierFilterComponent)
