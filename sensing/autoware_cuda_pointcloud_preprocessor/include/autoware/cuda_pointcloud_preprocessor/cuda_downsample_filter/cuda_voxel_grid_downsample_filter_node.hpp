// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_

#include "cuda_voxel_grid_downsample_filter.hpp"

#include <agnocast/agnocast.hpp>
#include <agnocast/cuda/types.hpp>

#include <memory>

namespace autoware::cuda_pointcloud_preprocessor
{
class CudaVoxelGridDownsampleFilterNode : public agnocast::Node
{
public:
  explicit CudaVoxelGridDownsampleFilterNode(const rclcpp::NodeOptions & node_options);

private:
  void cudaPointcloudCallback(
    agnocast::ipc_shared_ptr<const agnocast::cuda::PointCloud2> msg);

  // CUDA sub
  agnocast::Subscription<agnocast::cuda::PointCloud2>::SharedPtr sub_;

  // CUDA pub
  agnocast::Publisher<agnocast::cuda::PointCloud2>::SharedPtr pub_;

  std::unique_ptr<CudaVoxelGridDownsampleFilter> cuda_voxel_grid_downsample_filter_{};
};
}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_DOWNSAMPLE_FILTER__CUDA_VOXEL_GRID_DOWNSAMPLE_FILTER_NODE_HPP_
