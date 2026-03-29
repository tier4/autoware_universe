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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_CONCATENATE_AND_TIME_SYNC_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_CONCATENATE_AND_TIME_SYNC_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_traits.hpp"
#include "autoware/pointcloud_preprocessor/concatenate_data/concatenate_and_time_sync_node.hpp"

#include <rclcpp/rclcpp.hpp>

// Declare explicit specializations to prevent the generic template from being instantiated
// for CudaPointCloud2Traits. Definitions are in cuda_concatenate_and_time_sync_node.cpp.
namespace autoware::pointcloud_preprocessor
{
template <>
void PointCloudConcatenateDataSynchronizerComponentTemplated<
  CudaPointCloud2Traits>::initialize_pub_sub();

template <>
void PointCloudConcatenateDataSynchronizerComponentTemplated<CudaPointCloud2Traits>::publish_clouds(
  ConcatenatedCloudResult<CudaPointCloud2Traits> && concatenated_cloud_result,
  std::shared_ptr<CollectorInfoBase> collector_info);
}  // namespace autoware::pointcloud_preprocessor

namespace autoware::cuda_pointcloud_preprocessor
{

class CudaPointCloudConcatenateDataSynchronizerComponent
: public autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponentTemplated<
    autoware::pointcloud_preprocessor::CudaPointCloud2Traits>
{
public:
  explicit CudaPointCloudConcatenateDataSynchronizerComponent(
    const rclcpp::NodeOptions & node_options)
  : autoware::pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponentTemplated<
      autoware::pointcloud_preprocessor::CudaPointCloud2Traits>(node_options)
  {
  }
  ~CudaPointCloudConcatenateDataSynchronizerComponent() = default;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_CONCATENATE_AND_TIME_SYNC_NODE_HPP_
