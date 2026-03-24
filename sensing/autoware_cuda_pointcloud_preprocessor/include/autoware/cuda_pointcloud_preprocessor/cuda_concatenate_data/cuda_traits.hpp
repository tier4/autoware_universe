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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_TRAITS_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_TRAITS_HPP_

#include <agnocast/agnocast.hpp>
#include <agnocast/cuda/types.hpp>

namespace autoware::pointcloud_preprocessor
{

struct CudaPointCloud2Traits
{
  using PointCloudMessage = agnocast::cuda::PointCloud2;
  using UniquePtr = std::unique_ptr<PointCloudMessage>;
  using ConstSharedPtr = agnocast::ipc_shared_ptr<const PointCloudMessage>;
  using PublisherType = agnocast::Publisher<PointCloudMessage>;
  using SubscriberType = agnocast::Subscription<PointCloudMessage>;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_CONCATENATE_DATA__CUDA_TRAITS_HPP_
