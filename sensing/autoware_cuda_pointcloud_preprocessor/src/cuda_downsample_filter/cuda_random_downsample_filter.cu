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

#include "autoware/cuda_pointcloud_preprocessor/cuda_downsample_filter/cuda_random_downsample_filter.hpp"

#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/random.h>
#include <thrust/shuffle.h>
#include <thrust/sequence.h>

#include <cstdint>
#include <memory>
#include <random>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{
__global__ void randomSampleKernel(
  const InputPointType * __restrict__ input_points, OutputPointType * __restrict__ output_points,
  const std::uint32_t * __restrict__ selected_indices, size_t num_samples)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_samples) {
    return;
  }

  const std::uint32_t input_idx = selected_indices[idx];
  const InputPointType & input_point = input_points[input_idx];
  OutputPointType & output_point = output_points[idx];

  output_point.x = input_point.x;
  output_point.y = input_point.y;
  output_point.z = input_point.z;
  output_point.intensity = input_point.intensity;
  output_point.return_type = input_point.return_type;
  output_point.channel = input_point.channel;
}

}  // namespace

CudaRandomDownsampleFilter::CudaRandomDownsampleFilter(
  size_t sample_num, int64_t max_mem_pool_size_in_byte)
: sample_num_(sample_num)
{
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // Create memory pool
  {
    int current_device_id = 0;
    CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
    cudaMemPoolProps pool_props = {};
    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = current_device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;
    CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

    uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
    CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
      mem_pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));
  }
}

template <typename T>
T * CudaRandomDownsampleFilter::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(&buffer, num_elements * sizeof(T), mem_pool_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(buffer, 0, num_elements * sizeof(T), stream_));
  return buffer;
}

template <typename T>
void CudaRandomDownsampleFilter::returnBufferToPool(T * buffer)
{
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, stream_));
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaRandomDownsampleFilter::filter(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points)
{
  const size_t num_input_points = input_points->width * input_points->height;

  if (num_input_points == 0) {
    auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    output->header = input_points->header;
    output->width = 0;
    output->height = 1;
    output->fields = input_points->fields;
    output->is_bigendian = input_points->is_bigendian;
    output->point_step = input_points->point_step;
    output->row_step = 0;
    output->is_dense = input_points->is_dense;
    output->data.reset();
    return output;
  }

  // Determine actual number of samples (cannot exceed input size)
  const size_t num_samples = std::min(sample_num_, num_input_points);

  if (num_samples == 0) {
    auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    output->header = input_points->header;
    output->width = 0;
    output->height = 1;
    output->fields = input_points->fields;
    output->is_bigendian = input_points->is_bigendian;
    output->point_step = input_points->point_step;
    output->row_step = 0;
    output->is_dense = input_points->is_dense;
    output->data.reset();
    return output;
  }

  // Validate input point cloud layout
  if (input_points->point_step != sizeof(InputPointType)) {
    throw std::runtime_error(
      "Input pointcloud point_step does not match InputPointType size. "
      "This filter requires PointXYZIRCAEDT format.");
  }

  // Generate random seed
  std::random_device rd;
  std::mt19937 gen(rd());
  uint32_t seed = static_cast<uint32_t>(gen());

  // Allocate device buffers
  InputPointType * device_input_points = allocateBufferFromPool<InputPointType>(num_input_points);
  // Allocate full index array for shuffling
  std::uint32_t * device_all_indices = allocateBufferFromPool<std::uint32_t>(num_input_points);
  std::uint32_t * device_selected_indices = allocateBufferFromPool<std::uint32_t>(num_samples);
  OutputPointType * device_output_points = allocateBufferFromPool<OutputPointType>(num_samples);

  // Copy input points to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    device_input_points, input_points->data.get(), num_input_points * sizeof(InputPointType),
    cudaMemcpyHostToDevice, stream_));

  // Generate sequence of indices [0, 1, 2, ..., num_input_points-1]
  thrust::sequence(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<std::uint32_t>(device_all_indices),
    thrust::device_ptr<std::uint32_t>(device_all_indices + num_input_points), 0);

  // Shuffle the indices using random number generator
  thrust::default_random_engine rng(seed);
  thrust::shuffle(
    thrust::cuda::par_nosync.on(stream_), thrust::device_ptr<std::uint32_t>(device_all_indices),
    thrust::device_ptr<std::uint32_t>(device_all_indices + num_input_points), rng);

  // Copy first num_samples indices to selected_indices
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    device_selected_indices, device_all_indices, num_samples * sizeof(std::uint32_t),
    cudaMemcpyDeviceToDevice, stream_));

  // Sample points using random indices
  const int blocks_per_grid_sample =
    (num_samples + threads_per_block_ - 1) / threads_per_block_;
  randomSampleKernel<<<blocks_per_grid_sample, threads_per_block_, 0, stream_>>>(
    device_input_points, device_output_points, device_selected_indices, num_samples);

  // Allocate output point cloud
  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output->data = cuda_blackboard::make_unique<std::uint8_t[]>(num_samples * sizeof(OutputPointType));

  // Copy output points from device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output->data.get(), device_output_points, num_samples * sizeof(OutputPointType),
    cudaMemcpyDeviceToHost, stream_));

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Set output metadata
  output->header = input_points->header;
  output->width = num_samples;
  output->height = 1;
  output->point_step = sizeof(OutputPointType);
  output->row_step = num_samples * sizeof(OutputPointType);
  output->is_dense = true;
  output->is_bigendian = input_points->is_bigendian;

  // Set point fields
  auto make_point_field = [](
                            const std::string name, const uint32_t offset, const uint8_t datatype,
                            const uint32_t count) -> sensor_msgs::msg::PointField {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = count;
    return field;
  };

  output->fields.clear();
  output->fields.push_back(make_point_field("x", 0, sensor_msgs::msg::PointField::FLOAT32, 1));
  output->fields.push_back(make_point_field("y", 4, sensor_msgs::msg::PointField::FLOAT32, 1));
  output->fields.push_back(make_point_field("z", 8, sensor_msgs::msg::PointField::FLOAT32, 1));
  output->fields.push_back(
    make_point_field("intensity", 12, sensor_msgs::msg::PointField::UINT8, 1));
  output->fields.push_back(
    make_point_field("return_type", 13, sensor_msgs::msg::PointField::UINT8, 1));
  output->fields.push_back(
    make_point_field("channel", 14, sensor_msgs::msg::PointField::UINT16, 1));

  // Return buffers to pool
  returnBufferToPool(device_input_points);
  returnBufferToPool(device_all_indices);
  returnBufferToPool(device_selected_indices);
  returnBufferToPool(device_output_points);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  return output;
}

}  // namespace autoware::cuda_pointcloud_preprocessor

