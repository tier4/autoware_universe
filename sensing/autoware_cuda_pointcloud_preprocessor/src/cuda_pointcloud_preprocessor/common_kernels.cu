// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/cuda_pointcloud_preprocessor/common_kernels.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"
#include "autoware/cuda_pointcloud_preprocessor/types.hpp"

namespace autoware::cuda_pointcloud_preprocessor
{
__global__ void transformPointsKernel(
  const InputPointType * __restrict__ input_points, InputPointType * output_points, int num_points,
  TransformStruct transform)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    output_points[idx] = input_points[idx];

    const float x = input_points[idx].x;
    const float y = input_points[idx].y;
    const float z = input_points[idx].z;

    output_points[idx].x = transform.m11 * x + transform.m12 * y + transform.m13 * z + transform.x;
    output_points[idx].y = transform.m21 * x + transform.m22 * y + transform.m23 * z + transform.y;
    output_points[idx].z = transform.m31 * x + transform.m32 * y + transform.m33 * z + transform.z;
  }
}

__global__ void cropBoxKernel(
  InputPointType * __restrict__ d_points, std::uint32_t * __restrict__ output_crop_mask,
  std::uint8_t * __restrict__ output_nan_mask, int num_points,
  const CropBoxParameters * __restrict__ crop_box_parameters_ptr, int num_crop_boxes)
{
  for (int idx = blockIdx.x * blockDim.x + threadIdx.x; idx < num_points;
       idx += blockDim.x * gridDim.x) {
    if (d_points[idx].distance == 0.0f) {
      continue;
    }
    const float x = d_points[idx].x;
    const float y = d_points[idx].y;
    const float z = d_points[idx].z;

    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
      output_nan_mask[idx] = 1;
      continue;
    }

    std::uint32_t passed_crop_box_mask = 1;

    for (int i = 0; i < num_crop_boxes; i++) {
      const CropBoxParameters & crop_box_parameters = crop_box_parameters_ptr[i];
      const float & min_x = crop_box_parameters.min_x;
      const float & min_y = crop_box_parameters.min_y;
      const float & min_z = crop_box_parameters.min_z;
      const float & max_x = crop_box_parameters.max_x;
      const float & max_y = crop_box_parameters.max_y;
      const float & max_z = crop_box_parameters.max_z;
      const std::uint8_t negative = crop_box_parameters.negative;

      // Check if point is inside the box
      bool point_is_inside = (x > min_x && x < max_x) && (y > min_y && y < max_y) &&
                             (z > min_z && z < max_z);

      // If negative mode (negative == 1): remove points within the box → preserve points outside
      // If positive mode (negative == 0): remove points outside the box → preserve points inside
      bool should_preserve = (negative != 0) ? !point_is_inside : point_is_inside;
      passed_crop_box_mask &= (should_preserve ? 1 : 0);
    }

    output_crop_mask[idx] = passed_crop_box_mask;
  }
}

__global__ void combineMasksKernel(
  const std::uint32_t * __restrict__ mask1, const std::uint32_t * __restrict__ mask2,
  int num_points, std::uint32_t * __restrict__ output_mask)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    output_mask[idx] = mask1[idx] & mask2[idx];
  }
}

__global__ void extractPointsKernel(
  InputPointType * __restrict__ input_points, std::uint32_t * __restrict__ masks,
  std::uint32_t * __restrict__ indices, int num_points,
  OutputPointType * __restrict__ output_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    InputPointType & input_point = input_points[idx];
    OutputPointType & output_point = output_points[indices[idx] - 1];
    output_point.x = input_point.x;
    output_point.y = input_point.y;
    output_point.z = input_point.z;
    output_point.intensity = input_point.intensity;
    output_point.return_type = input_point.return_type;
    output_point.channel = input_point.channel;
  }
}

__global__ void extractInputPointsKernel(
  InputPointType * __restrict__ input_points, std::uint32_t * __restrict__ masks,
  std::uint32_t * __restrict__ indices, int num_points,
  InputPointType * __restrict__ output_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points && masks[idx] == 1) {
    InputPointType & input_point = input_points[idx];
    InputPointType & output_point = output_points[indices[idx] - 1];
    // Copy all fields including extended ones
    output_point = input_point;
  }
}

__global__ void convertPointXYZIRCToInputPointTypeKernel(
  const OutputPointType * __restrict__ input_points, InputPointType * __restrict__ output_points,
  int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    const OutputPointType & input_point = input_points[idx];
    InputPointType & output_point = output_points[idx];
    
    // Copy common fields
    output_point.x = input_point.x;
    output_point.y = input_point.y;
    output_point.z = input_point.z;
    output_point.intensity = input_point.intensity;
    output_point.return_type = input_point.return_type;
    output_point.channel = input_point.channel;
    
    // Initialize extended fields with default values
    // Set distance to 1.0f so points are not skipped by crop box kernel
    // (which checks if distance == 0.0f)
    output_point.azimuth = 0.0f;
    output_point.elevation = 0.0f;
    output_point.distance = 1.0f;  // Non-zero so point is not skipped
    output_point.time_stamp = 0U;
  }
}

void transformPointsLaunch(
  const InputPointType * input_points, InputPointType * output_points, int num_points,
  TransformStruct transform, int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  transformPointsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, output_points, num_points, transform);
}

void cropBoxLaunch(
  InputPointType * d_points, std::uint32_t * output_crop_mask, std::uint8_t * output_nan_mask,
  int num_points, const CropBoxParameters * crop_box_parameters_ptr, int num_crop_boxes,
  int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  cropBoxKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    d_points, output_crop_mask, output_nan_mask, num_points, crop_box_parameters_ptr,
    num_crop_boxes);
}

void combineMasksLaunch(
  const std::uint32_t * mask1, const std::uint32_t * mask2, int num_points,
  std::uint32_t * output_mask, int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  combineMasksKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    mask1, mask2, num_points, output_mask);
}

void extractPointsLaunch(
  InputPointType * input_points, std::uint32_t * masks, std::uint32_t * indices, int num_points,
  OutputPointType * output_points, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  extractPointsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, masks, indices, num_points, output_points);
}

void extractInputPointsLaunch(
  InputPointType * input_points, std::uint32_t * masks, std::uint32_t * indices, int num_points,
  InputPointType * output_points, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  extractInputPointsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, masks, indices, num_points, output_points);
}

void convertPointXYZIRCToInputPointTypeLaunch(
  const OutputPointType * input_points, InputPointType * output_points, int num_points,
  int threads_per_block, int blocks_per_grid, cudaStream_t & stream)
{
  convertPointXYZIRCToInputPointTypeKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, output_points, num_points);
}

// Pointcloud to costmap kernels
__global__ void assignPointsToGridCellsKernel(
  const InputPointType * __restrict__ input_points, int32_t * __restrict__ grid_indices,
  int num_points, const CostmapGridParams * __restrict__ params)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx < num_points) {
    const InputPointType & point = input_points[idx];
    const float x = point.x;
    const float y = point.y;

    // Check if point is valid
    if (!isfinite(x) || !isfinite(y)) {
      grid_indices[idx] = -1;  // Invalid index
      return;
    }

    // Calculate grid index following the same logic as CPU version
    // From points_to_costmap.cpp: fetchGridIndexFromPoint
    const float origin_x_offset = params->grid_length_x / 2.0f - params->grid_position_x;
    const float origin_y_offset = params->grid_length_y / 2.0f - params->grid_position_y;
    
    // Coordinate conversion for making index. Set bottom left to the origin of coordinate (0, 0)
    float mapped_x = (params->grid_length_x - origin_x_offset - x) / params->grid_resolution;
    float mapped_y = (params->grid_length_y - origin_y_offset - y) / params->grid_resolution;

    int32_t mapped_x_ind = static_cast<int32_t>(ceilf(mapped_x));
    int32_t mapped_y_ind = static_cast<int32_t>(ceilf(mapped_y));

    // Check if index is valid
    if (
      mapped_x_ind >= 0 && mapped_x_ind < params->grid_size_x && mapped_y_ind >= 0 &&
      mapped_y_ind < params->grid_size_y) {
      // Store as linear index: x * grid_size_y + y
      grid_indices[idx] = mapped_x_ind * params->grid_size_y + mapped_y_ind;
    } else {
      grid_indices[idx] = -1;  // Invalid index
    }
  }
}

__global__ void calculateCostmapFromGridCellsKernel(
  const InputPointType * __restrict__ input_points, const int32_t * __restrict__ grid_indices,
  float * __restrict__ costmap_data, int num_points, const CostmapGridParams * __restrict__ params)
{
  // First, initialize all costmap cells to min_value
  int grid_idx = blockIdx.x * blockDim.x + threadIdx.x;
  int total_grid_cells = params->grid_size_x * params->grid_size_y;
  if (grid_idx < total_grid_cells) {
    costmap_data[grid_idx] = params->grid_min_value;
  }

  __syncthreads();

  // Then, for each point, check if it should contribute to costmap
  int point_idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_idx < num_points) {
    const int32_t grid_cell_idx = grid_indices[point_idx];
    if (grid_cell_idx < 0 || grid_cell_idx >= total_grid_cells) {
      return;  // Invalid grid cell
    }

    const InputPointType & point = input_points[point_idx];
    const float z = point.z;

    // Check if z is within height thresholds
    if (z <= params->maximum_height_thres && z >= params->minimum_height_thres) {
      // Use atomic operation to set costmap value to max_value
      // This ensures thread safety when multiple points map to the same cell
      atomicExch(&costmap_data[grid_cell_idx], params->grid_max_value);
    }
  }
}

void assignPointsToGridCellsLaunch(
  const InputPointType * input_points, int32_t * grid_indices, int num_points,
  const CostmapGridParams * params, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  assignPointsToGridCellsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, grid_indices, num_points, params);
}

void calculateCostmapFromGridCellsLaunch(
  const InputPointType * input_points, const int32_t * grid_indices, float * costmap_data,
  int num_points, const CostmapGridParams * params, int threads_per_block, int blocks_per_grid,
  cudaStream_t & stream)
{
  // Calculate blocks for grid cells (for initialization)
  int total_grid_cells = params->grid_size_x * params->grid_size_y;
  int blocks_for_grid = (total_grid_cells + threads_per_block - 1) / threads_per_block;
  
  // Use max of blocks_for_grid and blocks_per_grid to handle both initialization and point processing
  int total_blocks = (blocks_for_grid > blocks_per_grid) ? blocks_for_grid : blocks_per_grid;
  
  calculateCostmapFromGridCellsKernel<<<total_blocks, threads_per_block, 0, stream>>>(
    input_points, grid_indices, costmap_data, num_points, params);
}

}  // namespace autoware::cuda_pointcloud_preprocessor
