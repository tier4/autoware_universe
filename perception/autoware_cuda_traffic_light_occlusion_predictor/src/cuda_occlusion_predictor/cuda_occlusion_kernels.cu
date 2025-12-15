// Copyright 2023-2026 the Autoware Foundation
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

#include "autoware/cuda_traffic_light_occlusion_predictor/cuda_occlusion_kernels.hpp"

#include <cuda_runtime.h>

namespace autoware::traffic_light
{
namespace kernels
{

// Constants
constexpr float RAD2DEG = 57.295779513f;  // 180.0 / M_PI

/**
 * @brief CUDA kernel: Extract XYZ from raw point cloud bytes
 */
__global__ void extractXYZKernel(
  const uint8_t * __restrict__ raw_points,
  PointXYZ * __restrict__ output_points,
  int num_points,
  int point_step)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }

  // Extract XYZ from byte array (assuming standard x,y,z float at offset 0,4,8)
  const float * pt = reinterpret_cast<const float *>(raw_points + static_cast<size_t>(idx) * point_step);
  
  // Validate and copy
  if (isfinite(pt[0]) && isfinite(pt[1]) && isfinite(pt[2])) {
    output_points[idx] = {pt[0], pt[1], pt[2]};
  } else {
    // Mark invalid points with NaN
    output_points[idx] = {NAN, NAN, NAN};
  }
}

void extractXYZLaunch(
  const uint8_t * raw_points,
  PointXYZ * output_points,
  int num_points,
  int point_step,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream)
{
  extractXYZKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    raw_points, output_points, num_points, point_step);
}

/**
 * @brief CUDA kernel: Transform points using 4x4 transformation matrix
 */
__global__ void transformPointCloudKernel(
  const PointXYZ * __restrict__ input_points,
  PointXYZ * __restrict__ output_points,
  const float * __restrict__ transform_matrix,
  int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  const PointXYZ & in_pt = input_points[idx];

  // Validate input
  if (!isfinite(in_pt.x) || !isfinite(in_pt.y) || !isfinite(in_pt.z)) {
    output_points[idx] = {NAN, NAN, NAN};
    return;
  }

  // Apply 4x4 transformation: P_out = T * [x, y, z, 1]^T
  // Transform matrix is row-major: [m00 m01 m02 m03; m10 m11 m12 m13; m20 m21 m22 m23; m30 m31 m32 m33]
  float x = transform_matrix[0] * in_pt.x + transform_matrix[1] * in_pt.y +
            transform_matrix[2] * in_pt.z + transform_matrix[3];
  float y = transform_matrix[4] * in_pt.x + transform_matrix[5] * in_pt.y +
            transform_matrix[6] * in_pt.z + transform_matrix[7];
  float z = transform_matrix[8] * in_pt.x + transform_matrix[9] * in_pt.y +
            transform_matrix[10] * in_pt.z + transform_matrix[11];

  output_points[idx] = {x, y, z};
}

/**
 * @brief CUDA kernel: Filter points by spatial bounds and distance
 */
__global__ void filterPointCloudKernel(
  const PointXYZ * __restrict__ input_points,
  uint32_t * __restrict__ output_mask,
  float min_x, float max_x,
  float min_y, float max_y,
  float min_z, float max_z,
  float min_dist_sq,
  float max_dist_sq,
  int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  const PointXYZ & pt = input_points[idx];

  // Check for invalid points
  if (!isfinite(pt.x) || !isfinite(pt.y) || !isfinite(pt.z)) {
    output_mask[idx] = 0;
    return;
  }

  // Check spatial bounds
  bool in_bounds = (pt.x >= min_x && pt.x <= max_x) &&
                   (pt.y >= min_y && pt.y <= max_y) &&
                   (pt.z >= min_z && pt.z <= max_z);

  if (!in_bounds) {
    output_mask[idx] = 0;
    return;
  }

  // Check distance (using squared distance to avoid sqrt)
  float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
  bool in_distance = (dist_sq >= min_dist_sq) && (dist_sq <= max_dist_sq);

  output_mask[idx] = in_distance ? 1 : 0;
}

/**
 * @brief CUDA kernel: Convert Cartesian to spherical coordinates
 */
__global__ void convertToSphericalKernel(
  const PointXYZ * __restrict__ input_points,
  Ray * __restrict__ output_rays,
  int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;

  const PointXYZ & pt = input_points[idx];

  // Validate input
  if (!isfinite(pt.x) || !isfinite(pt.y) || !isfinite(pt.z)) {
    output_rays[idx] = {NAN, NAN, NAN};
    return;
  }

  // Calculate distance
  float dist = sqrtf(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);

  // Calculate elevation: atan2(y, sqrt(x^2 + z^2))
  float horiz_dist = sqrtf(pt.x * pt.x + pt.z * pt.z);
  float elevation = atan2f(pt.y, horiz_dist) * RAD2DEG;

  // Calculate azimuth: atan2(x, z)
  float azimuth = atan2f(pt.x, pt.z) * RAD2DEG;

  output_rays[idx] = {azimuth, elevation, dist};
}

/**
 * @brief CUDA kernel: Sample points across ROI surface
 */
__global__ void sampleRoiKernel(
  const PointXYZ * __restrict__ roi_corners,
  PointXYZ * __restrict__ sampled_points,
  uint32_t horizontal_sample_num,
  uint32_t vertical_sample_num,
  int num_rois)
{
  // Each block handles one ROI
  int roi_idx = blockIdx.x;
  if (roi_idx >= num_rois) return;

  // Each thread handles one sample point
  int sample_idx = threadIdx.x;
  int total_samples = horizontal_sample_num * vertical_sample_num;
  if (sample_idx >= total_samples) return;

  // Get ROI corners
  const PointXYZ & top_left = roi_corners[roi_idx * 2];
  const PointXYZ & bottom_right = roi_corners[roi_idx * 2 + 1];

  // Calculate sample indices
  uint32_t i1 = sample_idx / vertical_sample_num;  // horizontal index
  uint32_t i2 = sample_idx % vertical_sample_num;  // vertical index

  // Bilinear interpolation
  float t1 = static_cast<float>(i1) / static_cast<float>(horizontal_sample_num - 1);
  float t2 = static_cast<float>(i2) / static_cast<float>(vertical_sample_num - 1);

  float x = top_left.x + (bottom_right.x - top_left.x) * t1;
  float y = top_left.y + (bottom_right.y - top_left.y) * t2;
  float z = top_left.z + (bottom_right.z - top_left.z) * t1;

  // Write output
  int output_idx = roi_idx * total_samples + sample_idx;
  sampled_points[output_idx] = {x, y, z};
}

/**
 * @brief CUDA kernel: Detect occlusion for sample points
 */
__global__ void detectOcclusionKernel(
  const PointXYZ * __restrict__ sample_points,
  const Ray * __restrict__ lidar_rays,
  const int * __restrict__ ray_azimuth_indices,
  const int * __restrict__ ray_elevation_indices,
  uint32_t * __restrict__ occlusion_mask,
  float azimuth_tolerance,
  float elevation_tolerance,
  float min_dist_diff,
  int num_samples,
  int num_lidar_rays)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_samples) return;

  const PointXYZ & sample_pt = sample_points[idx];

  // Validate input
  if (!isfinite(sample_pt.x) || !isfinite(sample_pt.y) || !isfinite(sample_pt.z)) {
    occlusion_mask[idx] = 0;
    return;
  }

  // Convert sample point to spherical coordinates
  float dist = sqrtf(sample_pt.x * sample_pt.x + sample_pt.y * sample_pt.y +
                     sample_pt.z * sample_pt.z);
  float horiz_dist = sqrtf(sample_pt.x * sample_pt.x + sample_pt.z * sample_pt.z);
  float elevation = atan2f(sample_pt.y, horiz_dist) * RAD2DEG;
  float azimuth = atan2f(sample_pt.x, sample_pt.z) * RAD2DEG;

  // Search through LiDAR rays
  // Note: This is a simple linear search. For better performance, consider
  // using a spatial data structure or sorting rays for binary search.
  bool occluded = false;
  for (int i = 0; i < num_lidar_rays && !occluded; i++) {
    const Ray & lidar_ray = lidar_rays[i];

    // Check if NaN
    if (!isfinite(lidar_ray.azimuth) || !isfinite(lidar_ray.elevation) ||
        !isfinite(lidar_ray.dist)) {
      continue;
    }

    // Check angular proximity
    float az_diff = fabsf(lidar_ray.azimuth - azimuth);
    float el_diff = fabsf(lidar_ray.elevation - elevation);

    // Handle azimuth wraparound (360 degrees)
    if (az_diff > 180.0f) {
      az_diff = 360.0f - az_diff;
    }

    if (az_diff <= azimuth_tolerance && el_diff <= elevation_tolerance) {
      // Check distance condition: lidar point is closer to camera
      if (lidar_ray.dist < dist - min_dist_diff) {
        occluded = true;
      }
    }
  }

  occlusion_mask[idx] = occluded ? 1 : 0;
}

// ============================================================================
// Point Compaction Kernel
// ============================================================================

__global__ void compactPointsKernel(
  const PointXYZ * __restrict__ input_points,
  const uint32_t * __restrict__ filter_mask,
  const uint32_t * __restrict__ scan_indices,
  PointXYZ * __restrict__ output_points,
  int num_points)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) return;
  
  if (filter_mask[idx] == 1) {
    // This point passed the filter
    int output_idx = (idx == 0) ? 0 : scan_indices[idx - 1];
    output_points[output_idx] = input_points[idx];
  }
}

// ============================================================================
// Launch functions
// ============================================================================

void transformPointCloudLaunch(
  const PointXYZ * input_points,
  PointXYZ * output_points,
  const float * transform_matrix,
  int num_points,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream)
{
  transformPointCloudKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, output_points, transform_matrix, num_points);
}

void filterPointCloudLaunch(
  const PointXYZ * input_points,
  uint32_t * output_mask,
  float min_x, float max_x,
  float min_y, float max_y,
  float min_z, float max_z,
  float min_dist_sq,
  float max_dist_sq,
  int num_points,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream)
{
  filterPointCloudKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, output_mask, min_x, max_x, min_y, max_y, min_z, max_z,
    min_dist_sq, max_dist_sq, num_points);
}

void convertToSphericalLaunch(
  const PointXYZ * input_points,
  Ray * output_rays,
  int num_points,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream)
{
  convertToSphericalKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, output_rays, num_points);
}

void sampleRoiLaunch(
  const PointXYZ * roi_corners,
  PointXYZ * sampled_points,
  uint32_t horizontal_sample_num,
  uint32_t vertical_sample_num,
  int num_rois,
  cudaStream_t & stream)
{
  int total_samples = horizontal_sample_num * vertical_sample_num;
  sampleRoiKernel<<<num_rois, total_samples, 0, stream>>>(
    roi_corners, sampled_points, horizontal_sample_num, vertical_sample_num, num_rois);
}

void detectOcclusionLaunch(
  const PointXYZ * sample_points,
  const Ray * lidar_rays,
  const int * ray_azimuth_indices,
  const int * ray_elevation_indices,
  uint32_t * occlusion_mask,
  float azimuth_tolerance,
  float elevation_tolerance,
  float min_dist_diff,
  int num_samples,
  int num_lidar_rays,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream)
{
  detectOcclusionKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    sample_points, lidar_rays, ray_azimuth_indices, ray_elevation_indices,
    occlusion_mask, azimuth_tolerance, elevation_tolerance, min_dist_diff,
    num_samples, num_lidar_rays);
}

void compactPointsLaunch(
  const PointXYZ * input_points,
  const uint32_t * filter_mask,
  const uint32_t * scan_indices,
  PointXYZ * output_points,
  int num_points,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream)
{
  compactPointsKernel<<<blocks_per_grid, threads_per_block, 0, stream>>>(
    input_points, filter_mask, scan_indices, output_points, num_points);
}

}  // namespace kernels
}  // namespace autoware::traffic_light

