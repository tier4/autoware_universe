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

#ifndef AUTOWARE__CUDA_TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__CUDA_OCCLUSION_KERNELS_HPP_
#define AUTOWARE__CUDA_TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__CUDA_OCCLUSION_KERNELS_HPP_

#include <cuda_runtime.h>
#include <cstdint>

namespace autoware::traffic_light
{

/**
 * @brief Structure to hold a 3D point in space
 */
struct PointXYZ
{
  float x;
  float y;
  float z;
};

/**
 * @brief Structure to hold spherical coordinates (ray representation)
 */
struct Ray
{
  float azimuth;    // Angle in horizontal plane (degrees)
  float elevation;  // Angle from horizontal plane (degrees)
  float dist;       // Distance from origin (meters)
};

namespace kernels
{

/**
 * @brief Extract XYZ coordinates from raw point cloud byte array
 *
 * @param raw_points Raw point cloud data (GPU pointer)
 * @param output_points Output PointXYZ array
 * @param num_points Number of points
 * @param point_step Bytes per point
 * @param threads_per_block CUDA threads per block
 * @param blocks_per_grid CUDA blocks per grid
 * @param stream CUDA stream for asynchronous execution
 */
void extractXYZLaunch(
  const uint8_t * raw_points,
  PointXYZ * output_points,
  int num_points,
  int point_step,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream);

/**
 * @brief Transform point cloud from one frame to another using 4x4 transformation matrix
 *
 * @param input_points Input points in source frame
 * @param output_points Output points in target frame
 * @param transform_matrix 4x4 transformation matrix (row-major, 16 floats)
 * @param num_points Number of points to transform
 * @param threads_per_block CUDA threads per block
 * @param blocks_per_grid CUDA blocks per grid
 * @param stream CUDA stream for async execution
 */
void transformPointCloudLaunch(
  const PointXYZ * input_points,
  PointXYZ * output_points,
  const float * transform_matrix,
  int num_points,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream);

/**
 * @brief Filter point cloud by spatial bounds and distance constraints
 *
 * @param input_points Input points to filter
 * @param output_mask Binary mask (1=keep, 0=discard)
 * @param min_x Minimum X coordinate
 * @param max_x Maximum X coordinate
 * @param min_y Minimum Y coordinate
 * @param max_y Maximum Y coordinate
 * @param min_z Minimum Z coordinate
 * @param max_z Maximum Z coordinate
 * @param min_dist_sq Minimum squared distance from origin
 * @param max_dist_sq Maximum squared distance from origin
 * @param num_points Number of points
 * @param threads_per_block CUDA threads per block
 * @param blocks_per_grid CUDA blocks per grid
 * @param stream CUDA stream for async execution
 */
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
  cudaStream_t & stream);

/**
 * @brief Compact filtered points using scan indices
 *
 * @param input_points Input points array
 * @param filter_mask Binary filter mask (1=keep, 0=discard)
 * @param scan_indices Inclusive scan of filter mask
 * @param output_points Compacted output points
 * @param num_points Number of input points
 * @param threads_per_block CUDA threads per block
 * @param blocks_per_grid CUDA blocks per grid
 * @param stream CUDA stream for async execution
 */
void compactPointsLaunch(
  const PointXYZ * input_points,
  const uint32_t * filter_mask,
  const uint32_t * scan_indices,
  PointXYZ * output_points,
  int num_points,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream);

/**
 * @brief Convert Cartesian coordinates to spherical (ray) representation
 *
 * @param input_points Input points in Cartesian coordinates
 * @param output_rays Output rays in spherical coordinates
 * @param num_points Number of points
 * @param threads_per_block CUDA threads per block
 * @param blocks_per_grid CUDA blocks per grid
 * @param stream CUDA stream for async execution
 */
void convertToSphericalLaunch(
  const PointXYZ * input_points,
  Ray * output_rays,
  int num_points,
  int threads_per_block,
  int blocks_per_grid,
  cudaStream_t & stream);

/**
 * @brief Sample points uniformly across ROI surface
 *
 * @param roi_corners Array of [top_left, bottom_right] pairs for each ROI
 * @param sampled_points Output sampled points
 * @param horizontal_sample_num Number of horizontal samples per ROI
 * @param vertical_sample_num Number of vertical samples per ROI
 * @param num_rois Number of ROIs
 * @param stream CUDA stream for async execution
 */
void sampleRoiLaunch(
  const PointXYZ * roi_corners,
  PointXYZ * sampled_points,
  uint32_t horizontal_sample_num,
  uint32_t vertical_sample_num,
  int num_rois,
  cudaStream_t & stream);

/**
 * @brief Detect occlusion for sampled ROI points using LiDAR rays
 *
 * This kernel checks if each sampled point is occluded by comparing with
 * nearby LiDAR rays in spherical coordinates.
 *
 * @param sample_points Sampled points from ROIs (in camera frame)
 * @param lidar_rays Sorted array of LiDAR rays
 * @param ray_azimuth_indices Start indices for each azimuth bin
 * @param ray_elevation_indices Start indices for each elevation bin
 * @param occlusion_mask Output mask (1=occluded, 0=visible)
 * @param azimuth_tolerance Angular tolerance for azimuth matching (degrees)
 * @param elevation_tolerance Angular tolerance for elevation matching (degrees)
 * @param min_dist_diff Minimum distance difference for occlusion (meters)
 * @param num_samples Number of sample points
 * @param num_lidar_rays Number of LiDAR rays
 * @param threads_per_block CUDA threads per block
 * @param blocks_per_grid CUDA blocks per grid
 * @param stream CUDA stream for async execution
 */
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
  cudaStream_t & stream);

}  // namespace kernels
}  // namespace autoware::traffic_light

#endif  // AUTOWARE__CUDA_TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__CUDA_OCCLUSION_KERNELS_HPP_

