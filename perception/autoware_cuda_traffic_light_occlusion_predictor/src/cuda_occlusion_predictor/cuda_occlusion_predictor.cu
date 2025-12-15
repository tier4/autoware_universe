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

#include "autoware/cuda_traffic_light_occlusion_predictor/cuda_occlusion_predictor.hpp"
#include "autoware/cuda_traffic_light_occlusion_predictor/cuda_occlusion_kernels.hpp"  // For PointXYZ and Ray definitions
#include "autoware/cuda_utils/cuda_check_error.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <thrust/device_ptr.h>
#include <thrust/scan.h>
#include <thrust/sort.h>

#include <stdexcept>
#include <vector>

namespace autoware::traffic_light
{

CudaOcclusionPredictor::CudaOcclusionPredictor(
  const CudaOcclusionPredictorParameters & params,
  int64_t max_mem_pool_size_in_byte)
: params_(params)
{
  // Validate GPU availability
  int device_count = 0;
  CHECK_CUDA_ERROR(cudaGetDeviceCount(&device_count));
  if (device_count == 0) {
    throw std::runtime_error("CUDA GPU required but not available");
  }

  // Create CUDA stream
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  // Create CUDA memory pool
  int current_device_id = 0;
  CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
  cudaMemPoolProps pool_props = {};
  pool_props.allocType = cudaMemAllocationTypePinned;
  pool_props.location.id = current_device_id;
  pool_props.location.type = cudaMemLocationTypeDevice;
  CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

  // Configure memory pool threshold
  uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
  CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
    mem_pool_, cudaMemPoolAttrReleaseThreshold,
    static_cast<void *>(&pool_release_threshold)));
}

CudaOcclusionPredictor::~CudaOcclusionPredictor()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
  if (mem_pool_) {
    cudaMemPoolDestroy(mem_pool_);
  }
}

template <typename T>
T * CudaOcclusionPredictor::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(cudaMallocFromPoolAsync(
    &buffer, num_elements * sizeof(T), mem_pool_, stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(
    buffer, 0, num_elements * sizeof(T), stream_));
  return buffer;
}

template <typename T>
void CudaOcclusionPredictor::returnBufferToPool(T * buffer)
{
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, stream_));
}

void CudaOcclusionPredictor::predict(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & cloud_msg,
  const float * camera2cloud_transform,
  const std::vector<PointXYZ> & roi_3d_points,
  std::vector<int> & occlusion_ratios)
{
  // Validate inputs
  if (!cloud_msg || !cloud_msg->data || camera2cloud_transform == nullptr) {
    return;
  }

  const size_t num_points = cloud_msg->width * cloud_msg->height;
  const size_t point_step = cloud_msg->point_step;
  const size_t num_rois = roi_3d_points.size() / 2;  // Each ROI has [top_left, bottom_right]
  
  occlusion_ratios.resize(num_rois, 0);

  if (num_points == 0 || num_rois == 0) {
    return;
  }

  try {
    // Step 1: Extract XYZ from CUDA blackboard point cloud (already on GPU - zero copy!)
    const uint8_t * d_pointcloud_data = cloud_msg->data.get();
    
    PointXYZ * d_input_points = allocateBufferFromPool<PointXYZ>(num_points);
    PointXYZ * d_transformed_points = allocateBufferFromPool<PointXYZ>(num_points);

    const int blocks_per_grid = (num_points + threads_per_block_ - 1) / threads_per_block_;
    
    kernels::extractXYZLaunch(
      d_pointcloud_data, d_input_points, static_cast<int>(num_points), static_cast<int>(point_step),
      threads_per_block_, blocks_per_grid, stream_);

    // Step 2: Transform points from LiDAR frame to camera frame
    kernels::transformPointCloudLaunch(
      d_input_points, d_transformed_points, camera2cloud_transform, static_cast<int>(num_points),
      threads_per_block_, blocks_per_grid, stream_);

    // Step 3: Filter points by spatial bounds and distance
    // Calculate bounding box from ROI 3D points
    float min_x = 0, max_x = 0, min_y = 0, max_y = 0, min_z = 0, max_z = 0;
    bool first = true;
    for (const auto & pt : roi_3d_points) {
      if (first) {
        min_x = max_x = pt.x;
        min_y = max_y = pt.y;
        min_z = max_z = pt.z;
        first = false;
      } else {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
        min_z = std::min(min_z, pt.z);
        max_z = std::max(max_z, pt.z);
      }
    }

    uint32_t * d_filter_mask = allocateBufferFromPool<uint32_t>(num_points);
    const float min_dist_sq = 1.0f * 1.0f;  // 1m minimum
    const float max_dist_sq =
      params_.max_valid_pt_distance * params_.max_valid_pt_distance;

    kernels::filterPointCloudLaunch(
      d_transformed_points, d_filter_mask, min_x, max_x, min_y, max_y, min_z, max_z,
      min_dist_sq, max_dist_sq, static_cast<int>(num_points), threads_per_block_,
      blocks_per_grid, stream_);

    // Step 4: Compact filtered points using Thrust
    // Count valid points
    uint32_t * d_indices = allocateBufferFromPool<uint32_t>(num_points);
    thrust::inclusive_scan(
      thrust::cuda::par_nosync.on(stream_),
      thrust::device_ptr<uint32_t>(d_filter_mask),
      thrust::device_ptr<uint32_t>(d_filter_mask + num_points),
      thrust::device_ptr<uint32_t>(d_indices));
    
    uint32_t num_filtered = 0;
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &num_filtered, d_indices + num_points - 1, sizeof(uint32_t),
      cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    
    if (num_filtered == 0) {
      // No points passed filtering
      returnBufferToPool(d_input_points);
      returnBufferToPool(d_transformed_points);
      returnBufferToPool(d_filter_mask);
      returnBufferToPool(d_indices);
      return;
    }
    
    // Compact the points
    PointXYZ * d_compacted_points = allocateBufferFromPool<PointXYZ>(num_filtered);
    const int compact_blocks = (num_points + threads_per_block_ - 1) / threads_per_block_;
    kernels::compactPointsLaunch(
      d_transformed_points, d_filter_mask, d_indices, d_compacted_points,
      static_cast<int>(num_points), threads_per_block_, compact_blocks, stream_);

    // Step 5: Convert to spherical coordinates
    Ray * d_rays = allocateBufferFromPool<Ray>(num_filtered);
    const int ray_blocks = (num_filtered + threads_per_block_ - 1) / threads_per_block_;
    kernels::convertToSphericalLaunch(
      d_compacted_points, d_rays, static_cast<int>(num_filtered),
      threads_per_block_, ray_blocks, stream_);

    // Step 6: Sort rays by azimuth then elevation (lexicographic)
    thrust::sort(
      thrust::cuda::par_nosync.on(stream_),
      thrust::device_ptr<Ray>(d_rays),
      thrust::device_ptr<Ray>(d_rays + num_filtered),
      [] __device__ (const Ray & a, const Ray & b) {
        if (a.azimuth != b.azimuth) return a.azimuth < b.azimuth;
        return a.elevation < b.elevation;
      });

    // Step 7-9: Sample ROI points and detect occlusion for all ROIs
    const int samples_per_roi = params_.horizontal_sample_num * params_.vertical_sample_num;
    const int total_samples = num_rois * samples_per_roi;
    
    if (total_samples > 0) {
      // Copy ROI 3D points to device
      PointXYZ * d_roi_corners = allocateBufferFromPool<PointXYZ>(num_rois * 2);
      CHECK_CUDA_ERROR(cudaMemcpyAsync(
        d_roi_corners, roi_3d_points.data(), num_rois * 2 * sizeof(PointXYZ),
        cudaMemcpyHostToDevice, stream_));
      
      // Allocate sample points for all ROIs
      PointXYZ * d_sample_points = allocateBufferFromPool<PointXYZ>(total_samples);
      uint32_t * d_occlusion_mask = allocateBufferFromPool<uint32_t>(total_samples);
      
      // Generate sample points for all ROIs at once
      kernels::sampleRoiLaunch(
        d_roi_corners, d_sample_points,
        params_.horizontal_sample_num, params_.vertical_sample_num,
        static_cast<int>(num_rois), stream_);
      
      // Detect occlusion for all samples
      // Note: Passing nullptr for spatial indices - kernel will do linear search
      // TODO: Create spatial indexing for better performance
      const int sample_blocks = (total_samples + threads_per_block_ - 1) / threads_per_block_;
      kernels::detectOcclusionLaunch(
        d_sample_points,                    // PointXYZ* (all samples)
        d_rays,                             // Ray* (LiDAR rays, sorted)
        nullptr,                            // int* (azimuth indices - TODO)
        nullptr,                            // int* (elevation indices - TODO)
        d_occlusion_mask,                   // uint32_t* (output mask)
        params_.azimuth_occlusion_resolution_deg,
        params_.elevation_occlusion_resolution_deg,
        params_.min_dist_from_occlusion_to_tl,
        total_samples,
        static_cast<int>(num_filtered),
        threads_per_block_,
        sample_blocks,
        stream_);
      
      // Calculate occlusion ratio for each ROI
      for (size_t roi_idx = 0; roi_idx < num_rois; ++roi_idx) {
        // Reduce for this ROI's samples
        uint32_t * roi_mask_start = d_occlusion_mask + roi_idx * samples_per_roi;
        uint32_t occluded_count = thrust::reduce(
          thrust::cuda::par_nosync.on(stream_),
          thrust::device_ptr<uint32_t>(roi_mask_start),
          thrust::device_ptr<uint32_t>(roi_mask_start + samples_per_roi),
          0U);
        
        occlusion_ratios[roi_idx] = (100 * occluded_count) / samples_per_roi;
      }
      
      // Cleanup
      returnBufferToPool(d_roi_corners);
      returnBufferToPool(d_sample_points);
      returnBufferToPool(d_occlusion_mask);
    }

    // Cleanup
    returnBufferToPool(d_input_points);
    returnBufferToPool(d_transformed_points);
    returnBufferToPool(d_filter_mask);
    returnBufferToPool(d_indices);
    returnBufferToPool(d_compacted_points);
    returnBufferToPool(d_rays);

    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  } catch (const std::exception & e) {
    // Re-throw to be handled by node
    throw;
  }
}

// Explicit template instantiations
template PointXYZ * CudaOcclusionPredictor::allocateBufferFromPool<PointXYZ>(size_t);
template Ray * CudaOcclusionPredictor::allocateBufferFromPool<Ray>(size_t);
template uint32_t * CudaOcclusionPredictor::allocateBufferFromPool<uint32_t>(size_t);
template float * CudaOcclusionPredictor::allocateBufferFromPool<float>(size_t);
template int * CudaOcclusionPredictor::allocateBufferFromPool<int>(size_t);

template void CudaOcclusionPredictor::returnBufferToPool<PointXYZ>(PointXYZ *);
template void CudaOcclusionPredictor::returnBufferToPool<Ray>(Ray *);
template void CudaOcclusionPredictor::returnBufferToPool<uint32_t>(uint32_t *);
template void CudaOcclusionPredictor::returnBufferToPool<float>(float *);
template void CudaOcclusionPredictor::returnBufferToPool<int>(int *);

}  // namespace autoware::traffic_light

