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

#ifndef AUTOWARE__CUDA_TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__CUDA_OCCLUSION_PREDICTOR_HPP_
#define AUTOWARE__CUDA_TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__CUDA_OCCLUSION_PREDICTOR_HPP_

#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_runtime.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::traffic_light
{

// Forward declarations for basic types (defined in kernels header)
struct PointXYZ;
struct Ray;

/**
 * @brief Parameters for CUDA occlusion predictor
 */
struct CudaOcclusionPredictorParameters
{
  float max_valid_pt_distance;              // Maximum distance for valid points (meters)
  float azimuth_occlusion_resolution_deg;   // Azimuth resolution for occlusion detection (degrees)
  float elevation_occlusion_resolution_deg; // Elevation resolution for occlusion detection (degrees)
  float min_dist_from_occlusion_to_tl;      // Minimum distance between occluder and traffic light (meters)
  uint32_t horizontal_sample_num;           // Number of horizontal samples per ROI
  uint32_t vertical_sample_num;             // Number of vertical samples per ROI
};

/**
 * @brief CUDA-accelerated occlusion predictor for traffic lights
 *
 * This class implements GPU-accelerated occlusion detection for traffic lights
 * using LiDAR point cloud data. It maintains the same algorithm as the CPU version
 * but leverages CUDA for parallel processing.
 */
class CudaOcclusionPredictor
{
public:
  /**
   * @brief Constructor
   *
   * @param params Configuration parameters
   * @param max_mem_pool_size_in_byte Maximum size of CUDA memory pool (bytes)
   */
  explicit CudaOcclusionPredictor(
    const CudaOcclusionPredictorParameters & params,
    int64_t max_mem_pool_size_in_byte = 1e9);

  /**
   * @brief Destructor - cleanup CUDA resources
   */
  ~CudaOcclusionPredictor();

  /**
   * @brief Predict occlusion ratios for traffic light ROIs
   *
   * @param cloud_msg Point cloud data (CUDA blackboard - zero-copy if from CUDA source!)
   * @param camera2cloud_transform 4x4 transformation matrix from camera to LiDAR frame (row-major, 16 floats)
   * @param roi_3d_points Pre-calculated 3D ROI corners in camera frame [top_left, bottom_right] pairs
   * @param occlusion_ratios Output vector of occlusion ratios (0-100) for each ROI
   */
  void predict(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & cloud_msg,
    const float * camera2cloud_transform,
    const std::vector<PointXYZ> & roi_3d_points,
    std::vector<int> & occlusion_ratios);

private:
  /**
   * @brief Allocate device memory from CUDA memory pool
   *
   * @tparam T Type of data to allocate
   * @param num_elements Number of elements to allocate
   * @return Pointer to allocated device memory
   */
  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  /**
   * @brief Return device memory to CUDA memory pool
   *
   * @tparam T Type of data to free
   * @param buffer Pointer to device memory
   */
  template <typename T>
  void returnBufferToPool(T * buffer);

  // Note: calcRoiVector3D is moved to node file (uses OpenCV, CPU-only)

  // Parameters
  CudaOcclusionPredictorParameters params_;

  // CUDA resources
  cudaStream_t stream_{};
  cudaMemPool_t mem_pool_{};

  // Thread configuration
  static constexpr int threads_per_block_ = 512;
};

}  // namespace autoware::traffic_light

#endif  // AUTOWARE__CUDA_TRAFFIC_LIGHT_OCCLUSION_PREDICTOR__CUDA_OCCLUSION_PREDICTOR_HPP_

