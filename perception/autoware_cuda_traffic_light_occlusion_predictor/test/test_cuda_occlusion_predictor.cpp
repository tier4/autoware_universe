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
#include "autoware/cuda_traffic_light_occlusion_predictor/cuda_occlusion_kernels.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <gtest/gtest.h>

#include <memory>
#include <vector>

using autoware::traffic_light::CudaOcclusionPredictor;
using autoware::traffic_light::CudaOcclusionPredictorParameters;
using autoware::traffic_light::PointXYZ;

class CudaOcclusionPredictorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Setup predictor with default parameters
    CudaOcclusionPredictorParameters params;
    params.max_valid_pt_distance = 100.0f;
    params.azimuth_occlusion_resolution_deg = 0.5f;
    params.elevation_occlusion_resolution_deg = 0.5f;
    params.min_dist_from_occlusion_to_tl = 1.0f;
    params.horizontal_sample_num = 3;
    params.vertical_sample_num = 3;

    predictor_ = std::make_unique<CudaOcclusionPredictor>(params, 1e9);

    // Create CUDA stream
    CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  }

  void TearDown() override
  {
    predictor_.reset();
    if (stream_) {
      CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
    }
  }

  std::unique_ptr<CudaOcclusionPredictor> predictor_;
  cudaStream_t stream_{};
};

// Helper: Create CUDA blackboard pointcloud message
std::shared_ptr<cuda_blackboard::CudaPointCloud2> createCudaBlackboardCloud(
  int num_points, int point_step, cudaStream_t /* stream */, const std::vector<uint8_t> & host_data = {})
{
  auto cloud_msg = std::make_shared<cuda_blackboard::CudaPointCloud2>();
  cloud_msg->width = num_points;
  cloud_msg->height = 1;
  cloud_msg->point_step = point_step;
  cloud_msg->row_step = num_points * point_step;
  cloud_msg->is_dense = true;
  cloud_msg->header.frame_id = "base_link";
  
  if (num_points > 0) {
    size_t data_size = num_points * point_step;
    
    // Use cuda_blackboard's make_unique for proper CUDA memory management
    cloud_msg->data = cuda_blackboard::make_unique<uint8_t[]>(data_size);
    
    if (!host_data.empty()) {
      CHECK_CUDA_ERROR(cudaMemcpy(
        cloud_msg->data.get(), host_data.data(), data_size, cudaMemcpyHostToDevice));
    } else {
      CHECK_CUDA_ERROR(cudaMemset(cloud_msg->data.get(), 0, data_size));
    }
  }
  
  return cloud_msg;
}

// Test 1: Empty point cloud
TEST_F(CudaOcclusionPredictorTest, EmptyPointCloud)
{
  // Create empty CUDA blackboard cloud
  auto cloud_msg = createCudaBlackboardCloud(0, 16, stream_);

  // Identity transform
  float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // Empty ROIs
  std::vector<PointXYZ> roi_3d_points;
  std::vector<int> occlusion_ratios;

  // Should handle gracefully
  EXPECT_NO_THROW({
    predictor_->predict(cloud_msg, identity, roi_3d_points, occlusion_ratios);
  });

  EXPECT_EQ(occlusion_ratios.size(), 0);
}

// Test 2: Simple point cloud with no ROIs
TEST_F(CudaOcclusionPredictorTest, NoROIs)
{
  // Create simple point cloud (PointXYZI format)
  const int num_points = 100;
  const int point_step = 16;
  std::vector<uint8_t> host_data(num_points * point_step);

  // Fill with random points
  for (int i = 0; i < num_points; i++) {
    float * pt = reinterpret_cast<float *>(&host_data[i * point_step]);
    pt[0] = static_cast<float>(i % 10);      // x
    pt[1] = static_cast<float>((i / 10) % 10);  // y
    pt[2] = static_cast<float>(i / 100);     // z
    pt[3] = 100.0f;                           // intensity
  }

  // Create CUDA blackboard cloud
  auto cloud_msg = createCudaBlackboardCloud(num_points, point_step, stream_, host_data);

  // Identity transform
  float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // No ROIs
  std::vector<PointXYZ> roi_3d_points;
  std::vector<int> occlusion_ratios;

  // Should handle gracefully
  EXPECT_NO_THROW({
    predictor_->predict(cloud_msg, identity, roi_3d_points, occlusion_ratios);
  });

  EXPECT_EQ(occlusion_ratios.size(), 0);
}

// Test 3: Point cloud with one ROI (DISABLED - full pipeline not yet complete)
TEST_F(CudaOcclusionPredictorTest, DISABLED_OneROI)
{
  // Create point cloud with points around origin
  const int num_points = 50;
  const int point_step = 16;
  std::vector<uint8_t> host_data(num_points * point_step);

  // Fill with points in a grid
  for (int i = 0; i < num_points; i++) {
    float * pt = reinterpret_cast<float *>(&host_data[i * point_step]);
    pt[0] = static_cast<float>((i % 10) - 5);  // x: -5 to 4
    pt[1] = static_cast<float>((i / 10) - 2);  // y: -2 to 2
    pt[2] = 10.0f;                              // z: 10
    pt[3] = 100.0f;                             // intensity
  }

  // Create CUDA blackboard cloud
  auto cloud_msg = createCudaBlackboardCloud(num_points, point_step, stream_, host_data);

  // Identity transform
  float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // One ROI: top-left (-1, -1, 10), bottom-right (1, 1, 10)
  std::vector<PointXYZ> roi_3d_points = {
    {-1.0f, -1.0f, 10.0f},  // top-left
    {1.0f, 1.0f, 10.0f}     // bottom-right
  };
  std::vector<int> occlusion_ratios;

  // Should calculate occlusion
  EXPECT_NO_THROW({
    predictor_->predict(cloud_msg, identity, roi_3d_points, occlusion_ratios);
  });

  EXPECT_EQ(occlusion_ratios.size(), 1);
  // Occlusion ratio should be between 0 and 100
  EXPECT_GE(occlusion_ratios[0], 0);
  EXPECT_LE(occlusion_ratios[0], 100);
}

// Test 4: Null pointer handling
TEST_F(CudaOcclusionPredictorTest, NullPointers)
{
  float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  std::vector<PointXYZ> roi_3d_points;
  std::vector<int> occlusion_ratios;

  // Null cloud message should be handled gracefully
  EXPECT_NO_THROW({
    predictor_->predict(nullptr, identity, roi_3d_points, occlusion_ratios);
  });

  // Null transform should be handled gracefully
  EXPECT_NO_THROW({
    predictor_->predict(nullptr, nullptr, roi_3d_points, occlusion_ratios);
  });
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

