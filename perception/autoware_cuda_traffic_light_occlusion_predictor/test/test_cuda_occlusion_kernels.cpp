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

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using autoware::traffic_light::PointXYZ;
using autoware::traffic_light::Ray;
using autoware::traffic_light::kernels::extractXYZLaunch;
using autoware::traffic_light::kernels::transformPointCloudLaunch;
using autoware::traffic_light::kernels::convertToSphericalLaunch;

class CudaOcclusionKernelsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  }

  void TearDown() override
  {
    if (stream_) {
      CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
    }
  }

  cudaStream_t stream_{};
};

// Test 1: Extract XYZ from raw point cloud bytes
TEST_F(CudaOcclusionKernelsTest, ExtractXYZKernel)
{
  // Create test point cloud data (PointXYZI format: x, y, z, intensity)
  const int num_points = 10;
  const int point_step = 16;  // 4 floats * 4 bytes
  std::vector<uint8_t> host_data(num_points * point_step);

  // Fill with test data
  for (int i = 0; i < num_points; i++) {
    float * pt = reinterpret_cast<float *>(&host_data[i * point_step]);
    pt[0] = static_cast<float>(i);        // x
    pt[1] = static_cast<float>(i * 2);    // y
    pt[2] = static_cast<float>(i * 3);    // z
    pt[3] = 100.0f;                       // intensity
  }

  // Copy to device
  uint8_t * d_data = nullptr;
  PointXYZ * d_output = nullptr;
  CHECK_CUDA_ERROR(cudaMallocAsync(&d_data, host_data.size(), stream_));
  CHECK_CUDA_ERROR(cudaMallocAsync(&d_output, num_points * sizeof(PointXYZ), stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_data, host_data.data(), host_data.size(), cudaMemcpyHostToDevice, stream_));

  // Launch kernel
  const int threads_per_block = 256;
  const int blocks_per_grid = (num_points + threads_per_block - 1) / threads_per_block;
  extractXYZLaunch(d_data, d_output, num_points, point_step, threads_per_block, blocks_per_grid, stream_);

  // Copy results back
  std::vector<PointXYZ> output(num_points);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output.data(), d_output, num_points * sizeof(PointXYZ), cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Verify
  for (int i = 0; i < num_points; i++) {
    EXPECT_FLOAT_EQ(output[i].x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(output[i].y, static_cast<float>(i * 2));
    EXPECT_FLOAT_EQ(output[i].z, static_cast<float>(i * 3));
  }

  // Cleanup
  CHECK_CUDA_ERROR(cudaFreeAsync(d_data, stream_));
  CHECK_CUDA_ERROR(cudaFreeAsync(d_output, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

// Test 2: Transform point cloud with identity matrix
TEST_F(CudaOcclusionKernelsTest, TransformIdentity)
{
  // Create test points
  const int num_points = 5;
  std::vector<PointXYZ> host_input = {
    {1.0f, 2.0f, 3.0f},
    {4.0f, 5.0f, 6.0f},
    {7.0f, 8.0f, 9.0f},
    {-1.0f, -2.0f, -3.0f},
    {0.0f, 0.0f, 0.0f}
  };

  // Identity transformation matrix (row-major)
  float identity[16] = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };

  // Allocate device memory
  PointXYZ * d_input = nullptr;
  PointXYZ * d_output = nullptr;
  float * d_transform = nullptr;

  CHECK_CUDA_ERROR(cudaMallocAsync(&d_input, num_points * sizeof(PointXYZ), stream_));
  CHECK_CUDA_ERROR(cudaMallocAsync(&d_output, num_points * sizeof(PointXYZ), stream_));
  CHECK_CUDA_ERROR(cudaMallocAsync(&d_transform, 16 * sizeof(float), stream_));

  // Copy to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_input, host_input.data(), num_points * sizeof(PointXYZ), cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_transform, identity, 16 * sizeof(float), cudaMemcpyHostToDevice, stream_));

  // Launch kernel
  const int threads_per_block = 256;
  const int blocks_per_grid = (num_points + threads_per_block - 1) / threads_per_block;
  transformPointCloudLaunch(
    d_input, d_output, d_transform, num_points, threads_per_block, blocks_per_grid, stream_);

  // Copy results back
  std::vector<PointXYZ> output(num_points);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output.data(), d_output, num_points * sizeof(PointXYZ), cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Verify (should be unchanged)
  for (int i = 0; i < num_points; i++) {
    EXPECT_FLOAT_EQ(output[i].x, host_input[i].x);
    EXPECT_FLOAT_EQ(output[i].y, host_input[i].y);
    EXPECT_FLOAT_EQ(output[i].z, host_input[i].z);
  }

  // Cleanup
  CHECK_CUDA_ERROR(cudaFreeAsync(d_input, stream_));
  CHECK_CUDA_ERROR(cudaFreeAsync(d_output, stream_));
  CHECK_CUDA_ERROR(cudaFreeAsync(d_transform, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

// Test 3: Transform with translation
TEST_F(CudaOcclusionKernelsTest, TransformTranslation)
{
  // Create test point
  std::vector<PointXYZ> host_input = {{1.0f, 2.0f, 3.0f}};

  // Translation matrix: translate by (10, 20, 30)
  float translation[16] = {
    1, 0, 0, 10,
    0, 1, 0, 20,
    0, 0, 1, 30,
    0, 0, 0, 1
  };

  // Allocate device memory
  PointXYZ * d_input = nullptr;
  PointXYZ * d_output = nullptr;
  float * d_transform = nullptr;

  CHECK_CUDA_ERROR(cudaMallocAsync(&d_input, sizeof(PointXYZ), stream_));
  CHECK_CUDA_ERROR(cudaMallocAsync(&d_output, sizeof(PointXYZ), stream_));
  CHECK_CUDA_ERROR(cudaMallocAsync(&d_transform, 16 * sizeof(float), stream_));

  // Copy to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_input, host_input.data(), sizeof(PointXYZ), cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_transform, translation, 16 * sizeof(float), cudaMemcpyHostToDevice, stream_));

  // Launch kernel
  transformPointCloudLaunch(d_input, d_output, d_transform, 1, 256, 1, stream_);

  // Copy result back
  PointXYZ output;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &output, d_output, sizeof(PointXYZ), cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Verify
  EXPECT_FLOAT_EQ(output.x, 11.0f);  // 1 + 10
  EXPECT_FLOAT_EQ(output.y, 22.0f);  // 2 + 20
  EXPECT_FLOAT_EQ(output.z, 33.0f);  // 3 + 30

  // Cleanup
  CHECK_CUDA_ERROR(cudaFreeAsync(d_input, stream_));
  CHECK_CUDA_ERROR(cudaFreeAsync(d_output, stream_));
  CHECK_CUDA_ERROR(cudaFreeAsync(d_transform, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

// Test 4: Convert to spherical coordinates
TEST_F(CudaOcclusionKernelsTest, ConvertToSpherical)
{
  // Create test points at known angles
  std::vector<PointXYZ> host_input = {
    {1.0f, 0.0f, 0.0f},   // 0° azimuth, 0° elevation
    {0.0f, 1.0f, 0.0f},   // 90° azimuth, 0° elevation
    {0.0f, 0.0f, 1.0f},   // undefined azimuth, 90° elevation
  };

  const int num_points = host_input.size();

  // Allocate device memory
  PointXYZ * d_input = nullptr;
  Ray * d_output = nullptr;

  CHECK_CUDA_ERROR(cudaMallocAsync(&d_input, num_points * sizeof(PointXYZ), stream_));
  CHECK_CUDA_ERROR(cudaMallocAsync(&d_output, num_points * sizeof(Ray), stream_));

  // Copy to device
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    d_input, host_input.data(), num_points * sizeof(PointXYZ), cudaMemcpyHostToDevice, stream_));

  // Launch kernel
  const int threads_per_block = 256;
  const int blocks_per_grid = (num_points + threads_per_block - 1) / threads_per_block;
  convertToSphericalLaunch(
    d_input, d_output, num_points, threads_per_block, blocks_per_grid, stream_);

  // Copy results back
  std::vector<Ray> output(num_points);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    output.data(), d_output, num_points * sizeof(Ray), cudaMemcpyDeviceToHost, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  // Verify
  EXPECT_NEAR(output[0].azimuth, 0.0f, 1e-3);
  EXPECT_NEAR(output[0].elevation, 0.0f, 1e-3);
  EXPECT_NEAR(output[0].dist, 1.0f, 1e-5);

  EXPECT_NEAR(output[1].azimuth, 90.0f, 1e-3);
  EXPECT_NEAR(output[1].elevation, 0.0f, 1e-3);
  EXPECT_NEAR(output[1].dist, 1.0f, 1e-5);

  EXPECT_NEAR(output[2].elevation, 90.0f, 1e-3);
  EXPECT_NEAR(output[2].dist, 1.0f, 1e-5);

  // Cleanup
  CHECK_CUDA_ERROR(cudaFreeAsync(d_input, stream_));
  CHECK_CUDA_ERROR(cudaFreeAsync(d_output, stream_));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

