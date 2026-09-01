// Copyright 2026 TIER IV, Inc.
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

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

struct DistanceMapTextureTestAccess
{
  template <class CostT>
  static bool updateGrid(
    CostT & cost, DistanceMapTextureGrid & grid, const int width, const int height,
    const float resolution)
  {
    return cost.updateDistanceMapGrid(grid, width, height, resolution);
  }

  template <class CostT>
  static cudaArray_t staticArray(const CostT & cost)
  {
    return cost.static_distance_array_;
  }

  template <class CostT>
  static cudaArray_t obstacleArray(const CostT & cost)
  {
    return cost.obstacle_distance_array_;
  }

  template <class CostT>
  static const DistanceMapTextureVisualizer * visualizer(const CostT & cost)
  {
    return cost.distance_map_visualizer_;
  }
};

namespace
{

constexpr int kTestHorizon = 80;
constexpr float kEmptyDistance = 1.0E8F;
constexpr float kPi = 3.14159265358979323846F;
using TestCost = FirstOrderDubinsBicycleCost<kTestHorizon>;
using TestParams = FirstOrderDubinsBicycleCostParams<kTestHorizon>;
using autoware::mppi_optimizer::Segment;

void cudaCheck(const cudaError_t error, const char * expression)
{
  if (error != cudaSuccess) {
    throw std::runtime_error(std::string(expression) + " failed: " + cudaGetErrorString(error));
  }
}

void cudaCheckNoThrow(const cudaError_t error, const char * expression) noexcept
{
  if (error != cudaSuccess) {
    std::fprintf(stderr, "%s failed during cleanup: %s\n", expression, cudaGetErrorString(error));
  }
}

#define CUDA_CHECK(expression) cudaCheck((expression), #expression)

class CudaStream
{
public:
  CudaStream() { CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking)); }

  ~CudaStream()
  {
    if (stream_ != nullptr) {
      cudaCheckNoThrow(cudaStreamDestroy(stream_), "cudaStreamDestroy(stream_)");
    }
  }

  CudaStream(const CudaStream &) = delete;
  CudaStream & operator=(const CudaStream &) = delete;

  cudaStream_t get() const { return stream_; }

private:
  cudaStream_t stream_{nullptr};
};

template <class T>
class DeviceBuffer
{
public:
  explicit DeviceBuffer(const std::size_t count) : count_(count)
  {
    CUDA_CHECK(cudaMalloc(reinterpret_cast<void **>(&data_), count_ * sizeof(T)));
  }

  ~DeviceBuffer()
  {
    if (data_ != nullptr) {
      cudaCheckNoThrow(cudaFree(data_), "cudaFree(data_)");
    }
  }

  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;

  T * get() const { return data_; }

  void copyFromHost(const T * source, const cudaStream_t stream)
  {
    CUDA_CHECK(cudaMemcpyAsync(data_, source, count_ * sizeof(T), cudaMemcpyHostToDevice, stream));
  }

  void copyToHost(T * destination, const cudaStream_t stream) const
  {
    CUDA_CHECK(
      cudaMemcpyAsync(destination, data_, count_ * sizeof(T), cudaMemcpyDeviceToHost, stream));
  }

private:
  T * data_{nullptr};
  std::size_t count_{0};
};

template <class T>
class PinnedBuffer
{
public:
  explicit PinnedBuffer(const std::size_t count) : count_(count)
  {
    CUDA_CHECK(
      cudaHostAlloc(reinterpret_cast<void **>(&data_), count_ * sizeof(T), cudaHostAllocDefault));
  }

  ~PinnedBuffer()
  {
    if (data_ != nullptr) {
      cudaCheckNoThrow(cudaFreeHost(data_), "cudaFreeHost(data_)");
    }
  }

  PinnedBuffer(const PinnedBuffer &) = delete;
  PinnedBuffer & operator=(const PinnedBuffer &) = delete;

  T * get() const { return data_; }

private:
  T * data_{nullptr};
  std::size_t count_{0};
};

__global__ void sampleStaticTextureKernel(
  const cudaTextureObject_t texture, const DistanceMapTextureGrid grid, const float world_x,
  const float world_y, float2 * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index == 0) {
    const float texture_x = (world_x - grid.origin_x) / grid.resolution;
    const float texture_y = (world_y - grid.origin_y) / grid.resolution;
    output[0] = tex2D<float2>(texture, texture_x, texture_y);
  }
}

__global__ void sampleObstacleTextureKernel(
  const cudaTextureObject_t texture, const DistanceMapTextureGrid grid, const float world_x,
  const float world_y, const int timestep, float * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index == 0) {
    const float texture_x = (world_x - grid.origin_x) / grid.resolution;
    const float texture_y = (world_y - grid.origin_y) / grid.resolution;
    output[0] = tex3D<float>(texture, texture_x, texture_y, static_cast<float>(timestep) + 0.5F);
  }
}

enum class DistanceQuery : int { Obstacle, RoadBorder, DrivableArea };

template <class CostT>
__global__ void sampleCostDistanceKernel(
  const CostT * cost, const float x, const float y, const float yaw, const int timestep,
  const DistanceQuery query, float * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index != 0) {
    return;
  }
  if (query == DistanceQuery::Obstacle) {
    output[0] = cost->distanceToClosestObstacle(x, y, yaw, timestep);
  } else if (query == DistanceQuery::RoadBorder) {
    output[0] = cost->distanceToRoadBorder(x, y, yaw);
  } else {
    output[0] = cost->distanceToDrivableArea(x, y, yaw);
  }
}

template <class CostT>
__global__ void computeStateCostKernel(
  CostT * cost, float * state, const int timestep, float * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index == 0) {
    int crash_status = 0;
    output[0] = cost->computeStateCost(state, timestep, nullptr, &crash_status);
  }
}

float2 sampleStaticTexture(
  const TestCost & cost, const float world_x, const float world_y, const cudaStream_t stream)
{
  DeviceBuffer<float2> output(1);
  sampleStaticTextureKernel<<<1, 32, 0, stream>>>(
    cost.static_distance_texture_, cost.static_distance_map_grid_, world_x, world_y, output.get());
  CUDA_CHECK(cudaGetLastError());
  float2 result{};
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

float sampleObstacleTexture(
  const TestCost & cost, const float world_x, const float world_y, const int timestep,
  const cudaStream_t stream)
{
  DeviceBuffer<float> output(1);
  sampleObstacleTextureKernel<<<1, 32, 0, stream>>>(
    cost.obstacle_distance_texture_, cost.obstacle_distance_map_grid_, world_x, world_y, timestep,
    output.get());
  CUDA_CHECK(cudaGetLastError());
  float result = 0.0F;
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

float sampleCostDistance(
  const TestCost & cost, const float x, const float y, const float yaw, const int timestep,
  const DistanceQuery query, const cudaStream_t stream)
{
  DeviceBuffer<float> output(1);
  sampleCostDistanceKernel<<<1, 32, 0, stream>>>(
    cost.cost_d_, x, y, yaw, timestep, query, output.get());
  CUDA_CHECK(cudaGetLastError());
  float result = 0.0F;
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

std::vector<float2> copyStaticMap(const TestCost & cost, const cudaStream_t stream)
{
  const DistanceMapTextureGrid & grid = cost.static_distance_map_grid_;
  const std::size_t count = static_cast<std::size_t>(grid.width) * grid.height;
  PinnedBuffer<float2> host(count);
  CUDA_CHECK(cudaMemcpy2DFromArrayAsync(
    host.get(), static_cast<std::size_t>(grid.width) * sizeof(float2),
    DistanceMapTextureTestAccess::staticArray(cost), 0, 0,
    static_cast<std::size_t>(grid.width) * sizeof(float2), grid.height, cudaMemcpyDeviceToHost,
    stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return std::vector<float2>(host.get(), host.get() + count);
}

void fillReference(TestCost & cost, const float offset_x = 0.0F, const float offset_y = 0.0F)
{
  std::array<float, kTestHorizon> x{};
  std::array<float, kTestHorizon> y{};
  std::array<float, kTestHorizon> velocity{};
  std::array<float, kTestHorizon> yaw{};
  for (int timestep = 0; timestep < kTestHorizon; ++timestep) {
    x[static_cast<std::size_t>(timestep)] = offset_x;
    y[static_cast<std::size_t>(timestep)] = offset_y;
  }
  cost.setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon, yaw.data());
}

TestParams distanceOnlyParams()
{
  TestParams params;
  params.spatial_overspeed_coeff = 0.0F;
  params.track_coeff = 0.0F;
  params.heading_coeff = 0.0F;
  params.lateral_distance_coeff = 0.0F;
  params.lateral_yaw_error_coeff = 0.0F;
  params.remaining_distance_coeff = 0.0F;
  params.path_overshoot_coeff = 0.0F;
  params.track_center_coeff = 0.0F;
  params.corner_buffer_coeff = 0.0F;
  params.lateral_boundary_barrier_weight = 0.0F;
  params.accel_cmd_coeff = 0.0F;
  params.steer_cmd_coeff = 0.0F;
  params.steer_rate_coeff = 0.0F;
  params.lateral_acceleration_coeff = 0.0F;
  params.lateral_jerk_coeff = 0.0F;
  params.longitudinal_jerk_coeff = 0.0F;
  params.overlimit_coeff = 0.0F;
  params.drivable_area_barrier_weight = 0.0F;
  return params;
}

class DistanceMapGridTest : public ::testing::Test
{
protected:
  void setStraightReference(const float start_x, const float end_x, const float y = 0.0F)
  {
    for (int timestep = 0; timestep < kTestHorizon; ++timestep) {
      const float fraction = static_cast<float>(timestep) / static_cast<float>(kTestHorizon - 1);
      cost_.ref_x_[timestep] = start_x + fraction * (end_x - start_x);
      cost_.ref_y_[timestep] = y;
    }
  }

  bool update(DistanceMapTextureGrid & grid)
  {
    return DistanceMapTextureTestAccess::updateGrid(cost_, grid, 1024, 1024, 0.15F);
  }

  TestCost cost_;
};

TEST_F(DistanceMapGridTest, SubResolutionShiftKeepsSnappedGrid)
{
  DistanceMapTextureGrid grid;
  setStraightReference(0.0F, 20.0F);
  ASSERT_TRUE(update(grid));
  const float original_x = grid.origin_x;
  const float original_y = grid.origin_y;

  setStraightReference(0.05F, 20.05F, 0.05F);
  EXPECT_FALSE(update(grid));
  EXPECT_FLOAT_EQ(grid.origin_x, original_x);
  EXPECT_FLOAT_EQ(grid.origin_y, original_y);
}

TEST_F(DistanceMapGridTest, ShiftBeyondResolutionMovesByWholeTexels)
{
  DistanceMapTextureGrid grid;
  setStraightReference(0.0F, 20.0F);
  ASSERT_TRUE(update(grid));
  const float original_x = grid.origin_x;

  setStraightReference(0.5F, 20.5F);
  ASSERT_TRUE(update(grid));
  const float texel_shift = (grid.origin_x - original_x) / grid.resolution;
  EXPECT_NEAR(texel_shift, std::round(texel_shift), 1.0E-4F);
  EXPECT_GE(std::fabs(grid.origin_x - original_x), grid.resolution);
}

TEST_F(DistanceMapGridTest, GridEnclosesReferenceTrajectory)
{
  DistanceMapTextureGrid grid;
  setStraightReference(-70.0F, 70.0F, 4.0F);
  ASSERT_TRUE(update(grid));
  const float maximum_x = grid.origin_x + static_cast<float>(grid.width) * grid.resolution;
  const float maximum_y = grid.origin_y + static_cast<float>(grid.height) * grid.resolution;
  for (int timestep = 0; timestep < kTestHorizon; ++timestep) {
    EXPECT_GE(cost_.ref_x_[timestep], grid.origin_x);
    EXPECT_LT(cost_.ref_x_[timestep], maximum_x);
    EXPECT_GE(cost_.ref_y_[timestep], grid.origin_y);
    EXPECT_LT(cost_.ref_y_[timestep], maximum_y);
  }
}

class DistanceMapGpuTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    const cudaError_t error = cudaGetDeviceCount(&device_count);
    if (error != cudaSuccess || device_count == 0) {
      GTEST_SKIP() << "A CUDA device is required: " << cudaGetErrorString(error);
    }
    stream_ = std::make_unique<CudaStream>();
    cost_ = std::make_unique<TestCost>(stream_->get());
    cost_->GPUSetup();
    fillReference(*cost_);
  }

  void TearDown() override
  {
    cost_.reset();
    stream_.reset();
  }

  cudaStream_t stream() const { return stream_->get(); }

  std::unique_ptr<CudaStream> stream_;
  std::unique_ptr<TestCost> cost_;
};

TEST_F(DistanceMapGpuTest, EmptyStaticEnvironmentFillsBothChannelsWithEmptyDistance)
{
  cost_->clearRoadBorders();
  cost_->clearDrivableAreaSegments();
  const std::vector<float2> map = copyStaticMap(*cost_, stream());
  EXPECT_TRUE(std::all_of(map.begin(), map.end(), [](const float2 value) {
    return value.x == kEmptyDistance && value.y == kEmptyDistance;
  }));
}

TEST_F(DistanceMapGpuTest, SingleSegmentProducesExpectedDistances)
{
  cost_->setRoadBorderSegments({Segment{0.0F, 0.0F, 0.0F, 10.0F}});
  EXPECT_NEAR(sampleStaticTexture(*cost_, 5.0F, 5.0F, stream()).x, 5.0F, 1.0E-3F);
  EXPECT_NEAR(sampleStaticTexture(*cost_, -5.0F, 5.0F, stream()).x, 5.0F, 1.0E-3F);
  EXPECT_NEAR(sampleStaticTexture(*cost_, 5.0F, -5.0F, stream()).x, std::sqrt(50.0F), 0.03F);
}

TEST_F(DistanceMapGpuTest, StaticTextureChannelsAreIndependent)
{
  cost_->setRoadBorderSegments({Segment{5.0F, -20.0F, 5.0F, 20.0F}});
  cost_->setDrivableAreaSegments({Segment{-20.0F, 5.0F, 20.0F, 5.0F}});
  const float2 sample = sampleStaticTexture(*cost_, 1.0F, 2.0F, stream());
  EXPECT_NEAR(sample.x, 4.0F, 1.0E-3F);
  EXPECT_NEAR(sample.y, 3.0F, 1.0E-3F);
}

TEST_F(DistanceMapGpuTest, StaticObstacleIsExtrudedAcrossTime)
{
  constexpr float x = 2.0F;
  constexpr float y = -1.0F;
  constexpr float yaw = 0.0F;
  constexpr float half_length = 0.8F;
  constexpr float half_width = 0.5F;
  cost_->setOrientedBoxObstacles(&x, &y, &yaw, &half_length, &half_width, 1);
  const float at_start = sampleObstacleTexture(*cost_, 4.0F, 0.0F, 0, stream());
  const float at_end = sampleObstacleTexture(*cost_, 4.0F, 0.0F, kTestHorizon - 1, stream());
  EXPECT_FLOAT_EQ(at_start, at_end);
}

TEST_F(DistanceMapGpuTest, DynamicObstacleFollowsTheTextureTimeAxis)
{
  std::array<float, kTestHorizon> x{};
  std::array<float, kTestHorizon> y{};
  std::array<float, kTestHorizon> yaw{};
  for (int timestep = 0; timestep < kTestHorizon; ++timestep) {
    x[static_cast<std::size_t>(timestep)] = static_cast<float>(timestep);
  }
  constexpr float half_length = 0.6F;
  constexpr float half_width = 0.5F;
  cost_->setOrientedBoxObstacleTrajectories(
    x.data(), y.data(), yaw.data(), &half_length, &half_width, 1, kTestHorizon);

  constexpr float query_x = 10.0F + half_length;
  EXPECT_NEAR(sampleObstacleTexture(*cost_, query_x, 0.0F, 10, stream()), 0.0F, 0.02F);
  EXPECT_NEAR(sampleObstacleTexture(*cost_, query_x, 0.0F, 0, stream()), 10.0F, 0.02F);
}

TEST_F(DistanceMapGpuTest, OrientedBoxDistanceMatchesAnalyticalValue)
{
  constexpr float x = 0.0F;
  constexpr float y = 0.0F;
  constexpr float yaw = kPi * 0.25F;
  constexpr float half_length = 2.0F;
  constexpr float half_width = 0.5F;
  cost_->setOrientedBoxObstacles(&x, &y, &yaw, &half_length, &half_width, 1);
  constexpr float expected_distance = 2.0F;
  const float query_radius = half_length + expected_distance;
  const float query_x = query_radius * 0.70710678118F;
  const float query_y = query_radius * 0.70710678118F;
  EXPECT_NEAR(
    sampleObstacleTexture(*cost_, query_x, query_y, 0, stream()), expected_distance, 0.03F);
}

TEST_F(DistanceMapGpuTest, OutOfBoundsRolloutQueryUsesAnalyticalFallback)
{
  cost_->clearRoadBorders();
  EXPECT_FLOAT_EQ(
    sampleCostDistance(*cost_, 1000.0F, -1000.0F, 0.0F, 0, DistanceQuery::RoadBorder, stream()),
    kEmptyDistance);
}

TEST_F(DistanceMapGpuTest, HardwareBilinearSamplingAveragesFourTexelsAtTheirMidpoint)
{
  cost_->setRoadBorderSegments({Segment{-2.0F, -20.0F, -2.0F, 20.0F}});
  const DistanceMapTextureGrid & grid = cost_->static_distance_map_grid_;
  constexpr int gx = 600;
  constexpr int gy = 600;
  const float x0 = grid.origin_x + (static_cast<float>(gx) + 0.5F) * grid.resolution;
  const float x1 = x0 + grid.resolution;
  const float y0 = grid.origin_y + (static_cast<float>(gy) + 0.5F) * grid.resolution;
  const float y1 = y0 + grid.resolution;
  const float expected = 0.25F * (sampleStaticTexture(*cost_, x0, y0, stream()).x +
                                  sampleStaticTexture(*cost_, x1, y0, stream()).x +
                                  sampleStaticTexture(*cost_, x0, y1, stream()).x +
                                  sampleStaticTexture(*cost_, x1, y1, stream()).x);
  const float actual = sampleStaticTexture(*cost_, 0.5F * (x0 + x1), 0.5F * (y0 + y1), stream()).x;
  EXPECT_NEAR(actual, expected, 1.0E-5F);
}

TEST_F(DistanceMapGpuTest, EgoSpineSubtractsTheConservativeSliceRadius)
{
  TestParams params = distanceOnlyParams();
  params.ego_length = 4.0F;
  params.ego_width = 2.0F;
  cost_->setParams(params);
  cost_->setRoadBorderSegments({Segment{-20.0F, 0.0F, 20.0F, 0.0F}});
  const float slice_half_length = 0.5F * params.ego_length / 4.0F;
  const float circle_radius = std::hypot(slice_half_length, 0.5F * params.ego_width);
  const float expected = 3.0F - circle_radius;
  EXPECT_NEAR(
    sampleCostDistance(*cost_, 0.0F, 3.0F, 0.0F, 0, DistanceQuery::RoadBorder, stream()), expected,
    0.02F);
  EXPECT_NEAR(cost_->distanceToRoadBorder(0.0F, 3.0F, 0.0F), expected, 1.0E-6F);
}

TEST_F(DistanceMapGpuTest, EmptyObstacleSetBypassesGenerationSafely)
{
  cost_->setOrientedBoxObstacles(nullptr, nullptr, nullptr, nullptr, nullptr, 0);
  CUDA_CHECK(cudaStreamSynchronize(stream()));
  EXPECT_EQ(cost_->num_obstacles_, 0);
  EXPECT_TRUE(cost_->obstacle_texture_valid_);
  EXPECT_FALSE(cost_->obstacle_texture_has_obstacles_);
  EXPECT_FLOAT_EQ(
    sampleCostDistance(*cost_, 0.0F, 0.0F, 0.0F, 0, DistanceQuery::Obstacle, stream()),
    kEmptyDistance);
}

TEST_F(DistanceMapGpuTest, GridUpdatesReuseAllocatedTextureResources)
{
  const Segment wall{-5.0F, 3.0F, 5.0F, 3.0F};
  cost_->setRoadBorderSegments({wall});
  const cudaArray_t original_static_array = DistanceMapTextureTestAccess::staticArray(*cost_);
  const cudaArray_t original_obstacle_array = DistanceMapTextureTestAccess::obstacleArray(*cost_);
  const cudaTextureObject_t original_static_texture = cost_->static_distance_texture_;
  const cudaTextureObject_t original_obstacle_texture = cost_->obstacle_distance_texture_;

  for (int iteration = 1; iteration <= 100; ++iteration) {
    fillReference(*cost_, 0.5F * static_cast<float>(iteration));
    cost_->setRoadBorderSegments({wall});
  }
  CUDA_CHECK(cudaStreamSynchronize(stream()));
  EXPECT_EQ(DistanceMapTextureTestAccess::staticArray(*cost_), original_static_array);
  EXPECT_EQ(DistanceMapTextureTestAccess::obstacleArray(*cost_), original_obstacle_array);
  EXPECT_EQ(cost_->static_distance_texture_, original_static_texture);
  EXPECT_EQ(cost_->obstacle_distance_texture_, original_obstacle_texture);
}

TEST_F(DistanceMapGpuTest, HostAndGpuStateCostsUseTheSameFootprintModel)
{
  TestParams params = distanceOnlyParams();
  params.ego_length = 4.0F;
  params.ego_width = 2.0F;
  params.ego_axle_to_box_center = 0.0F;
  params.obstacle_safe_margin = 2.0F;
  params.obstacle_barrier_weight = 1.0F;
  params.road_border_safe_margin = 2.0F;
  params.road_border_barrier_weight = 1.0F;
  cost_->setParams(params);
  cost_->setRoadBorderSegments({Segment{-20.0F, 3.0F, 20.0F, 3.0F}});
  constexpr float obstacle_x = 4.0F;
  constexpr float obstacle_y = 0.0F;
  constexpr float obstacle_yaw = 0.0F;
  constexpr float obstacle_half_length = 0.5F;
  constexpr float obstacle_half_width = 0.5F;
  cost_->setOrientedBoxObstacles(
    &obstacle_x, &obstacle_y, &obstacle_yaw, &obstacle_half_length, &obstacle_half_width, 1);

  TestCost::output_array state = TestCost::output_array::Zero();
  int crash_status = 0;
  const float host_cost = cost_->computeStateCost(state, 0, &crash_status);
  DeviceBuffer<float> device_state(static_cast<std::size_t>(TestCost::OUTPUT_DIM));
  DeviceBuffer<float> device_result(1);
  device_state.copyFromHost(state.data(), stream());
  computeStateCostKernel<<<1, 32, 0, stream()>>>(
    cost_->cost_d_, device_state.get(), 0, device_result.get());
  CUDA_CHECK(cudaGetLastError());
  float gpu_cost = 0.0F;
  device_result.copyToHost(&gpu_cost, stream());
  CUDA_CHECK(cudaStreamSynchronize(stream()));
  EXPECT_NEAR(gpu_cost, host_cost, 0.1F);
}

TEST_F(DistanceMapGpuTest, HeadlessVisualizerFailureLeavesNoLiveVisualizer)
{
  if (std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr) {
    GTEST_SKIP() << "Headless fallback requires an environment without a display server";
  }
  EXPECT_NO_THROW(cost_->setDistanceMapTextureDebugEnabled(true));
  EXPECT_EQ(DistanceMapTextureTestAccess::visualizer(*cost_), nullptr);
  EXPECT_NO_THROW(cost_->renderDistanceMapTextureDebug());
}

TEST_F(DistanceMapGpuTest, VisualizerCanBeEnabledRenderedAndDisabled)
{
  if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr) {
    GTEST_SKIP() << "Visualizer toggle requires a display server";
  }
  cost_->setRoadBorderSegments({Segment{-5.0F, 3.0F, 5.0F, 3.0F}});
  EXPECT_NO_THROW(cost_->setDistanceMapTextureDebugEnabled(true));
  if (DistanceMapTextureTestAccess::visualizer(*cost_) == nullptr) {
    GTEST_SKIP() << "CUDA/OpenGL interoperability is unavailable";
  }
  EXPECT_NO_THROW(cost_->renderDistanceMapTextureDebug());
  EXPECT_NO_THROW(cost_->setDistanceMapTextureDebugEnabled(false));
  EXPECT_EQ(DistanceMapTextureTestAccess::visualizer(*cost_), nullptr);
}

TEST(DistanceMapResourceTest, DestructorReleasesDistanceMapAllocations)
{
  int device_count = 0;
  const cudaError_t device_error = cudaGetDeviceCount(&device_count);
  if (device_error != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "A CUDA device is required: " << cudaGetErrorString(device_error);
  }
  CudaStream stream;
  const auto allocate_and_release = [&stream]() {
    TestCost cost(stream.get());
    cost.GPUSetup();
    fillReference(cost);
    cost.setRoadBorderSegments({Segment{-5.0F, 3.0F, 5.0F, 3.0F}});
    constexpr float obstacle_x = 2.0F;
    constexpr float obstacle_y = 0.0F;
    constexpr float obstacle_yaw = 0.0F;
    constexpr float half_length = 0.5F;
    constexpr float half_width = 0.5F;
    cost.setOrientedBoxObstacles(
      &obstacle_x, &obstacle_y, &obstacle_yaw, &half_length, &half_width, 1);
    CUDA_CHECK(cudaStreamSynchronize(stream.get()));
  };

  // Warm the CUDA module before measuring so one-time JIT/module allocations are excluded.
  allocate_and_release();
  CUDA_CHECK(cudaStreamSynchronize(stream.get()));
  std::size_t free_before = 0;
  std::size_t total_bytes = 0;
  CUDA_CHECK(cudaMemGetInfo(&free_before, &total_bytes));
  allocate_and_release();
  CUDA_CHECK(cudaStreamSynchronize(stream.get()));
  std::size_t free_after = 0;
  CUDA_CHECK(cudaMemGetInfo(&free_after, &total_bytes));
  constexpr std::size_t kAllocatorTolerance = 4U * 1024U * 1024U;
  EXPECT_GE(free_after + kAllocatorTolerance, free_before);
}

#undef CUDA_CHECK

}  // namespace
