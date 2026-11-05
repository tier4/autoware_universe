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

#include "projection_test_cases.hpp"

#include <mppi/core/mppi_common.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <tuple>
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
    return cost.texture_state_.static_distance_array_;
  }

  template <class CostT>
  static cudaArray_t obstacleArray(const CostT & cost)
  {
    return cost.texture_state_.obstacle_distance_array_;
  }

  template <class CostT>
  static cudaArray_t nearestSegmentArray(const CostT & cost)
  {
    return cost.texture_state_.nearest_segment_array_;
  }

  template <class CostT>
  static const DistanceMapTextureVisualizer * visualizer(const CostT & cost)
  {
    return cost.texture_state_.distance_map_visualizer_;
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
    const auto channels = tex2D<float4>(texture, texture_x, texture_y);
    output[0] = make_float2(channels.x, channels.y);
  }
}

__global__ void sampleVegetationTextureKernel(
  const cudaTextureObject_t texture, const DistanceMapTextureGrid grid, const float world_x,
  const float world_y, float * output)
{
  const float texture_x = (world_x - grid.origin_x) / grid.resolution;
  const float texture_y = (world_y - grid.origin_y) / grid.resolution;
  output[0] = tex2D<float4>(texture, texture_x, texture_y).z;
}

__global__ void sampleVegetationCollisionKernel(
  const TestCost * cost, const float x, const float y, bool * output)
{
  output[0] = cost->egoIntersectsVegetation(x, y, 0.0F);
}

__global__ void sampleVegetationRolloutSafetyKernel(
  const TestCost * cost, const float x, const float y, bool * output)
{
  float drivable_cost = 0.0F;
  float obstacle_cost = 0.0F;
  float road_cost = 0.0F;
  bool unsafe = false;
  cost->computeGradualCrashCosts(x, y, 0.0F, 0, drivable_cost, obstacle_cost, road_cost, &unsafe);
  output[0] = unsafe;
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

__global__ void sampleNearestSegmentTextureKernel(
  const cudaTextureObject_t texture, const DistanceMapTextureGrid grid, const float world_x,
  const float world_y, NearestSegmentIndex * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index == 0) {
    const float texture_x = (world_x - grid.origin_x) / grid.resolution;
    const float texture_y = (world_y - grid.origin_y) / grid.resolution;
    output[0] = tex2D<NearestSegmentIndex>(texture, texture_x, texture_y);
  }
}

template <class CostT>
__global__ void sampleProjectionSegmentKernel(
  const CostT * cost, const float x, const float y, int * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index == 0) {
    output[0] = cost->computeLateralPathMetrics(x, y, 0.0F).best_segment_i;
  }
}

enum class DistanceQuery : int { Obstacle, RoadBorder, DrivableArea, PreferredLaneCenter };

__global__ void setProjectionTextureEnabled(TestCost * cost, const bool enabled)
{
  cost->texture_state_.nearest_segment_texture_valid_ = enabled;
}

struct ProjectionSample
{
  TestCost::LateralPathMetrics metrics;
  float initial_hint;
};

struct DeviceProjectionSample
{
  mppi::cost::detail::PolylineProjection projection;
  int projection_status;
  int state_status;
  mppi::cost::detail::PolylineSearchStats stats;
};

// Separate kernel instantiations keep the original device arithmetic independent of the
// prepared and pruned projectors. All read identical uploaded vertices.
template <int Mode>  // 0: original, 1: prepared exhaustive, 2: grouped pruning.
__global__ void compareProjectionKernel(
  const TestCost * cost, const float2 * queries, const int query_count, const int hint_count,
  const bool use_shared, const float boundary_threshold, DeviceProjectionSample * output)
{
  extern __shared__ float theta[];
  cost->initializeCosts(nullptr, nullptr, theta, 0.0F, 0.1F);
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= query_count * hint_count) {
    return;
  }
  const auto & data = cost->runtimeData();
  const bool corridor = data.num_lateral_corridor_points_ >= 2;
  const int count = corridor ? data.num_lateral_corridor_points_ : kTestHorizon;
  const float * x = corridor ? data.lateral_corridor_x_ : data.ref_x_;
  const float * y = corridor ? data.lateral_corridor_y_ : data.ref_y_;
  if (use_shared) {
    x = theta + (corridor ? TestCost::kSharedCorridorXOffset : TestCost::kSharedRefXOffset);
    y = theta + (corridor ? TestCost::kSharedCorridorYOffset : TestCost::kSharedRefYOffset);
  }
  const auto query = queries[index / hint_count];
  const int hint = index % hint_count - 2;
  auto & sample = output[index];
  sample.stats = {};
  if constexpr (Mode == 2) {
    sample.projection = mppi::cost::detail::projectPointToPrunedPolyline<true>(
      query.x, query.y, x, y,
      corridor ? data.lateral_corridor_segment_delta_ : data.ref_segment_delta_,
      corridor ? data.lateral_corridor_group_bounds_ : data.ref_group_bounds_,
      corridor ? data.lateral_corridor_group_bounds_valid_ : data.ref_group_bounds_valid_, count,
      hint, &sample.stats);
  } else if constexpr (Mode == 1) {
    const auto * deltas = corridor ? data.lateral_corridor_segment_delta_ : data.ref_segment_delta_;
    sample.projection = mppi::cost::detail::projectPointToPreparedPolyline(
      query.x, query.y, x, y, deltas, count, hint);
  } else {
    sample.projection =
      mppi::cost::detail::projectPointToPolyline(query.x, query.y, x, y, count, hint);
  }
  constexpr int timestep = 7;
  sample.projection_status = fabsf(sample.projection.lateral_distance) >= boundary_threshold
                               ? mppi::safety::event(1, timestep)
                               : 0;
  sample.state_status = 0;
  if constexpr (Mode != 0) {
    // Check the real state-cost safety path against the original device projection too.
    // The fixture has no obstacles/borders, so only lateral safety can contribute a flag.
    using Output = FirstOrderDubinsBicycleParams::OutputIndex;
    float state[TestCost::OUTPUT_DIM] = {};
    state[static_cast<int>(Output::BASELINK_POS_I_X)] = query.x;
    state[static_cast<int>(Output::BASELINK_POS_I_Y)] = query.y;
    *cost->projectionHintSlot(theta) = static_cast<float>(hint);
    cost->computeStateCost(state, timestep, use_shared ? theta : nullptr, &sample.state_status);
  }
}

struct ProjectionBoundSample
{
  float lower_bound;
  float minimum_distance;
};

__global__ void projectionGroupBoundsKernel(
  const TestCost * cost, const float2 * queries, const int query_count,
  ProjectionBoundSample * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  const auto & data = cost->runtimeData();
  const bool corridor = data.num_lateral_corridor_points_ >= 2;
  const int count = corridor ? data.num_lateral_corridor_points_ : kTestHorizon;
  constexpr int group_size = mppi::cost::detail::kPolylineGroupSize;
  const int groups = (count - 2) / group_size + 1;
  if (index >= query_count * groups) return;
  const auto query = queries[index / groups];
  const int group = index % groups;
  const auto * bounds = corridor ? data.lateral_corridor_group_bounds_ : data.ref_group_bounds_;
  const auto * x = corridor ? data.lateral_corridor_x_ : data.ref_x_;
  const auto * y = corridor ? data.lateral_corridor_y_ : data.ref_y_;
  auto & sample = output[index];
  sample.lower_bound =
    mppi::cost::detail::distanceToPolylineGroupLowerBound(query.x, query.y, bounds[group]);
  sample.minimum_distance = FLT_MAX;
  for (int i = group * group_size; i < min(count - 1, (group + 1) * group_size); ++i) {
    float distance = 0.0F;
    float t_raw = 0.0F;
    // Independent original arithmetic, including endpoint subtraction on device.
    mppi::cost::detail::distanceToPolylineSegment(query.x, query.y, x, y, i, distance, t_raw);
    sample.minimum_distance = fminf(sample.minimum_distance, distance);
  }
}

void expectDeviceProjectionParity(
  TestCost & cost, const std::vector<float2> & queries, const cudaStream_t stream)
{
  ASSERT_FALSE(queries.empty());
  const auto & data = cost.runtimeData();
  const int count =
    data.num_lateral_corridor_points_ >= 2 ? data.num_lateral_corridor_points_ : kTestHorizon;
  const int hint_count = count + 4;  // Every valid segment, plus hints -2, -1, n-1, n, n+1.
  const int query_count = static_cast<int>(queries.size());
  const int sample_count = query_count * hint_count;
  const dim3 block(80, 1, 1);
  const dim3 grid((sample_count + block.x - 1) / block.x, 1, 1);
  const auto shared_bytes = mppi::kernels::calcClassSharedMemSize(&cost, block);
  DeviceBuffer<float2> device_queries(queries.size());
  device_queries.copyFromHost(queries.data(), stream);
  const bool valid_bounds = data.num_lateral_corridor_points_ >= 2
                              ? data.lateral_corridor_group_bounds_valid_
                              : data.ref_group_bounds_valid_;
  if (valid_bounds && mppi::cost::detail::polylinePruningMathSupported()) {
    const int groups = mppi::cost::detail::polylineGroupCount(count);
    DeviceBuffer<ProjectionBoundSample> device_bounds(query_count * groups);
    std::vector<ProjectionBoundSample> bounds(query_count * groups);
    projectionGroupBoundsKernel<<<(bounds.size() + 127) / 128, 128, 0, stream>>>(
      cost.cost_d_, device_queries.get(), query_count, device_bounds.get());
    CUDA_CHECK(cudaGetLastError());
    device_bounds.copyToHost(bounds.data(), stream);
    CUDA_CHECK(cudaStreamSynchronize(stream));
    for (std::size_t i = 0; i < bounds.size(); ++i) {
      SCOPED_TRACE("query=" + std::to_string(i / groups) + " group=" + std::to_string(i % groups));
      ASSERT_TRUE(std::isfinite(bounds[i].lower_bound));
      EXPECT_LE(bounds[i].lower_bound, bounds[i].minimum_distance);
    }
  }
  DeviceBuffer<DeviceProjectionSample> original(sample_count), prepared(sample_count),
    pruned(sample_count);
  std::vector<DeviceProjectionSample> expected(sample_count), actual(sample_count);
  std::vector<DeviceProjectionSample> grouped(sample_count);
  for (bool texture : {true, false}) {
    SCOPED_TRACE(texture ? "texture seed" : "no texture");
    setProjectionTextureEnabled<<<1, 1, 0, stream>>>(cost.cost_d_, texture);
    CUDA_CHECK(cudaGetLastError());
    for (bool shared : {false, true}) {
      SCOPED_TRACE(shared ? "shared vertices" : "global vertices");
      compareProjectionKernel<0><<<grid, block, shared_bytes, stream>>>(
        cost.cost_d_, device_queries.get(), query_count, hint_count, shared,
        cost.getParams().boundary_threshold, original.get());
      CUDA_CHECK(cudaGetLastError());
      compareProjectionKernel<1><<<grid, block, shared_bytes, stream>>>(
        cost.cost_d_, device_queries.get(), query_count, hint_count, shared,
        cost.getParams().boundary_threshold, prepared.get());
      CUDA_CHECK(cudaGetLastError());
      compareProjectionKernel<2><<<grid, block, shared_bytes, stream>>>(
        cost.cost_d_, device_queries.get(), query_count, hint_count, shared,
        cost.getParams().boundary_threshold, pruned.get());
      CUDA_CHECK(cudaGetLastError());
      original.copyToHost(expected.data(), stream);
      prepared.copyToHost(actual.data(), stream);
      pruned.copyToHost(grouped.data(), stream);
      CUDA_CHECK(cudaStreamSynchronize(stream));
      for (int i = 0; i < sample_count; ++i) {
        SCOPED_TRACE(i / hint_count);
        SCOPED_TRACE(i % hint_count - 2);
        const auto & a = expected[i];
        ASSERT_TRUE(std::isfinite(a.projection.best_t_raw));
        ASSERT_TRUE(std::isfinite(a.projection.lateral_distance));
        for (const auto * candidate : {&actual[i], &grouped[i]}) {
          const auto & b = *candidate;
          EXPECT_EQ(b.projection.best_i, a.projection.best_i);
          // EXPECT_EQ deliberately allows no float/ULP tolerance for device-to-device parity.
          EXPECT_EQ(b.projection.best_t_raw, a.projection.best_t_raw);
          EXPECT_EQ(b.projection.lateral_distance, a.projection.lateral_distance);
          EXPECT_EQ(b.projection_status, a.projection_status);
          EXPECT_EQ(b.state_status, a.projection_status);
        }
        EXPECT_LE(grouped[i].stats.segments_evaluated, static_cast<unsigned int>(count - 1));
      }
    }
  }
  setProjectionTextureEnabled<<<1, 1, 0, stream>>>(cost.cost_d_, true);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaStreamSynchronize(stream));
}

struct RawProjectionSample
{
  mppi::cost::detail::PolylineProjection original;
  mppi::cost::detail::PolylineProjection pruned;
  mppi::cost::detail::PolylineSearchStats stats;
  unsigned int bound_violations;
};

template <int GroupSize>
__global__ void rawPrunedProjectionKernel(
  const float * x, const float * y, const mppi::cost::detail::PolylineSegmentDelta * deltas,
  const mppi::cost::detail::PolylineGroupBounds * bounds, const bool valid, const int count,
  const float2 * queries, const int query_count, const int hint_count, RawProjectionSample * output)
{
  using namespace mppi::cost::detail;
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index >= query_count * hint_count) return;
  const auto query = queries[index / hint_count];
  const int hint = index % hint_count - 2;
  auto & result = output[index];
  result.original = projectPointToPolyline(query.x, query.y, x, y, count, hint);
  result.pruned = projectPointToPrunedPolyline<true, GroupSize>(
    query.x, query.y, x, y, deltas, bounds, valid, count, hint, &result.stats);
  result.bound_violations = 0;
  if (
    !valid || !polylinePruningMathSupported() ||
    !(fabsf(query.x) <= kPolylinePruningCoordinateLimit) ||
    !(fabsf(query.y) <= kPolylinePruningCoordinateLimit))
    return;
  for (int group = 0; group * GroupSize < count - 1; ++group) {
    const float lower = distanceToPolylineGroupLowerBound(query.x, query.y, bounds[group]);
    for (int i = group * GroupSize; i < min(count - 1, (group + 1) * GroupSize); ++i) {
      float distance = 0.0F;
      float t_raw = 0.0F;
      distanceToPolylineSegment(query.x, query.y, x, y, i, distance, t_raw);
      if (!(lower <= distance)) ++result.bound_violations;
    }
  }
}

template <int GroupSize = mppi::cost::detail::kPolylineGroupSize>
void expectRawPruningParity(
  std::vector<float> x, std::vector<float> y, const std::vector<float2> & queries,
  const cudaStream_t stream, const bool force_invalid = false, const bool report_work = false,
  const bool require_pruning = false)
{
  using namespace mppi::cost::detail;
  ASSERT_EQ(x.size(), y.size());
  ASSERT_FALSE(queries.empty());
  const int count = static_cast<int>(x.size());
  std::vector<PolylineSegmentDelta> deltas(std::max(1, count - 1));
  std::vector<PolylineGroupBounds> bounds(std::max(1, polylineGroupCount(count, GroupSize)));
  for (int i = 0; i < count - 1; ++i) {
    deltas[i] = makePolylineSegmentDelta(x[i], y[i], x[i + 1], y[i + 1]);
  }
  const bool built =
    buildPolylineGroupBounds<GroupSize>(x.data(), y.data(), deltas.data(), count, bounds.data());
  bool expected_valid = count >= 2;
  for (int i = 0; i < count; ++i) {
    expected_valid &= std::fabs(x[i]) <= kPolylinePruningCoordinateLimit &&
                      std::fabs(y[i]) <= kPolylinePruningCoordinateLimit;
  }
  ASSERT_EQ(built, expected_valid);
  const bool valid = built && !force_invalid;
  // Nonempty allocations allow the empty-polyline device case without dereferencing nullptr.
  if (x.empty()) {
    x.push_back(0.0F);
    y.push_back(0.0F);
  }
  DeviceBuffer<float> device_x(x.size()), device_y(y.size());
  DeviceBuffer<PolylineSegmentDelta> device_deltas(deltas.size());
  DeviceBuffer<PolylineGroupBounds> device_bounds(bounds.size());
  DeviceBuffer<float2> device_queries(queries.size());
  const int hints = count + 4;
  const int samples = static_cast<int>(queries.size()) * hints;
  DeviceBuffer<RawProjectionSample> device_results(samples);
  std::vector<RawProjectionSample> results(samples);
  device_x.copyFromHost(x.data(), stream);
  device_y.copyFromHost(y.data(), stream);
  device_deltas.copyFromHost(deltas.data(), stream);
  device_bounds.copyFromHost(bounds.data(), stream);
  device_queries.copyFromHost(queries.data(), stream);
  rawPrunedProjectionKernel<GroupSize><<<(samples + 127) / 128, 128, 0, stream>>>(
    device_x.get(), device_y.get(), device_deltas.get(), device_bounds.get(), valid, count,
    device_queries.get(), static_cast<int>(queries.size()), hints, device_results.get());
  CUDA_CHECK(cudaGetLastError());
  device_results.copyToHost(results.data(), stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  std::uint64_t examined = 0, pruned = 0, evaluated = 0, fallbacks = 0;
  for (int i = 0; i < samples; ++i) {
    SCOPED_TRACE("query=" + std::to_string(i / hints) + " hint=" + std::to_string(i % hints - 2));
    const auto & result = results[i];
    EXPECT_EQ(result.pruned.best_i, result.original.best_i);
    if (std::isnan(result.original.best_t_raw)) {
      EXPECT_TRUE(std::isnan(result.pruned.best_t_raw));
    } else {
      EXPECT_EQ(result.pruned.best_t_raw, result.original.best_t_raw);
    }
    if (std::isnan(result.original.lateral_distance)) {
      EXPECT_TRUE(std::isnan(result.pruned.lateral_distance));
    } else {
      EXPECT_EQ(result.pruned.lateral_distance, result.original.lateral_distance);
    }
    EXPECT_EQ(result.bound_violations, 0U);
    const auto query = queries[i / hints];
    const bool fallback = !valid || !polylinePruningMathSupported() || count <= GroupSize + 1 ||
                          !(std::fabs(query.x) <= kPolylinePruningCoordinateLimit) ||
                          !(std::fabs(query.y) <= kPolylinePruningCoordinateLimit);
    EXPECT_EQ(result.stats.exhaustive_fallbacks, fallback ? 1U : 0U);
    EXPECT_LE(result.stats.segments_evaluated, static_cast<unsigned int>(std::max(0, count - 1)));
    examined += result.stats.groups_examined;
    pruned += result.stats.groups_pruned;
    evaluated += result.stats.segments_evaluated;
    fallbacks += result.stats.exhaustive_fallbacks;
  }
  if (require_pruning && polylinePruningMathSupported()) {
    EXPECT_GT(pruned, 0U);
    EXPECT_LT(evaluated, static_cast<std::uint64_t>(samples) * (count - 1));
    EXPECT_EQ(fallbacks, 0U);
  }
  if (report_work) {
    std::printf(
      "[projection work] group_size=%d points=%d queries_with_hints=%d groups_examined=%llu "
      "groups_pruned=%llu segments_evaluated=%llu exhaustive_segments=%llu fallbacks=%llu\n",
      GroupSize, count, samples, static_cast<unsigned long long>(examined),
      static_cast<unsigned long long>(pruned), static_cast<unsigned long long>(evaluated),
      static_cast<unsigned long long>(samples) * std::max(0, count - 1),
      static_cast<unsigned long long>(fallbacks));
  }
}

__global__ void sampleProjectionSequenceKernel(
  TestCost * cost, const float2 * queries, const int count, const bool use_shared,
  ProjectionSample * output)
{
  extern __shared__ float theta[];
  cost->initializeCosts(nullptr, nullptr, theta, 0.0F, 0.1F);
  if (threadIdx.y == 0) {
    const float initial_hint = *cost->projectionHintSlot(theta);
    // Lane zero tests fresh initialization. Other lanes start from deliberately unrelated
    // hints. Later queries reuse the result written by the preceding query in that lane.
    if (threadIdx.x > 0) {
      *cost->projectionHintSlot(theta) = static_cast<float>(threadIdx.x - 1);
    }
    for (int i = 0; i < count; ++i) {
      auto & sample = output[threadIdx.x * count + i];
      sample.initial_hint = initial_hint;
      sample.metrics = cost->computeLateralPathMetrics(
        queries[i].x, queries[i].y, 0.3F, use_shared ? theta : nullptr);
    }
  }
}

void expectProjectionSequence(
  TestCost & cost, const std::vector<float2> & queries, const cudaStream_t stream)
{
  constexpr int lanes = 32;
  const dim3 block(lanes, 2, 1);
  const auto shared_bytes = mppi::kernels::calcClassSharedMemSize(&cost, block);
  DeviceBuffer<float2> device_queries(queries.size());
  CUDA_CHECK(cudaMemcpyAsync(
    device_queries.get(), queries.data(), queries.size() * sizeof(float2), cudaMemcpyHostToDevice,
    stream));
  DeviceBuffer<ProjectionSample> output(lanes * queries.size());
  std::vector<ProjectionSample> actual(lanes * queries.size());
  for (bool texture : {true, false}) {
    SCOPED_TRACE(texture ? "texture seed" : "no texture");
    setProjectionTextureEnabled<<<1, 1, 0, stream>>>(cost.cost_d_, texture);
    CUDA_CHECK(cudaGetLastError());
    for (bool shared : {true, false}) {
      SCOPED_TRACE(shared ? "persistent shared hint" : "no shared hint");
      sampleProjectionSequenceKernel<<<1, block, shared_bytes, stream>>>(
        cost.cost_d_, device_queries.get(), static_cast<int>(queries.size()), shared, output.get());
      CUDA_CHECK(cudaGetLastError());
      output.copyToHost(actual.data(), stream);
      CUDA_CHECK(cudaStreamSynchronize(stream));
      for (int lane = 0; lane < lanes; ++lane) {
        SCOPED_TRACE(lane);
        for (std::size_t i = 0; i < queries.size(); ++i) {
          SCOPED_TRACE(i);
          const auto expected = cost.computeLateralPathMetrics(queries[i].x, queries[i].y, 0.3F);
          const auto & sample = actual[lane * queries.size() + i];
          EXPECT_FLOAT_EQ(sample.initial_hint, -1.0F);
          const auto & value = sample.metrics;
          const auto & data = cost.runtimeData();
          const bool corridor = data.num_lateral_corridor_points_ >= 2;
          const float * oracle_x = corridor ? data.lateral_corridor_x_ : data.ref_x_;
          const float * oracle_y = corridor ? data.lateral_corridor_y_ : data.ref_y_;
          const int oracle_count = corridor ? data.num_lateral_corridor_points_ : kTestHorizon;
          // This is the unchanged exhaustive search, independent of the prepared deltas.
          const auto oracle = mppi::cost::detail::projectPointToPolyline(
            queries[i].x, queries[i].y, oracle_x, oracle_y, oracle_count,
            static_cast<int>(sample.initial_hint));
          const float * oracle_s = corridor ? data.lateral_corridor_s_ : data.ref_s_;
          const float total_s =
            corridor ? data.lateral_corridor_total_length_s_ : data.ref_s_[kTestHorizon - 1];
          float oracle_progress = 0.0F;
          float oracle_remaining = 0.0F;
          float oracle_overshoot = 0.0F;
          mppi::cost::detail::pathLengthAtProjection(
            oracle, oracle_x, oracle_y, oracle_s, oracle_count, total_s, oracle_progress,
            oracle_remaining, oracle_overshoot);
          const float * oracle_v = corridor ? data.lateral_corridor_ref_velocity_ : data.ref_v_;
          const float segment_t = std::clamp(oracle.best_t_raw, 0.0F, 1.0F);
          const float oracle_velocity =
            oracle_v[oracle.best_i] +
            segment_t * (oracle_v[oracle.best_i + 1] - oracle_v[oracle.best_i]);
          EXPECT_EQ(expected.best_segment_i, oracle.best_i);
          EXPECT_NEAR(expected.lateral_distance, oracle.lateral_distance, 1.0E-4F);
          EXPECT_EQ(value.best_segment_i, oracle.best_i);
          EXPECT_NEAR(value.lateral_distance, oracle.lateral_distance, 1.0E-4F);
          EXPECT_NEAR(value.path_length_s, oracle_progress, 1.0E-4F);
          EXPECT_NEAR(value.remaining_distance_s, oracle_remaining, 1.0E-4F);
          EXPECT_NEAR(value.overshoot_distance_s, oracle_overshoot, 1.0E-4F);
          EXPECT_NEAR(value.spatial_ref_velocity, oracle_velocity, 1.0E-4F);
          EXPECT_EQ(value.best_segment_i, expected.best_segment_i);
          // Analytical projection has no texture-resolution error allowance.
          EXPECT_NEAR(value.lateral_distance, expected.lateral_distance, 1.0E-4F);
          EXPECT_NEAR(value.lateral_yaw_error_sq, expected.lateral_yaw_error_sq, 1.0E-4F);
          EXPECT_NEAR(value.path_length_s, expected.path_length_s, 1.0E-4F);
          EXPECT_NEAR(value.spatial_s, expected.spatial_s, 1.0E-4F);
          EXPECT_NEAR(value.spatial_ref_velocity, expected.spatial_ref_velocity, 1.0E-4F);
          EXPECT_NEAR(value.remaining_distance_s, expected.remaining_distance_s, 1.0E-4F);
          EXPECT_NEAR(value.overshoot_distance_s, expected.overshoot_distance_s, 1.0E-4F);
        }
      }
    }
  }
  setProjectionTextureEnabled<<<1, 1, 0, stream>>>(cost.cost_d_, true);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaStreamSynchronize(stream));
}

template <class CostT>
__global__ void sampleCostDistanceKernel(
  const CostT * cost, const float x, const float y, const float yaw, const int timestep,
  const DistanceQuery query, float * output)
{
  const int index = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
  if (index != 0) {
    return;
  }
  if (query == DistanceQuery::PreferredLaneCenter) {
    output[0] = cost->computePreferredLaneCenterCost(x, y);
  } else if (query == DistanceQuery::Obstacle) {
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
    cost.texture_state_.static_distance_texture_, cost.texture_state_.static_distance_map_grid_,
    world_x, world_y, output.get());
  CUDA_CHECK(cudaGetLastError());
  float2 result{};
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

float sampleVegetationTexture(
  const TestCost & cost, const float world_x, const float world_y, const cudaStream_t stream)
{
  DeviceBuffer<float> output(1);
  sampleVegetationTextureKernel<<<1, 1, 0, stream>>>(
    cost.texture_state_.static_distance_texture_, cost.texture_state_.static_distance_map_grid_,
    world_x, world_y, output.get());
  CUDA_CHECK(cudaGetLastError());
  float result = 0.0F;
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

bool sampleVegetationCollision(
  const TestCost & cost, const float x, const float y, const cudaStream_t stream)
{
  DeviceBuffer<bool> output(1);
  sampleVegetationCollisionKernel<<<1, 1, 0, stream>>>(cost.cost_d_, x, y, output.get());
  CUDA_CHECK(cudaGetLastError());
  bool result = false;
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

bool sampleVegetationRolloutSafety(
  const TestCost & cost, const float x, const float y, const cudaStream_t stream)
{
  DeviceBuffer<bool> output(1);
  sampleVegetationRolloutSafetyKernel<<<1, 1, 0, stream>>>(cost.cost_d_, x, y, output.get());
  CUDA_CHECK(cudaGetLastError());
  bool result = false;
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
    cost.texture_state_.obstacle_distance_texture_, cost.texture_state_.obstacle_distance_map_grid_,
    world_x, world_y, timestep, output.get());
  CUDA_CHECK(cudaGetLastError());
  float result = 0.0F;
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

NearestSegmentIndex sampleNearestSegmentTexture(
  const TestCost & cost, const float world_x, const float world_y, const cudaStream_t stream)
{
  DeviceBuffer<NearestSegmentIndex> output(1);
  sampleNearestSegmentTextureKernel<<<1, 32, 0, stream>>>(
    cost.texture_state_.nearest_segment_texture_, cost.texture_state_.nearest_segment_map_grid_,
    world_x, world_y, output.get());
  CUDA_CHECK(cudaGetLastError());
  NearestSegmentIndex result = 0;
  output.copyToHost(&result, stream);
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return result;
}

int sampleProjectionSegment(
  const TestCost & cost, const float x, const float y, const cudaStream_t stream)
{
  DeviceBuffer<int> output(1);
  sampleProjectionSegmentKernel<<<1, 32, 0, stream>>>(cost.cost_d_, x, y, output.get());
  CUDA_CHECK(cudaGetLastError());
  int result = -1;
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

std::vector<float4> copyStaticMap(const TestCost & cost, const cudaStream_t stream)
{
  const DistanceMapTextureGrid & grid = cost.texture_state_.static_distance_map_grid_;
  const std::size_t count = static_cast<std::size_t>(grid.width) * grid.height;
  PinnedBuffer<float4> host(count);
  CUDA_CHECK(cudaMemcpy2DFromArrayAsync(
    host.get(), static_cast<std::size_t>(grid.width) * sizeof(float4),
    DistanceMapTextureTestAccess::staticArray(cost), 0, 0,
    static_cast<std::size_t>(grid.width) * sizeof(float4), grid.height, cudaMemcpyDeviceToHost,
    stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));
  return std::vector<float4>(host.get(), host.get() + count);
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
      cost_.runtimeData().ref_x_[timestep] = start_x + fraction * (end_x - start_x);
      cost_.runtimeData().ref_y_[timestep] = y;
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
    EXPECT_GE(cost_.runtimeData().ref_x_[timestep], grid.origin_x);
    EXPECT_LT(cost_.runtimeData().ref_x_[timestep], maximum_x);
    EXPECT_GE(cost_.runtimeData().ref_y_[timestep], grid.origin_y);
    EXPECT_LT(cost_.runtimeData().ref_y_[timestep], maximum_y);
  }
}

class DistanceMapGpuTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    const cudaError_t error = cudaGetDeviceCount(&device_count);
    if (error == cudaErrorNoDevice || (error == cudaSuccess && device_count == 0)) {
      GTEST_SKIP() << "A CUDA device is required: " << cudaGetErrorString(error);
    }
    ASSERT_EQ(error, cudaSuccess) << cudaGetErrorString(error);
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

TEST_F(DistanceMapGpuTest, PreferredLaneCenterTextureCacheAndExactFallback)
{
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_array_, nullptr);
  auto params = cost_->getParams();
  params.preferred_lane_center_coeff = 2.0F;
  cost_->setParams(params);
  cost_->beginDataUpdate();
  ASSERT_EQ(cost_->setPreferredLaneCenterSegments({{-200, 0, 200, 0}}), "active");
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 0U);
  cost_->commitDataUpdate();
  const auto array = cost_->texture_state_.preferred_lane_center_array_;
  ASSERT_NE(array, nullptr);
  ASSERT_TRUE(cost_->texture_state_.preferred_lane_center_texture_valid_);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 1U);
  const auto sample = [&](float x, float y) {
    return sampleCostDistance(*cost_, x, y, 0, 0, DistanceQuery::PreferredLaneCenter, stream());
  };
  for (float y : {-2.0F, -0.5F, 0.0F, 0.5F, 2.0F}) {
    const float d = std::abs(y), epsilon = 0.215F;
    EXPECT_NEAR(sample(0, y), 2 * d * d, 2 * (2 * d * epsilon + epsilon * epsilon));
  }
  // Both the outer half-texel band and points outside the field must be exact.
  const auto grid = cost_->texture_state_.preferred_lane_center_grid_;
  EXPECT_NEAR(sample(grid.origin_x + 0.1F * grid.resolution, 2), 8, 1.0E-4);
  EXPECT_NEAR(sample(300, 2), 20008, 0.01);
  cost_->setPreferredLaneCenterSegments({{-200, 0, 200, 0}});
  params.preferred_lane_center_coeff = 4;
  cost_->setParams(params);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 1U);
  EXPECT_NEAR(sample(0, 2), 16, 0.01);
  cost_->setPreferredLaneCenterSegments({{-200, 3, 200, 3}});
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 2U);
  EXPECT_NEAR(sample(0, 2), 4, 0.01);
  cost_->setPreferredLaneCenterTextureEnabled(false);
  EXPECT_FALSE(cost_->texture_state_.preferred_lane_center_texture_valid_);
  EXPECT_NEAR(sample(0, 3), 0, 1.0E-6);
  cost_->setPreferredLaneCenterTextureEnabled(true);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 3U);
  fillReference(*cost_, 1.0F);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 4U);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_array_, array);
  params.preferred_lane_center_coeff = 0;
  cost_->setParams(params);
  EXPECT_FLOAT_EQ(sample(0, 2), 0);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 4U);
  params.preferred_lane_center_coeff = 4;
  cost_->setParams(params);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_build_count_, 5U);
  cost_->setPreferredLaneCenterSegments({});
  EXPECT_FALSE(cost_->texture_state_.preferred_lane_center_texture_valid_);
  EXPECT_FLOAT_EQ(sample(0, 2), 0);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_array_, array);
  cost_->freeCudaMem();
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_array_, nullptr);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_surface_, 0U);
  EXPECT_EQ(cost_->texture_state_.preferred_lane_center_texture_, 0U);
  cost_->GPUSetup();
  cost_->setPreferredLaneCenterSegments({{-200, 0, 200, 0}});
  EXPECT_TRUE(cost_->texture_state_.preferred_lane_center_texture_valid_);
  EXPECT_NEAR(sample(0, 2), 16, 0.01);
}

TEST_F(DistanceMapGpuTest, EmptyStaticEnvironmentFillsBothChannelsWithEmptyDistance)
{
  cost_->clearRoadBorders();
  cost_->clearDrivableAreaSegments();
  const std::vector<float4> map = copyStaticMap(*cost_, stream());
  EXPECT_TRUE(std::all_of(map.begin(), map.end(), [](const float4 value) {
    return value.x == kEmptyDistance && value.y == kEmptyDistance;
  }));
}

TEST_F(DistanceMapGpuTest, VegetationUsesThirdStaticChannelAndClearsBetweenCycles)
{
  const auto road_before = sampleStaticTexture(*cost_, 5.0F, 0.0F, stream());
  cost_->setVegetationPoints({{5.0F, 0.0F}, {5.0F, 2.0F}});
  ASSERT_TRUE(cost_->texture_state_.vegetation_texture_valid_);
  EXPECT_LT(sampleVegetationTexture(*cost_, 5.0F, 0.0F, stream()), 0.22F);
  EXPECT_NEAR(sampleVegetationTexture(*cost_, 5.0F, 1.0F, stream()), 1.0F, 0.22F);
  cost_->beginDataUpdate();
  cost_->setRoadBorderSegments({Segment{-20.0F, 10.0F, 20.0F, 10.0F}});
  cost_->setDrivableAreaSegments({Segment{-20.0F, -10.0F, 20.0F, -10.0F}});
  cost_->commitDataUpdate();
  EXPECT_LT(sampleVegetationTexture(*cost_, 5.0F, 0.0F, stream()), 0.22F);
  const auto road_after = sampleStaticTexture(*cost_, 5.0F, 0.0F, stream());
  EXPECT_NE(road_after.x, road_before.x);
  EXPECT_NE(road_after.y, road_before.y);
  auto params = cost_->getParams();
  params.ego_length = 1.0F;
  params.ego_width = 1.0F;
  params.ego_axle_to_box_center = 0.0F;
  params.vegetation_collision_margin = 0.2F;
  cost_->setParams(params);
  EXPECT_TRUE(cost_->egoIntersectsVegetation(5.0F, 0.0F, 0.0F));
  EXPECT_FALSE(cost_->egoIntersectsVegetation(5.0F, 4.0F, 0.0F));
  EXPECT_TRUE(sampleVegetationCollision(*cost_, 5.0F, 0.0F, stream()));
  EXPECT_FALSE(sampleVegetationCollision(*cost_, 5.0F, 4.0F, stream()));
  cost_->setVegetationPoints({});
  EXPECT_FALSE(cost_->texture_state_.vegetation_texture_valid_);
  EXPECT_FALSE(cost_->egoIntersectsVegetation(5.0F, 0.0F, 0.0F));
  EXPECT_FALSE(sampleVegetationCollision(*cost_, 5.0F, 0.0F, stream()));
}

TEST_F(DistanceMapGpuTest, VegetationRolloutSafetyMatchesExactFootprintNearClearance)
{
  auto params = cost_->getParams();
  params.ego_length = 1.0F;
  params.ego_width = 1.0F;
  params.ego_axle_to_box_center = 0.0F;
  params.vegetation_collision_margin = 0.2F;
  cost_->setParams(params);
  cost_->setVegetationPoints({{5.0, 0.0}});

  for (const auto & [x, y, expected] : std::array<std::tuple<float, float, bool>, 4>{
         {{5.0F, 0.69F, true}, {5.0F, 0.8F, false}, {5.8F, 0.8F, false}, {500.0F, 0.0F, false}}}) {
    EXPECT_EQ(cost_->egoIntersectsVegetation(x, y, 0.0F), expected);
    EXPECT_EQ(sampleVegetationCollision(*cost_, x, y, stream()), expected);
    EXPECT_EQ(sampleVegetationRolloutSafety(*cost_, x, y, stream()), expected);
  }

  cost_->setVegetationPoints({{8.0, 0.0}, {5.0, 0.0}});
  EXPECT_TRUE(sampleVegetationCollision(*cost_, 8.0F, 0.0F, stream()));
  cost_->setVegetationPoints({{8.0, 0.0}});
  EXPECT_FALSE(sampleVegetationCollision(*cost_, 5.0F, 0.0F, stream()));
  EXPECT_TRUE(sampleVegetationRolloutSafety(*cost_, 8.0F, 0.0F, stream()));
}

TEST_F(DistanceMapGpuTest, SingleSegmentProducesExpectedDistances)
{
  cost_->setRoadBorderSegments({Segment{0.0F, 0.0F, 0.0F, 10.0F}});
  EXPECT_NEAR(sampleStaticTexture(*cost_, 5.0F, 5.0F, stream()).x, 5.0F, 1.0E-3F);
  EXPECT_NEAR(sampleStaticTexture(*cost_, -5.0F, 5.0F, stream()).x, 5.0F, 1.0E-3F);
  EXPECT_NEAR(sampleStaticTexture(*cost_, 5.0F, -5.0F, stream()).x, std::sqrt(50.0F), 0.03F);
}

TEST_F(DistanceMapGpuTest, DataUpdatePublishesGeometryOnlyAtCommit)
{
  cost_->setRoadBorderSegments({Segment{0.0F, -20.0F, 0.0F, 20.0F}});
  EXPECT_NEAR(sampleStaticTexture(*cost_, 2.0F, 0.0F, stream()).x, 2.0F, 1.0E-3F);

  cost_->beginDataUpdate();
  cost_->setRoadBorderSegments({Segment{10.0F, -20.0F, 10.0F, 20.0F}});
  EXPECT_NEAR(sampleStaticTexture(*cost_, 2.0F, 0.0F, stream()).x, 2.0F, 1.0E-3F);

  cost_->commitDataUpdate();
  EXPECT_NEAR(sampleStaticTexture(*cost_, 2.0F, 0.0F, stream()).x, 8.0F, 1.0E-3F);
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
  const DistanceMapTextureGrid & grid = cost_->texture_state_.static_distance_map_grid_;
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

TEST_F(DistanceMapGpuTest, NearestSegmentTextureSelectsAndSeedsTheExpectedSegment)
{
  TestParams params = distanceOnlyParams();
  params.lateral_distance_coeff = 1.0F;
  cost_->setParams(params);
  const std::array<float, 3> corridor_x{0.0F, 10.0F, 10.0F};
  const std::array<float, 3> corridor_y{0.0F, 0.0F, 10.0F};
  cost_->setLateralCorridor(
    corridor_x.data(), corridor_y.data(), static_cast<int>(corridor_x.size()));

  ASSERT_TRUE(cost_->texture_state_.nearest_segment_texture_valid_);
  ASSERT_NE(cost_->texture_state_.nearest_segment_texture_, 0U);
  EXPECT_EQ(sampleNearestSegmentTexture(*cost_, 5.0F, 1.0F, stream()), 0U);
  EXPECT_EQ(sampleNearestSegmentTexture(*cost_, 9.0F, 5.0F, stream()), 1U);
  EXPECT_EQ(sampleProjectionSegment(*cost_, 5.0F, 1.0F, stream()), 0);
  EXPECT_EQ(sampleProjectionSegment(*cost_, 9.0F, 5.0F, stream()), 1);

  const cudaArray_t original_array = DistanceMapTextureTestAccess::nearestSegmentArray(*cost_);
  const std::array<float, 3> shifted_y{1.0F, 1.0F, 11.0F};
  cost_->setLateralCorridor(
    corridor_x.data(), shifted_y.data(), static_cast<int>(corridor_x.size()));
  EXPECT_EQ(DistanceMapTextureTestAccess::nearestSegmentArray(*cost_), original_array);
}

TEST_F(DistanceMapGpuTest, OutOfBoundsProjectionRetainsAnalyticalFallback)
{
  TestParams params = distanceOnlyParams();
  params.lateral_distance_coeff = 1.0F;
  cost_->setParams(params);
  const std::array<float, 3> corridor_x{0.0F, 10.0F, 10.0F};
  const std::array<float, 3> corridor_y{0.0F, 0.0F, 10.0F};
  cost_->setLateralCorridor(
    corridor_x.data(), corridor_y.data(), static_cast<int>(corridor_x.size()));

  constexpr float query_x = 1000.0F;
  constexpr float query_y = 1000.0F;
  const int host_segment = cost_->computeLateralPathMetrics(query_x, query_y, 0.0F).best_segment_i;
  EXPECT_EQ(sampleProjectionSegment(*cost_, query_x, query_y, stream()), host_segment);
}

TEST_F(DistanceMapGpuTest, AdversarialProjectionMatchesHostWithAndWithoutPersistentHints)
{
  TestParams params = distanceOnlyParams();
  params.lateral_distance_coeff = 1.0F;
  cost_->setParams(params);
  for (const auto & c : autoware::mppi_optimizer::projection_test::cases()) {
    SCOPED_TRACE(c.name);
    std::vector<float> velocity(c.x.size());
    for (std::size_t i = 0; i < velocity.size(); ++i) {
      velocity[i] = 1.0F + static_cast<float>(i) * 0.1F;
    }
    cost_->setLateralCorridor(
      c.x.data(), c.y.data(), static_cast<int>(c.x.size()), nullptr, velocity.data());
    ASSERT_TRUE(cost_->texture_state_.nearest_segment_texture_valid_);
    const auto expected = cost_->computeLateralPathMetrics(c.query_x, c.query_y, 0.3F);
    EXPECT_EQ(expected.best_segment_i, c.segment);
    EXPECT_NEAR(expected.lateral_distance, c.lateral, 1.0E-5F);
    const auto & grid = cost_->texture_state_.nearest_segment_map_grid_;
    const float boundary_x = grid.origin_x + 128.0F * grid.resolution;
    const float boundary_y = grid.origin_y + 128.0F * grid.resolution;
    const float epsilon = 0.001F * grid.resolution;
    expectProjectionSequence(
      *cost_,
      {make_float2(c.x.front(), c.y.front()), make_float2(c.query_x, c.query_y),
       make_float2(boundary_x - epsilon, boundary_y - epsilon),
       make_float2(boundary_x + epsilon, boundary_y + epsilon),
       make_float2(grid.origin_x - 10.0F, c.query_y), make_float2(c.query_x, c.query_y)},
      stream());
  }
}

TEST_F(DistanceMapGpuTest, ReferenceOnlyProjectionUsesTheSameGlobalSearch)
{
  std::array<float, kTestHorizon> x{}, y{}, velocity{};
  x[1] = x[2] = 10.0F;
  for (int i = 2; i < kTestHorizon; ++i) {
    y[i] = 10.0F;
  }
  velocity.fill(2.0F);
  cost_->beginDataUpdate();
  cost_->clearLateralCorridor();
  cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon);
  cost_->commitDataUpdate();
  expectProjectionSequence(
    *cost_, {make_float2(1, 0), make_float2(1, 9), make_float2(-1000, 9)}, stream());
}

TEST_F(DistanceMapGpuTest, PreparedProjectionMatchesOriginalDeviceArithmeticAndSafety)
{
  auto params = distanceOnlyParams();
  params.boundary_threshold = 0.5F;
  cost_->setParams(params);
  for (const auto & c : autoware::mppi_optimizer::projection_test::cases()) {
    SCOPED_TRACE(c.name);
    cost_->setLateralCorridor(c.x.data(), c.y.data(), static_cast<int>(c.x.size()));
    expectDeviceProjectionParity(
      *cost_,
      {make_float2(c.query_x, c.query_y),
       make_float2(c.query_x, std::nextafter(c.query_y, c.query_y - 1.0F)),
       make_float2(c.query_x, std::nextafter(c.query_y, c.query_y + 1.0F))},
      stream());
  }

  // Exercise both signs immediately inside, on, and outside the hard lateral boundary.
  std::array<float, 33> line_x{}, line_y{};
  for (std::size_t i = 0; i < line_x.size(); ++i) line_x[i] = static_cast<float>(i);
  cost_->setLateralCorridor(line_x.data(), line_y.data(), static_cast<int>(line_x.size()));
  std::vector<float2> queries;
  for (float sign : {-1.0F, 1.0F}) {
    for (float offset : {std::nextafter(0.5F, 0.0F), 0.5F, std::nextafter(0.5F, 1.0F)}) {
      queries.push_back(make_float2(5.0F, sign * offset));
    }
  }
  expectDeviceProjectionParity(*cost_, queries, stream());

  // Fractional oblique vertices exercise rounded endpoint differences in reference fallback.
  const float ref_x[] = {-0.17F, 3.14159F, 3.7F, 0.13F};
  const float ref_y[] = {0.23F, 2.71828F, -0.41F, -0.19F};
  const float velocity[] = {2, 2, 2, 2};
  cost_->beginDataUpdate();
  cost_->setReferenceTrajectory(ref_x, ref_y, velocity, 4);
  cost_->clearLateralCorridor();
  cost_->commitDataUpdate();
  expectDeviceProjectionParity(
    *cost_, {make_float2(1.1F, 1.3F), make_float2(2.7F, -0.2F), make_float2(-1000, 9)}, stream());
}

TEST_F(DistanceMapGpuTest, PrunedProjectionPreservesDeviceResultsAcrossNumericRanges)
{
  using namespace mppi::cost::detail;
  const float tiny = std::numeric_limits<float>::denorm_min();
  const float limit = kPolylinePruningCoordinateLimit;
  const std::vector<float2> queries{
    make_float2(0, 0),
    make_float2(tiny, -tiny),
    make_float2(8, 0.5F),
    make_float2(std::nextafter(8.0F, 0.0F), std::nextafter(0.5F, 0.0F)),
    make_float2(std::nextafter(8.0F, 9.0F), std::nextafter(0.5F, 1.0F)),
    make_float2(-1000, 9),
    make_float2(limit, -limit),
    make_float2(std::nextafter(limit, FLT_MAX), 0),
    make_float2(std::numeric_limits<float>::infinity(), 0),
    make_float2(std::numeric_limits<float>::quiet_NaN(), 0)};
  expectRawPruningParity({}, {}, queries, stream());
  expectRawPruningParity({1}, {2}, queries, stream());
  std::mt19937 random(0x6d707069U);
  std::uniform_int_distribution<int> coordinate(-10000, 10000);
  for (int count : {2, 9, 10, 17, 80, 255, 256}) {
    SCOPED_TRACE(count);
    std::vector<float> x(count), y(count);
    for (float scale : {0x1p-100F, 0x1p-20F, 1.0F, 0x1p16F}) {
      SCOPED_TRACE(scale);
      for (int i = 0; i < count; ++i) {
        // Reproducible fractional, nonuniform, crossing and duplicate vertices.
        x[i] = static_cast<float>(coordinate(random)) * 0.001F * scale;
        y[i] = static_cast<float>(coordinate(random)) * 0.001F * scale;
        if (i > 0 && i % 7 == 0) {
          x[i] = x[i - 1];
          y[i] = y[i - 1];
        }
      }
      auto random_queries = queries;
      for (int i = 0; i < 16; ++i) {
        random_queries.push_back(make_float2(
          static_cast<float>(coordinate(random)) * 0.001F * scale,
          static_cast<float>(coordinate(random)) * 0.001F * scale));
      }
      expectRawPruningParity(x, y, random_queries, stream());
    }
  }
  std::vector<float> x(34, 1), y(34, -1);
  x.front() = 100000000.0F;
  y.front() = -100000000.0F;
  expectRawPruningParity(x, y, queries, stream());        // Rounded endpoint leaves the ideal AABB.
  expectRawPruningParity(x, y, queries, stream(), true);  // Explicit invalid-bounds fallback.
  x.front() = std::nextafter(limit, FLT_MAX);
  expectRawPruningParity(x, y, queries, stream());  // Unsupported geometry, still finite.
  x.assign(34, 0.0F);
  y.assign(34, 0.0F);
  expectRawPruningParity(x, y, queries, stream());  // Entirely degenerate groups.
  x.resize(256);
  y.assign(256, 0.0F);
  for (int i = 0; i < 256; ++i) x[i] = static_cast<float>(i);
  expectRawPruningParity(x, y, {make_float2(80.5F, 0.5F)}, stream(), false, false, true);
}

// Deliberately disabled: work counts are diagnostic, never part of a default test/benchmark run.
// These synthetic queries/all hints are not a trace of the adaptive optimizer workload.
TEST_F(DistanceMapGpuTest, DISABLED_ProjectionPruningWorkCounts)
{
  using namespace mppi::cost::detail;
  ASSERT_TRUE(polylinePruningMathSupported()) << "Rebuild with standard IEEE CUDA math flags";
  std::vector<float> x(256), y(256);
  std::vector<float2> queries;
  for (int i = 0; i < 256; ++i) {
    x[i] = static_cast<float>(i);
    y[i] = 0.0F;
    if (i % 16 == 0) queries.push_back(make_float2(x[i] + 0.25F, 0.5F));
  }
  std::puts("[projection work] scene=separated_straight_groups");
  expectRawPruningParity<4>(x, y, queries, stream(), false, true);
  expectRawPruningParity<8>(x, y, queries, stream(), false, true);
  expectRawPruningParity<16>(x, y, queries, stream(), false, true);
  for (int i = 0; i < 256; ++i) x[i] = i % 2 == 0 ? -10.0F : 10.0F;
  std::puts("[projection work] scene=overlapping_groups");
  expectRawPruningParity<4>(x, y, {make_float2(0, 0)}, stream(), false, true);
  expectRawPruningParity<8>(x, y, {make_float2(0, 0)}, stream(), false, true);
  expectRawPruningParity<16>(x, y, {make_float2(0, 0)}, stream(), false, true);
}

TEST_F(DistanceMapGpuTest, PreparedProjectionTracksReferenceCorridorAndTranslatedVertices)
{
  std::array<float, kTestHorizon> x{}, y{}, velocity{};
  velocity.fill(2.0F);
  x[1] = x[2] = 10.0F;
  for (int i = 2; i < kTestHorizon; ++i) y[i] = 10.0F;
  cost_->beginDataUpdate();
  cost_->clearLateralCorridor();
  cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon);
  cost_->commitDataUpdate();
  EXPECT_FLOAT_EQ(cost_->runtimeData().ref_segment_delta_[0].dx, 10.0F);
  expectProjectionSequence(*cost_, {make_float2(1, 9), make_float2(-1000, 9)}, stream());

  const float corridor_x[] = {0, 10, 10, 0};
  const float corridor_y[] = {0, 0, 2, 2};
  cost_->setLateralCorridor(corridor_x, corridor_y, 4);
  EXPECT_FLOAT_EQ(cost_->runtimeData().lateral_corridor_segment_delta_[2].dx, -10.0F);
  expectProjectionSequence(*cost_, {make_float2(1, 1), make_float2(1, 1.000001F)}, stream());

  // Reference changes while the corridor is active must be ready when it is removed.
  x[1] = x[2] = 20.0F;
  cost_->beginDataUpdate();
  cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon);
  cost_->clearLateralCorridor();
  cost_->commitDataUpdate();
  EXPECT_FLOAT_EQ(cost_->runtimeData().ref_segment_delta_[0].dx, 20.0F);
  expectProjectionSequence(*cost_, {make_float2(18, 1), make_float2(1, 9)}, stream());

  // Simulate replacement by translated local vertices at the cost-setter boundary.
  // The interface's PublishesPreciseMapPositionsAcrossOriginChange test triggers recentering.
  const float replacement_x[] = {100, 120, 120, 100};
  const float replacement_y[] = {50, 50, 52, 52};
  for (int i = 0; i < kTestHorizon; ++i) {
    x[i] += 100.0F;
    y[i] += 50.0F;
  }
  cost_->beginDataUpdate();
  cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon);
  cost_->setLateralCorridor(replacement_x, replacement_y, 4);
  cost_->commitDataUpdate();
  EXPECT_FLOAT_EQ(cost_->runtimeData().lateral_corridor_segment_delta_[0].dx, 20.0F);
  const float below_tie = std::nextafter(51.0F, 50.0F);
  const float above_tie = std::nextafter(51.0F, 52.0F);
  ASSERT_LT(below_tie, 51.0F);
  ASSERT_GT(above_tie, 51.0F);
  const std::array<float, 3> query_y{below_tie, 51.0F, above_tie};
  const std::array<int, 3> expected_segments{0, 0, 2};
  for (std::size_t i = 0; i < query_y.size(); ++i) {
    SCOPED_TRACE(i);
    EXPECT_EQ(
      cost_->computeLateralPathMetrics(101, query_y[i], 0.0F).best_segment_i, expected_segments[i]);
    EXPECT_EQ(sampleProjectionSegment(*cost_, 101, query_y[i], stream()), expected_segments[i]);
  }
  const std::vector<float2> translated_queries{
    make_float2(101, below_tie), make_float2(101, 51), make_float2(101, above_tie)};
  expectProjectionSequence(*cost_, translated_queries, stream());
  expectDeviceProjectionParity(*cost_, translated_queries, stream());
  cost_->clearLateralCorridor();
  expectProjectionSequence(*cost_, {make_float2(118, 51), make_float2(101, 59)}, stream());
}

TEST_F(DistanceMapGpuTest, PrunedProjectionBoundsFollowGeometryLifecycle)
{
  using namespace mppi::cost::detail;
  std::array<float, 256> x{}, y{}, velocity{};
  velocity.fill(2.0F);
  for (int i = 0; i < 256; ++i) x[i] = static_cast<float>(i);
  cost_->beginDataUpdate();
  cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon);
  cost_->setLateralCorridor(x.data(), y.data(), 256, nullptr, velocity.data());
  cost_->commitDataUpdate();
  ASSERT_TRUE(cost_->runtimeData().ref_group_bounds_valid_);
  ASSERT_TRUE(cost_->runtimeData().lateral_corridor_group_bounds_valid_);
  const auto & data = cost_->runtimeData();
  const std::vector<PolylineGroupBounds> reference_bounds(
    std::begin(data.ref_group_bounds_), std::end(data.ref_group_bounds_));
  const std::vector<PolylineGroupBounds> corridor_bounds(
    std::begin(data.lateral_corridor_group_bounds_), std::end(data.lateral_corridor_group_bounds_));
  velocity.fill(3.0F);
  cost_->beginDataUpdate();
  cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon);
  cost_->setLateralCorridor(x.data(), y.data(), 256, nullptr, velocity.data());
  cost_->commitDataUpdate();
  const auto equal_bounds = [](const auto & before, const auto & after) {
    for (std::size_t i = 0; i < before.size(); ++i) {
      EXPECT_EQ(before[i].min_x, after[i].min_x);
      EXPECT_EQ(before[i].min_y, after[i].min_y);
      EXPECT_EQ(before[i].max_x, after[i].max_x);
      EXPECT_EQ(before[i].max_y, after[i].max_y);
    }
  };
  equal_bounds(reference_bounds, data.ref_group_bounds_);
  equal_bounds(corridor_bounds, data.lateral_corridor_group_bounds_);
  expectProjectionSequence(*cost_, {make_float2(8, 0.5F), make_float2(200, 0)}, stream());
  // Shrink, grow, replace vertices, remove, and recreate after a local-frame translation.
  for (int count : {10, 17, 256}) {
    SCOPED_TRACE(count);
    for (int i = 0; i < 256; ++i) {
      x[i] = 100.0F + 0.25F * i;
      y[i] = 50.0F + (i % 9 == 0 ? 0.125F : 0.0F);
    }
    cost_->beginDataUpdate();
    cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon);
    cost_->setLateralCorridor(x.data(), y.data(), count, nullptr, velocity.data());
    cost_->commitDataUpdate();
    const std::vector<float2> queries{make_float2(101, 50.5F), make_float2(150, 50.25F)};
    expectDeviceProjectionParity(*cost_, queries, stream());
    expectProjectionSequence(*cost_, queries, stream());
    cost_->clearLateralCorridor();
    EXPECT_FALSE(data.lateral_corridor_group_bounds_valid_);
    // The reference was changed while a corridor was active, and is now the projection path.
    expectDeviceProjectionParity(*cost_, queries, stream());
    expectProjectionSequence(*cost_, queries, stream());
  }
  cost_->setLateralCorridor(x.data(), y.data(), 1);
  EXPECT_FALSE(data.lateral_corridor_group_bounds_valid_);
  cost_->setLateralCorridor(x.data(), y.data(), 256);
  cost_->freeCudaMem();
  cost_->GPUSetup();
  expectDeviceProjectionParity(*cost_, {make_float2(101, 50.5F)}, stream());
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
  EXPECT_EQ(cost_->runtimeData().num_obstacles_, 0);
  EXPECT_TRUE(cost_->texture_state_.obstacle_texture_valid_);
  EXPECT_FALSE(cost_->texture_state_.obstacle_texture_has_obstacles_);
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
  const cudaTextureObject_t original_static_texture =
    cost_->texture_state_.static_distance_texture_;
  const cudaTextureObject_t original_obstacle_texture =
    cost_->texture_state_.obstacle_distance_texture_;

  for (int iteration = 1; iteration <= 100; ++iteration) {
    fillReference(*cost_, 0.5F * static_cast<float>(iteration));
    cost_->setRoadBorderSegments({wall});
  }
  CUDA_CHECK(cudaStreamSynchronize(stream()));
  EXPECT_EQ(DistanceMapTextureTestAccess::staticArray(*cost_), original_static_array);
  EXPECT_EQ(DistanceMapTextureTestAccess::obstacleArray(*cost_), original_obstacle_array);
  EXPECT_EQ(cost_->texture_state_.static_distance_texture_, original_static_texture);
  EXPECT_EQ(cost_->texture_state_.obstacle_distance_texture_, original_obstacle_texture);
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
  if (device_error == cudaErrorNoDevice || (device_error == cudaSuccess && device_count == 0)) {
    GTEST_SKIP() << "A CUDA device is required: " << cudaGetErrorString(device_error);
  }
  ASSERT_EQ(device_error, cudaSuccess) << cudaGetErrorString(device_error);
  CudaStream stream;
  const auto allocate_and_release = [&stream]() {
    TestCost cost(stream.get());
    cost.GPUSetup();
    TestParams params = distanceOnlyParams();
    params.lateral_distance_coeff = 1.0F;
    cost.setParams(params);
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
