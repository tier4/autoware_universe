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

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/tracked_objects_obstacles.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{
constexpr std::size_t kMeasuredIterations = 50U;
constexpr std::size_t kHorizon = 80U;
constexpr int kOptimizerIterations = 20;
// Matches FirstOrderDubinsRuntimeData::kMaxRoadBorderSegments.
constexpr std::size_t kRoadBorderSegments = 256U;
// Matches FirstOrderDubinsRuntimeData::kMaxLateralCorridorPoints.
constexpr std::size_t kProjectionPathPoints = 256U;
constexpr char kLoggerName[] = "first_order_dubins_mppi";
static_assert(detail::kMppiHorizon == kHorizon, "Rebaseline if the MPPI horizon changes");
static_assert(detail::kMppiDt == 0.1F, "Rebaseline trajectory timestamps if MPPI dt changes");

Trajectory makeBenchmarkTrajectory()
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 123;
  trajectory.points.reserve(kHorizon);
  for (std::size_t i = 0; i < kHorizon; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    // Post-step reference: x[t + 1] at 2 m/s with dt = 0.1 s.
    point.pose.position.x = 2.0 * detail::kMppiDt * static_cast<double>(i + 1U);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    point.time_from_start.sec = static_cast<std::int32_t>((i + 1U) / 10U);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(((i + 1U) % 10U) * 100000000U);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

Trajectory makeHairpinBenchmarkTrajectory()
{
  auto trajectory = makeBenchmarkTrajectory();
  trajectory.points.resize(kProjectionPathPoints);
  constexpr double pi = 3.14159265358979323846;
  constexpr double straight_length = 16.0;
  constexpr double radius = 3.0;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    auto & point = trajectory.points[i];
    const double distance = 2.0 * detail::kMppiDt * static_cast<double>(i + 1U);
    double yaw = 0.0;
    if (distance <= straight_length) {
      point.pose.position.x = distance;
      point.pose.position.y = 0.0;
    } else if (distance < straight_length + pi * radius) {
      yaw = (distance - straight_length) / radius;
      point.pose.position.x = straight_length + radius * std::sin(yaw);
      point.pose.position.y = radius * (1.0 - std::cos(yaw));
    } else {
      yaw = pi;
      point.pose.position.x = straight_length - (distance - straight_length - pi * radius);
      point.pose.position.y = 2.0 * radius;
    }
    point.pose.orientation.z = std::sin(0.5 * yaw);
    point.pose.orientation.w = std::cos(0.5 * yaw);
    point.longitudinal_velocity_mps = 2.0F;
    point.time_from_start.sec = static_cast<std::int32_t>((i + 1U) / 10U);
    point.time_from_start.nanosec = static_cast<std::uint32_t>(((i + 1U) % 10U) * 100000000U);
  }
  return trajectory;
}

TrackedObjects makeHeavyObstacles()
{
  TrackedObjects objects;
  objects.header.frame_id = "map";
  objects.header.stamp.sec = 123;
  objects.objects.resize(kMaxMppiObstacles);
  for (std::size_t i = 0; i < objects.objects.size(); ++i) {
    auto & object = objects.objects[i];
    object.object_id.uuid[0] = static_cast<std::uint8_t>(i + 1U);
    object.existence_probability = 1.0F;
    auto & pose = object.kinematics.pose_with_covariance.pose;
    // Deterministic staggered rows on both sides of the ego/reference, from x=-4 to x=13.5.
    // Leave the centerline clear while keeping obstacles near sampled trajectories.
    pose.position.x = -4.0 + 2.5 * static_cast<double>(i % 8U);
    const double side = (i / 8U) % 2U == 0U ? 1.0 : -1.0;
    pose.position.y = side * (1.2 + 0.8 * static_cast<double>(i / 16U));
    pose.orientation.w = 1.0;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 0.8;
    object.shape.dimensions.y = 0.5;
    object.shape.dimensions.z = 1.0;
    // Nonzero velocity prevents static-obstacle extrusion from bypassing temporal work.
    object.kinematics.twist_with_covariance.twist.linear.x =
      0.2 + 0.05 * static_cast<double>(i % 4U);
  }
  return objects;
}

std::vector<Segment> makeTightCorridor()
{
  constexpr std::size_t segments_per_side = kRoadBorderSegments / 2U;
  std::vector<Segment> borders;
  borders.reserve(kRoadBorderSegments);
  const auto half_width = [](const float x) { return 0.65F + 0.03F * std::sin(x); };
  for (std::size_t i = 0; i < segments_per_side; ++i) {
    const float x0 = -2.0F + 20.0F * static_cast<float>(i) / segments_per_side;
    const float x1 = -2.0F + 20.0F * static_cast<float>(i + 1U) / segments_per_side;
    // Two continuous walls close to the default 0.42 m ego width plus collision margins.
    borders.push_back(Segment{x0, half_width(x0), x1, half_width(x1)});
    borders.push_back(Segment{x0, -half_width(x0), x1, -half_width(x1)});
  }
  return borders;
}

::testing::AssertionResult completedOptimization(
  const FirstOrderDubinsMppiOptimizationResult & result, const std::size_t input_points)
{
  if (
    !result.debug.applied_plant.valid || result.optimized_point_count != kHorizon ||
    result.trajectory.points.size() != input_points || !std::isfinite(result.debug.baseline_cost)) {
    return ::testing::AssertionFailure() << "Expected a completed, finite 80-step optimization";
  }
  return ::testing::AssertionSuccess();
}

class MppiPerformanceBenchmark : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    const cudaError_t status = cudaGetDeviceCount(&device_count);
    if (status == cudaErrorNoDevice || (status == cudaSuccess && device_count == 0)) {
      GTEST_SKIP() << "A CUDA GPU is required for MPPI performance benchmarks";
    }
    ASSERT_EQ(status, cudaSuccess) << cudaGetErrorString(status);
    int device = 0;
    ASSERT_EQ(cudaGetDevice(&device), cudaSuccess);
    ASSERT_EQ(cudaGetDeviceProperties(&device_properties_, device), cudaSuccess);
    ASSERT_EQ(cudaRuntimeGetVersion(&runtime_version_), cudaSuccess);
    ASSERT_EQ(cudaDriverGetVersion(&driver_version_), cudaSuccess);

    // Exclude console/debug I/O from the baseline; restore the logger after each case.
    ASSERT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
    previous_log_level_ = rcutils_logging_get_logger_level(kLoggerName);
    ASSERT_GE(previous_log_level_, 0);
    ASSERT_EQ(
      rcutils_logging_set_logger_level(kLoggerName, RCUTILS_LOG_SEVERITY_ERROR), RCUTILS_RET_OK);
    restore_log_level_ = true;
    interface_ = std::make_unique<FirstOrderDubinsMppiInterface>();
    FirstOrderDubinsMppiCostParams cost_params;
    cost_params.max_iter = kOptimizerIterations;
    interface_->setCostParams(cost_params);
    FirstOrderDubinsMppiRuntimeOptions options;
    options.enable_debug_trajectory_log = false;
    options.enable_distance_map_texture_debug = false;
    options.enable_iteration_rollout_debug = false;
    options.force_cold_start_each_step = false;
    options.use_last_control_as_nominal = false;
    options.use_temporal_mpt_as_nominal = false;
    options.skip_if_invalid = false;
    interface_->setRuntimeOptions(options);
  }

  void TearDown() override
  {
    interface_.reset();
    if (restore_log_level_) {
      EXPECT_EQ(rcutils_logging_set_logger_level(kLoggerName, previous_log_level_), RCUTILS_RET_OK);
    }
  }

  void benchmark(
    const TrackedObjects & objects, const std::vector<Segment> & borders,
    const Trajectory & trajectory = makeBenchmarkTrajectory())
  {
    Odometry odometry;
    odometry.header = trajectory.header;
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.orientation.w = 1.0;
    odometry.twist.twist.linear.x = 2.0;
    const std::vector<Segment> drivable_area;
    const FirstOrderDubinsMppiKinematicLimits limits;
    const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> acceleration;
    const std::optional<autoware_vehicle_msgs::msg::SteeringReport> steering;

    // Exactly one unmeasured optimizeTrajectory call per case. Keep the same interface,
    // inputs and timestamps for all samples; persistent caches/control history remain live.
    {
      const auto warmup = interface_->optimizeTrajectory(
        trajectory, odometry, acceleration, steering, objects, borders, drivable_area, limits);
      const cudaError_t status = cudaDeviceSynchronize();
      ASSERT_EQ(status, cudaSuccess) << cudaGetErrorString(status);
      ASSERT_TRUE(interface_->isInitialized());
      ASSERT_TRUE(completedOptimization(warmup, trajectory.points.size()));
    }

    using Clock = std::chrono::high_resolution_clock;
    std::array<double, kMeasuredIterations> elapsed_ms{};
    for (std::size_t i = 0; i < kMeasuredIterations; ++i) {
      // Drain any preceding work outside the measured interval.
      ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
      const auto start = Clock::now();
      const auto result = interface_->optimizeTrajectory(
        trajectory, odometry, acceleration, steering, objects, borders, drivable_area, limits);
      const cudaError_t status = cudaDeviceSynchronize();
      const auto stop = Clock::now();

      // Assertions, statistics, printing and destruction of the result are outside timing.
      ASSERT_EQ(status, cudaSuccess) << "Iteration " << i << ": " << cudaGetErrorString(status);
      ASSERT_TRUE(completedOptimization(result, trajectory.points.size())) << "Iteration " << i;
      elapsed_ms[i] = std::chrono::duration<double, std::milli>(stop - start).count();
      ASSERT_GE(elapsed_ms[i], 0.0) << "high_resolution_clock moved backwards";
    }

    const double average =
      std::accumulate(elapsed_ms.begin(), elapsed_ms.end(), 0.0) / kMeasuredIterations;
    const auto extrema = std::minmax_element(elapsed_ms.begin(), elapsed_ms.end());
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(3) << "\n[MPPI latency] "
            << ::testing::UnitTest::GetInstance()->current_test_info()->name()
            << "\n  GPU: " << device_properties_.name << " | CUDA runtime: " << runtime_version_
            << " | driver: " << driver_version_ << "\n  horizon=" << kHorizon
            << " | optimizer_iterations=" << kOptimizerIterations
            << " | projection_path_points=" << trajectory.points.size()
            << " | obstacles=" << objects.objects.size() << " | road_borders=" << borders.size()
            << " | warmup=1 | measured=" << kMeasuredIterations << "\n  average=" << average
            << " ms | minimum=" << *extrema.first << " ms | maximum=" << *extrema.second << " ms\n";
    std::cout << summary.str() << std::flush;
    RecordProperty("average_ms", std::to_string(average));
    RecordProperty("minimum_ms", std::to_string(*extrema.first));
    RecordProperty("maximum_ms", std::to_string(*extrema.second));
  }

private:
  std::unique_ptr<FirstOrderDubinsMppiInterface> interface_;
  cudaDeviceProp device_properties_{};
  int runtime_version_{0};
  int driver_version_{0};
  int previous_log_level_{RCUTILS_LOG_SEVERITY_UNSET};
  bool restore_log_level_{false};
};

TEST_F(MppiPerformanceBenchmark, Benchmark_FreeSpace)
{
  benchmark(TrackedObjects{}, {});
}

TEST_F(MppiPerformanceBenchmark, Benchmark_HeavyObstacles)
{
  const auto objects = makeHeavyObstacles();
  ASSERT_EQ(objects.objects.size(), kMaxMppiObstacles);
  benchmark(objects, {});
}

TEST_F(MppiPerformanceBenchmark, Benchmark_TightCorridor)
{
  const auto borders = makeTightCorridor();
  ASSERT_EQ(borders.size(), kRoadBorderSegments);
  benchmark(TrackedObjects{}, borders);
}

TEST_F(MppiPerformanceBenchmark, Benchmark_256PointHairpin)
{
  const auto trajectory = makeHairpinBenchmarkTrajectory();
  ASSERT_EQ(trajectory.points.size(), kProjectionPathPoints);
  // The full input feeds the lateral corridor; optimization still uses 80 time steps.
  // This measures maximum-capacity projection work with a nearby returning branch.
  benchmark(TrackedObjects{}, {}, trajectory);
}
}  // namespace
}  // namespace autoware::mppi_optimizer
