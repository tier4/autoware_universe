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
#include "autoware/mppi_optimizer/input_collision_gate.hpp"
#include "autoware/mppi_optimizer/tracked_objects_obstacles.hpp"
#include "performance_benchmark_support.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>
#include <rcutils/logging.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{
using benchmark::Json;
constexpr std::size_t kHorizon = 80U;
constexpr std::size_t kRoadBorderSegments = 256U;
constexpr std::size_t kProjectionPathPoints = 256U;
constexpr char kLoggerName[] = "first_order_dubins_mppi";
static_assert(detail::kMppiHorizon == kHorizon, "Update benchmark trajectories with the horizon");
static_assert(detail::kMppiDt == 0.1F, "Update benchmark timestamps with the model timestep");

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

TrackedObjects makeObstacles(std::size_t count, double clearance)
{
  TrackedObjects objects;
  objects.header.frame_id = "map";
  objects.objects.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto & object = objects.objects[i];
    object.object_id.uuid[0] = static_cast<std::uint8_t>(i + 1);
    object.existence_probability = 1.0F;
    auto & pose = object.kinematics.pose_with_covariance.pose;
    pose.position.x = -4.0 + 2.5 * static_cast<double>(i % 8);
    const double side = (i / 8) % 2 == 0 ? 1.0 : -1.0;
    pose.position.y = side * (clearance + 0.8 * static_cast<double>(i / 16));
    pose.orientation.w = 1.0;
    object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    object.shape.dimensions.x = 0.8;
    object.shape.dimensions.y = 0.8;
    object.shape.dimensions.z = 1.0;
    // Zero velocity: these are stationary targets, including in the dirty-map scenarios.
  }
  return objects;
}

std::vector<Segment> makeCorridor(double half_width)
{
  std::vector<Segment> borders;
  borders.reserve(kRoadBorderSegments);
  constexpr std::size_t per_side = kRoadBorderSegments / 2;
  for (std::size_t i = 0; i < per_side; ++i) {
    const double x0 = -2.0 + 20.0 * static_cast<double>(i) / per_side;
    const double x1 = -2.0 + 20.0 * static_cast<double>(i + 1) / per_side;
    borders.push_back({x0, half_width, x1, half_width});
    borders.push_back({x0, -half_width, x1, -half_width});
  }
  return borders;
}

enum class SceneChange { cached, obstacle_geometry, grid_crossing };
enum class ReferenceIssue { none, static_collision, border_near_collision };

struct Scene
{
  Trajectory trajectory{makeBenchmarkTrajectory()};
  TrackedObjects objects{};
  std::vector<Segment> borders;
  PreferredLaneCenterlineInput preferred;
  SceneChange change{SceneChange::cached};
  ReferenceIssue reference_issue{ReferenceIssue::none};
  bool expect_rejection{false};
  bool exact_centerline{false};
};

const char * changeName(SceneChange change)
{
  switch (change) {
    case SceneChange::cached:
      return "cached";
    case SceneChange::obstacle_geometry:
      return "obstacle_geometry";
    case SceneChange::grid_crossing:
      return "grid_crossing";
  }
  return "unknown";
}

std::size_t phaseCalls(const FirstOrderDubinsMppiPerformanceReport & report, const char * name)
{
  for (const auto & phase : report.phases)
    if (phase.name == name) return phase.calls;
  return 0;
}

Json phaseJson(const FirstOrderDubinsMppiPerformanceReport & report)
{
  Json phases = Json::object();
  for (const auto & phase : report.phases) {
    phases[phase.name] = {
      {"calls", phase.calls},
      {"gpu_calls", phase.gpu_calls},
      {"host_ms", phase.host_ms},
      {"gpu_ms", phase.gpu_calls ? Json(phase.gpu_ms) : Json(nullptr)}};
  }
  return phases;
}

class MppiPerformanceBenchmark : public ::testing::Test
{
protected:
  void SetUp() override
  {
    settings_ = std::make_unique<benchmark::Settings>();
    profile_ = benchmark::loadProfile(*settings_);
    report_path_ = benchmark::reportPath(
      *settings_, ::testing::UnitTest::GetInstance()->current_test_info()->name());
    int device_count = 0;
    const auto status = cudaGetDeviceCount(&device_count);
    if (status == cudaErrorNoDevice || (status == cudaSuccess && device_count == 0)) {
      GTEST_SKIP() << "A CUDA GPU is required; no performance samples were produced";
    }
    ASSERT_EQ(status, cudaSuccess) << cudaGetErrorString(status);
    ASSERT_LT(settings_->device, device_count);
    ASSERT_EQ(cudaSetDevice(settings_->device), cudaSuccess);
    cudaDeviceProp device{};
    ASSERT_EQ(cudaGetDeviceProperties(&device, settings_->device), cudaSuccess);
    int runtime = 0;
    int driver = 0;
    ASSERT_EQ(cudaRuntimeGetVersion(&runtime), cudaSuccess);
    ASSERT_EQ(cudaDriverGetVersion(&driver), cudaSuccess);
    char pci_bus_id[32]{};
    ASSERT_EQ(
      cudaDeviceGetPCIBusId(pci_bus_id, sizeof(pci_bus_id), settings_->device), cudaSuccess);
    gpu_info_ = {
      {"name", device.name},
      {"visible_device_index", settings_->device},
      {"pci_bus_id", pci_bus_id},
      {"compute_capability", std::to_string(device.major) + "." + std::to_string(device.minor)},
      {"multiprocessors", device.multiProcessorCount},
      {"global_memory_bytes", device.totalGlobalMem},
      {"cuda_runtime", runtime},
      {"cuda_driver", driver},
      {"CUDA_VISIBLE_DEVICES", benchmark::environment("CUDA_VISIBLE_DEVICES")}};
    ASSERT_EQ(rcutils_logging_initialize(), RCUTILS_RET_OK);
    previous_log_level_ = rcutils_logging_get_logger_level(kLoggerName);
    ASSERT_GE(previous_log_level_, 0);
    ASSERT_EQ(
      rcutils_logging_set_logger_level(kLoggerName, RCUTILS_LOG_SEVERITY_ERROR), RCUTILS_RET_OK);
    restore_log_level_ = true;
    interface_ = std::make_unique<FirstOrderDubinsMppiInterface>();
    interface_->setVehicleParams(profile_.vehicle);
    interface_->setCostParams(profile_.cost);
    interface_->setRuntimeOptions(profile_.runtime);
    interface_->setPerformanceInstrumentationEnabled(settings_->phase_timing);
  }

  void TearDown() override
  {
    interface_.reset();
    if (restore_log_level_) {
      EXPECT_EQ(rcutils_logging_set_logger_level(kLoggerName, previous_log_level_), RCUTILS_RET_OK);
    }
  }

  Scene parkedScene(std::size_t count = 16) const
  {
    Scene scene;
    scene.objects = makeObstacles(
      count, profile_.vehicle.ego_width * 0.5 + profile_.cost.obstacle_safe_margin + 1.5);
    return scene;
  }

  double corridorHalfWidth() const
  {
    return profile_.vehicle.ego_width * 0.5 + profile_.cost.road_border_safe_margin + 0.25;
  }

  void runBenchmark(const Scene & base)
  {
    ASSERT_FALSE(profile_.runtime.ignore_obstacles)
      << "Benchmark profiles must retain obstacle checks";
    ASSERT_FALSE(profile_.runtime.ignore_road_borders)
      << "Benchmark profiles must retain border checks";
    if (!base.preferred.segments.empty()) {
      profile_.cost.preferred_lane_center_coeff = 100.0F;
      profile_.description["cost"]["preferred_lane_center_coeff"] = 100.0F;
      interface_->setCostParams(profile_.cost);
    }
    interface_->setPreferredLaneCenterTextureEnabled(!base.exact_centerline);
    profile_.description["preferred_lane_center_texture_enabled"] = !base.exact_centerline;
    const auto hard_input = checkInputTrajectoryCollisions(
      base.trajectory, base.objects, base.borders, profile_.vehicle,
      profile_.cost.obstacle_collision_margin, profile_.cost.road_border_collision_margin,
      {true, true});
    const auto soft_input = checkInputTrajectoryCollisions(
      base.trajectory, base.objects, base.borders, profile_.vehicle,
      profile_.cost.obstacle_safe_margin, profile_.cost.road_border_safe_margin, {true, true});
    if (base.reference_issue == ReferenceIssue::static_collision) {
      ASSERT_TRUE(hard_input.static_object);
    }
    if (base.reference_issue == ReferenceIssue::border_near_collision) {
      ASSERT_TRUE(soft_input.road_border);
      ASSERT_FALSE(hard_input.road_border);
    }

    Json measured = Json::array();
    Json first_call;
    std::vector<double> wall_times;
    std::vector<double> cpu_times;
    wall_times.reserve(settings_->samples);
    cpu_times.reserve(settings_->samples);
    FirstOrderDubinsMppiPerformanceReport configuration;
    std::size_t unexpected_outcomes = 0;
    const auto total_cycles = settings_->warmup + settings_->samples;
    for (int cycle = 0; cycle < total_cycles; ++cycle) {
      SCOPED_TRACE("cycle=" + std::to_string(cycle));
      Scene scene = base;
      Odometry odometry;
      // Advance epochs without pretending to execute the optimized command. Alternate bounded
      // offsets for dirty maps so the local frame never recenters and workload size stays fixed.
      const double offset = base.change == SceneChange::grid_crossing && cycle % 2 != 0 ? 0.6 : 0.0;
      scene.trajectory.header.stamp.sec = 123 + cycle / 10;
      scene.trajectory.header.stamp.nanosec = static_cast<std::uint32_t>(cycle % 10) * 100000000U;
      for (auto & point : scene.trajectory.points) point.pose.position.x += offset;
      scene.objects.header = scene.trajectory.header;
      for (auto & object : scene.objects.objects)
        object.kinematics.pose_with_covariance.pose.position.x += offset;
      for (auto & border : scene.borders) {
        border.x0 += offset;
        border.x1 += offset;
      }
      for (auto & segment : scene.preferred.segments) {
        segment.x0 += offset;
        segment.x1 += offset;
      }
      if (base.change == SceneChange::obstacle_geometry) {
        ASSERT_FALSE(scene.objects.objects.empty());
        scene.objects.objects.front().kinematics.pose_with_covariance.pose.position.y +=
          cycle % 2 == 0 ? 0.0 : 0.125;
      }
      odometry.header = scene.trajectory.header;
      odometry.child_frame_id = "base_link";
      odometry.pose.pose.position.x = offset;
      odometry.pose.pose.orientation.w = 1.0;
      odometry.twist.twist.linear.x = 2.0;
      const std::vector<Segment> drivable_area;
      const FirstOrderDubinsMppiKinematicLimits limits;
      const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> acceleration;
      const std::optional<autoware_vehicle_msgs::msg::SteeringReport> steering;

      // Only the optimizer call and its completion wait are timed. Event collection, assertions,
      // input construction, result destruction and all report I/O are outside these boundaries.
      ASSERT_EQ(cudaDeviceSynchronize(), cudaSuccess);
      const double cpu_start = benchmark::processCpuMilliseconds();
      const auto start = std::chrono::steady_clock::now();
      const auto result = interface_->optimizeTrajectory(
        scene.trajectory, odometry, acceleration, steering, scene.objects, scene.borders,
        drivable_area, limits, {}, true, std::nullopt, scene.preferred);
      const auto completion = cudaDeviceSynchronize();
      const auto stop = std::chrono::steady_clock::now();
      const double cpu_ms = benchmark::processCpuMilliseconds() - cpu_start;
      const double wall_ms = std::chrono::duration<double, std::milli>(stop - start).count();
      ASSERT_EQ(completion, cudaSuccess) << cudaGetErrorString(completion);
      configuration = interface_->performanceReport();
      ASSERT_TRUE(configuration.initialized);
      EXPECT_GE(wall_ms, 0.0);
      EXPECT_GE(cpu_ms, 0.0);
      const bool ran_full_workload =
        result.optimized_point_count == kHorizon &&
        result.trajectory.points.size() == scene.trajectory.points.size() &&
        result.debug.rollout_iteration_diagnostics.size() ==
          static_cast<std::size_t>(profile_.cost.max_iter);
      const bool outcome_ok = base.expect_rejection
                                ? result.debug.was_rejected &&
                                    result.debug.failed_rollout_iteration >= 0 &&
                                    hasInvalidityReason(
                                      result.debug.validation.reasons,
                                      FirstOrderDubinsMppiInvalidityReason::no_eligible_rollouts)
                                : !result.debug.was_rejected && result.debug.validation.isValid() &&
                                    result.debug.failed_rollout_iteration == -1 &&
                                    std::isfinite(result.debug.baseline_cost);
      EXPECT_TRUE(ran_full_workload) << "Optimizer skipped or changed the requested workload";
      EXPECT_TRUE(outcome_ok) << "Unexpected outcome: "
                              << to_string(result.debug.validation.reasons);
      if (!ran_full_workload || !outcome_ok) ++unexpected_outcomes;

      if (settings_->phase_timing) {
        for (const char * phase :
             {"MPPI/random_sampling", "MPPI/rollout", "MPPI/weight_statistics",
              "MPPI/weighted_control_reduction"}) {
          EXPECT_EQ(
            phaseCalls(configuration, phase), static_cast<std::size_t>(profile_.cost.max_iter))
            << phase;
        }
        if (cycle > 0) {
          const auto obstacle_rebuilds =
            !scene.objects.objects.empty() && base.change != SceneChange::cached ? 1U : 0U;
          const auto grid_rebuilds = base.change == SceneChange::grid_crossing ? 1U : 0U;
          EXPECT_EQ(phaseCalls(configuration, "MPPI/distance_map_obstacle"), obstacle_rebuilds);
          EXPECT_EQ(phaseCalls(configuration, "MPPI/distance_map_static"), grid_rebuilds);
          EXPECT_EQ(phaseCalls(configuration, "MPPI/distance_map_nearest_segment"), grid_rebuilds);
          const auto preferred_rebuilds =
            !scene.preferred.segments.empty() && !base.exact_centerline ? grid_rebuilds : 0U;
          EXPECT_EQ(
            phaseCalls(configuration, "MPPI/distance_map_preferred_lane_center"),
            preferred_rebuilds);
        }
      }
      Json sample = {
        {"cycle", cycle},
        {"wall_ms", wall_ms},
        {"process_cpu_ms", cpu_ms},
        {"workload_complete", ran_full_workload},
        {"outcome_expected", outcome_ok},
        {"rejected", result.debug.was_rejected},
        {"validation", to_string(result.debug.validation.reasons)},
        {"optimized_points", result.optimized_point_count},
        {"iterations_completed", result.debug.rollout_iteration_diagnostics.size()},
        {"failed_iteration", result.debug.failed_rollout_iteration},
        {"eligible_rollouts", result.debug.eligible_rollout_count},
        {"baseline_cost", benchmark::finiteNumber(result.debug.baseline_cost)},
        {"lambda_used", benchmark::finiteNumber(result.debug.lambda_used)},
        {"lambda_next", benchmark::finiteNumber(result.debug.lambda_next)},
        {"phases", phaseJson(configuration)}};
      if (cycle == 0) first_call = sample;
      if (cycle >= settings_->warmup) {
        measured.push_back(std::move(sample));
        wall_times.push_back(wall_ms);
        cpu_times.push_back(cpu_ms);
      }
    }

    Json phase_summary = Json::object();
    std::set<std::string> phase_names;
    for (const auto & sample : measured) {
      for (auto phase = sample["phases"].begin(); phase != sample["phases"].end(); ++phase)
        phase_names.insert(phase.key());
    }
    for (const auto & name : phase_names) {
      std::vector<double> host_ms;
      std::vector<double> gpu_ms;
      std::size_t calls = 0;
      std::size_t gpu_calls = 0;
      for (const auto & sample : measured) {
        const auto & phases = sample["phases"];
        if (phases.contains(name)) {
          const auto & phase = phases[name];
          host_ms.push_back(phase["host_ms"].get<double>());
          gpu_ms.push_back(phase["gpu_ms"].is_null() ? 0.0 : phase["gpu_ms"].get<double>());
          calls += phase["calls"].get<std::size_t>();
          gpu_calls += phase["gpu_calls"].get<std::size_t>();
        } else {
          host_ms.push_back(0.0);
          gpu_ms.push_back(0.0);
        }
      }
      phase_summary[name] = {
        {"calls", calls},
        {"gpu_calls", gpu_calls},
        {"host", benchmark::statistics(host_ms)},
        {"gpu", gpu_calls ? benchmark::statistics(gpu_ms) : Json(nullptr)}};
    }
    const auto wall_summary = benchmark::statistics(wall_times);
    const auto cpu_summary = benchmark::statistics(cpu_times);
    const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
    const Json report = {
      {"schema_version", 1},
      {"scenario", name},
      {"measurements_valid", !HasFailure()},
      {"unexpected_outcomes_including_warmup", unexpected_outcomes},
      {"mode", settings_->phase_timing ? "instrumented" : "latency_only"},
      {"scope", "optimizer_interface_call_and_completion_wait"},
      {"warmup_calls_including_first", settings_->warmup},
      {"measured_calls", settings_->samples},
      {"scene_change", changeName(base.change)},
      {"reference_points", base.trajectory.points.size()},
      {"objects", base.objects.objects.size()},
      {"hypotheses", base.objects.objects.size()},
      {"road_border_segments", base.borders.size()},
      {"preferred_centerline_segments", base.preferred.segments.size()},
      {"input_hard_collision", hard_input.triggered()},
      {"input_soft_collision", soft_input.triggered()},
      {"rollouts", configuration.rollouts},
      {"horizon", configuration.horizon},
      {"iterations", configuration.iterations},
      {"dt_s", configuration.dt},
      {"seed", configuration.seed},
      {"rollout_kernel", configuration.kernel},
      {"resolved_input_profile", profile_.description},
      {"host", benchmark::hostInformation()},
      {"gpu", gpu_info_},
      {"build", benchmark::buildInformation()},
      {"environment_note", settings_->environment_note},
      {"first_optimizer_call", first_call},
      {"wall", wall_summary},
      {"process_cpu", cpu_summary},
      {"phase_summary", phase_summary},
      {"samples", measured}};
    benchmark::writeReport(report_path_, report);
    RecordProperty("benchmark_report", std::filesystem::absolute(report_path_).string());
    RecordProperty("instrumented", settings_->phase_timing ? "true" : "false");
    RecordProperty("samples", settings_->samples);
    RecordProperty("wall_mean_ms", std::to_string(wall_summary["mean_ms"].get<double>()));
    RecordProperty("wall_median_ms", std::to_string(wall_summary["median_ms"].get<double>()));
    RecordProperty("wall_p95_ms", std::to_string(wall_summary["p95_ms"].get<double>()));
    RecordProperty("wall_max_ms", std::to_string(wall_summary["max_ms"].get<double>()));
    RecordProperty("process_cpu_mean_ms", std::to_string(cpu_summary["mean_ms"].get<double>()));
    std::cout << "\n[MPPI benchmark] " << name << " | " << report["mode"]
              << " | median=" << wall_summary["median_ms"] << " ms | p95=" << wall_summary["p95_ms"]
              << " ms | CPU mean=" << cpu_summary["mean_ms"] << " ms\n  " << report_path_ << '\n';
  }

  std::unique_ptr<benchmark::Settings> settings_;
  benchmark::Profile profile_;
  std::unique_ptr<FirstOrderDubinsMppiInterface> interface_;

private:
  Json gpu_info_;
  std::filesystem::path report_path_;
  int previous_log_level_{RCUTILS_LOG_SEVERITY_UNSET};
  bool restore_log_level_{false};
};

TEST_F(MppiPerformanceBenchmark, FreeSpaceCached)
{
  runBenchmark(Scene{});
}

TEST_F(MppiPerformanceBenchmark, StaticObstacles16Cached)
{
  runBenchmark(parkedScene());
}

TEST_F(MppiPerformanceBenchmark, StaticObstacles64Cached)
{
  runBenchmark(parkedScene(kMaxMppiObstacles));
}

TEST_F(MppiPerformanceBenchmark, StaticObstacleReferenceCollision)
{
  auto scene = parkedScene(1);
  auto & pose = scene.objects.objects.front().kinematics.pose_with_covariance.pose;
  pose.position.x = 8.0;
  // A shallow hard-margin overlap, leaving a feasible lateral correction within the corridor.
  pose.position.y =
    profile_.vehicle.ego_width * 0.5 + 0.4 + profile_.cost.obstacle_collision_margin - 0.05;
  scene.reference_issue = ReferenceIssue::static_collision;
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, RoadBorderReferenceNearCollision)
{
  ASSERT_GT(profile_.cost.road_border_safe_margin, profile_.cost.road_border_collision_margin);
  Scene scene;
  const double y =
    profile_.vehicle.ego_width * 0.5 +
    0.5 * (profile_.cost.road_border_collision_margin + profile_.cost.road_border_safe_margin);
  scene.borders.push_back({4.0, y, 14.0, y});
  scene.reference_issue = ReferenceIssue::border_near_collision;
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, TightCorridor256Cached)
{
  Scene scene;
  scene.borders = makeCorridor(corridorHalfWidth());
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, StaticObstacleGeometryUpdates)
{
  auto scene = parkedScene();
  scene.change = SceneChange::obstacle_geometry;
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, GridBoundaryCrossings)
{
  auto scene = parkedScene();
  scene.borders = makeCorridor(corridorHalfWidth());
  scene.change = SceneChange::grid_crossing;
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, Hairpin256Cached)
{
  Scene scene;
  scene.trajectory = makeHairpinBenchmarkTrajectory();
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, NoEligibleRollouts)
{
  auto scene = parkedScene(1);
  auto & object = scene.objects.objects.front();
  object.kinematics.pose_with_covariance.pose.position.x = 0.0;
  object.kinematics.pose_with_covariance.pose.position.y = 0.0;
  object.shape.dimensions.x = 200.0;
  object.shape.dimensions.y = 200.0;
  scene.expect_rejection = true;
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, PreferredCenterlineTextureCached)
{
  Scene scene;
  scene.preferred = {{{-100, 0, 200, 0}}, "active", 1};
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, PreferredCenterlineExactCached)
{
  Scene scene;
  scene.preferred = {{{-100, 0, 200, 0}}, "active", 1};
  scene.exact_centerline = true;
  runBenchmark(scene);
}

TEST_F(MppiPerformanceBenchmark, PreferredCenterline256GridCrossings)
{
  Scene scene;
  scene.preferred.status = "active";
  for (int i = 0; i < 256; ++i) {
    const double x = -100.0 + i * 1.5;
    scene.preferred.segments.push_back({x, 0, x + 1.5, 0});
  }
  scene.change = SceneChange::grid_crossing;
  runBenchmark(scene);
}

}  // namespace
}  // namespace autoware::mppi_optimizer
