// Copyright 2026 TIER IV, Inc.
// SPDX-License-Identifier: Apache-2.0

#ifndef PERFORMANCE_BENCHMARK_SUPPORT_HPP_
#define PERFORMANCE_BENCHMARK_SUPPORT_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "performance_build_info.hpp"

#include <nlohmann/json.hpp>

#include <sys/utsname.h>
#include <time.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <type_traits>
#include <vector>

namespace autoware::mppi_optimizer::benchmark
{
using Json = nlohmann::json;

inline std::string environment(const char * name, const std::string & fallback = "")
{
  const char * value = std::getenv(name);
  return value ? value : fallback;
}

inline int integerOption(const char * name, int fallback, int minimum, int maximum)
{
  const auto text = environment(name, std::to_string(fallback));
  int value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (
    parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() || value < minimum ||
    value > maximum) {
    throw std::invalid_argument(
      std::string(name) + " must be an integer in [" + std::to_string(minimum) + ", " +
      std::to_string(maximum) + "]");
  }
  return value;
}

struct Settings
{
  int warmup{integerOption("MPPI_BENCHMARK_WARMUP", 10, 1, 100000)};
  int samples{integerOption("MPPI_BENCHMARK_SAMPLES", 100, 1, 100000)};
  int iterations{integerOption("MPPI_BENCHMARK_ITERATIONS", 0, 0, kMaxMppiIterations)};
  int device{integerOption("MPPI_BENCHMARK_DEVICE", 0, 0, 1024)};
  bool phase_timing{integerOption("MPPI_BENCHMARK_PHASE_TIMING", 0, 0, 1) != 0};
  bool projection_pruning{integerOption("MPPI_BENCHMARK_PROJECTION_PRUNING", 1, 0, 1) != 0};
  std::string cost_profile{environment("MPPI_BENCHMARK_COST_PROFILE", kDefaultCostProfile)};
  std::string vehicle_profile{
    environment("MPPI_BENCHMARK_VEHICLE_PROFILE", kDefaultVehicleProfile)};
  std::filesystem::path output_dir{
    environment("MPPI_BENCHMARK_OUTPUT_DIR", "mppi_benchmark_results")};
  std::string environment_note{environment("MPPI_BENCHMARK_ENVIRONMENT")};
};

inline std::string readText(const std::string & path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot read benchmark profile: " + path);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

template <typename T>
void scalar(const YAML::Node & node, const char * key, T & value, Json & snapshot, bool required)
{
  if (node[key]) {
    value = node[key].as<T>();
  } else if (required) {
    throw std::invalid_argument(std::string("Missing benchmark profile field: ") + key);
  }
  if constexpr (std::is_floating_point_v<T>) {
    if (!std::isfinite(value)) throw std::invalid_argument(std::string("Nonfinite field: ") + key);
  }
  snapshot[key] = value;
}

struct Profile
{
  FirstOrderDubinsMppiCostParams cost;
  FirstOrderDubinsMppiRuntimeOptions runtime;
  FirstOrderDubinsMppiVehicleParams vehicle;
  Json description;
};

inline Profile loadProfile(const Settings & settings)
{
  Profile profile;
  auto & cost = profile.cost;
  auto & runtime = profile.runtime;
  auto & vehicle = profile.vehicle;
  const auto cost_text = readText(settings.cost_profile);
  const auto vehicle_text = readText(settings.vehicle_profile);
  const auto cost_root = YAML::Load(cost_text);
  const auto node = cost_root["/**"]["ros__parameters"]["mppi_optimizer"];
  const auto vehicle_root = YAML::Load(vehicle_text);
  const auto vehicle_node = vehicle_root["vehicle"];
  if (!node.IsMap() || !vehicle_node.IsMap()) {
    throw std::invalid_argument(
      "Expected /**.ros__parameters.mppi_optimizer and vehicle YAML maps");
  }
  auto & cost_json = profile.description["cost"];
  auto & runtime_json = profile.description["runtime"];
  auto & vehicle_json = profile.description["vehicle"];
  scalar(node, "lambda", cost.lambda, cost_json, true);
  scalar(node, "lambda_min", cost.lambda_min, cost_json, true);
  scalar(node, "lambda_max", cost.lambda_max, cost_json, true);
  scalar(node, "target_ess_ratio", cost.target_ess_ratio, cost_json, true);
  scalar(node, "lambda_adaptation_gain", cost.lambda_adaptation_gain, cost_json, true);
  scalar(
    node, "unsafe_rollout_fraction_threshold", cost.unsafe_rollout_fraction_threshold, cost_json,
    true);
  scalar(
    node, "cost_normalization_percentile", cost.cost_normalization_percentile, cost_json, true);
  scalar(node, "max_iter", cost.max_iter, cost_json, true);
  scalar(node, "spatial_overspeed_coeff", cost.spatial_overspeed_coeff, cost_json, true);
  scalar(node, "track_coeff", cost.track_coeff, cost_json, true);
  scalar(node, "track_terminal_scale", cost.track_terminal_scale, cost_json, true);
  scalar(node, "terminal_error_coeff", cost.terminal_error_coeff, cost_json, true);
  scalar(node, "terminal_heading_coeff", cost.terminal_heading_coeff, cost_json, true);
  scalar(node, "heading_coeff", cost.heading_coeff, cost_json, true);
  scalar(node, "lateral_distance_coeff", cost.lateral_distance_coeff, cost_json, true);
  scalar(node, "lateral_yaw_error_coeff", cost.lateral_yaw_error_coeff, cost_json, true);
  scalar(node, "remaining_distance_coeff", cost.remaining_distance_coeff, cost_json, true);
  scalar(node, "path_overshoot_coeff", cost.path_overshoot_coeff, cost_json, true);
  scalar(node, "preferred_lane_center_coeff", cost.preferred_lane_center_coeff, cost_json, true);
  scalar(node, "track_center_coeff", cost.track_center_coeff, cost_json, true);
  scalar(node, "corner_buffer_coeff", cost.corner_buffer_coeff, cost_json, true);
  scalar(node, "corner_safe_margin", cost.corner_safe_margin, cost_json, true);
  scalar(node, "boundary_threshold", cost.boundary_threshold, cost_json, true);
  scalar(node, "lateral_boundary_soft_margin", cost.lateral_boundary_soft_margin, cost_json, true);
  scalar(
    node, "lateral_boundary_barrier_weight", cost.lateral_boundary_barrier_weight, cost_json,
    false);
  scalar(node, "accel_cmd_coeff", cost.accel_cmd_coeff, cost_json, true);
  scalar(node, "steer_cmd_coeff", cost.steer_cmd_coeff, cost_json, true);
  scalar(node, "steer_rate_coeff", cost.steer_rate_coeff, cost_json, true);
  scalar(node, "initial_steer_rate_coeff", cost.initial_steer_rate_coeff, cost_json, true);
  scalar(node, "accel_cmd_rate_coeff", cost.accel_cmd_rate_coeff, cost_json, true);
  scalar(node, "steer_cmd_rate_coeff", cost.steer_cmd_rate_coeff, cost_json, true);
  scalar(node, "overlimit_coeff", cost.overlimit_coeff, cost_json, true);
  scalar(node, "accel_cmd_std_dev", cost.accel_cmd_std_dev, cost_json, true);
  scalar(node, "steer_cmd_std_dev", cost.steer_cmd_std_dev, cost_json, true);
  scalar(node, "std_dev_decay", cost.std_dev_decay, cost_json, true);
  scalar(node, "accel_cmd_noise_exponent", cost.accel_cmd_noise_exponent, cost_json, true);
  scalar(node, "steer_cmd_noise_exponent", cost.steer_cmd_noise_exponent, cost_json, true);
  scalar(
    node, "nominal_curvature_min_chord_length_m", cost.nominal_curvature_min_chord_length_m,
    cost_json, true);
  scalar(
    node, "nominal_curvature_fit_window_m", cost.nominal_curvature_fit_window_m, cost_json, true);
  scalar(node, "lateral_acceleration_coeff", cost.lateral_acceleration_coeff, cost_json, true);
  scalar(node, "lateral_jerk_coeff", cost.lateral_jerk_coeff, cost_json, true);
  scalar(node, "longitudinal_jerk_coeff", cost.longitudinal_jerk_coeff, cost_json, true);
  scalar(node, "obstacle_collision_margin", cost.obstacle_collision_margin, cost_json, true);
  scalar(node, "road_border_collision_margin", cost.road_border_collision_margin, cost_json, true);
  scalar(node, "obstacle_safe_margin", cost.obstacle_safe_margin, cost_json, true);
  scalar(node, "obstacle_barrier_weight", cost.obstacle_barrier_weight, cost_json, false);
  scalar(node, "vegetation_collision_margin", cost.vegetation_collision_margin, cost_json, true);
  scalar(node, "vegetation_safe_margin", cost.vegetation_safe_margin, cost_json, true);
  scalar(node, "vegetation_barrier_weight", cost.vegetation_barrier_weight, cost_json, false);
  scalar(node, "road_border_safe_margin", cost.road_border_safe_margin, cost_json, true);
  scalar(node, "road_border_barrier_weight", cost.road_border_barrier_weight, cost_json, false);
  scalar(node, "drivable_area_safe_margin", cost.drivable_area_safe_margin, cost_json, true);
  scalar(node, "drivable_area_barrier_weight", cost.drivable_area_barrier_weight, cost_json, true);
  scalar(node, "crash_contact_penalty", cost.crash_contact_penalty, cost_json, true);
  scalar(
    node, "enable_debug_trajectory_log", runtime.enable_debug_trajectory_log, runtime_json, false);
  scalar(
    node, "enable_distance_map_texture_debug", runtime.enable_distance_map_texture_debug,
    runtime_json, false);
  scalar(
    node, "enable_iteration_rollout_debug", runtime.enable_iteration_rollout_debug, runtime_json,
    false);
  scalar(node, "ignore_obstacles", runtime.ignore_obstacles, runtime_json, false);
  scalar(
    node, "dynamic_obstacle_horizon_s", runtime.dynamic_obstacle_horizon_s, runtime_json, false);
  scalar(node, "ignore_road_borders", runtime.ignore_road_borders, runtime_json, false);
  scalar(node, "ignore_drivable_area", runtime.ignore_drivable_area, runtime_json, false);
  scalar(
    node, "force_cold_start_each_step", runtime.force_cold_start_each_step, runtime_json, false);
  scalar(node, "skip_if_invalid", runtime.skip_if_invalid, runtime_json, false);
  scalar(
    node, "validation_interpolation_substeps", runtime.validation_interpolation_substeps,
    runtime_json, false);
  scalar(node, "min_optimization_length", runtime.min_optimization_length, runtime_json, false);
  scalar(
    node, "steering_hold_reference_length_threshold_m",
    runtime.steering_hold_reference_length_threshold_m, runtime_json, false);
  scalar(node, "min_trajectory_progress_m", runtime.min_trajectory_progress_m, runtime_json, false);
  scalar(
    node, "use_last_control_as_nominal", runtime.use_last_control_as_nominal, runtime_json, false);
  scalar(
    node, "nominal_initial_steering_max_deviation_rad",
    runtime.nominal_initial_steering_max_deviation_rad, runtime_json, false);
  scalar(
    node, "last_control_warm_start_max_age_s", runtime.last_control_warm_start_max_age_s,
    runtime_json, false);
  scalar(
    node, "last_control_warm_start_max_position_error_m",
    runtime.last_control_warm_start_max_position_error_m, runtime_json, false);
  scalar(
    node, "last_control_warm_start_max_yaw_error_rad",
    runtime.last_control_warm_start_max_yaw_error_rad, runtime_json, false);
  scalar(
    node, "last_control_warm_start_max_velocity_error_mps",
    runtime.last_control_warm_start_max_velocity_error_mps, runtime_json, false);
  scalar(
    node, "last_control_warm_start_max_reference_position_error_m",
    runtime.last_control_warm_start_max_reference_position_error_m, runtime_json, false);
  scalar(
    node, "last_control_warm_start_max_reference_yaw_error_rad",
    runtime.last_control_warm_start_max_reference_yaw_error_rad, runtime_json, false);
  scalar(
    node, "last_control_warm_start_max_reference_velocity_error_mps",
    runtime.last_control_warm_start_max_reference_velocity_error_mps, runtime_json, false);
  scalar(
    node, "last_control_warm_start_stop_enter_velocity_mps",
    runtime.last_control_warm_start_stop_enter_velocity_mps, runtime_json, false);
  scalar(
    node, "last_control_warm_start_stop_exit_velocity_mps",
    runtime.last_control_warm_start_stop_exit_velocity_mps, runtime_json, false);
  scalar(node, "prevent_reverse_velocity", runtime.prevent_reverse_velocity, runtime_json, false);
  scalar(
    node, "enable_input_delay_compensation", runtime.enable_input_delay_compensation, runtime_json,
    false);
  scalar(vehicle_node, "ego_length", vehicle.ego_length, vehicle_json, true);
  scalar(vehicle_node, "ego_width", vehicle.ego_width, vehicle_json, true);
  scalar(
    vehicle_node, "ego_axle_to_box_center", vehicle.ego_axle_to_box_center, vehicle_json, true);
  scalar(vehicle_node, "wheel_base", vehicle.wheel_base, vehicle_json, true);
  scalar(vehicle_node, "max_steer_angle", vehicle.max_steer_angle, vehicle_json, true);
  scalar(vehicle_node, "acc_time_constant", vehicle.acc_time_constant, vehicle_json, true);
  scalar(vehicle_node, "steer_time_constant", vehicle.steer_time_constant, vehicle_json, true);
  scalar(vehicle_node, "steer_rate_lim", vehicle.steer_rate_lim, vehicle_json, true);
  scalar(vehicle_node, "max_lateral_jerk_mps3", vehicle.max_lateral_jerk_mps3, vehicle_json, true);
  scalar(
    vehicle_node, "standstill_steer_rate_lim", vehicle.standstill_steer_rate_lim, vehicle_json,
    true);
  scalar(
    vehicle_node, "restart_steer_command_rate_lim", vehicle.restart_steer_command_rate_lim,
    vehicle_json, true);
  scalar(
    vehicle_node, "restart_steer_command_acceleration_lim",
    vehicle.restart_steer_command_acceleration_lim, vehicle_json, true);
  scalar(
    vehicle_node, "restart_velocity_threshold_mps", vehicle.restart_velocity_threshold_mps,
    vehicle_json, true);
  scalar(vehicle_node, "vel_rate_lim", vehicle.vel_rate_lim, vehicle_json, true);
  scalar(vehicle_node, "acc_time_delay", vehicle.acc_time_delay, vehicle_json, true);
  scalar(vehicle_node, "steer_time_delay", vehicle.steer_time_delay, vehicle_json, true);

  if (settings.iterations != 0) cost.max_iter = settings.iterations;
  cost_json["max_iter"] = cost.max_iter;
  validateFirstOrderDubinsMppiCostParams(cost);
  if (cost.max_iter < 1)
    throw std::invalid_argument("Benchmarks require at least one MPPI iteration");
  // Repeat snapshots without accumulating assumed actuator commands from unapplied outputs.
  // RNG and cycle-level ESS adaptation persist; parameter resets are excluded from warm timing.
  runtime.enable_debug_trajectory_log = false;
  runtime.enable_distance_map_texture_debug = false;
  runtime.enable_iteration_rollout_debug = false;
  runtime.force_cold_start_each_step = false;
  runtime.use_last_control_as_nominal = false;
  runtime.skip_if_invalid = true;
  for (const char * key :
       {"enable_debug_trajectory_log", "enable_distance_map_texture_debug",
        "enable_iteration_rollout_debug", "force_cold_start_each_step",
        "use_last_control_as_nominal"}) {
    runtime_json[key] = false;
  }
  runtime_json["skip_if_invalid"] = true;
  runtime_json["enable_rollout_visualization"] = false;
  runtime_json["debug_trajectory_log_directory"] = "";
  profile.description["history_policy"] = "defer_commit_without_acceptance";
  profile.description["cost_profile_path"] = settings.cost_profile;
  profile.description["vehicle_profile_path"] = settings.vehicle_profile;
  profile.description["cost_profile_source"] = cost_text;
  profile.description["vehicle_profile_source"] = vehicle_text;
  return profile;
}

inline double processCpuMilliseconds()
{
  timespec time{};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time) != 0) {
    throw std::system_error(errno, std::generic_category(), "CLOCK_PROCESS_CPUTIME_ID");
  }
  return static_cast<double>(time.tv_sec) * 1000.0 + static_cast<double>(time.tv_nsec) * 1e-6;
}

inline Json finiteNumber(double value)
{
  return std::isfinite(value) ? Json(value) : Json(nullptr);
}

inline Json statistics(std::vector<double> values)
{
  if (values.empty()) throw std::invalid_argument("No benchmark samples");
  std::sort(values.begin(), values.end());
  const auto count = values.size();
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) / count;
  double variance = 0.0;
  for (const auto value : values) variance += (value - mean) * (value - mean);
  const double median =
    count % 2 == 0 ? (values[count / 2 - 1] + values[count / 2]) * 0.5 : values[count / 2];
  const auto p95_index = static_cast<std::size_t>(std::ceil(0.95 * count)) - 1;
  return {
    {"count", count},
    {"mean_ms", mean},
    {"median_ms", median},
    {"p95_ms", values[p95_index]},
    {"min_ms", values.front()},
    {"max_ms", values.back()},
    {"stddev_ms", std::sqrt(variance / count)}};
}

inline Json hostInformation()
{
  std::string cpu = "unavailable";
  std::ifstream cpuinfo("/proc/cpuinfo");
  std::string line;
  while (std::getline(cpuinfo, line)) {
    if (line.rfind("model name", 0) == 0 || line.rfind("Hardware", 0) == 0) {
      const auto colon = line.find(':');
      if (colon != std::string::npos) cpu = line.substr(colon + 1);
      break;
    }
  }
  utsname system{};
  const bool have_system = uname(&system) == 0;
  return {
    {"cpu", cpu},
    {"logical_cpus", std::thread::hardware_concurrency()},
    {"os", have_system ? system.sysname : "unavailable"},
    {"kernel", have_system ? system.release : "unavailable"},
    {"machine", have_system ? system.machine : "unavailable"}};
}

inline Json buildInformation()
{
  return {
    {"package_revision_at_configure", kPackageRevision},
    {"vendor_revision_at_configure", kVendorRevision},
    {"build_type", kBuildType},
    {"compiler", kCompiler},
    {"cxx_flags", kCxxFlags},
    {"cuda_flags", kCudaFlags},
    {"vendor_compile_definitions", kVendorDefinitions},
    {"cuda_toolkit", kCudaToolkit}};
}

inline std::filesystem::path reportPath(const Settings & settings, const std::string & name)
{
  std::filesystem::create_directories(settings.output_dir);
  const auto epoch_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  return settings.output_dir /
         (name + "_" + std::to_string(epoch_ns) + "_" + std::to_string(getpid()) + ".json");
}

inline void writeReport(const std::filesystem::path & path, const Json & report)
{
  std::ofstream output(path);
  if (!output) throw std::runtime_error("Cannot open benchmark result: " + path.string());
  output << report.dump(2) << '\n';
  output.close();
  if (!output) throw std::runtime_error("Cannot write benchmark result: " + path.string());
}

}  // namespace autoware::mppi_optimizer::benchmark

#endif  // PERFORMANCE_BENCHMARK_SUPPORT_HPP_
