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

/**
 * Receding-horizon straight-line tracking using the MPPI delay-bicycle as the plant.
 *
 * Each cycle: build an MPPI horizon from the ego's continuous arc length on a fixed
 * straight reference (point 0 at the ego projection); optimizeTrajectory; feedback from
 * MPPI internal applied_plant.
 *
 * Example:
 *   ros2 run autoware_mppi_optimizer mppi_open_loop_line_sim -- \
 *     --params-yaml $(ros2 pkg prefix
 * autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml \
 *     --simulator-yaml $(ros2 pkg prefix
 * j6_gen2_description)/share/j6_gen2_description/config/simulator_model.param.yaml \
 *     --vehicle-info-yaml $(ros2 pkg prefix
 * j6_gen2_description)/share/j6_gen2_description/config/vehicle_info.param.yaml \
 *     --out-dir "$HOME/.cache/autoware/mppi_open_loop_line_sim" --plot
 */

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp"
#include "autoware/mppi_optimizer/mppi_debug_trajectory_io.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{

using autoware::mppi_optimizer::FirstOrderDubinsMppiControl;
using autoware::mppi_optimizer::FirstOrderDubinsMppiCostParams;
using autoware::mppi_optimizer::FirstOrderDubinsMppiInterface;
using autoware::mppi_optimizer::FirstOrderDubinsMppiOptimizationResult;
using autoware::mppi_optimizer::FirstOrderDubinsMppiRuntimeOptions;
using autoware::mppi_optimizer::FirstOrderDubinsMppiVehicleParams;
using autoware::mppi_optimizer::quaternionFromYaw;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

constexpr float kDt = autoware::mppi_optimizer::detail::kMppiDt;
constexpr int kHorizon = autoware::mppi_optimizer::detail::kMppiHorizon;

float wrapPi(const float yaw);

struct SimPlantState
{
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float velocity{0.0F};
  float acceleration{0.0F};
  float steering{0.0F};
  float sim_time{0.0F};
  std::vector<float> accel_cmd_buffer;
  std::vector<float> steer_cmd_buffer;
};

void printUsage(const char * argv0)
{
  std::cerr
    << "Usage: " << argv0
    << " [options]\n"
       "  --params-yaml FILE   Cost/runtime yaml (default: package share config)\n"
       "  --simulator-yaml FILE  simulator_model.param.yaml (default: j6_gen2 if installed)\n"
       "  --vehicle-info-yaml FILE  vehicle_info.param.yaml (default: j6_gen2 if installed)\n"
       "  --out-dir DIR        CSV/PNG output directory\n"
       "  --duration S         Closed-loop duration [s] (default 8)\n"
       "  --speed MPS          Reference speed [m/s] (default 8)\n"
       "  --goal-x M           Goal x on the line [m] (default speed * duration)\n"
       "  --y-offset M         Initial lateral offset [m] (default 0.5)\n"
       "  --yaw-offset-deg DEG Initial heading error [deg] (default 0)\n"
       "  --no-temporal-mpt    Seed u_nom from the geometric reference, not t-MPT\n"
       "  --no-delay           Zero acc/steer dead time in the MPPI plant\n"
       "  --plot               Run the matplotlib plotter after the sim\n"
       "  --help\n";
}

std::string trim(const std::string & s)
{
  const auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return "";
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

float wrapPi(const float yaw)
{
  float wrapped = yaw;
  while (wrapped > static_cast<float>(M_PI)) {
    wrapped -= 2.0F * static_cast<float>(M_PI);
  }
  while (wrapped < -static_cast<float>(M_PI)) {
    wrapped += 2.0F * static_cast<float>(M_PI);
  }
  return wrapped;
}

std::optional<bool> parseBool(const std::string & value)
{
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }
  if (value == "false" || value == "False" || value == "0") {
    return false;
  }
  return std::nullopt;
}

void loadParamsYaml(
  const std::string & path, FirstOrderDubinsMppiCostParams & cost,
  FirstOrderDubinsMppiRuntimeOptions & runtime)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Failed to open params yaml: " + path);
  }

  std::unordered_map<std::string, float *> cost_fields = {
    {"lambda", &cost.lambda},
    {"lambda_min", &cost.lambda_min},
    {"lambda_max", &cost.lambda_max},
    {"target_ess_ratio", &cost.target_ess_ratio},
    {"lambda_adaptation_gain", &cost.lambda_adaptation_gain},
    {"unsafe_rollout_fraction_threshold", &cost.unsafe_rollout_fraction_threshold},
    {"cost_normalization_percentile", &cost.cost_normalization_percentile},
    {"spatial_overspeed_coeff", &cost.spatial_overspeed_coeff},
    {"track_coeff", &cost.track_coeff},
    {"track_terminal_scale", &cost.track_terminal_scale},
    {"terminal_error_coeff", &cost.terminal_error_coeff},
    {"terminal_heading_coeff", &cost.terminal_heading_coeff},
    {"heading_coeff", &cost.heading_coeff},
    {"lateral_distance_coeff", &cost.lateral_distance_coeff},
    {"lateral_yaw_error_coeff", &cost.lateral_yaw_error_coeff},
    {"remaining_distance_coeff", &cost.remaining_distance_coeff},
    {"path_overshoot_coeff", &cost.path_overshoot_coeff},
    {"track_center_coeff", &cost.track_center_coeff},
    {"corner_buffer_coeff", &cost.corner_buffer_coeff},
    {"corner_safe_margin", &cost.corner_safe_margin},
    {"boundary_threshold", &cost.boundary_threshold},
    {"lateral_boundary_soft_margin", &cost.lateral_boundary_soft_margin},
    {"accel_cmd_coeff", &cost.accel_cmd_coeff},
    {"steer_cmd_coeff", &cost.steer_cmd_coeff},
    {"steer_rate_coeff", &cost.steer_rate_coeff},
    {"overlimit_coeff", &cost.overlimit_coeff},
    {"accel_cmd_std_dev", &cost.accel_cmd_std_dev},
    {"steer_cmd_std_dev", &cost.steer_cmd_std_dev},
    {"accel_cmd_noise_exponent", &cost.accel_cmd_noise_exponent},
    {"steer_cmd_noise_exponent", &cost.steer_cmd_noise_exponent},
    {"nominal_curvature_min_chord_length_m", &cost.nominal_curvature_min_chord_length_m},
    {"lateral_acceleration_coeff", &cost.lateral_acceleration_coeff},
    {"lateral_jerk_coeff", &cost.lateral_jerk_coeff},
    {"longitudinal_jerk_coeff", &cost.longitudinal_jerk_coeff},
    {"obstacle_collision_margin", &cost.obstacle_collision_margin},
    {"road_border_collision_margin", &cost.road_border_collision_margin},
    {"obstacle_safe_margin", &cost.obstacle_safe_margin},
    {"road_border_safe_margin", &cost.road_border_safe_margin},
    {"drivable_area_safe_margin", &cost.drivable_area_safe_margin},
    {"drivable_area_barrier_weight", &cost.drivable_area_barrier_weight},
    {"crash_contact_penalty", &cost.crash_contact_penalty},
  };

  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#' || line.find(':') == std::string::npos) {
      continue;
    }
    const auto colon = line.find(':');
    const std::string key = trim(line.substr(0, colon));
    std::string value = trim(line.substr(colon + 1));
    if (!value.empty() && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }

    if (const auto flag = parseBool(value)) {
      if (key == "use_temporal_mpt_as_nominal") {
        runtime.use_temporal_mpt_as_nominal = *flag;
      } else if (key == "use_last_control_as_nominal") {
        runtime.use_last_control_as_nominal = *flag;
      } else if (key == "enable_input_delay_compensation") {
        runtime.enable_input_delay_compensation = *flag;
      } else if (key == "prevent_reverse_velocity") {
        runtime.prevent_reverse_velocity = *flag;
      } else if (key == "enable_curvature_adaptive_steering_filter") {
        runtime.enable_curvature_adaptive_steering_filter = *flag;
      }
      continue;
    }

    if (key == "steering_filter_alpha_straight") {
      runtime.steering_filter_alpha_straight = std::stof(value);
      continue;
    }
    if (key == "steering_filter_alpha_turn") {
      runtime.steering_filter_alpha_turn = std::stof(value);
      continue;
    }
    if (key == "steering_filter_turn_angle_rad") {
      runtime.steering_filter_turn_angle_rad = std::stof(value);
      continue;
    }

    const auto it = cost_fields.find(key);
    if (it == cost_fields.end()) {
      continue;
    }
    try {
      *it->second = std::stof(value);
    } catch (const std::exception &) {
    }
  }
}

std::string defaultParamsYaml()
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory("autoware_mppi_optimizer");
    const auto path = std::filesystem::path(share) / "config" / "mppi_optimizer.param.yaml";
    if (std::filesystem::is_regular_file(path)) {
      return path.string();
    }
  } catch (const std::exception &) {
  }
  return "";
}

std::string defaultVehicleDescriptionYaml(const char * filename)
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory("j6_gen2_description");
    const auto path = std::filesystem::path(share) / "config" / filename;
    if (std::filesystem::is_regular_file(path)) {
      return path.string();
    }
  } catch (const std::exception &) {
  }
  return "";
}

FirstOrderDubinsMppiVehicleParams loadVehicleParamsFromYaml(
  const std::string & vehicle_info_yaml, const std::string & simulator_yaml)
{
  if (vehicle_info_yaml.empty() || simulator_yaml.empty()) {
    throw std::runtime_error(
      "Both --vehicle-info-yaml and --simulator-yaml are required to load actuator params");
  }
  if (!std::filesystem::is_regular_file(vehicle_info_yaml)) {
    throw std::runtime_error("vehicle info yaml not found: " + vehicle_info_yaml);
  }
  if (!std::filesystem::is_regular_file(simulator_yaml)) {
    throw std::runtime_error("simulator yaml not found: " + simulator_yaml);
  }

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.arguments(
    {"--ros-args", "--params-file", vehicle_info_yaml, "--params-file", simulator_yaml});
  auto node = rclcpp::Node::make_shared("mppi_open_loop_line_sim_params", options);
  autoware::mppi_optimizer::declare_first_order_dubins_mppi_vehicle_dynamics_params(*node);
  return autoware::mppi_optimizer::get_first_order_dubins_mppi_vehicle_params(*node);
}

void logVehicleParams(const FirstOrderDubinsMppiVehicleParams & vehicle)
{
  std::cerr << "Vehicle params: wheel_base=" << vehicle.wheel_base
            << " acc_tau=" << vehicle.acc_time_constant
            << " steer_tau=" << vehicle.steer_time_constant
            << " acc_delay=" << vehicle.acc_time_delay
            << " steer_delay=" << vehicle.steer_time_delay
            << " steer_rate_lim=" << vehicle.steer_rate_lim
            << " vel_rate_lim=" << vehicle.vel_rate_lim << "\n";
}

std::string defaultOutDir()
{
  const char * cache = std::getenv("XDG_CACHE_HOME");
  std::filesystem::path root;
  if (cache && cache[0] != '\0') {
    root = std::filesystem::path(cache);
  } else {
    const char * home = std::getenv("HOME");
    root = (home && home[0] != '\0') ? std::filesystem::path(home) / ".cache"
                                     : std::filesystem::temp_directory_path();
  }
  return (root / "autoware" / "mppi_open_loop_line_sim").string();
}

/** Fixed Δs along y=0; matches one MPPI stage at the reference speed. */
float referenceSpacingMeters(const float speed)
{
  return speed * kDt;
}

/**
 * Build a world-fixed straight reference from x=0 through goal_x plus horizon margin.
 * Point i is at arc length i·ds on y=0; overlapping MPC windows share the same coordinates.
 */
Trajectory makeFixedStraightLinePath(
  const float goal_x, const float speed, const float ds, const int sim_steps)
{
  if (ds <= 0.0F) {
    throw std::runtime_error("reference spacing must be positive");
  }
  const float margin = static_cast<float>(kHorizon + sim_steps) * ds;
  const float path_length = std::max(goal_x, ds) + margin;
  const std::size_t point_count = static_cast<std::size_t>(std::ceil(
                                    static_cast<double>(path_length) / static_cast<double>(ds))) +
                                  1U;

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points.reserve(point_count);
  for (std::size_t i = 0; i < point_count; ++i) {
    const float s = static_cast<float>(i) * ds;
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(s);
    point.pose.position.y = 0.0;
    point.pose.orientation = quaternionFromYaw(0.0);
    point.longitudinal_velocity_mps = speed;
    const int nanos = static_cast<int>(std::lround(static_cast<double>(s / speed) * 1.0e9));
    point.time_from_start.sec = nanos / 1000000000;
    point.time_from_start.nanosec = static_cast<std::uint32_t>(nanos % 1000000000);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

float maxPathArcLengthMeters(const Trajectory & fixed_path)
{
  if (fixed_path.points.empty()) {
    return 0.0F;
  }
  return static_cast<float>(fixed_path.points.back().pose.position.x);
}

/** Ego arc length on the y=0 straight path (x is the path coordinate in this sim). */
float egoArcLengthOnPath(const SimPlantState & plant)
{
  return plant.x;
}

TrajectoryPoint makeStraightPathPointAtArcLength(
  const float s, const float speed, const float s_max)
{
  const float s_clamped = std::clamp(s, 0.0F, s_max);
  TrajectoryPoint point;
  point.pose.position.x = static_cast<double>(s_clamped);
  point.pose.position.y = 0.0;
  point.pose.orientation = quaternionFromYaw(0.0);
  point.longitudinal_velocity_mps = speed;
  const int nanos = static_cast<int>(std::lround(static_cast<double>(s_clamped / speed) * 1.0e9));
  point.time_from_start.sec = nanos / 1000000000;
  point.time_from_start.nanosec = static_cast<std::uint32_t>(nanos % 1000000000);
  return point;
}

/**
 * MPPI horizon from continuous ego arc length s_ego (not a rounded grid index).
 * Point k is at s_ego + k·ds so reference[0] stays at the ego projection each step.
 */
Trajectory sliceReferenceHorizonFromArcLength(
  const Trajectory & fixed_path, const float s_ego, const float ds, const std::size_t count,
  const float speed)
{
  Trajectory trajectory;
  trajectory.header = fixed_path.header;
  if (fixed_path.points.empty() || count == 0U || ds <= 0.0F) {
    return trajectory;
  }
  const float s_max = maxPathArcLengthMeters(fixed_path);
  trajectory.points.reserve(count);
  for (std::size_t k = 0; k < count; ++k) {
    const float s = std::max(0.0F, s_ego + static_cast<float>(k) * ds);
    trajectory.points.push_back(makeStraightPathPointAtArcLength(s, speed, s_max));
  }
  return trajectory;
}

void updatePlantFromResult(
  SimPlantState & plant, FirstOrderDubinsMppiControl & command,
  const FirstOrderDubinsMppiOptimizationResult & result)
{
  if (!result.debug.applied_plant.valid) {
    throw std::runtime_error("MPPI did not publish applied_plant snapshot");
  }
  const auto & snap = result.debug.applied_plant;
  // Single source of truth: host plant after runStep (matches delay FIFOs and lag states).
  plant.x = snap.x;
  plant.y = snap.y;
  plant.yaw = snap.yaw;
  plant.velocity = snap.velocity;
  plant.acceleration = snap.acceleration;
  plant.steering = snap.steering;
  plant.sim_time = snap.sim_time;
  plant.accel_cmd_buffer = snap.accel_cmd_delay_buffer;
  plant.steer_cmd_buffer = snap.steer_cmd_delay_buffer;
  command = snap.applied_control;
}

Odometry makeOdometry(const SimPlantState & plant, const float sim_time_s)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.child_frame_id = "base_link";
  odometry.header.stamp.sec = static_cast<int32_t>(sim_time_s);
  odometry.header.stamp.nanosec =
    static_cast<uint32_t>((sim_time_s - static_cast<float>(odometry.header.stamp.sec)) * 1.0E9F);
  odometry.pose.pose.position.x = plant.x;
  odometry.pose.pose.position.y = plant.y;
  odometry.pose.pose.orientation = quaternionFromYaw(plant.yaw);
  odometry.twist.twist.linear.x = plant.velocity;
  return odometry;
}

geometry_msgs::msg::AccelWithCovarianceStamped makeAccel(const SimPlantState & plant)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header.frame_id = "base_link";
  accel.accel.accel.linear.x = plant.acceleration;
  return accel;
}

autoware_vehicle_msgs::msg::SteeringReport makeSteering(const SimPlantState & plant)
{
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.steering_tire_angle = plant.steering;
  return steering;
}

bool writePlantCsv(const std::string & path, const std::vector<std::string> & rows)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "t,x,y,yaw,v,accel,steer,accel_cmd,steer_cmd,cross_track_m,heading_error_rad\n";
  for (const auto & row : rows) {
    out << row << "\n";
  }
  return true;
}

std::string formatRow(
  const float t, const SimPlantState & plant, const FirstOrderDubinsMppiControl & command)
{
  std::ostringstream oss;
  oss << std::setprecision(9) << std::fixed << t << "," << plant.x << "," << plant.y << ","
      << plant.yaw << "," << plant.velocity << "," << plant.acceleration << "," << plant.steering
      << "," << command.accel_cmd << "," << command.steer_cmd << "," << plant.y << ","
      << wrapPi(plant.yaw);
  return oss.str();
}

std::filesystem::path findPlotScript(const char * argv0)
{
  const std::filesystem::path exe(argv0);
  const auto sibling = exe.parent_path() / "mppi_open_loop_line_sim_plot.py";
  if (std::filesystem::is_regular_file(sibling)) {
    return sibling;
  }
  if (const char * env = std::getenv("MPPI_OPEN_LOOP_PLOT_SCRIPT")) {
    const std::filesystem::path from_env(env);
    if (std::filesystem::is_regular_file(from_env)) {
      return from_env;
    }
  }
  return {};
}

}  // namespace

int run(int argc, char ** argv);

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "mppi_open_loop_line_sim failed: " << e.what() << "\n";
    return 1;
  }
}

int run(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
  }

  struct RclcppGuard
  {
    RclcppGuard(int argc_in, char ** argv_in) { rclcpp::init(argc_in, argv_in); }
    ~RclcppGuard()
    {
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  } rclcpp_guard(argc, argv);

  std::string params_yaml = defaultParamsYaml();
  std::string simulator_yaml = defaultVehicleDescriptionYaml("simulator_model.param.yaml");
  std::string vehicle_info_yaml = defaultVehicleDescriptionYaml("vehicle_info.param.yaml");
  std::string out_dir = defaultOutDir();
  float duration_s = 8.0F;
  float speed = 8.0F;
  float goal_x = 0.0F;
  bool goal_x_set = false;
  float y_offset = 0.5F;
  float yaw_offset_rad = 0.0F;
  bool no_temporal_mpt = false;
  bool no_delay = false;
  bool plot = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto require_value = [&](const char * name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };
    if (arg == "--params-yaml") {
      params_yaml = require_value("--params-yaml");
    } else if (arg == "--simulator-yaml") {
      simulator_yaml = require_value("--simulator-yaml");
    } else if (arg == "--vehicle-info-yaml") {
      vehicle_info_yaml = require_value("--vehicle-info-yaml");
    } else if (arg == "--out-dir") {
      out_dir = require_value("--out-dir");
    } else if (arg == "--duration") {
      duration_s = std::stof(require_value("--duration"));
    } else if (arg == "--speed") {
      speed = std::stof(require_value("--speed"));
    } else if (arg == "--goal-x") {
      goal_x = std::stof(require_value("--goal-x"));
      goal_x_set = true;
    } else if (arg == "--y-offset") {
      y_offset = std::stof(require_value("--y-offset"));
    } else if (arg == "--yaw-offset-deg") {
      yaw_offset_rad =
        std::stof(require_value("--yaw-offset-deg")) * static_cast<float>(M_PI) / 180.0F;
    } else if (arg == "--no-temporal-mpt") {
      no_temporal_mpt = true;
    } else if (arg == "--no-delay") {
      no_delay = true;
    } else if (arg == "--plot") {
      plot = true;
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (duration_s <= 0.0F || speed <= 0.0F) {
    throw std::runtime_error("duration and speed must be positive");
  }
  if (!goal_x_set) {
    // Keep the goal beyond the sim window so remaining_distance cost does not spike at t=T.
    goal_x = speed * duration_s + static_cast<float>(kHorizon) * speed * kDt;
  }

  FirstOrderDubinsMppiCostParams cost_params;
  FirstOrderDubinsMppiRuntimeOptions runtime;
  runtime.ignore_obstacles = true;
  runtime.ignore_road_borders = true;
  runtime.ignore_drivable_area = true;
  runtime.force_cold_start_each_step = false;
  runtime.skip_if_invalid = false;
  runtime.min_optimization_length = 0.0F;
  runtime.use_temporal_mpt_as_nominal = true;
  runtime.use_last_control_as_nominal = true;
  runtime.prevent_reverse_velocity = true;
  runtime.enable_input_delay_compensation = false;
  runtime.enable_debug_trajectory_log = false;

  if (!params_yaml.empty()) {
    loadParamsYaml(params_yaml, cost_params, runtime);
    std::cerr << "Loaded params from " << params_yaml << "\n";
  } else {
    std::cerr << "WARNING: no params yaml found; using compiled cost defaults\n";
  }
  runtime.skip_if_invalid = false;
  runtime.ignore_obstacles = true;
  runtime.ignore_road_borders = true;
  runtime.ignore_drivable_area = true;
  runtime.enable_debug_trajectory_log = false;
  if (no_temporal_mpt) {
    runtime.use_temporal_mpt_as_nominal = false;
  }
  if (no_delay) {
    runtime.enable_input_delay_compensation = false;
  }

  FirstOrderDubinsMppiVehicleParams vehicle;
  if (!simulator_yaml.empty() && !vehicle_info_yaml.empty()) {
    vehicle = loadVehicleParamsFromYaml(vehicle_info_yaml, simulator_yaml);
    std::cerr << "Loaded actuator params via get_first_order_dubins_mppi_vehicle_params\n"
              << "  vehicle_info: " << vehicle_info_yaml << "\n"
              << "  simulator: " << simulator_yaml << "\n";
    logVehicleParams(vehicle);
  } else {
    std::cerr << "WARNING: no vehicle/simulator yaml found; using compiled actuator defaults\n";
    vehicle.wheel_base = 4.76F;
    vehicle.ego_length = 5.0F;
    vehicle.ego_width = 1.9F;
    vehicle.ego_axle_to_box_center = 1.5F;
    vehicle.max_steer_angle = 0.7F;
    vehicle.acc_time_constant = 0.1F;
    vehicle.steer_time_constant = 0.27F;
    vehicle.steer_rate_lim = 5.0F;
    vehicle.vel_rate_lim = 7.0F;
    vehicle.acc_time_delay = 0.1F;
    vehicle.steer_time_delay = 0.24F;
  }
  if (no_delay) {
    vehicle.acc_time_delay = 0.0F;
    vehicle.steer_time_delay = 0.0F;
  }

  FirstOrderDubinsMppiInterface mppi;
  mppi.setVehicleParams(vehicle);
  mppi.setCostParams(cost_params);
  mppi.setRuntimeOptions(runtime);

  SimPlantState plant;
  plant.x = 0.0F;
  plant.y = y_offset;
  plant.yaw = yaw_offset_rad;
  plant.velocity = speed;
  plant.acceleration = 0.0F;
  plant.steering = 0.0F;

  FirstOrderDubinsMppiControl command;
  std::vector<std::string> rows;
  rows.push_back(formatRow(0.0F, plant, command));

  std::filesystem::create_directories(out_dir);
  const auto plant_csv = (std::filesystem::path(out_dir) / "plant.csv").string();
  const auto horizon_csv = (std::filesystem::path(out_dir) / "horizon0.csv").string();

  const int steps = static_cast<int>(std::lround(duration_s / kDt));
  const float ref_ds = referenceSpacingMeters(speed);
  const Trajectory fixed_reference = makeFixedStraightLinePath(goal_x, speed, ref_ds, steps);
  std::cerr << "Straight-line plant sim: steps=" << steps << " dt=" << kDt << " v=" << speed
            << " goal_x=" << goal_x << " y0=" << y_offset
            << " t-MPT=" << (runtime.use_temporal_mpt_as_nominal ? "on" : "off")
            << " delay=" << (runtime.enable_input_delay_compensation ? "on" : "off") << "\n";
  if (runtime.use_temporal_mpt_as_nominal && runtime.enable_input_delay_compensation) {
    std::cerr << "NOTE: t-MPT OCP has no delay FIFOs; MPPI shifts its nominal by acc/steer delay "
                 "steps before seeding u_nom.\n";
  }

  autoware_perception_msgs::msg::TrackedObjects objects;
  autoware::mppi_optimizer::FirstOrderDubinsMppiKinematicLimits limits;
  const std::size_t reference_points = static_cast<std::size_t>(kHorizon);
  std::cerr << "  fixed reference: " << fixed_reference.points.size() << " points, ds=" << ref_ds
            << " m; horizon anchored at ego arc length each step\n";
  for (int step = 0; step < steps; ++step) {
    const float sim_t = static_cast<float>(step) * kDt;
    const float s_ego = egoArcLengthOnPath(plant);
    const auto reference =
      sliceReferenceHorizonFromArcLength(fixed_reference, s_ego, ref_ds, reference_points, speed);
    const auto result = mppi.optimizeTrajectory(
      reference, makeOdometry(plant, sim_t), makeAccel(plant), makeSteering(plant), objects, {}, {},
      limits);
    if (step == 0) {
      if (!autoware::mppi_optimizer::writeMppiDebugOptimalHorizonCsv(
            horizon_csv, result.debug.optimal_horizon, kDt)) {
        throw std::runtime_error("Failed to write first optimal horizon " + horizon_csv);
      }
    }
    updatePlantFromResult(plant, command, result);
    const float t = static_cast<float>(step + 1) * kDt;
    rows.push_back(formatRow(t, plant, command));
    if ((step + 1) % 10 == 0 || step + 1 == steps) {
      const auto & pred = result.debug.prediction_accuracy;
      std::cerr << "  t=" << std::fixed << std::setprecision(2) << t << " x=" << plant.x
                << " y=" << plant.y << " yaw=" << plant.yaw << " v=" << plant.velocity
                << " u_a=" << command.accel_cmd << " u_d=" << command.steer_cmd;
      if (pred.valid) {
        std::cerr << " pred_pos_err=" << pred.pos_error_m << "m pred_yaw_err=" << pred.yaw_error_rad
                  << "rad elapsed=" << pred.elapsed_s << "s steps=" << pred.full_steps << "+"
                  << pred.remainder_s;
      }
      std::cerr << "\n";
    }
  }

  if (!writePlantCsv(plant_csv, rows)) {
    throw std::runtime_error("Failed to write " + plant_csv);
  }
  std::cerr << "Wrote " << plant_csv << "\n";
  std::cerr << "Wrote first optimal-horizon rollout " << horizon_csv << "\n";

  if (plot) {
    const auto script = findPlotScript(argv[0]);
    if (script.empty()) {
      std::cerr << "Plot script not found. Run:\n  ros2 run autoware_mppi_optimizer "
                   "mppi_open_loop_line_sim_plot.py -- --csv "
                << plant_csv << "\n";
    } else {
      const auto png = (std::filesystem::path(out_dir) / "plant.png").string();
      const std::string cmd = "python3 \"" + script.string() + "\" --csv \"" + plant_csv +
                              "\" --horizon-csv \"" + horizon_csv + "\" --out \"" + png + "\"";
      std::cerr << "Plotting: " << cmd << "\n";
      if (std::system(cmd.c_str()) != 0) {
        std::cerr << "WARNING: plotter exited non-zero\n";
      }
    }
  } else {
    std::cerr
      << "Plot with:\n  ros2 run autoware_mppi_optimizer mppi_open_loop_line_sim_plot.py -- "
         "--csv "
      << plant_csv << "\n";
  }
  return 0;
}
