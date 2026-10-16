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
 * Receding-horizon tracking on a fixed left-then-right turn reference (MPPI delay-bicycle plant).
 *
 * Path: straight → left arc → straight → right arc → straight (world-fixed polyline).
 *
 * Example:
 *   ros2 run autoware_mppi_optimizer mppi_open_loop_lr_turn_sim -- \
 *   ros2 run autoware_mppi_optimizer mppi_open_loop_lr_turn_sim -- --plot
 *   ros2 run autoware_mppi_optimizer mppi_open_loop_lr_turn_sim -- --static-obstacle --plot
 *   ros2 run autoware_mppi_optimizer mppi_open_loop_lr_turn_sim -- --road-borders --plot
 */

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/tracked_objects_obstacles.hpp"
#include "mppi_open_loop_arc_path.hpp"
#include "mppi_open_loop_sim_params.hpp"
#include "mppi_open_loop_sim_io.hpp"
#include "mppi_open_loop_sim_paths.hpp"
#include "autoware/mppi_optimizer/mppi_debug_trajectory_io.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp"

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

using autoware::mppi_optimizer::Segment;
using autoware::mppi_optimizer::FirstOrderDubinsMppiControl;
using autoware::mppi_optimizer::FirstOrderDubinsMppiCostParams;
using autoware::mppi_optimizer::FirstOrderDubinsMppiInterface;
using autoware::mppi_optimizer::FirstOrderDubinsMppiOptimizationResult;
using autoware::mppi_optimizer::FirstOrderDubinsMppiRuntimeOptions;
using autoware::mppi_optimizer::FirstOrderDubinsMppiVehicleParams;
using autoware::mppi_optimizer::buildObstacleTrajectoryBuffersFromTrackedObjects;
using autoware::mppi_optimizer::makeStationaryTrackedBoxObjects;
using autoware::mppi_optimizer::open_loop_sim::ArcLengthPath;
using autoware::mppi_optimizer::open_loop_sim::defaultOutDir;
using autoware::mppi_optimizer::open_loop_sim::writeEgoFootprintCsv;
using autoware::mppi_optimizer::open_loop_sim::defaultPlanningMppiParamsYaml;
using autoware::mppi_optimizer::open_loop_sim::defaultVehicleDescriptionYaml;
using autoware::mppi_optimizer::open_loop_sim::loadMppiOptimizerParamsFromYaml;
using autoware::mppi_optimizer::open_loop_sim::makeLeftThenRightTurnPath;
using autoware::mppi_optimizer::open_loop_sim::kOpenLoopMaxRoadBorderSegments;
using autoware::mppi_optimizer::open_loop_sim::lrTurnPathLayout;
using autoware::mppi_optimizer::open_loop_sim::makeLrTurnRoadBorders;
using autoware::mppi_optimizer::open_loop_sim::minRoadBorderHalfWidthForEgo;
using autoware::mppi_optimizer::quaternionFromYaw;
using autoware::mppi_optimizer::trackedObjectObstacleCount;
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
       "  --params-yaml FILE   Cost/runtime yaml (default: autoware_launch neural_net, else package)\n"
       "  --simulator-yaml FILE  simulator_model.param.yaml (default: j6_gen2 if installed)\n"
       "  --vehicle-info-yaml FILE  vehicle_info.param.yaml (default: j6_gen2 if installed)\n"
       "  --out-dir DIR        CSV/PNG output directory\n"
       "  --duration S         Closed-loop duration [s] (default 22)\n"
       "  --speed MPS          Reference speed [m/s] (default 8)\n"
       "  --straight-before M  Straight segment before first turn [m] (default 20)\n"
       "  --straight-mid M     Straight between turns [m] (default 15)\n"
       "  --straight-after M   Straight after second turn [m] (default 40)\n"
       "  --turn-radius M      Constant turn radius [m] (default 25)\n"
       "  --turn-angle-deg DEG Each turn angle [deg], left then right (default 45)\n"
       "  --y-offset M         Initial lateral offset [m] (default 0.3)\n"
       "  --yaw-offset-deg DEG Initial heading error [deg] (default 0)\n"
       "  --no-temporal-mpt    Seed u_nom from the geometric reference, not t-MPT\n"
       "  --no-delay           Zero acc/steer dead time in the MPPI plant\n"
       "  --plot               Run the matplotlib plotter after the sim\n"
       "  --static-obstacle    Enable one stationary box on the reference path (MPPI avoidance)\n"
       "  --obstacle-path-s M  Obstacle arc length on path [m] (default: max(22, straight_before-3))\n"
       "  --obstacle-lateral M Lateral offset from path (+ = left) [m] (default 0)\n"
       "  --obstacle-length M  Box length [m] (default 2.5)\n"
       "  --obstacle-width M   Box width [m] (default 1.0)\n"
       "  --road-borders       Enable lane-line segments parallel to the reference (MPPI borders)\n"
       "  --road-border-half-width M  Corridor half-width on straights [m] (default 3.0)\n"
       "  --road-border-turn-half-width M  Tighter outer edge in turns [m] (default 2.25)\n"
       "  --help\n";
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

/** Fixed Δs along the path; matches one MPPI stage at the reference speed. */
float referenceSpacingMeters(const float speed)
{
  return speed * kDt;
}

autoware_perception_msgs::msg::TrackedObjects makeStationaryBoxOnPath(
  const ArcLengthPath & path_geom, const float path_s, const float lateral_m, const float length,
  const float width)
{
  const auto on_path = path_geom.sampleAtArcLength(path_s);
  const float nx = -std::sin(on_path.yaw);
  const float ny = std::cos(on_path.yaw);
  return makeStationaryTrackedBoxObjects(
    static_cast<double>(on_path.x + lateral_m * nx),
    static_cast<double>(on_path.y + lateral_m * ny), static_cast<double>(on_path.yaw), length,
    width);
}

void logTrackedObjectsForMppi(const autoware_perception_msgs::msg::TrackedObjects & objects)
{
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> yaw;
  std::vector<float> half_length;
  std::vector<float> half_width;
  buildObstacleTrajectoryBuffersFromTrackedObjects(
    objects, kDt, kHorizon, x, y, yaw, half_length, half_width, 0.0F);
  std::cerr << "  MPPI obstacle buffers: count=" << trackedObjectObstacleCount(objects);
  if (!half_length.empty()) {
    std::cerr << " half_len=" << half_length.front() << " half_w=" << half_width.front()
              << " x0=" << x.front() << " y0=" << y.front();
  }
  std::cerr << "\n";
}

bool writeRoadBordersCsv(const std::string & path, const std::vector<Segment> & segments)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "x0,y0,x1,y1\n";
  out << std::setprecision(9) << std::fixed;
  for (const auto & seg : segments) {
    out << seg.x0 << "," << seg.y0 << "," << seg.x1 << "," << seg.y1 << "\n";
  }
  return true;
}

bool writeObstaclesCsv(
  const std::string & path, const autoware_perception_msgs::msg::TrackedObjects & objects)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "x,y,yaw,length,width\n";
  for (const auto & object : objects.objects) {
    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const float yaw = static_cast<float>(tf2::getYaw(pose.orientation));
    out << std::setprecision(9) << std::fixed << pose.position.x << "," << pose.position.y << ","
        << yaw << "," << object.shape.dimensions.x << "," << object.shape.dimensions.y << "\n";
  }
  return true;
}

bool writeReferenceCsv(const std::string & path, const ArcLengthPath & path_geom)
{
  std::ofstream out(path);
  if (!out) {
    return false;
  }
  out << "s,x,y,yaw\n";
  for (const auto & sample : path_geom.samples()) {
    out << std::setprecision(9) << std::fixed << sample.s << "," << sample.x << "," << sample.y
        << "," << sample.yaw << "\n";
  }
  return true;
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
  odometry.header.stamp.nanosec = static_cast<uint32_t>(
    (sim_time_s - static_cast<float>(odometry.header.stamp.sec)) * 1.0E9F);
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
  const float t, const SimPlantState & plant, const FirstOrderDubinsMppiControl & command,
  const float cross_track_m, const float heading_error_rad)
{
  std::ostringstream oss;
  oss << std::setprecision(9) << std::fixed << t << "," << plant.x << "," << plant.y << ","
      << plant.yaw << "," << plant.velocity << "," << plant.acceleration << "," << plant.steering
      << "," << command.accel_cmd << "," << command.steer_cmd << "," << cross_track_m << ","
      << heading_error_rad;
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
    std::cerr << "mppi_open_loop_lr_turn_sim failed: " << e.what() << "\n";
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

  std::string params_yaml = defaultPlanningMppiParamsYaml();
  std::string simulator_yaml = defaultVehicleDescriptionYaml("simulator_model.param.yaml");
  std::string vehicle_info_yaml = defaultVehicleDescriptionYaml("vehicle_info.param.yaml");
  std::string out_dir = defaultOutDir("mppi_open_loop_lr_turn_sim");
  float duration_s = 22.0F;
  float speed = 8.0F;
  float straight_before = 20.0F;
  float straight_mid = 15.0F;
  float straight_after = 40.0F;
  float turn_radius = 25.0F;
  float turn_angle_deg = 45.0F;
  float y_offset = 0.3F;
  float yaw_offset_rad = 0.0F;
  bool no_temporal_mpt = false;
  bool no_delay = false;
  bool plot = false;
  bool static_obstacle = false;
  bool road_borders_enabled = false;
  float road_border_half_width = 3.0F;
  float road_border_turn_half_width = 2.25F;
  float obstacle_path_s = -1.0F;
  float obstacle_lateral = 0.0F;
  float obstacle_length = 2.5F;
  float obstacle_width = 1.0F;

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
    } else if (arg == "--straight-before") {
      straight_before = std::stof(require_value("--straight-before"));
    } else if (arg == "--straight-mid") {
      straight_mid = std::stof(require_value("--straight-mid"));
    } else if (arg == "--straight-after") {
      straight_after = std::stof(require_value("--straight-after"));
    } else if (arg == "--turn-radius") {
      turn_radius = std::stof(require_value("--turn-radius"));
    } else if (arg == "--turn-angle-deg") {
      turn_angle_deg = std::stof(require_value("--turn-angle-deg"));
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
    } else if (arg == "--static-obstacle") {
      static_obstacle = true;
    } else if (arg == "--obstacle-path-s") {
      obstacle_path_s = std::stof(require_value("--obstacle-path-s"));
    } else if (arg == "--obstacle-lateral") {
      obstacle_lateral = std::stof(require_value("--obstacle-lateral"));
    } else if (arg == "--obstacle-length") {
      obstacle_length = std::stof(require_value("--obstacle-length"));
    } else if (arg == "--obstacle-width") {
      obstacle_width = std::stof(require_value("--obstacle-width"));
    } else if (arg == "--road-borders") {
      road_borders_enabled = true;
    } else if (arg == "--road-border-half-width") {
      road_border_half_width = std::stof(require_value("--road-border-half-width"));
    } else if (arg == "--road-border-turn-half-width") {
      road_border_turn_half_width = std::stof(require_value("--road-border-turn-half-width"));
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
  if (straight_before <= 0.0F || straight_mid < 0.0F || straight_after <= 0.0F || turn_radius <= 0.0F) {
    throw std::runtime_error("path geometry lengths and turn radius must be positive");
  }
  if (turn_angle_deg <= 0.0F || turn_angle_deg >= 180.0F) {
    throw std::runtime_error("turn-angle-deg must be in (0, 180)");
  }

  FirstOrderDubinsMppiCostParams cost_params;
  FirstOrderDubinsMppiRuntimeOptions runtime;
  if (!params_yaml.empty()) {
    loadMppiOptimizerParamsFromYaml(params_yaml, cost_params, runtime);
    std::cerr << "Loaded mppi_optimizer params from " << params_yaml << " (lambda=" << cost_params.lambda
              << " skip_if_invalid=" << (runtime.skip_if_invalid ? "true" : "false") << ")\n";
  } else {
    std::cerr << "WARNING: no params yaml found; using struct defaults\n";
  }
  runtime.ignore_obstacles = !static_obstacle;
  runtime.ignore_road_borders = !road_borders_enabled;
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

  const float turn_angle_rad =
    turn_angle_deg * static_cast<float>(M_PI) / 180.0F;
  constexpr float kPathIntegrationStep = 0.25F;
  ArcLengthPath path_geom = makeLeftThenRightTurnPath(
    straight_before, turn_radius, turn_angle_rad, straight_mid, straight_after,
    kPathIntegrationStep);

  if (static_obstacle && obstacle_path_s < 0.0F) {
    // On the pre-turn straight so the ego can steer around before the box.
    obstacle_path_s = std::max(22.0F, straight_before - 3.0F);
  }
  if (static_obstacle) {
    if (obstacle_path_s < 0.0F || obstacle_path_s > path_geom.totalLength()) {
      throw std::runtime_error("obstacle-path-s must lie on the reference path");
    }
    if (obstacle_length <= 0.0F || obstacle_width <= 0.0F) {
      throw std::runtime_error("obstacle length and width must be positive");
    }
  }
  if (road_borders_enabled && road_border_half_width <= 0.0F) {
    throw std::runtime_error("road-border-half-width must be positive");
  }
  if (road_borders_enabled && road_border_turn_half_width <= 0.0F) {
    throw std::runtime_error("road-border-turn-half-width must be positive");
  }
  if (road_borders_enabled && road_border_turn_half_width >= road_border_half_width) {
    throw std::runtime_error(
      "road-border-turn-half-width must be smaller than road-border-half-width (outer constriction)");
  }
  if (road_borders_enabled) {
    const float min_half = minRoadBorderHalfWidthForEgo(
      vehicle.ego_width, cost_params.road_border_collision_margin);
    const float min_turn_outer = min_half + 0.35F;
    if (road_border_half_width < min_half) {
      std::cerr << "WARNING: road-border-half-width " << road_border_half_width
                << " m is below ego clearance (~" << min_half
                << " m); raising to fit ego_width + collision margin\n";
      road_border_half_width = min_half;
    }
    if (road_border_turn_half_width < min_turn_outer) {
      std::cerr << "WARNING: road-border-turn-half-width " << road_border_turn_half_width
                << " m is too tight for ego on turn outer edge (~" << min_turn_outer
                << " m with tracking slack); raising\n";
      road_border_turn_half_width = min_turn_outer;
      if (road_border_turn_half_width >= road_border_half_width) {
        road_border_half_width = road_border_turn_half_width + 0.25F;
      }
    }
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
  float cross_track = y_offset;
  float heading_error = yaw_offset_rad;
  rows.push_back(formatRow(0.0F, plant, command, cross_track, heading_error));

  std::filesystem::create_directories(out_dir);
  const auto plant_csv = (std::filesystem::path(out_dir) / "plant.csv").string();
  const auto reference_csv = (std::filesystem::path(out_dir) / "reference.csv").string();
  const auto obstacles_csv = (std::filesystem::path(out_dir) / "obstacles.csv").string();
  const auto road_borders_csv = (std::filesystem::path(out_dir) / "road_borders.csv").string();
  const auto ego_footprint_csv = (std::filesystem::path(out_dir) / "ego_footprint.csv").string();

  const int steps = static_cast<int>(std::lround(duration_s / kDt));
  const float ref_ds = referenceSpacingMeters(speed);
  const float margin = static_cast<float>(kHorizon + steps) * ref_ds;
  const Trajectory fixed_reference = path_geom.discretizeReference(margin, ref_ds, speed);
  (void)fixed_reference;
  if (!writeReferenceCsv(reference_csv, path_geom)) {
    throw std::runtime_error("Failed to write " + reference_csv);
  }
  if (!writeEgoFootprintCsv(ego_footprint_csv, vehicle)) {
    throw std::runtime_error("Failed to write " + ego_footprint_csv);
  }
  std::cerr << "Left-then-right turn plant sim: steps=" << steps << " dt=" << kDt << " v=" << speed
            << " path_len=" << path_geom.totalLength() << " y0=" << y_offset
            << " R=" << turn_radius << " turn=" << turn_angle_deg << "deg"
            << " obstacles=" << (static_obstacle ? "on" : "off")
            << " road_borders=" << (road_borders_enabled ? "on" : "off")
            << " skip_if_invalid=" << (runtime.skip_if_invalid ? "on" : "off")
            << " t-MPT=" << (runtime.use_temporal_mpt_as_nominal ? "on" : "off")
            << " delay=" << (runtime.enable_input_delay_compensation ? "on" : "off") << "\n";
  if (runtime.use_temporal_mpt_as_nominal && runtime.enable_input_delay_compensation) {
    std::cerr << "NOTE: t-MPT OCP has no delay FIFOs; MPPI shifts its nominal by acc/steer delay "
                 "steps before seeding u_nom.\n";
  }

  autoware_perception_msgs::msg::TrackedObjects objects;
  if (static_obstacle) {
    objects = makeStationaryBoxOnPath(
      path_geom, obstacle_path_s, obstacle_lateral, obstacle_length, obstacle_width);
    if (!writeObstaclesCsv(obstacles_csv, objects)) {
      throw std::runtime_error("Failed to write " + obstacles_csv);
    }
    const auto & pose = objects.objects.front().kinematics.pose_with_covariance.pose;
    std::cerr << "  static obstacle: path_s=" << obstacle_path_s << " m center=(" << pose.position.x
              << "," << pose.position.y << ") size=" << obstacle_length << "x" << obstacle_width
              << " m\n";
    logTrackedObjectsForMppi(objects);
  }
  std::vector<Segment> road_borders;
  if (road_borders_enabled) {
    const auto layout =
      lrTurnPathLayout(straight_before, turn_radius, turn_angle_rad, straight_mid);
    road_borders = makeLrTurnRoadBorders(
      path_geom, layout, road_border_half_width, road_border_turn_half_width,
      kPathIntegrationStep);
    if (!writeRoadBordersCsv(road_borders_csv, road_borders)) {
      throw std::runtime_error("Failed to write " + road_borders_csv);
    }
    std::cerr << "  road borders: segments=" << road_borders.size() << " (cap "
              << kOpenLoopMaxRoadBorderSegments << ") straight_half=" << road_border_half_width
              << " m turn_outer_half=" << road_border_turn_half_width << " m\n";
  }
  autoware::mppi_optimizer::FirstOrderDubinsMppiKinematicLimits limits;
  const std::size_t reference_points = static_cast<std::size_t>(kHorizon);
  std::cerr << "  fixed reference: " << fixed_reference.points.size() << " points, ds=" << ref_ds
            << " m; horizon anchored at ego arc length each step\n";
  int rejected_steps = 0;
  for (int step = 0; step < steps; ++step) {
    const float sim_t = static_cast<float>(step) * kDt;
    float s_ego = 0.0F;
    float lat_unused = 0.0F;
    float heading_unused = 0.0F;
    path_geom.projectEgo(plant.x, plant.y, s_ego, lat_unused, heading_unused);
    const auto reference = path_geom.sliceHorizon(s_ego, ref_ds, reference_points, speed);
    if (!plant.accel_cmd_buffer.empty() || !plant.steer_cmd_buffer.empty()) {
      mppi.setInputDelayBuffer(plant.accel_cmd_buffer, plant.steer_cmd_buffer);
    }
    const auto result = mppi.optimizeTrajectory(
      reference, makeOdometry(plant, sim_t), makeAccel(plant), makeSteering(plant), objects,
      road_borders, {}, limits);
    updatePlantFromResult(plant, command, result);
    if (result.debug.was_rejected) {
      ++rejected_steps;
    }
    const float t = static_cast<float>(step + 1) * kDt;
    float cross_track_m = 0.0F;
    float heading_err_rad = 0.0F;
    path_geom.projectEgo(plant.x, plant.y, s_ego, cross_track_m, heading_err_rad);
    const auto ref_at_s = path_geom.sampleAtArcLength(s_ego);
    heading_err_rad = wrapPi(plant.yaw - ref_at_s.yaw);
    rows.push_back(formatRow(t, plant, command, cross_track_m, heading_err_rad));
    if ((step + 1) % 10 == 0 || step + 1 == steps) {
      const auto & pred = result.debug.prediction_accuracy;
      std::cerr << "  t=" << std::fixed << std::setprecision(2) << t << " x=" << plant.x
                << " y=" << plant.y << " yaw=" << plant.yaw << " v=" << plant.velocity
                << " u_a=" << command.accel_cmd << " u_d=" << command.steer_cmd
                << (result.debug.was_rejected ? " rejected=1" : "");
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
  std::cerr << "Summary: final x=" << plant.x << " y=" << plant.y << " v=" << plant.velocity
            << " rejected_steps=" << rejected_steps << "/" << steps << "\n";
  std::cerr << "Wrote " << plant_csv << "\n";
  std::cerr << "Wrote " << reference_csv << "\n";
  std::cerr << "Wrote " << ego_footprint_csv << "\n";
  if (static_obstacle) {
    std::cerr << "Wrote " << obstacles_csv << "\n";
  }
  if (road_borders_enabled) {
    std::cerr << "Wrote " << road_borders_csv << "\n";
  }

  std::string plot_title = "MPPI open-loop left/right turn";
  if (static_obstacle && road_borders_enabled) {
    plot_title = "MPPI open-loop left/right turn (obstacle + road borders)";
  } else if (static_obstacle) {
    plot_title = "MPPI open-loop left/right turn (static obstacle)";
  } else if (road_borders_enabled) {
    plot_title = "MPPI open-loop left/right turn (road borders)";
  }
  std::string obstacles_arg =
    static_obstacle ? (" --obstacles-csv \"" + obstacles_csv + "\"") : std::string();
  const std::string road_borders_arg =
    road_borders_enabled ? (" --road-borders-csv \"" + road_borders_csv + "\"") : std::string();
  const std::string ego_footprint_arg = " --ego-footprint-csv \"" + ego_footprint_csv + "\"";

  if (plot) {
    const auto script = findPlotScript(argv[0]);
    if (script.empty()) {
      std::cerr << "Plot script not found. Run:\n  ros2 run autoware_mppi_optimizer "
                   "mppi_open_loop_line_sim_plot.py -- --csv "
                << plant_csv << " --reference-csv " << reference_csv << obstacles_arg
                << road_borders_arg << ego_footprint_arg << "\n";
    } else {
      const auto png = (std::filesystem::path(out_dir) / "plant.png").string();
      const std::string cmd = "python3 \"" + script.string() + "\" --csv \"" + plant_csv +
                              "\" --reference-csv \"" + reference_csv + "\"" + obstacles_arg +
                              road_borders_arg + ego_footprint_arg + " --out \"" + png +
                              "\" --title \"" + plot_title + "\"";
      std::cerr << "Plotting: " << cmd << "\n";
      if (std::system(cmd.c_str()) != 0) {
        std::cerr << "WARNING: plotter exited non-zero\n";
      }
    }
  } else {
    std::cerr
      << "Plot with:\n  ros2 run autoware_mppi_optimizer mppi_open_loop_line_sim_plot.py -- "
         "--csv "
      << plant_csv << " --reference-csv " << reference_csv << obstacles_arg << road_borders_arg
      << ego_footprint_arg << "\n";
  }
  return 0;
}
