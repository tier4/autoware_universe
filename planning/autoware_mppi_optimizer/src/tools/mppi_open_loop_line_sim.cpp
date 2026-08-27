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
 * Each cycle: build a reference polyline from the ego projection onto y=0 through the
 * goal, resampled to kMppiHorizon points; optimizeTrajectory; advance plant from output.
 *
 * Example:
 *   ros2 run autoware_mppi_optimizer mppi_open_loop_line_sim -- \
 *     --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml \
 *     --out-dir "$HOME/.cache/autoware/mppi_open_loop_line_sim" --plot
 */

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"
#include "autoware/mppi_optimizer/mppi_debug_trajectory_io.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
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

struct SimPlantState
{
  float x{0.0F};
  float y{0.0F};
  float yaw{0.0F};
  float velocity{0.0F};
  float acceleration{0.0F};
  float steering{0.0F};
};

void printUsage(const char * argv0)
{
  std::cerr << "Usage: " << argv0
            << " [options]\n"
               "  --params-yaml FILE   Cost/runtime yaml (default: package share config)\n"
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
    {"speed_coeff", &cost.speed_coeff},
    {"track_coeff", &cost.track_coeff},
    {"track_terminal_scale", &cost.track_terminal_scale},
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
      }
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

/** Projection of ego onto the straight reference line y=0 (tangent +x). */
float projectedArcLengthOnLine(const SimPlantState & plant)
{
  return plant.x;
}

/**
 * Resample the straight segment [s_start, s_end] on y=0 to point_count points for MPPI.
 * points[0] is the projection; points.back() is the goal (or held if s_end <= s_start).
 */
Trajectory makeReferenceFromProjectionToGoal(
  const SimPlantState & plant, const float goal_x, const float speed,
  const std::size_t point_count)
{
  if (point_count < 2U) {
    throw std::runtime_error("MPPI reference requires at least 2 points");
  }

  const float s_start = projectedArcLengthOnLine(plant);
  const float s_end = std::max(goal_x, s_start + speed * kDt);

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points.reserve(point_count);

  const float length = std::max(s_end - s_start, 1.0e-3F);
  for (std::size_t i = 0; i < point_count; ++i) {
    const float alpha =
      static_cast<float>(i) / static_cast<float>(point_count - 1U);
    const float s = s_start + alpha * length;
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(s);
    point.pose.position.y = 0.0;
    point.pose.orientation = quaternionFromYaw(0.0);
    point.longitudinal_velocity_mps = speed;
    const float t = (s - s_start) / std::max(speed, 1.0e-3F);
    const int nanos = static_cast<int>(std::lround(static_cast<double>(t) * 1.0e9));
    point.time_from_start.sec = nanos / 1000000000;
    point.time_from_start.nanosec = static_cast<std::uint32_t>(nanos % 1000000000);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

Odometry makeOdometry(const SimPlantState & plant)
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.child_frame_id = "base_link";
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

bool writePlantCsv(
  const std::string & path, const std::vector<std::string> & rows)
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

void updatePlantFromResult(
  SimPlantState & plant, FirstOrderDubinsMppiControl & command,
  const FirstOrderDubinsMppiOptimizationResult & result,
  const FirstOrderDubinsMppiVehicleParams & vehicle)
{
  if (result.trajectory.points.empty()) {
    throw std::runtime_error("MPPI returned an empty optimized trajectory");
  }
  const auto & point = result.trajectory.points.front();
  const float prev_accel = plant.acceleration;
  const float prev_steer = plant.steering;
  plant.x = static_cast<float>(point.pose.position.x);
  plant.y = static_cast<float>(point.pose.position.y);
  plant.yaw = static_cast<float>(tf2::getYaw(point.pose.orientation));
  plant.velocity = point.longitudinal_velocity_mps;
  command.accel_cmd = point.acceleration_mps2;
  command.steer_cmd = point.front_wheel_angle_rad;
  const float alpha_a = kDt / std::max(vehicle.acc_time_constant, 1.0e-6F);
  const float alpha_s = kDt / std::max(vehicle.steer_time_constant, 1.0e-6F);
  plant.acceleration = prev_accel + alpha_a * (command.accel_cmd - prev_accel);
  plant.steering = prev_steer + alpha_s * (command.steer_cmd - prev_steer);
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
      yaw_offset_rad = std::stof(require_value("--yaw-offset-deg")) * static_cast<float>(M_PI) /
                       180.0F;
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
    goal_x = speed * duration_s;
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
  runtime.enable_input_delay_compensation = true;
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
  std::cerr << "Straight-line plant sim: steps=" << steps << " dt=" << kDt
            << " v=" << speed << " goal_x=" << goal_x << " y0=" << y_offset
            << " t-MPT=" << (runtime.use_temporal_mpt_as_nominal ? "on" : "off")
            << " delay=" << (runtime.enable_input_delay_compensation ? "on" : "off") << "\n";

  autoware_perception_msgs::msg::TrackedObjects objects;
  autoware::mppi_optimizer::FirstOrderDubinsMppiKinematicLimits limits;
  const std::size_t reference_points = static_cast<std::size_t>(kHorizon);
  std::cerr << "  reference points=" << reference_points
            << " (projection→goal resample, rebuilt each step)\n";
  for (int step = 0; step < steps; ++step) {
    const auto reference =
      makeReferenceFromProjectionToGoal(plant, goal_x, speed, reference_points);
    const auto result = mppi.optimizeTrajectory(
      reference, makeOdometry(plant), makeAccel(plant), makeSteering(plant), objects, {}, {},
      limits);
    if (step == 0) {
      autoware::mppi_optimizer::writeMppiDebugTrajectoryCsv(horizon_csv, result.trajectory);
    }
    updatePlantFromResult(plant, command, result, vehicle);
    const float t = static_cast<float>(step + 1) * kDt;
    rows.push_back(formatRow(t, plant, command));
    if ((step + 1) % 10 == 0 || step + 1 == steps) {
      std::cerr << "  t=" << std::fixed << std::setprecision(2) << t << " x=" << plant.x
                << " y=" << plant.y << " yaw=" << plant.yaw << " v=" << plant.velocity
                << " u_a=" << command.accel_cmd << " u_d=" << command.steer_cmd << "\n";
    }
  }

  if (!writePlantCsv(plant_csv, rows)) {
    throw std::runtime_error("Failed to write " + plant_csv);
  }
  std::cerr << "Wrote " << plant_csv << "\n";
  std::cerr << "Wrote first-horizon rollout " << horizon_csv << "\n";

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
    std::cerr << "Plot with:\n  ros2 run autoware_mppi_optimizer mppi_open_loop_line_sim_plot.py -- "
                 "--csv "
              << plant_csv << "\n";
  }
  return 0;
}
