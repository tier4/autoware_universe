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
 * Offline MPPI retune tool.
 *
 * Replays logged diffusion references from enable_debug_trajectory_log CSVs through
 * FirstOrderDubinsMppiInterface with optionally overridden cost params, and writes
 * retuned optimized trajectories for mppi_offline_tuner.py / mppi_debug_visualizer.py.
 *
 * Example:
 *   ros2 run autoware_mppi_optimizer mppi_offline_retune -- \
 *     --log-dir "$HOME/.cache/autoware/mppi_debug_log" \
 *     --out-dir "$HOME/.cache/autoware/mppi_retune" \
 *     --frame 12 --set track_coeff=2000 --set steer_rate_coeff=5000
 */

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"
#include "autoware/mppi_optimizer/mppi_debug_trajectory_io.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/utils.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

using autoware::mppi_optimizer::FirstOrderDubinsMppiCostParams;
using autoware::mppi_optimizer::FirstOrderDubinsMppiInterface;
using autoware::mppi_optimizer::FirstOrderDubinsMppiKinematicLimits;
using autoware::mppi_optimizer::FirstOrderDubinsMppiRuntimeOptions;
using autoware::mppi_optimizer::FirstOrderDubinsMppiVehicleParams;
using autoware::mppi_optimizer::formatMppiDebugFrameId;
using autoware::mppi_optimizer::listMppiDebugFrameIds;
using autoware::mppi_optimizer::loadMppiDebugControlHistoryCsv;
using autoware::mppi_optimizer::loadMppiDebugEgoCsv;
using autoware::mppi_optimizer::loadMppiDebugKeyValueCsv;
using autoware::mppi_optimizer::loadMppiDebugNominalCsv;
using autoware::mppi_optimizer::loadMppiDebugObjectsCsv;
using autoware::mppi_optimizer::loadMppiDebugRuntimeOptionsCsv;
using autoware::mppi_optimizer::loadMppiDebugSegmentsCsv;
using autoware::mppi_optimizer::loadMppiDebugTrajectoryCsv;
using autoware::mppi_optimizer::MppiDebugEgoState;
using autoware::mppi_optimizer::Segment;
using autoware::mppi_optimizer::writeMppiDebugCostsCsv;
using autoware::mppi_optimizer::writeMppiDebugNominalCsv;
using autoware::mppi_optimizer::writeMppiDebugRolloutsCsv;
using autoware::mppi_optimizer::writeMppiDebugTrajectoryCsv;
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

void printUsage(const char * argv0)
{
  std::cerr << "Usage: " << argv0
            << " --log-dir DIR --out-dir DIR [options]\n"
               "  --log-dir DIR          Input log directory (index.csv + *_reference.csv)\n"
               "  --out-dir DIR          Output directory for retuned CSVs\n"
               "  --frame N              Only retune frame N (default: all)\n"
               "  --params-yaml FILE     Optional ROS-style yaml with cost params\n"
               "  --set key=value        Override a cost param (repeatable)\n"
               "  --wheel-base M         Vehicle wheel base [m] (default 4.76)\n"
               "  --ego-length M         Ego length [m] (default 5.0)\n"
               "  --ego-width M          Ego width [m] (default 1.9)\n"
               "  --max-steer-angle RAD  Max steer [rad] (default 0.7)\n"
               "  --steer-tau S          Steer time constant [s] (default 0.27)\n"
               "  --copy-reference       Also copy reference CSVs into out-dir\n"
               "  --nominal-csv FILE     Force u_nom from this CSV (overrides log *_nominal.csv)\n"
               "\n"
               "Exact online parity: loads cost/vehicle/runtime params plus per-frame\n"
               "*_ego.csv, *_nominal.csv (REQUIRED warm-start u_nom), *_road_borders.csv,\n"
               "*_drivable.csv, *_objects.csv, *_control_history.csv, *_delay_buffer.csv.\n"
               "Also restores per-frame *_kinematic_limits.csv when present.\n"
               "CLI --set and vehicle flags override logged values.\n"
               "After each frame, writes <out-dir>/<tag>_seed_nominal.csv from optimized u_opt\n"
               "for iterative re-seed via --nominal-csv.\n"
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

bool parseKeyValue(const std::string & token, std::string & key, std::string & value)
{
  const auto eq = token.find('=');
  if (eq == std::string::npos) {
    return false;
  }
  key = trim(token.substr(0, eq));
  value = trim(token.substr(eq + 1));
  return !key.empty() && !value.empty();
}

void applyCostParam(
  FirstOrderDubinsMppiCostParams & params, const std::string & key, const float value)
{
  if (key == "lambda") {
    params.lambda = value;
  } else if (key == "speed_coeff") {
    params.speed_coeff = value;
  } else if (key == "track_coeff") {
    params.track_coeff = value;
  } else if (key == "track_terminal_scale") {
    params.track_terminal_scale = value;
  } else if (key == "heading_coeff") {
    params.heading_coeff = value;
  } else if (key == "lateral_distance_coeff") {
    params.lateral_distance_coeff = value;
  } else if (key == "lateral_yaw_error_coeff") {
    params.lateral_yaw_error_coeff = value;
  } else if (key == "remaining_distance_coeff") {
    params.remaining_distance_coeff = value;
  } else if (key == "path_overshoot_coeff") {
    params.path_overshoot_coeff = value;
  } else if (key == "track_center_coeff") {
    params.track_center_coeff = value;
  } else if (key == "corner_buffer_coeff") {
    params.corner_buffer_coeff = value;
  } else if (key == "corner_safe_margin") {
    params.corner_safe_margin = value;
  } else if (key == "boundary_threshold") {
    params.boundary_threshold = value;
  } else if (key == "lateral_boundary_soft_margin") {
    params.lateral_boundary_soft_margin = value;
  } else if (key == "accel_cmd_coeff") {
    params.accel_cmd_coeff = value;
  } else if (key == "steer_cmd_coeff") {
    params.steer_cmd_coeff = value;
  } else if (key == "steer_rate_coeff") {
    params.steer_rate_coeff = value;
  } else if (key == "overlimit_coeff") {
    params.overlimit_coeff = value;
  } else if (key == "accel_cmd_std_dev") {
    params.accel_cmd_std_dev = value;
  } else if (key == "steer_cmd_std_dev") {
    params.steer_cmd_std_dev = value;
  } else if (key == "accel_cmd_noise_exponent") {
    params.accel_cmd_noise_exponent = value;
  } else if (key == "steer_cmd_noise_exponent") {
    params.steer_cmd_noise_exponent = value;
  } else if (key == "nominal_curvature_min_chord_length_m") {
    params.nominal_curvature_min_chord_length_m = value;
  } else if (key == "lateral_acceleration_coeff") {
    params.lateral_acceleration_coeff = value;
  } else if (key == "lateral_jerk_coeff") {
    params.lateral_jerk_coeff = value;
  } else if (key == "longitudinal_jerk_coeff") {
    params.longitudinal_jerk_coeff = value;
  } else if (key == "obstacle_collision_margin") {
    params.obstacle_collision_margin = value;
  } else if (key == "road_border_collision_margin") {
    params.road_border_collision_margin = value;
  } else if (key == "obstacle_safe_margin") {
    params.obstacle_safe_margin = value;
  } else if (key == "road_border_safe_margin") {
    params.road_border_safe_margin = value;
  } else if (key == "drivable_area_safe_margin") {
    params.drivable_area_safe_margin = value;
  } else if (key == "drivable_area_barrier_weight") {
    params.drivable_area_barrier_weight = value;
  } else if (key == "crash_contact_penalty") {
    params.crash_contact_penalty = value;
  } else {
    // Unknown keys must not abort retune: the visualizer may send a superset of
    // slider names / logged fields that older or newer builds don't share.
    std::cerr << "WARNING: ignoring unknown cost param '" << key << "'=" << value << "\n";
  }
}

void loadParamsYaml(const std::string & path, FirstOrderDubinsMppiCostParams & params)
{
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error("Failed to open params yaml: " + path);
  }
  std::string line;
  while (std::getline(in, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#' || line.find(':') == std::string::npos) {
      continue;
    }
    const auto colon = line.find(':');
    std::string key = trim(line.substr(0, colon));
    std::string value = trim(line.substr(colon + 1));
    if (!value.empty() && value.front() == '"' && value.back() == '"') {
      value = value.substr(1, value.size() - 2);
    }
    // Skip non-numeric / bool flags.
    if (
      key == "enable_debug_trajectory_log" || key == "debug_trajectory_log_directory" ||
      key == "ros__parameters" || key == "/**") {
      continue;
    }
    try {
      applyCostParam(params, key, std::stof(value));
    } catch (const std::exception &) {
      // Ignore unknown / non-float keys in the yaml.
    }
  }
}

void applyVehicleParam(
  FirstOrderDubinsMppiVehicleParams & params, const std::string & key, const float value)
{
  if (key == "ego_length") {
    params.ego_length = value;
  } else if (key == "ego_width") {
    params.ego_width = value;
  } else if (key == "ego_axle_to_box_center") {
    params.ego_axle_to_box_center = value;
  } else if (key == "wheel_base") {
    params.wheel_base = value;
  } else if (key == "max_steer_angle") {
    params.max_steer_angle = value;
  } else if (key == "acc_time_constant") {
    params.acc_time_constant = value;
  } else if (key == "steer_time_constant") {
    params.steer_time_constant = value;
  } else if (key == "steer_rate_lim") {
    params.steer_rate_lim = value;
  } else if (key == "vel_rate_lim") {
    params.vel_rate_lim = value;
  } else if (key == "acc_time_delay") {
    params.acc_time_delay = value;
  } else if (key == "steer_time_delay") {
    params.steer_time_delay = value;
  } else {
    throw std::runtime_error("Unknown vehicle param: " + key);
  }
}

void loadCostParamsFromLog(const std::string & log_dir, FirstOrderDubinsMppiCostParams & params)
{
  std::unordered_map<std::string, float> kv;
  if (!loadMppiDebugKeyValueCsv(log_dir + "/cost_params.csv", kv)) {
    return;
  }
  for (const auto & [key, value] : kv) {
    try {
      applyCostParam(params, key, value);
    } catch (const std::exception &) {
      // Ignore unknown keys.
    }
  }
  std::cerr << "Loaded cost params from " << log_dir << "/cost_params.csv\n";
}

void loadVehicleParamsFromLog(
  const std::string & log_dir, FirstOrderDubinsMppiVehicleParams & params)
{
  std::unordered_map<std::string, float> kv;
  if (!loadMppiDebugKeyValueCsv(log_dir + "/vehicle_params.csv", kv)) {
    return;
  }
  for (const auto & [key, value] : kv) {
    try {
      applyVehicleParam(params, key, value);
    } catch (const std::exception &) {
      // Ignore unknown keys.
    }
  }
  std::cerr << "Loaded vehicle params from " << log_dir << "/vehicle_params.csv\n";
}

FirstOrderDubinsMppiRuntimeOptions loadRuntimeOptionsFromLog(const std::string & log_dir)
{
  // Defaults match config/mppi_optimizer.param.yaml online stack.
  FirstOrderDubinsMppiRuntimeOptions options;
  options.ignore_obstacles = false;
  options.ignore_road_borders = false;
  options.ignore_drivable_area = true;
  options.force_cold_start_each_step = false;
  options.skip_if_invalid = true;
  options.use_last_control_as_nominal = true;
  options.prevent_reverse_velocity = true;
  options.enable_input_delay_compensation = true;
  if (loadMppiDebugRuntimeOptionsCsv(log_dir + "/runtime_options.csv", options)) {
    std::cerr << "Loaded runtime options from " << log_dir << "/runtime_options.csv\n";
  } else {
    std::cerr << "WARNING: missing " << log_dir
              << "/runtime_options.csv; using online yaml defaults "
                 "(ignore_drivable_area=true, skip_if_invalid=true, "
                 "use_last_control_as_nominal=true).\n";
  }
  // Offline retune must not write into the input log directory.
  options.enable_debug_trajectory_log = false;
  return options;
}

Odometry odometryFromEgo(const MppiDebugEgoState & ego, const Trajectory & reference)
{
  Odometry odom;
  odom.header = reference.header;
  if (odom.header.frame_id.empty()) {
    odom.header.frame_id = "map";
  }
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = ego.x;
  odom.pose.pose.position.y = ego.y;
  odom.pose.pose.position.z = ego.z;
  odom.pose.pose.orientation = autoware::mppi_optimizer::quaternionFromYaw(ego.yaw);
  odom.twist.twist.linear.x = ego.v;
  return odom;
}

geometry_msgs::msg::AccelWithCovarianceStamped accelFromEgo(
  const MppiDebugEgoState & ego, const Trajectory & reference)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header = reference.header;
  accel.accel.accel.linear.x = ego.accel;
  return accel;
}

autoware_vehicle_msgs::msg::SteeringReport steeringFromEgo(
  const MppiDebugEgoState & ego, const Trajectory & reference)
{
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = reference.header.stamp;
  steering.steering_tire_angle = static_cast<float>(ego.steer);
  return steering;
}

Odometry odometryFromReference(const Trajectory & reference)
{
  Odometry odom;
  odom.header = reference.header;
  if (reference.header.frame_id.empty()) {
    odom.header.frame_id = "map";
  }
  odom.child_frame_id = "base_link";
  const auto & p0 = reference.points.front();
  odom.pose.pose = p0.pose;
  odom.twist.twist.linear.x = p0.longitudinal_velocity_mps;
  return odom;
}

geometry_msgs::msg::AccelWithCovarianceStamped accelFromReference(const Trajectory & reference)
{
  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header = reference.header;
  accel.accel.accel.linear.x = reference.points.front().acceleration_mps2;
  return accel;
}

autoware_vehicle_msgs::msg::SteeringReport steeringFromReference(const Trajectory & reference)
{
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = reference.header.stamp;
  steering.steering_tire_angle = reference.points.front().front_wheel_angle_rad;
  return steering;
}

FirstOrderDubinsMppiKinematicLimits loadKinematicLimits(const std::string & path, bool & file_found)
{
  std::unordered_map<std::string, float> values;
  file_found = loadMppiDebugKeyValueCsv(path, values);
  FirstOrderDubinsMppiKinematicLimits result;
  if (!file_found || values["active"] == 0.0F) {
    return result;
  }
  const auto set_if_present = [&](const char * key, std::optional<float> & destination) {
    const auto it = values.find(key);
    if (it != values.end()) {
      destination = it->second;
    }
  };
  set_if_present("max_velocity", result.max_velocity);
  set_if_present("min_longitudinal_acceleration", result.min_longitudinal_acceleration);
  set_if_present("max_longitudinal_acceleration", result.max_longitudinal_acceleration);
  set_if_present("min_longitudinal_jerk", result.min_longitudinal_jerk);
  set_if_present("max_longitudinal_jerk", result.max_longitudinal_jerk);
  return result;
}

}  // namespace

int run(int argc, char ** argv);

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::cerr << "mppi_offline_retune failed: " << e.what() << "\n";
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

  std::string log_dir;
  std::string out_dir;
  std::string params_yaml;
  std::string nominal_csv_override;
  std::optional<uint64_t> frame_filter;
  bool copy_reference = false;
  bool reseed_nominal_from_reference = false;
  FirstOrderDubinsMppiCostParams cost_params;
  FirstOrderDubinsMppiVehicleParams vehicle_params;
  // Defaults closer to j6_gen2 for Autoware replay.
  vehicle_params.wheel_base = 4.76F;
  vehicle_params.ego_length = 5.0F;
  vehicle_params.ego_width = 1.9F;
  vehicle_params.ego_axle_to_box_center = 1.5F;
  vehicle_params.max_steer_angle = 0.7F;
  vehicle_params.steer_time_constant = 0.27F;
  vehicle_params.acc_time_constant = 0.1F;
  vehicle_params.vel_rate_lim = 7.0F;
  vehicle_params.steer_rate_lim = 5.0F;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char * name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
    if (arg == "--log-dir") {
      log_dir = need("--log-dir");
    } else if (arg == "--out-dir") {
      out_dir = need("--out-dir");
    } else if (arg == "--frame") {
      frame_filter = std::stoull(need("--frame"));
    } else if (arg == "--params-yaml") {
      params_yaml = need("--params-yaml");
    } else if (arg == "--set") {
      std::string key;
      std::string value;
      if (!parseKeyValue(need("--set"), key, value)) {
        throw std::runtime_error("Expected --set key=value");
      }
      applyCostParam(cost_params, key, std::stof(value));
    } else if (arg == "--wheel-base") {
      vehicle_params.wheel_base = std::stof(need("--wheel-base"));
    } else if (arg == "--ego-length") {
      vehicle_params.ego_length = std::stof(need("--ego-length"));
    } else if (arg == "--ego-width") {
      vehicle_params.ego_width = std::stof(need("--ego-width"));
    } else if (arg == "--max-steer-angle") {
      vehicle_params.max_steer_angle = std::stof(need("--max-steer-angle"));
    } else if (arg == "--steer-tau") {
      vehicle_params.steer_time_constant = std::stof(need("--steer-tau"));
    } else if (arg == "--copy-reference") {
      copy_reference = true;
    } else if (arg == "--nominal-csv") {
      nominal_csv_override = need("--nominal-csv");
    } else if (arg == "--reseed-nominal-from-reference") {
      reseed_nominal_from_reference = true;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      printUsage(argv[0]);
      return 1;
    }
  }

  if (log_dir.empty() || out_dir.empty()) {
    printUsage(argv[0]);
    return 1;
  }

  // Prefer params captured at log time; then yaml; then CLI --set / vehicle flags.
  loadCostParamsFromLog(log_dir, cost_params);
  loadVehicleParamsFromLog(log_dir, vehicle_params);
  const FirstOrderDubinsMppiRuntimeOptions runtime_options = loadRuntimeOptionsFromLog(log_dir);

  if (!params_yaml.empty()) {
    loadParamsYaml(params_yaml, cost_params);
  }
  // Re-apply --set / vehicle CLI so they win over log + yaml.
  for (int i = 1; i + 1 < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--set") {
      std::string key;
      std::string value;
      if (parseKeyValue(argv[i + 1], key, value)) {
        applyCostParam(cost_params, key, std::stof(value));
      }
    } else if (arg == "--wheel-base") {
      vehicle_params.wheel_base = std::stof(argv[i + 1]);
    } else if (arg == "--ego-length") {
      vehicle_params.ego_length = std::stof(argv[i + 1]);
    } else if (arg == "--ego-width") {
      vehicle_params.ego_width = std::stof(argv[i + 1]);
    } else if (arg == "--max-steer-angle") {
      vehicle_params.max_steer_angle = std::stof(argv[i + 1]);
    } else if (arg == "--steer-tau") {
      vehicle_params.steer_time_constant = std::stof(argv[i + 1]);
    }
  }

  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::cerr << "Failed to create out-dir: " << ec.message() << "\n";
    return 1;
  }

  std::cout << "applied_params lambda=" << cost_params.lambda
            << " track_coeff=" << cost_params.track_coeff
            << " speed_coeff=" << cost_params.speed_coeff
            << " overlimit_coeff=" << cost_params.overlimit_coeff
            << " heading_coeff=" << cost_params.heading_coeff
            << " steer_rate_coeff=" << cost_params.steer_rate_coeff << "\n";
  if (cost_params.lambda >= 5000.0F) {
    std::cerr
      << "WARNING: lambda=" << cost_params.lambda
      << " is very high — softmax weights stay near-uniform and cost-weight edits "
         "will barely move the trajectory. Try lambda around 100–1500 to see retune effects.\n";
  }

  const auto frame_ids = listMppiDebugFrameIds(log_dir);
  if (frame_ids.empty()) {
    std::cerr << "No frames found in " << log_dir << "\n";
    return 1;
  }

  std::ofstream index_out(out_dir + "/index.csv");
  index_out << "frame_id,stamp_sec,stamp_nsec,baseline_cost,n_reference,n_optimized,crash_status\n";

  size_t processed = 0;
  for (const uint64_t frame_id : frame_ids) {
    if (frame_filter && *frame_filter != frame_id) {
      continue;
    }
    const std::string tag = formatMppiDebugFrameId(frame_id);
    const std::string ref_path = log_dir + "/" + tag + "_reference.csv";
    Trajectory reference;
    if (!loadMppiDebugTrajectoryCsv(ref_path, reference)) {
      std::cerr << "Failed to load " << ref_path << "\n";
      return 1;
    }
    reference.header.frame_id = "map";

    const std::string ego_path = log_dir + "/" + tag + "_ego.csv";
    MppiDebugEgoState ego;
    Odometry odom;
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    autoware_vehicle_msgs::msg::SteeringReport steering;
    if (loadMppiDebugEgoCsv(ego_path, ego)) {
      odom = odometryFromEgo(ego, reference);
      accel = accelFromEgo(ego, reference);
      steering = steeringFromEgo(ego, reference);
    } else {
      static bool warned_missing_ego = false;
      if (!warned_missing_ego) {
        std::cerr << "WARNING: missing " << tag
                  << "_ego.csv (and possibly others). Falling back to reference[0] as ego IC.\n"
                     "Re-log with a build that writes *_ego.csv for faithful replay.\n";
        warned_missing_ego = true;
      }
      odom = odometryFromReference(reference);
      accel = accelFromReference(reference);
      steering = steeringFromReference(reference);
    }

    // Fresh controller each frame; warm-start u_nom comes from *_nominal.csv (same as online).
    FirstOrderDubinsMppiInterface frame_mppi;
    frame_mppi.setCostParams(cost_params);
    frame_mppi.setVehicleParams(vehicle_params);
    frame_mppi.setRuntimeOptions(runtime_options);
    frame_mppi.setDebugTrajectoryLogging(false);
    frame_mppi.setRolloutVisualizationEnabled(true);

    const std::string nominal_path =
      !nominal_csv_override.empty() ? nominal_csv_override : (log_dir + "/" + tag + "_nominal.csv");
    std::vector<float> nominal_accel;
    std::vector<float> nominal_steer;

    float hist_a0 = 0.0F;
    float hist_s0 = 0.0F;
    float hist_a1 = 0.0F;
    float hist_s1 = 0.0F;
    if (loadMppiDebugControlHistoryCsv(
          log_dir + "/" + tag + "_control_history.csv", hist_a0, hist_s0, hist_a1, hist_s1)) {
      frame_mppi.setControlHistory(hist_a0, hist_s0, hist_a1, hist_s1);
    } else {
      static bool warned_missing_history = false;
      if (!warned_missing_history) {
        std::cerr << "WARNING: missing *_control_history.csv; Savitzky–Golay edge taps are zero "
                     "(re-log for exact online match).\n";
        warned_missing_history = true;
      }
    }
    const bool force_nominal = !nominal_csv_override.empty() || !reseed_nominal_from_reference;
    if (force_nominal) {
      if (!loadMppiDebugNominalCsv(nominal_path, nominal_accel, nominal_steer)) {
        std::cerr << "ERROR: missing warm-start nominal control " << nominal_path
                  << "\n*_nominal.csv is the online u_nom warm-start. Re-log with a build that "
                     "writes it, or pass --nominal-csv from a prior retune *_seed_nominal.csv.\n";
        return 1;
      }
      frame_mppi.setForcedNominalControl(nominal_accel, nominal_steer);
      if (!nominal_csv_override.empty()) {
        std::cout << "frame " << frame_id << " seeding u_nom from " << nominal_path << "\n";
      }
    } else {
      std::cout << "frame " << frame_id
                << " reseeding u_nom from reference with nominal curvature chord "
                << cost_params.nominal_curvature_min_chord_length_m << " m\n";
    }

    {
      const std::string delay_path = log_dir + "/" + tag + "_delay_buffer.csv";
      std::vector<float> delay_accel;
      std::vector<float> delay_steer;
      if (std::filesystem::exists(delay_path)) {
        if (loadMppiDebugNominalCsv(delay_path, delay_accel, delay_steer) && !delay_accel.empty()) {
          frame_mppi.setInputDelayBuffer(delay_accel, delay_steer);
        }
      } else {
        static bool warned_missing_delay = false;
        if (!warned_missing_delay) {
          std::cerr << "WARNING: missing *_delay_buffer.csv; delay FIFO seeds from measured "
                       "accel/steer (re-log for exact online match).\n";
          warned_missing_delay = true;
        }
      }
    }

    std::vector<Segment> road_borders;
    std::vector<Segment> drivable_area;
    if (!loadMppiDebugSegmentsCsv(log_dir + "/" + tag + "_road_borders.csv", road_borders)) {
      static bool warned_missing_borders = false;
      if (!warned_missing_borders) {
        std::cerr << "WARNING: missing *_road_borders.csv; using empty borders "
                     "(re-log for exact online match).\n";
        warned_missing_borders = true;
      }
    }
    if (!loadMppiDebugSegmentsCsv(log_dir + "/" + tag + "_drivable.csv", drivable_area)) {
      static bool warned_missing_drivable = false;
      if (!warned_missing_drivable) {
        std::cerr << "WARNING: missing *_drivable.csv; using empty drivable segments "
                     "(re-log for exact online match).\n";
        warned_missing_drivable = true;
      }
    }

    autoware_perception_msgs::msg::TrackedObjects tracked_objects;
    if (!loadMppiDebugObjectsCsv(log_dir + "/" + tag + "_objects.csv", tracked_objects)) {
      static bool warned_missing_objects = false;
      if (!warned_missing_objects) {
        std::cerr << "WARNING: missing *_objects.csv; using empty obstacles "
                     "(re-log for exact online match).\n";
        warned_missing_objects = true;
      }
    }

    bool kinematic_limits_file_found = false;
    const auto kinematic_limits = loadKinematicLimits(
      log_dir + "/" + tag + "_kinematic_limits.csv", kinematic_limits_file_found);
    if (!kinematic_limits_file_found) {
      static bool warned_missing_kinematic_limits = false;
      if (!warned_missing_kinematic_limits) {
        std::cerr << "WARNING: missing *_kinematic_limits.csv; optional kinematic barrier is "
                     "inactive (re-log for exact online match).\n";
        warned_missing_kinematic_limits = true;
      }
    }

    const auto result = frame_mppi.optimizeTrajectory(
      reference, odom, accel, steering, tracked_objects, road_borders, drivable_area,
      kinematic_limits);

    const std::string opt_path = out_dir + "/" + tag + "_optimized.csv";
    if (!writeMppiDebugTrajectoryCsv(opt_path, result.debug.optimized_trajectory)) {
      std::cerr << "Failed to write " << opt_path << "\n";
      return 1;
    }

    {
      std::vector<float> seed_accel;
      std::vector<float> seed_steer;
      if (frame_mppi.copyLastOptimizedControl(seed_accel, seed_steer)) {
        const std::string seed_path = out_dir + "/" + tag + "_seed_nominal.csv";
        if (!writeMppiDebugNominalCsv(seed_path, seed_accel, seed_steer)) {
          std::cerr << "WARNING: failed to write " << seed_path << "\n";
        }
      } else {
        std::cerr << "WARNING: could not copy optimized controls for frame " << frame_id << "\n";
      }
    }

    // Stride keeps the CSV / histogram manageable (~4k samples from 32k rollouts).
    std::vector<float> raw_costs;
    std::vector<float> normalized_weights;
    constexpr int kCostVizStride = 8;
    if (frame_mppi.copySampleCostDistribution(raw_costs, normalized_weights, kCostVizStride)) {
      const std::string costs_path = out_dir + "/" + tag + "_costs.csv";
      if (!writeMppiDebugCostsCsv(costs_path, raw_costs, normalized_weights)) {
        std::cerr << "Failed to write " << costs_path << "\n";
        return 1;
      }
    } else {
      std::cerr << "WARNING: could not copy sample cost distribution for frame " << frame_id
                << "\n";
    }

    const std::string rollouts_path = out_dir + "/" + tag + "_rollouts.csv";
    if (!writeMppiDebugRolloutsCsv(rollouts_path, result.debug.rollouts)) {
      std::cerr << "Failed to write " << rollouts_path << "\n";
      return 1;
    }

    if (copy_reference) {
      const std::string out_ref = out_dir + "/" + tag + "_reference.csv";
      if (!writeMppiDebugTrajectoryCsv(out_ref, reference)) {
        std::cerr << "Failed to write " << out_ref << "\n";
        return 1;
      }
    }

    {
      const std::string crash_path = out_dir + "/" + tag + "_crash_status.csv";
      std::ofstream crash_out(crash_path);
      if (crash_out) {
        crash_out << "crash_status,label\n";
        crash_out << static_cast<int>(result.debug.validation.reasons) << ","
                  << to_string(result.debug.validation.reasons) << "\n";
      }
    }

    {
      const auto & breakdown = result.debug.cost_breakdown;
      const std::string breakdown_path = out_dir + "/" + tag + "_cost_breakdown.csv";
      std::ofstream breakdown_out(breakdown_path);
      if (!breakdown_out) {
        std::cerr << "Failed to write " << breakdown_path << "\n";
        return 1;
      }
      breakdown_out << "key,value\n";
      breakdown_out << "controller_baseline_cost," << result.debug.baseline_cost << "\n";
      breakdown_out << "output_total_cost," << breakdown.total << "\n";
      breakdown_out << "output_minus_baseline_cost," << breakdown.total - result.debug.baseline_cost
                    << "\n";
      breakdown_out << "running_total," << breakdown.running_total << "\n";
      breakdown_out << "terminal_total," << breakdown.terminal_total << "\n";
      breakdown_out << "evaluated_timesteps," << breakdown.evaluated_timesteps << "\n";
      breakdown_out << "state/speed," << breakdown.speed << "\n";
      breakdown_out << "state/track," << breakdown.track << "\n";
      breakdown_out << "state/heading," << breakdown.heading << "\n";
      breakdown_out << "state/lateral_distance," << breakdown.lateral_distance << "\n";
      breakdown_out << "state/lateral_boundary," << breakdown.lateral_boundary << "\n";
      breakdown_out << "state/lateral_yaw_error," << breakdown.lateral_yaw_error << "\n";
      breakdown_out << "state/remaining_distance," << breakdown.remaining_distance << "\n";
      breakdown_out << "state/path_overshoot," << breakdown.path_overshoot << "\n";
      breakdown_out << "state/track_center," << breakdown.track_center << "\n";
      breakdown_out << "state/corner_buffer," << breakdown.corner_buffer << "\n";
      breakdown_out << "state/drivable_area," << breakdown.drivable_area << "\n";
      breakdown_out << "state/obstacle," << breakdown.obstacle << "\n";
      breakdown_out << "state/road_border," << breakdown.road_border << "\n";
      breakdown_out << "control/acceleration_command," << breakdown.acceleration_command << "\n";
      breakdown_out << "control/steering_command," << breakdown.steering_command << "\n";
      breakdown_out << "comfort/lateral_acceleration," << breakdown.lateral_acceleration << "\n";
      breakdown_out << "comfort/lateral_jerk," << breakdown.lateral_jerk << "\n";
      breakdown_out << "comfort/longitudinal_jerk," << breakdown.longitudinal_jerk << "\n";
      breakdown_out << "comfort/steering_rate," << breakdown.steering_rate << "\n";
      breakdown_out << "kinematic/velocity_overlimit," << breakdown.kinematic_velocity_overlimit
                    << "\n";
      breakdown_out << "kinematic/acceleration_overlimit,"
                    << breakdown.kinematic_acceleration_overlimit << "\n";
      breakdown_out << "kinematic/jerk_overlimit," << breakdown.kinematic_jerk_overlimit << "\n";
    }

    index_out << frame_id << "," << reference.header.stamp.sec << ","
              << reference.header.stamp.nanosec << "," << result.debug.baseline_cost << ","
              << reference.points.size() << "," << result.debug.optimized_trajectory.points.size()
              << "," << static_cast<int>(result.debug.validation.reasons) << "\n";
    ++processed;
    std::cout << "frame " << frame_id << " baseline_cost=" << result.debug.baseline_cost
              << " crash_status=" << to_string(result.debug.validation.reasons)
              << " (min rollout cost among all samples)"
              << " points=" << result.debug.optimized_trajectory.points.size()
              << " rollouts=" << result.debug.rollouts.size() << "\n";
    {
      bool any_best = false;
      float viz_min = 0.0F;
      for (const auto & rollout : result.debug.rollouts) {
        if (rollout.is_worst) {
          continue;
        }
        if (!any_best || rollout.cost < viz_min) {
          viz_min = rollout.cost;
          any_best = true;
        }
      }
      if (any_best) {
        std::cout << "frame " << frame_id << " min_exported_rollout_cost=" << viz_min << "\n";
      }
    }
  }

  if (processed == 0U) {
    std::cerr << "No frames processed (check --frame filter).\n";
    return 1;
  }
  std::cout << "Wrote " << processed << " retuned frame(s) to " << out_dir << "\n";
  return 0;
}
