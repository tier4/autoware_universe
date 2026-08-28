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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_LOGGER_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_LOGGER_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_runtime_options.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params.hpp"

#include <rclcpp/logging.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::mppi_optimizer
{

/** Prefer $XDG_CACHE_HOME, else $HOME/.cache (user-private, not world-writable). */
inline std::string default_mppi_debug_log_directory()
{
  if (const char * xdg = std::getenv("XDG_CACHE_HOME"); xdg != nullptr && xdg[0] != '\0') {
    return std::string(xdg) + "/autoware/mppi_debug_log";
  }
  if (const char * home = std::getenv("HOME"); home != nullptr && home[0] != '\0') {
    return std::string(home) + "/.cache/autoware/mppi_debug_log";
  }
  return {};
}

/** Ego state used by MPPI at the start of a cycle (for offline replay). */
struct MppiDebugEgoState
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double v{0.0};
  double accel{0.0};
  double steer{0.0};
};

/**
 * Optional CSV logger for reference / optimized trajectories so
 * mppi_debug_visualizer.py can replay offline via --log-dir.
 *
 * Layout:
 *   <log_dir>/index.csv
 *   <log_dir>/vehicle_params.csv   (once)
 *   <log_dir>/cost_params.csv      (once)
 *   <log_dir>/runtime_options.csv  (once)
 *   <log_dir>/000000_reference.csv
 *   <log_dir>/000000_optimized.csv
 *   <log_dir>/000000_ego.csv
 *   <log_dir>/000000_nominal.csv          (u_nom warm-start accel/steer cmds)
 *   <log_dir>/000000_nominal_traj.csv     (open-loop XY of u_nom = RViz mppi_nominal)
 *   <log_dir>/000000_road_borders.csv
 *   <log_dir>/000000_drivable.csv
 *   <log_dir>/000000_objects.csv
 *   <log_dir>/000000_control_history.csv  (legacy vendor history at cycle start)
 *   <log_dir>/000000_delay_buffer.csv     (per-channel FIFOs before optimize)
 *   <log_dir>/000000_applied.csv          (u[0] applied this cycle)
 *   ...
 *
 * Trajectory CSV columns:
 *   t_from_start_s,x,y,z,yaw,v,a,steer,steer_rate
 * Ego CSV columns:
 *   x,y,z,yaw,v,accel,steer
 * Nominal CSV columns:
 *   t_idx,accel_cmd,steer_cmd
 */
class MppiDebugTrajectoryLogger
{
public:
  void configure(const bool enable, const std::string & directory)
  {
    enabled_ = enable;
    directory_ = directory;
    frame_id_ = 0;
    index_initialized_ = false;
    params_written_ = false;
    runtime_written_ = false;
    if (!enabled_) {
      return;
    }
    if (directory_.empty()) {
      directory_ = default_mppi_debug_log_directory();
    }
    if (directory_.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("mppi_debug_trajectory_logger"),
        "Debug trajectory logging enabled but no writable cache dir "
        "(set debug_trajectory_log_directory or HOME/XDG_CACHE_HOME); disabling.");
      enabled_ = false;
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    if (ec) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"),
        "Failed to create log directory '%s': %s", directory_.c_str(), ec.message().c_str());
      enabled_ = false;
      return;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("mppi_debug_trajectory_logger"),
      "MPPI debug trajectory logging enabled -> %s", directory_.c_str());
  }

  bool enabled() const { return enabled_; }

  void writeParamsOnce(
    const FirstOrderDubinsMppiCostParams & cost, const FirstOrderDubinsMppiVehicleParams & vehicle)
  {
    if (!enabled_ || params_written_) {
      return;
    }
    {
      std::ofstream out(directory_ + "/cost_params.csv");
      if (out) {
        out << "key,value\n";
        out << std::setprecision(9) << std::fixed;
        out << "lambda," << cost.lambda << "\n";
        out << "speed_coeff," << cost.speed_coeff << "\n";
        out << "track_coeff," << cost.track_coeff << "\n";
        out << "track_terminal_scale," << cost.track_terminal_scale << "\n";
        out << "heading_coeff," << cost.heading_coeff << "\n";
        out << "lateral_distance_coeff," << cost.lateral_distance_coeff << "\n";
        out << "lateral_yaw_error_coeff," << cost.lateral_yaw_error_coeff << "\n";
        out << "remaining_distance_coeff," << cost.remaining_distance_coeff << "\n";
        out << "path_overshoot_coeff," << cost.path_overshoot_coeff << "\n";
        out << "track_center_coeff," << cost.track_center_coeff << "\n";
        out << "corner_buffer_coeff," << cost.corner_buffer_coeff << "\n";
        out << "corner_safe_margin," << cost.corner_safe_margin << "\n";
        out << "boundary_threshold," << cost.boundary_threshold << "\n";
        out << "lateral_boundary_soft_margin," << cost.lateral_boundary_soft_margin << "\n";
        out << "accel_cmd_coeff," << cost.accel_cmd_coeff << "\n";
        out << "steer_cmd_coeff," << cost.steer_cmd_coeff << "\n";
        out << "steer_rate_coeff," << cost.steer_rate_coeff << "\n";
        out << "overlimit_coeff," << cost.overlimit_coeff << "\n";
        out << "accel_cmd_std_dev," << cost.accel_cmd_std_dev << "\n";
        out << "steer_cmd_std_dev," << cost.steer_cmd_std_dev << "\n";
        out << "accel_cmd_noise_exponent," << cost.accel_cmd_noise_exponent << "\n";
        out << "steer_cmd_noise_exponent," << cost.steer_cmd_noise_exponent << "\n";
        out << "nominal_curvature_min_chord_length_m," << cost.nominal_curvature_min_chord_length_m
            << "\n";
        out << "lateral_acceleration_coeff," << cost.lateral_acceleration_coeff << "\n";
        out << "lateral_jerk_coeff," << cost.lateral_jerk_coeff << "\n";
        out << "longitudinal_jerk_coeff," << cost.longitudinal_jerk_coeff << "\n";
        out << "obstacle_collision_margin," << cost.obstacle_collision_margin << "\n";
        out << "road_border_collision_margin," << cost.road_border_collision_margin << "\n";
        out << "obstacle_safe_margin," << cost.obstacle_safe_margin << "\n";
        out << "road_border_safe_margin," << cost.road_border_safe_margin << "\n";
        out << "drivable_area_safe_margin," << cost.drivable_area_safe_margin << "\n";
        out << "drivable_area_barrier_weight," << cost.drivable_area_barrier_weight << "\n";
        out << "crash_contact_penalty," << cost.crash_contact_penalty << "\n";
      }
    }
    {
      std::ofstream out(directory_ + "/vehicle_params.csv");
      if (out) {
        out << "key,value\n";
        out << std::setprecision(9) << std::fixed;
        out << "ego_length," << vehicle.ego_length << "\n";
        out << "ego_width," << vehicle.ego_width << "\n";
        out << "ego_axle_to_box_center," << vehicle.ego_axle_to_box_center << "\n";
        out << "wheel_base," << vehicle.wheel_base << "\n";
        // First-order Dubins L=lf+lr and lags used by TemporalMptNominalSeeder.
        {
          const float wb = std::max(vehicle.wheel_base, 1.0e-3F);
          float lr = vehicle.ego_axle_to_box_center;
          lr = std::min(std::max(lr, 1.0e-3F), wb - 1.0e-3F);
          const float lf = wb - lr;
          out << "lf," << lf << "\n";
          out << "lr," << lr << "\n";
        }
        out << "max_steer_angle," << vehicle.max_steer_angle << "\n";
        out << "acc_time_constant," << vehicle.acc_time_constant << "\n";
        out << "steer_time_constant," << vehicle.steer_time_constant << "\n";
        out << "steer_rate_lim," << vehicle.steer_rate_lim << "\n";
        out << "vel_rate_lim," << vehicle.vel_rate_lim << "\n";
        out << "acc_time_delay," << vehicle.acc_time_delay << "\n";
        out << "steer_time_delay," << vehicle.steer_time_delay << "\n";
      }
    }
    params_written_ = true;
  }

  void writeRuntimeOptionsOnce(const FirstOrderDubinsMppiRuntimeOptions & options)
  {
    if (!enabled_ || runtime_written_) {
      return;
    }
    std::ofstream out(directory_ + "/runtime_options.csv");
    if (out) {
      out << "key,value\n";
      out << "ignore_obstacles," << (options.ignore_obstacles ? 1 : 0) << "\n";
      out << "ignore_road_borders," << (options.ignore_road_borders ? 1 : 0) << "\n";
      out << "ignore_drivable_area," << (options.ignore_drivable_area ? 1 : 0) << "\n";
      out << "force_cold_start_each_step," << (options.force_cold_start_each_step ? 1 : 0) << "\n";
      out << "skip_if_invalid," << (options.skip_if_invalid ? 1 : 0) << "\n";
      out << "min_optimization_length," << options.min_optimization_length << "\n";
      out << "use_last_control_as_nominal," << (options.use_last_control_as_nominal ? 1 : 0)
          << "\n";
      out << "use_temporal_mpt_as_nominal," << (options.use_temporal_mpt_as_nominal ? 1 : 0)
          << "\n";
      out << "prevent_reverse_velocity," << (options.prevent_reverse_velocity ? 1 : 0) << "\n";
      out << "enable_input_delay_compensation," << (options.enable_input_delay_compensation ? 1 : 0)
          << "\n";
    }
    runtime_written_ = true;
  }

  void logFrame(
    const autoware_planning_msgs::msg::Trajectory & reference,
    const autoware_planning_msgs::msg::Trajectory & optimized,
    const autoware_planning_msgs::msg::Trajectory & nominal_trajectory,
    const MppiDebugEgoState & ego, const double baseline_cost,
    const std::vector<float> & nominal_accel_cmd, const std::vector<float> & nominal_steer_cmd,
    const std::vector<Segment> & road_borders, const std::vector<Segment> & drivable_area,
    const autoware_perception_msgs::msg::TrackedObjects & tracked_objects,
    const float hist_accel_tm2, const float hist_steer_tm2, const float hist_accel_tm1,
    const float hist_steer_tm1, const std::vector<float> & delay_accel_cmd,
    const std::vector<float> & delay_steer_cmd, const float applied_accel_cmd,
    const float applied_steer_cmd, const FirstOrderDubinsMppiKinematicLimits & kinematic_limits)
  {
    if (!enabled_) {
      return;
    }

    ensureIndexHeader();
    const std::string frame_tag = formatFrameId(frame_id_);
    const std::string ref_path = directory_ + "/" + frame_tag + "_reference.csv";
    const std::string opt_path = directory_ + "/" + frame_tag + "_optimized.csv";
    const std::string ego_path = directory_ + "/" + frame_tag + "_ego.csv";
    const std::string nominal_path = directory_ + "/" + frame_tag + "_nominal.csv";
    const std::string nominal_traj_path = directory_ + "/" + frame_tag + "_nominal_traj.csv";
    if (
      !writeTrajectoryCsv(ref_path, reference) || !writeTrajectoryCsv(opt_path, optimized) ||
      !writeEgoCsv(ego_path, ego)) {
      return;
    }
    if (!nominal_trajectory.points.empty()) {
      if (!writeTrajectoryCsv(nominal_traj_path, nominal_trajectory)) {
        return;
      }
    }
    if (!nominal_accel_cmd.empty() || !nominal_steer_cmd.empty()) {
      if (!writeNominalCsv(nominal_path, nominal_accel_cmd, nominal_steer_cmd)) {
        return;
      }
    }
    writeSegmentsCsv(directory_ + "/" + frame_tag + "_road_borders.csv", road_borders);
    writeSegmentsCsv(directory_ + "/" + frame_tag + "_drivable.csv", drivable_area);
    writeObjectsCsv(directory_ + "/" + frame_tag + "_objects.csv", tracked_objects);
    writeControlHistoryCsv(
      directory_ + "/" + frame_tag + "_control_history.csv", hist_accel_tm2, hist_steer_tm2,
      hist_accel_tm1, hist_steer_tm1);
    writeNominalCsv(
      directory_ + "/" + frame_tag + "_delay_buffer.csv", delay_accel_cmd, delay_steer_cmd);
    writeAppliedCsv(
      directory_ + "/" + frame_tag + "_applied.csv", applied_accel_cmd, applied_steer_cmd);
    writeKinematicLimitsCsv(
      directory_ + "/" + frame_tag + "_kinematic_limits.csv", kinematic_limits);

    const auto & stamp = reference.header.stamp.sec != 0 || reference.header.stamp.nanosec != 0
                           ? reference.header.stamp
                           : optimized.header.stamp;
    std::ofstream index(directory_ + "/index.csv", std::ios::app);
    if (!index) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"), "Failed to append index.csv in %s",
        directory_.c_str());
      return;
    }
    index << frame_id_ << "," << stamp.sec << "," << stamp.nanosec << "," << baseline_cost << ","
          << reference.points.size() << "," << optimized.points.size() << "\n";
    ++frame_id_;
  }

private:
  static std::string formatFrameId(const uint64_t id)
  {
    std::ostringstream oss;
    oss << std::setw(6) << std::setfill('0') << id;
    return oss.str();
  }

  void ensureIndexHeader()
  {
    if (index_initialized_) {
      return;
    }
    const std::string index_path = directory_ + "/index.csv";
    bool needs_header = true;
    if (std::filesystem::exists(index_path) && std::filesystem::file_size(index_path) > 0) {
      std::ifstream in(index_path);
      std::string first;
      if (std::getline(in, first) && first.rfind("frame_id", 0) == 0) {
        needs_header = false;
      }
    }
    if (needs_header && !std::filesystem::exists(index_path)) {
      std::ofstream index(index_path);
      if (index) {
        index << "frame_id,stamp_sec,stamp_nsec,baseline_cost,n_reference,n_optimized\n";
      }
    } else if (needs_header && std::filesystem::exists(index_path)) {
      // File exists but header is missing/corrupt (e.g. truncated mid-session). Leave
      // existing rows; visualizer tolerates headerless index.csv.
      RCLCPP_WARN(
        rclcpp::get_logger("mppi_debug_trajectory_logger"),
        "index.csv exists without a frame_id header; appending without rewriting");
    }
    index_initialized_ = true;
  }

  static bool writeTrajectoryCsv(
    const std::string & path, const autoware_planning_msgs::msg::Trajectory & trajectory)
  {
    std::ofstream out(path);
    if (!out) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"), "Failed to write %s", path.c_str());
      return false;
    }
    out << "t_from_start_s,x,y,z,yaw,v,a,steer,steer_rate\n";
    out << std::setprecision(9) << std::fixed;
    for (const auto & point : trajectory.points) {
      const double t = static_cast<double>(point.time_from_start.sec) +
                       static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
      const double yaw = tf2::getYaw(point.pose.orientation);
      out << t << "," << point.pose.position.x << "," << point.pose.position.y << ","
          << point.pose.position.z << "," << yaw << "," << point.longitudinal_velocity_mps << ","
          << point.acceleration_mps2 << "," << point.front_wheel_angle_rad << ","
          << point.heading_rate_rps << "\n";
    }
    return true;
  }

  static bool writeEgoCsv(const std::string & path, const MppiDebugEgoState & ego)
  {
    std::ofstream out(path);
    if (!out) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"), "Failed to write %s", path.c_str());
      return false;
    }
    out << "x,y,z,yaw,v,accel,steer\n";
    out << std::setprecision(9) << std::fixed;
    out << ego.x << "," << ego.y << "," << ego.z << "," << ego.yaw << "," << ego.v << ","
        << ego.accel << "," << ego.steer << "\n";
    return true;
  }

  static bool writeNominalCsv(
    const std::string & path, const std::vector<float> & accel_cmd,
    const std::vector<float> & steer_cmd)
  {
    const size_t n = std::min(accel_cmd.size(), steer_cmd.size());
    std::ofstream out(path);
    if (!out) {
      RCLCPP_ERROR(
        rclcpp::get_logger("mppi_debug_trajectory_logger"), "Failed to write %s", path.c_str());
      return false;
    }
    out << "t_idx,accel_cmd,steer_cmd\n";
    out << std::setprecision(9) << std::fixed;
    for (size_t i = 0; i < n; ++i) {
      out << i << "," << accel_cmd[i] << "," << steer_cmd[i] << "\n";
    }
    return true;
  }

  static void writeSegmentsCsv(const std::string & path, const std::vector<Segment> & segments)
  {
    std::ofstream out(path);
    if (!out) {
      return;
    }
    out << "x0,y0,x1,y1\n";
    out << std::setprecision(9) << std::fixed;
    for (const auto & seg : segments) {
      out << seg.x0 << "," << seg.y0 << "," << seg.x1 << "," << seg.y1 << "\n";
    }
  }

  static void writeObjectsCsv(
    const std::string & path, const autoware_perception_msgs::msg::TrackedObjects & objects)
  {
    std::ofstream out(path);
    if (!out) {
      return;
    }
    out << "x,y,yaw,v,length,width\n";
    out << std::setprecision(9) << std::fixed;
    for (const auto & object : objects.objects) {
      const auto & pose = object.kinematics.pose_with_covariance.pose;
      const double yaw = tf2::getYaw(pose.orientation);
      const double v = object.kinematics.twist_with_covariance.twist.linear.x;
      double length = 4.5;
      double width = 1.8;
      if (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
        length = object.shape.dimensions.x;
        width = object.shape.dimensions.y;
      }
      out << pose.position.x << "," << pose.position.y << "," << yaw << "," << v << "," << length
          << "," << width << "\n";
    }
  }

  static void writeControlHistoryCsv(
    const std::string & path, const float accel_tm2, const float steer_tm2, const float accel_tm1,
    const float steer_tm1)
  {
    std::ofstream out(path);
    if (!out) {
      return;
    }
    out << "accel_tm2,steer_tm2,accel_tm1,steer_tm1\n";
    out << std::setprecision(9) << std::fixed;
    out << accel_tm2 << "," << steer_tm2 << "," << accel_tm1 << "," << steer_tm1 << "\n";
  }

  static void writeAppliedCsv(
    const std::string & path, const float accel_cmd, const float steer_cmd)
  {
    std::ofstream out(path);
    if (!out) {
      return;
    }
    out << "accel_cmd,steer_cmd\n";
    out << std::setprecision(9) << std::fixed;
    out << accel_cmd << "," << steer_cmd << "\n";
  }

  static void writeKinematicLimitsCsv(
    const std::string & path, const FirstOrderDubinsMppiKinematicLimits & limits)
  {
    std::ofstream out(path);
    if (!out) {
      return;
    }
    out << "key,value\n";
    out << std::setprecision(9) << std::fixed;
    const bool has_pointwise_velocity_limit = std::any_of(
      limits.max_velocity_by_reference_point.begin(), limits.max_velocity_by_reference_point.end(),
      [](const auto & value) { return value.has_value(); });
    const bool active = limits.max_velocity || has_pointwise_velocity_limit ||
                        limits.min_longitudinal_acceleration ||
                        limits.max_longitudinal_acceleration || limits.min_longitudinal_jerk ||
                        limits.max_longitudinal_jerk;
    out << "active," << (active ? 1 : 0) << "\n";
    if (limits.max_velocity) {
      out << "min_velocity,0.0\n";
      out << "max_velocity," << *limits.max_velocity << "\n";
    }
    for (std::size_t index = 0; index < limits.max_velocity_by_reference_point.size(); ++index) {
      if (limits.max_velocity_by_reference_point[index]) {
        out << "max_velocity_point_" << index << ","
            << *limits.max_velocity_by_reference_point[index] << "\n";
      }
    }
    if (limits.min_longitudinal_acceleration && limits.max_longitudinal_acceleration) {
      out << "min_longitudinal_acceleration," << *limits.min_longitudinal_acceleration << "\n";
      out << "max_longitudinal_acceleration," << *limits.max_longitudinal_acceleration << "\n";
    }
    if (limits.min_longitudinal_jerk && limits.max_longitudinal_jerk) {
      out << "min_longitudinal_jerk," << *limits.min_longitudinal_jerk << "\n";
      out << "max_longitudinal_jerk," << *limits.max_longitudinal_jerk << "\n";
    }
  }

  bool enabled_{false};
  bool index_initialized_{false};
  bool params_written_{false};
  bool runtime_written_{false};
  std::string directory_;
  uint64_t frame_id_{0};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_LOGGER_HPP_
