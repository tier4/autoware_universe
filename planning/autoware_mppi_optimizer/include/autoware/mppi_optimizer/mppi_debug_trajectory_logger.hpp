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

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/utils.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace autoware::mppi_optimizer
{

/**
 * Optional CSV logger for reference / optimized trajectories so
 * mppi_debug_visualizer.py can replay offline via --log-dir.
 *
 * Layout:
 *   <log_dir>/index.csv
 *   <log_dir>/000000_reference.csv
 *   <log_dir>/000000_optimized.csv
 *   ...
 *
 * Trajectory CSV columns:
 *   t_from_start_s,x,y,z,yaw,v,a,steer,steer_rate
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
    if (!enabled_) {
      return;
    }
    if (directory_.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("mppi_debug_trajectory_logger"),
        "Debug trajectory logging enabled but directory is empty; disabling.");
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

  void logFrame(
    const autoware_planning_msgs::msg::Trajectory & reference,
    const autoware_planning_msgs::msg::Trajectory & optimized, const double baseline_cost = 0.0)
  {
    if (!enabled_) {
      return;
    }

    ensureIndexHeader();
    const std::string frame_tag = formatFrameId(frame_id_);
    const std::string ref_path = directory_ + "/" + frame_tag + "_reference.csv";
    const std::string opt_path = directory_ + "/" + frame_tag + "_optimized.csv";
    if (!writeTrajectoryCsv(ref_path, reference) || !writeTrajectoryCsv(opt_path, optimized)) {
      return;
    }

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
    if (!std::filesystem::exists(index_path)) {
      std::ofstream index(index_path);
      if (index) {
        index << "frame_id,stamp_sec,stamp_nsec,baseline_cost,n_reference,n_optimized\n";
      }
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
      const double t =
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
      const double yaw = tf2::getYaw(point.pose.orientation);
      out << t << "," << point.pose.position.x << "," << point.pose.position.y << ","
          << point.pose.position.z << "," << yaw << "," << point.longitudinal_velocity_mps << ","
          << point.acceleration_mps2 << "," << point.front_wheel_angle_rad << ","
          << point.heading_rate_rps << "\n";
    }
    return true;
  }

  bool enabled_{false};
  bool index_initialized_{false};
  std::string directory_;
  uint64_t frame_id_{0};
};

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_TRAJECTORY_LOGGER_HPP_
