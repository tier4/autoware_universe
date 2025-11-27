// Copyright 2025 TIER IV, Inc.
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

#include "class_helper.hpp"

#include "data_types.hpp"
#include "hermite_spline.hpp"
#include "wait_time_accumulator.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <fmt/format.h>
#include <tf2/utils.h>

#include <algorithm>
#include <deque>
#include <optional>
#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter::helper
{
std::optional<std::string> check_odometry_msg(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_ptr, const rclcpp::Time & now,
  const double timeout_s)
{
  if (!odom_ptr) {
    return "Waiting for odometry data...";
  }

  const auto msg_time = rclcpp::Time(odom_ptr->header.stamp, now.get_clock_type());
  const double delay = (now - msg_time).seconds();

  if (delay > timeout_s) {
    return fmt::format("Odometry is stale. Delay: {:.3f}s (Threshold: {:.3f}s)", delay, timeout_s);
  }

  return std::nullopt;
}

std::optional<std::string> check_trajectory_msg(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & traj_ptr,
  const rclcpp::Time & now, const double timeout_s)
{
  if (!traj_ptr || traj_ptr->points.empty()) {
    return {"waiting for trajectory..."};
  }

  const auto msg_time = rclcpp::Time(traj_ptr->header.stamp, now.get_clock_type());
  const double delay = (now - msg_time).seconds();

  if (delay > timeout_s) {
    return fmt::format(
      "Trajectory msg is stale. Delay: {:.3f}s (Threshold: {:.3f}s)", delay, timeout_s);
  }

  if (auto is_non_monotonic = has_non_monotonic(traj_ptr->points); is_non_monotonic) {
    return is_non_monotonic;
  }

  return std::nullopt;
}

std::optional<std::string> has_non_monotonic(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & trajectory_points)
{
  // if there is no points, we assume it is monotonic.
  if (trajectory_points.size() < 2) {
    return std::nullopt;
  }

  const auto to_sec = [](const auto & tp) {
    return rclcpp::Duration(tp.time_from_start).seconds();
  };

  auto monotonic_check_it = std::adjacent_find(
    trajectory_points.begin(), trajectory_points.end(),
    [&](const auto & prev, const auto & curr) { return to_sec(prev) - g_math_eps > to_sec(curr); });

  if (monotonic_check_it != trajectory_points.end()) {
    size_t idx_prev = std::distance(trajectory_points.begin(), monotonic_check_it);
    size_t idx_curr = idx_prev + 1;

    return fmt::format(
      "Trajectory has not increasing time at index {} ({:.6f} -> {:.6f})", idx_curr,
      to_sec(*monotonic_check_it), to_sec(*(monotonic_check_it + 1)));
  }

  return std::nullopt;
}

double solve_time_step(double current_s, double next_s, const HermiteSpline & sv)
{
  const double ds = next_s - current_s;

  if (ds < g_math_eps) {
    return 0.0;
  }

  const double s_mid = current_s + ds * 0.5;

  // 1. Query Velocities
  double v_start = sv.compute(current_s);
  double v_mid = sv.compute(s_mid);
  double v_end = sv.compute(next_s);

  // Sanity clamp negative values
  if (v_start < 0.0) v_start = 0.0;
  if (v_mid < 0.0) v_mid = 0.0;
  if (v_end < 0.0) v_end = 0.0;

  // 2. Hybrid Strategy: Avoid Simpson's singularity at low speeds.
  // Simpson's rule integrates (1/v). As v -> 0, this term explodes.
  // If we are in a "Low Velocity" regime (e.g. < 0.1 m/s), the kinematic assumption
  // (Linear Velocity Ramp) is numerically much more stable and physically sufficient.
  constexpr double k_low_speed_threshold = 0.1;  // m/s

  if (v_start < k_low_speed_threshold || v_end < k_low_speed_threshold) {
    // Linear Average Velocity: t = ds / v_avg
    constexpr double k_min_divisor = 1e-3;  // Prevent division by zero
    double v_avg = (v_start + v_end) / 2.0;
    if (v_avg < k_min_divisor) v_avg = k_min_divisor;
    return ds / v_avg;
  }

  // 3. Normal Simpson's Integration (for v > 0.1)
  // We still clamp to a safety floor, but it won't be hit often due to the check above.
  constexpr double min_simpson_vel = 0.01;
  if (v_start < min_simpson_vel) v_start = min_simpson_vel;
  if (v_mid < min_simpson_vel) v_mid = min_simpson_vel;
  if (v_end < min_simpson_vel) v_end = min_simpson_vel;

  const double integrand_start = 1.0 / v_start;
  const double integrand_mid = 1.0 / v_mid;
  const double integrand_end = 1.0 / v_end;

  return (ds / 6.0) * (integrand_start + 4.0 * integrand_mid + integrand_end);
}

SplineData resample(
  const SplineData & spline_data, const double resolution, const bool recompute_acceleration)
{
  if (spline_data.s.size() < 2) {
    return spline_data;
  }

  // 1. Prepare Interpolators
  const SplineSet splines = SplineSet::build(spline_data, recompute_acceleration);

  // 2. Prepare Output
  const double total_s = spline_data.s.back();
  SplineData resampled;
  resampled.reset(static_cast<size_t>(std::ceil(total_s / resolution)) + 5);

  // 3. Initialize Loop State
  double current_s = 0.0;
  double t_accum = spline_data.t.front();
  const double step_size = std::max(0.1, resolution);

  // Create stateful helper for wait times
  WaitTimeAccumulator wait_processor(spline_data);

  // 4. Generation Loop
  while (current_s <= total_s + g_math_eps) {
    PlannerPoint p{};

    p.pos.x = splines.sx.compute(current_s);
    p.pos.y = splines.sy.compute(current_s);
    p.pos.z = splines.sz.compute(current_s);
    p.v = std::max(0.0, splines.sv.compute(current_s));

    p.a = splines.compute_acceleration(current_s, p.v, total_s);

    if (!resampled.s.empty()) {
      t_accum += solve_time_step(resampled.s.back(), current_s, splines.sv);
    }

    p.t = t_accum;
    t_accum += wait_processor.collect(current_s);

    resampled.add_point(current_s, p, 0.0);

    // Advance
    if (std::abs(current_s - total_s) < g_math_eps) break;
    current_s += step_size;
    if (current_s > total_s) current_s = total_s;
  }

  // 5. Post-Process
  if (recompute_acceleration) {
    resampled.smooth_acceleration(resolution);
  }

  return resampled;
}

double calculate_yaw_at_index(const SplineData & spline, size_t i, double last_valid_yaw)
{
  const size_t size = spline.x.size();

  // 1. Primary Strategy: Look Ahead (Tangent to future path)
  if (i + 1 < size) {
    double dx = spline.x[i + 1] - spline.x[i];
    double dy = spline.y[i + 1] - spline.y[i];

    if (std::hypot(dx, dy) > g_geom_eps) {
      return std::atan2(dy, dx);
    }
  }

  // 2. Secondary Strategy: Look Behind (Tangent from past path)
  // We reach here if we are at the last point OR if we are stopped (dist to next is < g_geom_eps).
  if (i > 0) {
    double dx = spline.x[i] - spline.x[i - 1];
    double dy = spline.y[i] - spline.y[i - 1];

    if (std::hypot(dx, dy) > g_geom_eps) {
      return std::atan2(dy, dx);
    }
  }

  return last_valid_yaw;
}

autoware_planning_msgs::msg::Trajectory convert_spline_data_to_trajectory_msg(
  const SplineData & spline, const std_msgs::msg::Header & header)
{
  autoware_planning_msgs::msg::Trajectory traj_msg;
  traj_msg.header = header;
  traj_msg.points = convert_spline_data_to_trajectory_points(spline);
  return traj_msg;
}

std::vector<PlannerPoint> convert_msg_to_planner_points(
  const autoware_planning_msgs::msg::Trajectory & traj)
{
  std::vector<PlannerPoint> planner_points;
  planner_points.reserve(traj.points.size());
  for (const auto & traj_pt : traj.points) {
    decltype(planner_points)::value_type planner_pt;
    planner_pt.t = rclcpp::Duration(traj_pt.time_from_start).seconds();
    planner_pt.pos.x = traj_pt.pose.position.x;
    planner_pt.pos.y = traj_pt.pose.position.y;
    planner_pt.pos.z = traj_pt.pose.position.z;
    planner_pt.v = traj_pt.longitudinal_velocity_mps;
    planner_pt.a = traj_pt.acceleration_mps2;
    planner_pt.yaw = tf2::getYaw(traj_pt.pose.orientation);
    planner_points.push_back(planner_pt);
  }
  return planner_points;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> convert_spline_data_to_trajectory_points(
  const SplineData & spline)
{
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj_points;

  const size_t size = spline.s.size();
  if (size == 0) return traj_points;

  if (spline.x.size() != size || spline.t.size() != size) {
    return traj_points;
  }

  traj_points.reserve(size);
  double current_yaw = 0.0;

  for (size_t i = 0; i < size; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p;

    // 1. Geometry
    p.pose.position.x = spline.x[i];
    p.pose.position.y = spline.y[i];
    p.pose.position.z = (i < spline.z.size()) ? spline.z[i] : 0.0;

    current_yaw = calculate_yaw_at_index(spline, i, current_yaw);
    double half_yaw = current_yaw * 0.5;

    p.pose.orientation.w = std::cos(half_yaw);
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = std::sin(half_yaw);

    p.time_from_start = rclcpp::Duration::from_seconds(spline.t[i]);
    p.longitudinal_velocity_mps = static_cast<float>(spline.v[i]);
    p.acceleration_mps2 = static_cast<float>((i < spline.a.size()) ? spline.a[i] : 0.0);

    traj_points.push_back(p);
  }

  return traj_points;
}
std::vector<PlannerPoint> convert_odometry_history_to_planner_points(
  const std::deque<nav_msgs::msg::Odometry> & odom_history)
{
  if (odom_history.empty()) {
    return {};
  }

  std::vector<PlannerPoint> planner_points;
  planner_points.reserve(odom_history.size());

  for (auto it = odom_history.rbegin(); it != odom_history.rend(); ++it) {
    decltype(planner_points)::value_type planner_pt;
    planner_pt.pos.x = it->pose.pose.position.x;
    planner_pt.pos.y = it->pose.pose.position.y;
    planner_pt.pos.z = it->pose.pose.position.z;
    planner_pt.v = it->twist.twist.linear.x;
    planner_pt.a = 0.0;
    planner_pt.yaw = tf2::getYaw(it->pose.pose.orientation);
    planner_pt.t = 0.0;

    planner_points.push_back(planner_pt);
  }
  return planner_points;
}
}  // namespace autoware::time_to_space_trajectory_converter::helper
