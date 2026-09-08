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

#include "closed_loop_simulator.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner::testing
{

namespace
{

double to_seconds(const builtin_interfaces::msg::Duration & d)
{
  return rclcpp::Duration(d).seconds();
}

double to_seconds(const builtin_interfaces::msg::Time & t)
{
  return rclcpp::Time(t).seconds();
}

bool is_finite_pose(const Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

}  // namespace

std::vector<std::string> validate_trajectory(
  const Trajectory & trajectory, const Pose & ego_pose, const VehicleInfo & vehicle_info)
{
  // Thresholds follow the defaults of planning_validator
  constexpr double max_interval_m = 10.0;
  constexpr double max_ego_offset_m = 2.0;
  constexpr double max_yaw_jump_rad = M_PI / 2.0;
  constexpr double yaw_check_min_interval_m = 0.1;

  std::vector<std::string> violations;
  const auto & points = trajectory.points;

  if (points.size() < 2) {
    violations.push_back("trajectory has fewer than 2 points");
    return violations;
  }

  for (size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i];
    if (
      !is_finite_pose(p.pose) || !std::isfinite(p.longitudinal_velocity_mps) ||
      !std::isfinite(p.acceleration_mps2) || !std::isfinite(p.front_wheel_angle_rad)) {
      violations.push_back("non-finite value at point " + std::to_string(i));
      return violations;  // the geometric checks below are meaningless
    }
  }

  if (autoware_utils_geometry::calc_distance2d(points.front().pose, ego_pose) > max_ego_offset_m) {
    violations.push_back("first point is far from ego");
  }

  for (size_t i = 1; i < points.size(); ++i) {
    const auto & prev = points[i - 1];
    const auto & curr = points[i];
    if (to_seconds(curr.time_from_start) < to_seconds(prev.time_from_start)) {
      violations.push_back("time_from_start decreases at point " + std::to_string(i));
      break;
    }
    const double interval = autoware_utils_geometry::calc_distance2d(prev.pose, curr.pose);
    if (interval > max_interval_m) {
      violations.push_back("interval exceeds threshold at point " + std::to_string(i));
      break;
    }
    if (interval > yaw_check_min_interval_m) {
      const double yaw_diff = std::abs(
        autoware_utils_geometry::normalize_radian(
          autoware_utils_geometry::get_rpy(curr.pose).z -
          autoware_utils_geometry::get_rpy(prev.pose).z));
      if (yaw_diff > max_yaw_jump_rad) {
        violations.push_back("yaw jumps at point " + std::to_string(i));
        break;
      }
    }
  }

  for (size_t i = 0; i < points.size(); ++i) {
    if (points[i].longitudinal_velocity_mps < -1e-3) {
      violations.push_back("negative velocity at point " + std::to_string(i));
      break;
    }
  }
  for (size_t i = 0; i < points.size(); ++i) {
    if (std::abs(points[i].front_wheel_angle_rad) > vehicle_info.max_steer_angle_rad) {
      violations.push_back("steer angle exceeds max_steer_angle at point " + std::to_string(i));
      break;
    }
  }

  return violations;
}

void write_result_csv(const ClosedLoopResult & result, const std::string & path_prefix)
{
  std::ofstream ego(path_prefix + "_ego.csv");
  ego << "step,t,x,y,z,yaw,v,a,steer,num_trajectory_points\n";
  std::ofstream traj(path_prefix + "_trajectories.csv");
  traj << "step,t,idx,time_from_start,x,y,z,yaw,v,a,steer\n";
  ego.precision(9);
  traj.precision(9);

  const auto write_ego_row = [&ego](const size_t i, const StepRecord & s) {
    const auto & pose = s.odometry.pose.pose;
    ego << i << "," << to_seconds(s.odometry.header.stamp) << "," << pose.position.x << ","
        << pose.position.y << "," << pose.position.z << ","
        << autoware_utils_geometry::get_rpy(pose).z << "," << s.odometry.twist.twist.linear.x << ","
        << s.acceleration.accel.accel.linear.x << "," << s.steering.steering_tire_angle << ","
        << s.trajectory.points.size() << "\n";
  };

  for (size_t i = 0; i < result.steps.size(); ++i) {
    const auto & s = result.steps[i];
    const double t = to_seconds(s.odometry.header.stamp);
    write_ego_row(i, s);
    for (size_t j = 0; j < s.trajectory.points.size(); ++j) {
      const auto & p = s.trajectory.points[j];
      traj << i << "," << t << "," << j << "," << to_seconds(p.time_from_start) << ","
           << p.pose.position.x << "," << p.pose.position.y << "," << p.pose.position.z << ","
           << autoware_utils_geometry::get_rpy(p.pose).z << "," << p.longitudinal_velocity_mps
           << "," << p.acceleration_mps2 << "," << p.front_wheel_angle_rad << "\n";
    }
  }
  write_ego_row(result.steps.size(), result.final_state);
}

ClosedLoopSimulator::ClosedLoopSimulator(
  const Params & params, const VehicleInfo & vehicle_info, const LaneletMapBin & map_bin,
  const LaneletRoute & route, const PredictedObjects & predicted_objects,
  const ClosedLoopConfig & config)
: params_(params),
  config_(config),
  map_bin_(map_bin),
  route_(route),
  predicted_objects_(predicted_objects),
  planner_(params, std::make_shared<TimeKeeper>())
{
  input_.vehicle_info = vehicle_info;
  input_.odometry.header.frame_id = "map";
  input_.odometry.pose.pose = route.start_pose;
  input_.acceleration.header.frame_id = "map";
  input_.steering.steering_tire_angle = 0.0;
  input_.goal_pose = route.goal_pose;
  predicted_objects_.header.frame_id = "map";
}

bool ClosedLoopSimulator::update_route_manager()
{
  const auto & current_pose = input_.odometry.pose.pose;
  if (input_.route_manager) {
    input_.route_manager = std::move(*input_.route_manager)
                             .update_current_pose(
                               current_pose, params_.ego_nearest_lanelet.dist_threshold_m,
                               params_.ego_nearest_lanelet.yaw_threshold_rad);
    if (input_.route_manager) {
      return true;
    }
  }
  input_.route_manager = RouteManager::create(map_bin_, route_, current_pose);
  return input_.route_manager.has_value();
}

void ClosedLoopSimulator::advance_ego(const Trajectory & trajectory)
{
  const auto & points = trajectory.points;
  const double t_target = config_.dt_s;

  // The first point is the ego (time_from_start = 0). Interpolate linearly between the two
  // points that bracket dt
  size_t idx = points.size() - 1;
  for (size_t i = 1; i < points.size(); ++i) {
    if (to_seconds(points[i].time_from_start) >= t_target) {
      idx = i;
      break;
    }
  }
  const auto & p0 = points[idx - 1];
  const auto & p1 = points[idx];
  const double t0 = to_seconds(p0.time_from_start);
  const double t1 = to_seconds(p1.time_from_start);
  const double ratio = (t1 - t0) > 1e-6 ? std::clamp((t_target - t0) / (t1 - t0), 0.0, 1.0) : 1.0;

  // Points coincide while stopped, so slerp the orientation instead of deriving it from the
  // position difference
  input_.odometry.pose.pose =
    autoware_utils_geometry::calc_interpolated_pose(p0.pose, p1.pose, ratio, false);
  input_.odometry.twist.twist.linear.x =
    (1.0 - ratio) * p0.longitudinal_velocity_mps + ratio * p1.longitudinal_velocity_mps;
  input_.acceleration.accel.accel.linear.x =
    (1.0 - ratio) * p0.acceleration_mps2 + ratio * p1.acceleration_mps2;
  input_.steering.steering_tire_angle =
    static_cast<float>((1.0 - ratio) * p0.front_wheel_angle_rad + ratio * p1.front_wheel_angle_rad);
}

ClosedLoopResult ClosedLoopSimulator::run()
{
  ClosedLoopResult result;
  const auto finish = [this, &result](const std::string & reason, const bool goal_reached = false) {
    result.goal_reached = goal_reached;
    result.termination_reason = reason;
    result.final_state =
      StepRecord{input_.odometry, input_.acceleration, input_.steering, Trajectory{}, {}};
    return result;
  };

  double best_goal_distance = std::numeric_limits<double>::infinity();
  size_t last_progress_step = 0;

  for (step_ = 0; step_ < config_.max_steps; ++step_) {
    const auto stamp = rclcpp::Time(static_cast<int64_t>(step_ * config_.dt_s * 1e9));
    input_.odometry.header.stamp = stamp;
    input_.acceleration.header.stamp = stamp;
    input_.steering.stamp = stamp;
    predicted_objects_.header.stamp = stamp;
    input_.predicted_objects = std::make_shared<const PredictedObjects>(predicted_objects_);

    const double goal_distance =
      autoware_utils_geometry::calc_distance2d(input_.odometry.pose.pose, input_.goal_pose);
    if (
      goal_distance < config_.goal_distance_threshold_m &&
      std::abs(input_.odometry.twist.twist.linear.x) < config_.goal_velocity_threshold_mps) {
      return finish("goal reached at step " + std::to_string(step_), true);
    }
    if (goal_distance < best_goal_distance - config_.stall_progress_m) {
      best_goal_distance = goal_distance;
      last_progress_step = step_;
    } else if (step_ - last_progress_step >= config_.stall_window_steps) {
      return finish(
        "stalled at step " + std::to_string(step_) +
        " (goal distance = " + std::to_string(goal_distance) + " m)");
    }

    if (!update_route_manager()) {
      return finish("failed to create RouteManager at step " + std::to_string(step_));
    }

    const auto planned = planner_.plan(input_);
    if (!planned) {
      result.violations.push_back(
        "step " + std::to_string(step_) + ": plan failed: " + planned.error());
      return finish("plan failed at step " + std::to_string(step_));
    }
    if (!planned->normal_trajectory) {
      result.violations.push_back("step " + std::to_string(step_) + ": normal_trajectory is empty");
      return finish("no trajectory at step " + std::to_string(step_));
    }
    const auto & trajectory = *planned->normal_trajectory;
    std::vector<std::pair<double, double>> reference_path_xy;
    const auto & reference_path = planned->debug.reference_path;
    for (double s = 0.0; s <= reference_path.length(); s += 1.0) {
      const auto & p = reference_path.compute(s).point.pose.position;
      reference_path_xy.emplace_back(p.x, p.y);
    }
    result.steps.push_back(
      StepRecord{
        input_.odometry, input_.acceleration, input_.steering, trajectory,
        std::move(reference_path_xy)});

    const auto violations =
      validate_trajectory(trajectory, input_.odometry.pose.pose, input_.vehicle_info);
    for (const auto & v : violations) {
      result.violations.push_back("step " + std::to_string(step_) + ": " + v);
    }
    if (!violations.empty()) {
      // Advancing the ego along a broken trajectory would make the following cycles meaningless
      return finish("invalid trajectory at step " + std::to_string(step_));
    }

    advance_ego(trajectory);
  }

  return finish("reached max_steps (" + std::to_string(config_.max_steps) + ")");
}

}  // namespace autoware::safety_planner::testing
