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

#include "autoware/trajectory_processor/time_sequence_raw/trajectory_optimizer.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>

namespace autoware::trajectory_processor::time_sequence_raw
{
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace
{
constexpr double max_warm_start_age_s = 0.5;
constexpr double min_speed_for_steering_estimation_mps = 0.5;

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

TrajectoryOptimizer::TrajectoryOptimizer(
  const TrajectoryOptimizationParams & params,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const size_t batch_size)
: params_(params),
  wheelbase_m_(vehicle_info.wheel_base_m),
  max_steering_angle_rad_(vehicle_info.max_steer_angle_rad),
  solver_(
    std::make_unique<AcadosSolverWrapper>(params, wheelbase_m_, vehicle_info.max_steer_angle_rad)),
  previous_solutions_(batch_size)
{
}

OptimizationResult TrajectoryOptimizer::optimize(
  const Trajectory & raw_trajectory, const Odometry & ego_odometry,
  const std::optional<double> & current_steering_angle_rad, const size_t batch_index)
{
  OptimizationResult result;
  result.trajectory = raw_trajectory;

  if (raw_trajectory.points.size() < opt_horizon || batch_index >= previous_solutions_.size()) {
    return result;
  }

  const auto & ego_pose = ego_odometry.pose.pose;
  const double base_x = ego_pose.position.x;
  const double base_y = ego_pose.position.y;
  const double yaw0 = yaw_from_quaternion(ego_pose.orientation);
  const double v0 = std::clamp(
    static_cast<double>(ego_odometry.twist.twist.linear.x), params_.min_velocity_mps,
    params_.max_velocity_mps);

  double delta0 = 0.0;
  if (current_steering_angle_rad.has_value()) {
    delta0 = *current_steering_angle_rad;
  } else {
    const double yaw_rate = ego_odometry.twist.twist.angular.z;
    const double speed = std::max(std::abs(v0), min_speed_for_steering_estimation_mps);
    delta0 = std::atan(wheelbase_m_ * yaw_rate / speed);
  }
  delta0 = std::clamp(delta0, -max_steering_angle_rad_, max_steering_angle_rad_);

  const std::array<double, opt_nx> initial_state{0.0, 0.0, yaw0, v0, delta0};

  std::array<StageReference, opt_horizon> references;
  double previous_yaw = yaw0;
  for (size_t k = 0; k < opt_horizon; ++k) {
    const auto & point = raw_trajectory.points[k];
    StageReference & ref = references[k];
    ref.x = point.pose.position.x - base_x;
    ref.y = point.pose.position.y - base_y;
    const double raw_yaw = yaw_from_quaternion(point.pose.orientation);
    ref.yaw = previous_yaw + autoware_utils_math::normalize_radian(raw_yaw - previous_yaw);
    previous_yaw = ref.yaw;
  }

  const rclcpp::Time stamp(raw_trajectory.header.stamp);
  SolverSolution warm_start;
  const SolverSolution * warm_start_ptr = nullptr;
  auto & previous = previous_solutions_[batch_index];
  if (previous.has_value()) {
    const double age_s = (stamp - previous->stamp).seconds();
    if (age_s >= 0.0 && age_s <= max_warm_start_age_s) {
      warm_start = previous->solution;
      for (auto & state : warm_start.states) {
        state[0] -= base_x;
        state[1] -= base_y;
      }
      warm_start_ptr = &warm_start;
    } else {
      previous.reset();
    }
  }

  SolverSolution solution = solver_->solve(initial_state, references, warm_start_ptr);
  result.solver_status = solution.status;
  result.solve_time_ms = solution.solve_time_s * 1e3;

  if (!solution.success()) {
    previous.reset();
    return result;
  }

  Trajectory optimized;
  optimized.header = raw_trajectory.header;
  optimized.points.reserve(opt_horizon);
  for (size_t i = 1; i <= opt_horizon; ++i) {
    const auto & state = solution.states[i];
    TrajectoryPoint point;
    const double time_s = opt_dt_s * static_cast<double>(i);
    point.time_from_start.sec = static_cast<int32_t>(time_s);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((time_s - point.time_from_start.sec) * 1e9);
    point.pose.position.x = state[0] + base_x;
    point.pose.position.y = state[1] + base_y;
    point.pose.position.z = raw_trajectory.points[i - 1].pose.position.z;
    point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(
      autoware_utils_math::normalize_radian(state[2]));
    point.longitudinal_velocity_mps = static_cast<float>(state[3]);
    point.front_wheel_angle_rad = static_cast<float>(state[4]);
    point.acceleration_mps2 = (i < opt_horizon) ? static_cast<float>(solution.inputs[i][0]) : 0.0F;
    point.heading_rate_rps = static_cast<float>(state[3] * std::tan(state[4]) / wheelbase_m_);
    optimized.points.push_back(point);
  }
  result.trajectory = std::move(optimized);
  result.optimized = true;

  for (auto & state : solution.states) {
    state[0] += base_x;
    state[1] += base_y;
  }
  previous = PreviousSolution{solution, stamp};

  return result;
}

void TrajectoryOptimizer::clear_warm_start(const size_t batch_index)
{
  if (batch_index < previous_solutions_.size()) {
    previous_solutions_[batch_index].reset();
  }
}

}  // namespace autoware::trajectory_processor::time_sequence_raw
