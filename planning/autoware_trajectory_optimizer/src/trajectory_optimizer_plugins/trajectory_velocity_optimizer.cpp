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

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_velocity_optimizer.hpp"

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/plugin_utils/trajectory_velocity_optimizer_utils.hpp"
#include "autoware/trajectory_optimizer/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_rclcpp/parameter.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
namespace autoware::trajectory_optimizer::plugin
{

void TrajectoryVelocityOptimizer::initialize(
  const std::string & name, rclcpp::Node * node_ptr,
  const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper)
{
  TrajectoryOptimizerPluginBase::initialize(name, node_ptr, time_keeper);

  set_up_params();

  sub_planning_velocity_ =
    std::make_shared<autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit>>(
      node_ptr, "~/input/external_velocity_limit_mps", rclcpp::QoS{1});

  pub_velocity_limit_ = node_ptr->create_publisher<VelocityLimit>(
    "~/output/current_velocity_limit_mps", rclcpp::QoS{1}.transient_local());

  // publish default max velocity
  VelocityLimit max_vel_msg{};
  max_vel_msg.stamp = node_ptr->now();
  max_vel_msg.max_velocity = static_cast<float>(velocity_params_.default_max_velocity_mps);
  pub_velocity_limit_->publish(max_vel_msg);
}

void TrajectoryVelocityOptimizer::optimize_trajectory(
  TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
  const TrajectoryOptimizerData & data)
{
  if (!params.use_velocity_optimizer) {
    return;
  }

  if (traj_points.empty()) {
    return;
  }

  const auto & current_odometry = data.current_odometry;

  const TrajectoryPoints reference_trajectory = traj_points;

  TrajectoryPoints before_constraints = traj_points;

  if (velocity_params_.limit_lateral_acceleration) {
    trajectory_velocity_optimizer_utils::limit_lateral_acceleration(
      traj_points, velocity_params_.max_lateral_accel_mps2, data.current_odometry);
  }

  if (velocity_params_.limit_speed) {
    const auto external_velocity_limit = sub_planning_velocity_->take_data();
    const auto max_speed_mps = (external_velocity_limit)
                                 ? static_cast<float>(external_velocity_limit->max_velocity)
                                 : static_cast<float>(velocity_params_.default_max_velocity_mps);
    trajectory_velocity_optimizer_utils::set_max_velocity(traj_points, max_speed_mps);
    if (external_velocity_limit) {
      pub_velocity_limit_->publish(*external_velocity_limit);
    }
  }

  TrajectoryPoints after_constraints = traj_points;

  const auto external_velocity_limit_qp = sub_planning_velocity_->take_data();
  const auto max_speed_mps_qp = (external_velocity_limit_qp)
                                  ? static_cast<float>(external_velocity_limit_qp->max_velocity)
                                  : static_cast<float>(velocity_params_.default_max_velocity_mps);

  if (velocity_params_.smooth_velocities) {
    if (!is_smoothing_needed(after_constraints)) {
      RCLCPP_DEBUG_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
        "Trajectory already smooth, skipping QP optimization");
      return;
    }

    const auto constrained_indices =
      identify_constrained_points(before_constraints, after_constraints);

    RCLCPP_DEBUG_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
      "QP smoothing: %zu constrained points detected", constrained_indices.size());

    if (!optimize_velocity_with_qp(
          reference_trajectory, traj_points, constrained_indices, current_odometry,
          max_speed_mps_qp)) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
        "QP velocity optimization failed, using constrained trajectory");
      traj_points = after_constraints;
    }
  }
}

bool TrajectoryVelocityOptimizer::is_smoothing_needed(const TrajectoryPoints & trajectory) const
{
  if (trajectory.size() < 3) {
    return false;
  }

  const double dt = velocity_params_.qp_time_step_s;
  const double jerk_threshold = velocity_params_.qp_jerk_threshold_mps3;

  double max_jerk = 0.0;
  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    const double a_curr = trajectory[i].acceleration_mps2;
    const double a_next = trajectory[i + 1].acceleration_mps2;
    const double jerk = std::abs((a_next - a_curr) / dt);
    max_jerk = std::max(max_jerk, jerk);
  }

  return max_jerk > jerk_threshold;
}

std::vector<size_t> TrajectoryVelocityOptimizer::identify_constrained_points(
  const TrajectoryPoints & before_constraints, const TrajectoryPoints & after_constraints) const
{
  std::vector<size_t> constrained_indices;
  constexpr double velocity_tolerance = 1e-3;

  const size_t N = std::min(before_constraints.size(), after_constraints.size());
  for (size_t i = 0; i < N; ++i) {
    const double v_before = before_constraints[i].longitudinal_velocity_mps;
    const double v_after = after_constraints[i].longitudinal_velocity_mps;

    if (v_before - v_after > velocity_tolerance) {
      constrained_indices.push_back(i);
    }
  }

  return constrained_indices;
}

std::vector<double> TrajectoryVelocityOptimizer::compute_fidelity_weights(
  const TrajectoryPoints & trajectory,
  [[maybe_unused]] const std::vector<size_t> & constrained_indices) const
{
  const size_t n_points = trajectory.size();
  std::vector<double> weights(n_points, velocity_params_.qp_base_fidelity_weight);

  return weights;
}

void TrajectoryVelocityOptimizer::prepare_velocity_qp_matrices(
  const TrajectoryPoints & reference_trajectory, const TrajectoryPoints & current_trajectory,
  const std::vector<size_t> & constrained_indices,
  [[maybe_unused]] const Odometry & current_odometry, Eigen::MatrixXd & h_matrix,
  Eigen::MatrixXd & a_matrix, std::vector<double> & f_vec, std::vector<double> & l_vec,
  std::vector<double> & u_vec, const double max_velocity_limit) const
{
  const int n_points = static_cast<int>(reference_trajectory.size());
  const double dt = velocity_params_.qp_time_step_s;
  const double dt_sq = dt * dt;
  const double w_jerk = velocity_params_.qp_weight_jerk;

  h_matrix = Eigen::MatrixXd::Zero(n_points, n_points);
  f_vec.resize(n_points, 0.0);

  for (int i = 0; i < n_points - 2; ++i) {
    const double coeff = w_jerk / (dt_sq * dt_sq);

    h_matrix(i, i) += coeff;
    h_matrix(i, i + 1) += -2.0 * coeff;
    h_matrix(i, i + 2) += coeff;

    h_matrix(i + 1, i) += -2.0 * coeff;
    h_matrix(i + 1, i + 1) += 4.0 * coeff;
    h_matrix(i + 1, i + 2) += -2.0 * coeff;

    h_matrix(i + 2, i) += coeff;
    h_matrix(i + 2, i + 1) += -2.0 * coeff;
    h_matrix(i + 2, i + 2) += coeff;
  }

  const auto fidelity_weights = compute_fidelity_weights(reference_trajectory, constrained_indices);

  for (int i = 0; i < n_points; ++i) {
    const double w_i = fidelity_weights[i];
    const double v_ref = reference_trajectory[i].longitudinal_velocity_mps;

    h_matrix(i, i) += w_i;
    f_vec[i] = -w_i * v_ref;
  }

  l_vec.clear();
  u_vec.clear();

  int constraint_idx = 0;
  int total_constraints = constrained_indices.size() + n_points;

  if (velocity_params_.qp_enforce_acceleration_bounds) {
    total_constraints += (n_points - 1);
  }

  a_matrix = Eigen::MatrixXd::Zero(total_constraints, n_points);
  l_vec.reserve(total_constraints);
  u_vec.reserve(total_constraints);

  for (const auto idx : constrained_indices) {
    if (static_cast<int>(idx) < n_points) {
      a_matrix(constraint_idx, static_cast<int>(idx)) = 1.0;
      l_vec.push_back(0.0);
      u_vec.push_back(current_trajectory[idx].longitudinal_velocity_mps);
      constraint_idx++;
    }
  }

  for (int i = 0; i < n_points; ++i) {
    a_matrix(constraint_idx, i) = 1.0;
    l_vec.push_back(0.0);
    u_vec.push_back(max_velocity_limit);
    constraint_idx++;
  }

  if (velocity_params_.qp_enforce_acceleration_bounds) {
    const double a_max = velocity_params_.qp_max_acceleration_mps2;
    const double a_min = velocity_params_.qp_min_acceleration_mps2;

    for (int i = 0; i < n_points - 1; ++i) {
      a_matrix(constraint_idx, i) = -1.0;
      a_matrix(constraint_idx, i + 1) = 1.0;
      l_vec.push_back(a_min * dt);
      u_vec.push_back(a_max * dt);
      constraint_idx++;
    }
  }
}

bool TrajectoryVelocityOptimizer::optimize_velocity_with_qp(
  const TrajectoryPoints & reference_trajectory, TrajectoryPoints & output_trajectory,
  const std::vector<size_t> & constrained_indices, const Odometry & current_odometry,
  const double max_velocity_limit)
{
  const int n_points = static_cast<int>(reference_trajectory.size());

  if (n_points < 3) {
    RCLCPP_DEBUG(
      get_node_ptr()->get_logger(), "Trajectory too short for QP velocity optimization (n=%d)",
      n_points);
    return false;
  }

  Eigen::MatrixXd h_matrix;
  Eigen::MatrixXd a_matrix;
  std::vector<double> f_vec;
  std::vector<double> l_vec;
  std::vector<double> u_vec;

  prepare_velocity_qp_matrices(
    reference_trajectory, output_trajectory, constrained_indices, current_odometry, h_matrix,
    a_matrix, f_vec, l_vec, u_vec, max_velocity_limit);

  autoware::osqp_interface::OSQPInterface osqp_solver(velocity_params_.qp_osqp_eps_abs, true);
  osqp_solver.updateEpsRel(velocity_params_.qp_osqp_eps_rel);
  osqp_solver.updateMaxIter(velocity_params_.qp_osqp_max_iter);
  osqp_solver.updateVerbose(velocity_params_.qp_osqp_verbose);

  auto result = osqp_solver.optimize(h_matrix, a_matrix, f_vec, l_vec, u_vec);

  if (result.solution_status != 1) {
    RCLCPP_ERROR(
      get_node_ptr()->get_logger(),
      "QP velocity optimization FAILED! Status: %d (%s), Iterations: %d, n_points=%d",
      result.solution_status, osqp_solver.getStatusMessage().c_str(), result.iteration_status,
      n_points);
    return false;
  }

  const auto has_nan = std::any_of(
    result.primal_solution.begin(), result.primal_solution.end(),
    [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(
      get_node_ptr()->get_logger(), "QP velocity optimization: Solution contains NaN values");
    return false;
  }

  for (int i = 0; i < n_points; ++i) {
    output_trajectory[i].longitudinal_velocity_mps = static_cast<float>(result.primal_solution[i]);
  }

  autoware::trajectory_optimizer::utils::recalculate_longitudinal_acceleration(
    output_trajectory, true, velocity_params_.qp_time_step_s);

  motion_utils::calculate_time_from_start(output_trajectory, current_odometry.pose.pose.position);

  double max_jerk = 0.0;
  for (size_t i = 0; i < output_trajectory.size() - 1; ++i) {
    const double jerk = std::abs(
      (output_trajectory[i + 1].acceleration_mps2 - output_trajectory[i].acceleration_mps2) /
      velocity_params_.qp_time_step_s);
    max_jerk = std::max(max_jerk, jerk);
  }

  RCLCPP_DEBUG_THROTTLE(
    get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 5000,
    "QP velocity optimization: n_points=%d, constrained_pts=%zu, iters=%d, "
    "v=[%.2f, %.2f] m/s, max_jerk=%.2f m/s³",
    n_points, constrained_indices.size(), result.iteration_status,
    output_trajectory.front().longitudinal_velocity_mps,
    output_trajectory.back().longitudinal_velocity_mps, max_jerk);

  return true;
}

void TrajectoryVelocityOptimizer::set_up_params()
{
  auto node_ptr = get_node_ptr();
  using autoware_utils_rclcpp::get_or_declare_parameter;

  velocity_params_.default_max_velocity_mps =
    get_or_declare_parameter<double>(*node_ptr, "max_vel");

  velocity_params_.limit_speed =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.limit_speed");
  velocity_params_.limit_lateral_acceleration = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_velocity_optimizer.limit_lateral_acceleration");
  velocity_params_.smooth_velocities =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.smooth_velocities");

  velocity_params_.max_lateral_accel_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.max_lateral_accel_mps2");

  velocity_params_.qp_weight_jerk =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_velocity_optimizer.qp_weight_jerk");
  velocity_params_.qp_base_fidelity_weight = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.qp_base_fidelity_weight");
  velocity_params_.qp_time_step_s =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_velocity_optimizer.qp_time_step_s");
  velocity_params_.qp_jerk_threshold_mps3 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.qp_jerk_threshold_mps3");
  velocity_params_.qp_osqp_eps_abs =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_velocity_optimizer.qp_osqp_eps_abs");
  velocity_params_.qp_osqp_eps_rel =
    get_or_declare_parameter<double>(*node_ptr, "trajectory_velocity_optimizer.qp_osqp_eps_rel");
  velocity_params_.qp_osqp_max_iter =
    get_or_declare_parameter<int>(*node_ptr, "trajectory_velocity_optimizer.qp_osqp_max_iter");
  velocity_params_.qp_osqp_verbose =
    get_or_declare_parameter<bool>(*node_ptr, "trajectory_velocity_optimizer.qp_osqp_verbose");
  velocity_params_.qp_max_acceleration_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.qp_max_acceleration_mps2");
  velocity_params_.qp_min_acceleration_mps2 = get_or_declare_parameter<double>(
    *node_ptr, "trajectory_velocity_optimizer.qp_min_acceleration_mps2");
  velocity_params_.qp_enforce_acceleration_bounds = get_or_declare_parameter<bool>(
    *node_ptr, "trajectory_velocity_optimizer.qp_enforce_acceleration_bounds");
}

rcl_interfaces::msg::SetParametersResult TrajectoryVelocityOptimizer::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils_rclcpp::update_param;

  update_param(
    parameters, "trajectory_velocity_optimizer.limit_speed", velocity_params_.limit_speed);
  update_param(
    parameters, "trajectory_velocity_optimizer.limit_lateral_acceleration",
    velocity_params_.limit_lateral_acceleration);
  update_param(
    parameters, "trajectory_velocity_optimizer.smooth_velocities",
    velocity_params_.smooth_velocities);

  update_param(
    parameters, "trajectory_velocity_optimizer.max_lateral_accel_mps2",
    velocity_params_.max_lateral_accel_mps2);

  update_param(
    parameters, "trajectory_velocity_optimizer.qp_weight_jerk", velocity_params_.qp_weight_jerk);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_base_fidelity_weight",
    velocity_params_.qp_base_fidelity_weight);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_time_step_s", velocity_params_.qp_time_step_s);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_jerk_threshold_mps3",
    velocity_params_.qp_jerk_threshold_mps3);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_osqp_eps_abs", velocity_params_.qp_osqp_eps_abs);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_osqp_eps_rel", velocity_params_.qp_osqp_eps_rel);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_osqp_max_iter",
    velocity_params_.qp_osqp_max_iter);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_osqp_verbose", velocity_params_.qp_osqp_verbose);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_max_acceleration_mps2",
    velocity_params_.qp_max_acceleration_mps2);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_min_acceleration_mps2",
    velocity_params_.qp_min_acceleration_mps2);
  update_param(
    parameters, "trajectory_velocity_optimizer.qp_enforce_acceleration_bounds",
    velocity_params_.qp_enforce_acceleration_bounds);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

}  // namespace autoware::trajectory_optimizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer,
  autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase)
