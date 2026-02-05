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

// NOLINTNEXTLINE
#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_
// NOLINTNEXTLINE
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_
#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"

#include <Eigen/Dense>
#include <autoware/osqp_interface/osqp_interface.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/velocity_limit.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware_internal_planning_msgs::msg::VelocityLimit;

struct TrajectoryVelocityOptimizerParams
{
  double default_max_velocity_mps{8.33};
  bool limit_speed{true};
  bool limit_lateral_acceleration{false};
  bool smooth_velocities{false};
  double max_lateral_accel_mps2{1.5};

  double qp_weight_jerk{50.0};
  double qp_base_fidelity_weight{1.0};
  double qp_time_step_s{0.1};
  double qp_jerk_threshold_mps3{5.0};
  double qp_osqp_eps_abs{1e-4};
  double qp_osqp_eps_rel{1e-4};
  int qp_osqp_max_iter{4000};
  bool qp_osqp_verbose{false};
  double qp_max_acceleration_mps2{2.0};
  double qp_min_acceleration_mps2{-3.0};
  bool qp_enforce_acceleration_bounds{true};
};

class TrajectoryVelocityOptimizer : public TrajectoryOptimizerPluginBase
{
public:
  TrajectoryVelocityOptimizer() = default;
  ~TrajectoryVelocityOptimizer() = default;

  void initialize(
    const std::string & name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> & time_keeper) override;
  void optimize_trajectory(
    TrajectoryPoints & traj_points, const TrajectoryOptimizerParams & params,
    const TrajectoryOptimizerData & data) override;
  void set_up_params() override;
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters) override;

private:
  TrajectoryVelocityOptimizerParams velocity_params_;
  std::shared_ptr<autoware_utils_rclcpp::InterProcessPollingSubscriber<VelocityLimit>>
    sub_planning_velocity_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;

  bool is_smoothing_needed(const TrajectoryPoints & trajectory) const;

  std::vector<size_t> identify_constrained_points(
    const TrajectoryPoints & before_constraints, const TrajectoryPoints & after_constraints) const;

  std::vector<double> compute_fidelity_weights(
    const TrajectoryPoints & trajectory, const std::vector<size_t> & constrained_indices) const;

  bool optimize_velocity_with_qp(
    const TrajectoryPoints & reference_trajectory, TrajectoryPoints & output_trajectory,
    const std::vector<size_t> & constrained_indices, const Odometry & current_odometry,
    const double max_velocity_limit);

  void prepare_velocity_qp_matrices(
    const TrajectoryPoints & reference_trajectory, const TrajectoryPoints & current_trajectory,
    const std::vector<size_t> & constrained_indices, const Odometry & current_odometry,
    Eigen::MatrixXd & h_matrix, Eigen::MatrixXd & a_matrix, std::vector<double> & f_vec,
    std::vector<double> & l_vec, std::vector<double> & u_vec,
    const double max_velocity_limit) const;
};
}  // namespace autoware::trajectory_optimizer::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_PLUGINS__TRAJECTORY_VELOCITY_OPTIMIZER_HPP_
