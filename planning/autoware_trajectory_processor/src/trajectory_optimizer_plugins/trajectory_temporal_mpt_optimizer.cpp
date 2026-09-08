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

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_temporal_mpt_optimizer.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_processor::plugin
{
namespace
{
/** Overlay MPC state (x, y, psi, v) and steering from u[1] on reference points for debug
 * visualization. Used when acados status != 0 so we still publish the solver's last iterate
 * (reference alone has no optimized steering). */
TrajectoryPoints trajectory_from_solution_overlay(
  const TrajectoryPoints & reference, const temporal_mpt::AcadosSolution & solution, size_t n_out)
{
  TrajectoryPoints out;
  const size_t n = std::min({n_out, reference.size(), static_cast<size_t>(temporal_mpt::N + 1)});
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint p = reference[i];
    p.pose.position.x = solution.xtraj[i][0];
    p.pose.position.y = solution.xtraj[i][1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, solution.xtraj[i][2]);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.longitudinal_velocity_mps = static_cast<float>(std::max(0.0, solution.xtraj[i][3]));
    p.acceleration_mps2 = static_cast<float>(solution.xtraj[i][4]);
    p.front_wheel_angle_rad = static_cast<float>(solution.xtraj[i][5]);
    out.push_back(std::move(p));
  }
  if (out.size() >= 2) {
    const double v0 = std::max(0.0, solution.xtraj[0][3]);
    const double v1 = std::max(0.0, solution.xtraj[1][3]);
    out[0].longitudinal_velocity_mps = static_cast<float>(std::max(v0, v1));
  }
  return out;
}

std::string expand_user_path_string(const std::string & p)
{
  if (p.empty()) {
    return p;
  }
  if (p.front() == '~' && (p.size() == 1 || p[1] == '/')) {
    if (const char * home = std::getenv("HOME")) {
      return std::string(home) + p.substr(1);
    }
  }
  return p;
}
}  // namespace

void TrajectoryTemporalMPTOptimizer::set_mpt_params(
  const trajectory_processor_params::Params::TrajectoryTemporalMptOptimizer & params)
{
  mpt_params_.min_points_for_optimization =
    static_cast<size_t>(std::max<int64_t>(2, params.min_points_for_optimization));
  mpt_params_.tau_a = std::max(1.0e-4, params.acc_time_constant);
  mpt_params_.tau_d = std::max(1.0e-4, params.steer_time_constant);
  mpt_params_.max_steer_rate = std::max(1.0e-6, params.max_steer_rate);
  mpt_params_.use_previous_solution_warm_start = params.use_previous_solution_warm_start;
  mpt_params_.enable_debug_info = params.enable_debug_info;
  mpt_params_.publish_debug_topics = params.publish_debug_topics;
  mpt_params_.write_replay_fixture = params.write_replay_fixture;
  mpt_params_.replay_fixture_directory = params.replay_fixture_directory;
  mpt_params_.log_replay_fixture_to_console = params.log_replay_fixture_to_console;
  mpt_params_.reroute_output = params.reroute_output;
}

void TrajectoryTemporalMPTOptimizer::on_initialize(const TrajectoryProcessorParams & params)
{
  auto node_ptr = get_node_ptr();

  // SQP max_iter / tol: from codegen (generators/path_tracking_mpc_temporal.py → acados_ocp.json),
  // applied inside kinematic_bicycle_temporal_acados_create — not overridden in C++.

  enabled_ = params.use_temporal_mpt_optimizer;
  set_mpt_params(params.trajectory_temporal_mpt_optimizer);

  // Bicycle lf/lr from vehicle_info: CG at geometric box center (same as MPPI
  // ego_axle_to_box_center).
  {
    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*node_ptr).getVehicleInfo();
    const double wb = std::max(1.0e-3, vehicle_info.wheel_base_m);
    double lr = 0.5 * vehicle_info.vehicle_length_m - vehicle_info.rear_overhang_m;
    lr = std::clamp(lr, 1.0e-3, wb - 1.0e-3);
    mpt_params_.lr = lr;
    mpt_params_.lf = wb - lr;
  }
  RCLCPP_INFO(
    node_ptr->get_logger(),
    "Temporal MPT: lf=%.3f m lr=%.3f m tau_a=%.3f s tau_d=%.3f s max_steer_rate=%.3f rad/s",
    mpt_params_.lf, mpt_params_.lr, mpt_params_.tau_a, mpt_params_.tau_d,
    mpt_params_.max_steer_rate);

  if (mpt_params_.write_replay_fixture && !mpt_params_.replay_fixture_directory.empty()) {
    RCLCPP_INFO(
      node_ptr->get_logger(),
      "Temporal MPT: writing replay fixtures to %s (feed files to "
      "generators/example_trajectory_file_xyv.py)",
      expand_user_path_string(mpt_params_.replay_fixture_directory).c_str());
  }

  create_or_reset_solver();
}

void TrajectoryTemporalMPTOptimizer::update_params(const TrajectoryProcessorParams & params)
{
  const bool warm_start_was_enabled = mpt_params_.use_previous_solution_warm_start;
  enabled_ = params.use_temporal_mpt_optimizer;
  set_mpt_params(params.trajectory_temporal_mpt_optimizer);
  if (warm_start_was_enabled && !mpt_params_.use_previous_solution_warm_start) {
    have_prev_solution_ = false;
  }
}

ProcessingResult TrajectoryTemporalMPTOptimizer::process(
  TrajectoryPoints & traj_points, TrajectoryProcessorData & data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());
  if (!enabled_ || !data.current_odometry) {
    return ProcessingResult::Unchanged;
  }

  if (!acados_interface_) {
    create_or_reset_solver();
  }
  if (!acados_interface_) {
    return ProcessingResult::Unchanged;
  }

  const size_t min_points = std::max<size_t>(2, mpt_params_.min_points_for_optimization);
  if (traj_points.size() < min_points) {
    return ProcessingResult::Unchanged;
  }

  const TrajectoryPoints reference_snapshot = traj_points;

  const std::array<double, temporal_mpt::NP> model_params = {
    mpt_params_.lf, mpt_params_.lr, mpt_params_.tau_a, mpt_params_.tau_d};
  acados_interface_->setParametersAllStages(model_params);
  acados_interface_->setSteerRateLimit(mpt_params_.max_steer_rate);

  // Initial state: pose/speed from traj[0]; applied a/δ from that point (or 0).
  const auto & p0 = traj_points.front();
  const std::array<double, temporal_mpt::NX> x0 = {
    p0.pose.position.x,
    p0.pose.position.y,
    tf2::getYaw(p0.pose.orientation),
    std::max(0.0, static_cast<double>(p0.longitudinal_velocity_mps)),
    static_cast<double>(p0.acceleration_mps2),
    static_cast<double>(p0.front_wheel_angle_rad)};

  // Horizon references use the incoming trajectory as a **time-ordered discrete sequence** (planner
  // sample period is implicit in the message): start_idx = index closest to x0 (typically 0 when x0
  // is traj_points[0]), then start_idx+k for stage k. No arc-length resampling or
  // re-parameterization.
  const size_t n_pts = traj_points.size();
  size_t start_idx = 0;
  {
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n_pts; ++i) {
      const double dx = traj_points[i].pose.position.x - x0[0];
      const double dy = traj_points[i].pose.position.y - x0[1];
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        start_idx = i;
      }
    }
  }

  // Shift reference yaw by k*2π so the path heading at start_idx matches x0 psi in LINEAR_LS (same
  // idea as generators/utils.py run_closed_loop_mpc / solve_autoware_temporal_mpc). Avoids false
  // ~2π heading error when planner yaw branches differ.
  constexpr double two_pi = 2.0 * M_PI;
  const double yaw_at_start = tf2::getYaw(traj_points.at(start_idx).pose.orientation);
  const double psi_bias = std::round((x0[2] - yaw_at_start) / two_pi) * two_pi;

  // Ego-centered XY for the NLP (same as generators/utils.py translate_xy_horizon_to_local):
  // subtract x0[:2] from all position references and from the initial state passed to the solver,
  // then add the offset back to the state trajectory (unshift_sol_x_xy).
  const double x_off = x0[0];
  const double y_off = x0[1];

  // World-frame yref [x,y,psi,v,a,δ,a_cmd,δ_cmd]. Path-frame Vx rotates x,y.
  const size_t max_k = temporal_mpt::N;
  for (size_t k = 0; k < max_k; ++k) {
    if (k == 0) {
      // Anchor pose/velocity on ego; actuator refs match k>=1 (see path_tracking_solver.cpp).
      const std::array<double, temporal_mpt::NY> yref = {x0[0] - x_off, x0[1] - y_off, x0[2], x0[3],
                                                         0.0,           0.0,           0.0,   0.0};
      acados_interface_->setStageReference(static_cast<int>(k), yref);
      continue;
    }
    const size_t idx = std::min(start_idx + k, n_pts - 1);
    const auto & p = traj_points.at(idx);
    const double yaw = tf2::getYaw(p.pose.orientation) + psi_bias;
    const double v_ref = std::max(0.0, static_cast<double>(p.longitudinal_velocity_mps));
    const std::array<double, temporal_mpt::NY> yref = {
      p.pose.position.x - x_off, p.pose.position.y - y_off, yaw, v_ref, 0.0, 0.0, 0.0, 0.0};
    acados_interface_->setStageReference(static_cast<int>(k), yref);
  }

  const size_t terminal_idx = std::min(start_idx + temporal_mpt::N, n_pts - 1);
  const auto & terminal_point = traj_points.at(terminal_idx);
  const double terminal_yaw = tf2::getYaw(terminal_point.pose.orientation) + psi_bias;
  const double terminal_v_ref =
    std::max(0.0, static_cast<double>(terminal_point.longitudinal_velocity_mps));
  acados_interface_->setTerminalReference(
    {terminal_point.pose.position.x - x_off, terminal_point.pose.position.y - y_off, terminal_yaw,
     terminal_v_ref, 0.0, 0.0});

  const std::array<double, temporal_mpt::NX> x0_local = {x0[0] - x_off, x0[1] - y_off, x0[2],
                                                         x0[3],         x0[4],         x0[5]};

  // Receding-horizon warm-start from previous successful solve (shifted by one stage).
  if (mpt_params_.use_previous_solution_warm_start && have_prev_solution_) {
    std::array<std::array<double, temporal_mpt::NX>, temporal_mpt::N + 1> x_guess{};
    std::array<std::array<double, temporal_mpt::NU>, temporal_mpt::N> u_guess{};
    for (size_t k = 0; k < temporal_mpt::N; ++k) {
      const size_t src = std::min(k + 1, temporal_mpt::N - 1);
      u_guess[k] = prev_u_[src];
    }
    x_guess[0] = x0_local;
    for (size_t k = 1; k <= temporal_mpt::N; ++k) {
      const size_t src = std::min(k + 1, temporal_mpt::N);
      auto xw = prev_x_world_[src];
      xw[0] -= x_off;
      xw[1] -= y_off;
      x_guess[k] = xw;
    }
    acados_interface_->setWarmStartTrajectory(x_guess, u_guess);
  } else {
    acados_interface_->setWarmStart(x0_local, {0.0, 0.0});
  }

  auto solution = acados_interface_->getControl(x0_local);

  for (size_t i = 0; i <= temporal_mpt::N; ++i) {
    solution.xtraj[i][0] += x_off;
    solution.xtraj[i][1] += y_off;
  }
  if (solution.status != 0) {
    write_temporal_mpt_replay_fixture(x0, reference_snapshot, solution.status, "failed");

    if (mpt_params_.enable_debug_info) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 2000,
        "Temporal MPT acados solve failed with status %d", solution.status);

      rclcpp::Logger logger = get_node_ptr()->get_logger();
      RCLCPP_INFO(logger, "Temporal MPT optimize: plugin=%s", get_name().c_str());

      RCLCPP_INFO(
        logger,
        "x0 (first trajectory point): x=%.6f y=%.6f yaw=%.6f v=%.6f | MPC start_idx=%zu "
        "terminal_idx=%zu",
        x0[0], x0[1], x0[2], x0[3], start_idx, terminal_idx);
      RCLCPP_INFO(
        logger, "odom twist linear (map/body per msg): x=%.6f y=%.6f z=%.6f",
        data.current_odometry->twist.twist.linear.x, data.current_odometry->twist.twist.linear.y,
        data.current_odometry->twist.twist.linear.z);

      std::ostringstream oss;
      oss << "trajectory points (" << traj_points.size() << "): [";
      for (size_t i = 0; i < traj_points.size(); ++i) {
        const auto & p = traj_points[i];
        const double pyaw = tf2::getYaw(p.pose.orientation);
        oss << "(" << p.pose.position.x << ", " << p.pose.position.y << ", " << pyaw << ")";
        if (i + 1 < traj_points.size()) {
          oss << ", ";
        }
      }
      oss << "]";

      RCLCPP_INFO(logger, "%s", oss.str().c_str());
    }

    if (mpt_params_.publish_debug_topics) {
      const size_t n_out =
        std::min(reference_snapshot.size(), static_cast<size_t>(temporal_mpt::N + 1));
      const TrajectoryPoints debug_traj =
        trajectory_from_solution_overlay(reference_snapshot, solution, n_out);
      publish_temporal_mpt_debug_io(
        reference_snapshot, *data.current_odometry, debug_traj, debug_traj.size(), solution.status,
        &solution);
    }
    return ProcessingResult::Unchanged;
  } else {
    if (mpt_params_.enable_debug_info) {
      RCLCPP_INFO_THROTTLE(
        get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 2000,
        "Temporal MPT acados solve succeeded with status %d", solution.status);
    }
    if (mpt_params_.use_previous_solution_warm_start) {
      prev_u_ = solution.utraj;
      prev_x_world_ = solution.xtraj;
      have_prev_solution_ = true;
    }
  }

  // When reroute_output is true, write the optimized state to a copy only (debug topics) and
  // leave traj_points unchanged for downstream plugins.
  TrajectoryPoints rerouted_output;
  TrajectoryPoints * apply_traj = &traj_points;
  if (mpt_params_.reroute_output) {
    rerouted_output = traj_points;
    apply_traj = &rerouted_output;
  }

  const size_t n_apply = std::min(apply_traj->size(), static_cast<size_t>(temporal_mpt::N + 1));
  for (size_t i = 0; i < n_apply; ++i) {
    auto & p = apply_traj->at(i);
    p.pose.position.x = solution.xtraj[i][0];
    p.pose.position.y = solution.xtraj[i][1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, solution.xtraj[i][2]);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    p.longitudinal_velocity_mps = std::max(0.0, solution.xtraj[i][3]);
    p.acceleration_mps2 = static_cast<float>(solution.xtraj[i][4]);
    p.front_wheel_angle_rad = static_cast<float>(solution.xtraj[i][5]);
  }

  if (mpt_params_.publish_debug_topics) {
    publish_temporal_mpt_debug_io(
      reference_snapshot, *data.current_odometry, *apply_traj, apply_traj->size(), 0, &solution);
  }
  return mpt_params_.reroute_output ? ProcessingResult::Unchanged : ProcessingResult::Modified;
}

void TrajectoryTemporalMPTOptimizer::write_temporal_mpt_replay_fixture(
  const std::array<double, temporal_mpt::NX> & x0, const TrajectoryPoints & reference_trajectory,
  const int acados_status, const char * tag)
{
  const std::string dir_raw = mpt_params_.replay_fixture_directory;
  const bool want_file = mpt_params_.write_replay_fixture && !dir_raw.empty();
  const bool want_console = mpt_params_.log_replay_fixture_to_console;
  if (!want_file && !want_console) {
    return;
  }

  std::ostringstream body;
  body
    << "# example_trajectory_file_xyv.py: python3 generators/example_trajectory_file_xyv.py \\\n";
  body << "#   --reference-file <this_file> --N " << temporal_mpt::N << " --dt 0.1\n";
  body << "# x0: x[m] y[m] yaw[rad] v[m/s] a[m/s^2] delta[rad]; then x,y,yaw,v_ref per line.\n";
  body << std::setprecision(std::numeric_limits<double>::max_digits10);
  body << "x0: " << x0[0] << " " << x0[1] << " " << x0[2] << " " << x0[3] << " " << x0[4] << " "
       << x0[5] << "\n";
  for (const auto & p : reference_trajectory) {
    const double pyaw = tf2::getYaw(p.pose.orientation);
    const double v_ref = std::max(0.0, static_cast<double>(p.longitudinal_velocity_mps));
    body << p.pose.position.x << ", " << p.pose.position.y << ", " << pyaw << ", " << v_ref << "\n";
  }
  const std::string text = body.str();

  rclcpp::Logger logger = get_node_ptr()->get_logger();

  if (want_console) {
    RCLCPP_ERROR(
      logger, "Temporal MPT replay fixture (%s, status=%d):\n%s", tag, acados_status, text.c_str());
  }

  if (!want_file) {
    return;
  }

  const std::string dir_exp = expand_user_path_string(dir_raw);
  std::error_code ec;
  std::filesystem::create_directories(dir_exp, ec);
  if (ec) {
    RCLCPP_WARN(
      logger, "Temporal MPT: could not create replay fixture directory %s: %s", dir_exp.c_str(),
      ec.message().c_str());
    return;
  }

  const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  const std::filesystem::path out_path =
    std::filesystem::path(dir_exp) /
    ("temporal_mpt_replay_" + std::string(tag) + "_" + std::to_string(wall_ms) + "_status" +
     std::to_string(acados_status) + ".txt");

  std::ofstream out(out_path);
  if (!out) {
    RCLCPP_WARN(logger, "Temporal MPT: could not open replay fixture %s", out_path.c_str());
    return;
  }
  out << text;
  RCLCPP_WARN(
    logger, "Temporal MPT: wrote replay fixture for example_trajectory_file_xyv.py: %s",
    out_path.c_str());
}

void TrajectoryTemporalMPTOptimizer::create_or_reset_solver()
{
  acados_interface_ = std::make_unique<temporal_mpt::AcadosInterface>();
  have_prev_solution_ = false;
}

void TrajectoryTemporalMPTOptimizer::ensure_debug_publishers()
{
  if (!mpt_params_.publish_debug_topics || debug_input_trajectory_pub_) {
    return;
  }
  rclcpp::Node * const n = get_node_ptr();
  // Best effort (typical for high-rate planning). Subscribers must use matching reliability
  // (default rclpy reliable will not receive these messages).
  const auto qos = rclcpp::QoS(10).best_effort();
  debug_input_trajectory_pub_ = n->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/temporal_mpt/input/reference_trajectory", qos);
  debug_input_initial_state_pub_ =
    n->create_publisher<nav_msgs::msg::Odometry>("~/debug/temporal_mpt/input/initial_state", qos);
  debug_output_trajectory_pub_ = n->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/temporal_mpt/output/trajectory", qos);
  debug_solve_status_pub_ =
    n->create_publisher<std_msgs::msg::Int32>("~/debug/temporal_mpt/output/solve_status", qos);
  debug_control_accel_pub_ = n->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug/temporal_mpt/output/control_acceleration_mps2", qos);
  debug_control_delta_cmd_pub_ = n->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug/temporal_mpt/output/control_delta_cmd_rad", qos);
}

void TrajectoryTemporalMPTOptimizer::publish_temporal_mpt_debug_io(
  const TrajectoryPoints & reference_before, const nav_msgs::msg::Odometry & initial_odom,
  const TrajectoryPoints & trajectory_after, const size_t output_point_count,
  const int acados_status, const temporal_mpt::AcadosSolution * mpc_solution)
{
  ensure_debug_publishers();
  if (!debug_input_trajectory_pub_) {
    return;
  }

  rclcpp::Node * const node = get_node_ptr();
  std_msgs::msg::Header header;
  header.stamp = node->now();
  header.frame_id = initial_odom.header.frame_id.empty() ? "map" : initial_odom.header.frame_id;

  autoware_planning_msgs::msg::Trajectory input_traj;
  input_traj.header = header;
  input_traj.points = reference_before;

  autoware_planning_msgs::msg::Trajectory output_traj;
  output_traj.header = header;
  const size_t n = std::min(output_point_count, trajectory_after.size());
  if (n > 0) {
    output_traj.points.assign(trajectory_after.begin(), trajectory_after.begin() + n);
  }

  std_msgs::msg::Int32 status_msg;
  status_msg.data = acados_status;

  // Publish u = [a_long, delta] for every stage whenever the solver returned a trajectory,
  // including failed solves (status != 0): values are the last SQP iterate and are useful for
  // debug.
  std_msgs::msg::Float64MultiArray accel_msg;
  std_msgs::msg::Float64MultiArray delta_cmd_msg;
  if (mpc_solution != nullptr) {
    accel_msg.data.reserve(temporal_mpt::N);
    delta_cmd_msg.data.reserve(temporal_mpt::N);
    for (size_t k = 0; k < temporal_mpt::N; ++k) {
      accel_msg.data.push_back(mpc_solution->utraj[k][0]);
      delta_cmd_msg.data.push_back(mpc_solution->utraj[k][1]);
    }
  }

  debug_input_trajectory_pub_->publish(std::move(input_traj));
  debug_input_initial_state_pub_->publish(initial_odom);
  debug_output_trajectory_pub_->publish(std::move(output_traj));
  debug_solve_status_pub_->publish(std::move(status_msg));
  if (debug_control_accel_pub_) {
    debug_control_accel_pub_->publish(std::move(accel_msg));
  }
  if (debug_control_delta_cmd_pub_) {
    debug_control_delta_cmd_pub_->publish(std::move(delta_cmd_msg));
  }
}

}  // namespace autoware::trajectory_processor::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_processor::plugin::TrajectoryTemporalMPTOptimizer,
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase)
