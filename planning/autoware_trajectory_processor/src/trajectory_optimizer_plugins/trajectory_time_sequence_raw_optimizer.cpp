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

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_time_sequence_raw_optimizer.hpp"

#include "autoware/trajectory_processor/trajectory_modifier_utils/utils.hpp"
#include "autoware/trajectory_processor/time_sequence_raw/acados_solver_wrapper.hpp"

#include <rclcpp/logging.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/header.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace autoware::trajectory_processor::plugin
{
namespace
{
autoware_planning_msgs::msg::Trajectory to_trajectory_msg(
  const TrajectoryPoints & points, const std_msgs::msg::Header & header)
{
  autoware_planning_msgs::msg::Trajectory trajectory;
  trajectory.header = header;
  trajectory.points = points;
  return trajectory;
}

time_sequence_raw::TrajectoryOptimizationParams to_opt_params(
  const trajectory_processor_params::Params::TimeSequenceRawOptimizer & p)
{
  time_sequence_raw::TrajectoryOptimizationParams out;
  out.weight_longitudinal = p.weight_longitudinal;
  out.weight_lateral = p.weight_lateral;
  out.weight_yaw = p.weight_yaw;
  out.weight_jerk = p.weight_jerk;
  out.weight_steering_rate = p.weight_steering_rate;
  out.terminal_weight_scale = p.terminal_weight_scale;
  out.min_velocity_mps = p.min_velocity_mps;
  out.max_velocity_mps = p.max_velocity_mps;
  out.min_acceleration_mps2 = p.min_acceleration_mps2;
  out.max_acceleration_mps2 = p.max_acceleration_mps2;
  out.min_jerk_mps3 = p.min_jerk_mps3;
  out.max_jerk_mps3 = p.max_jerk_mps3;
  out.max_steering_rate_rps = p.max_steering_rate_rps;
  out.max_lateral_acceleration_mps2 = p.max_lateral_acceleration_mps2;
  out.max_sqp_iterations = static_cast<int>(p.max_sqp_iterations);
  return out;
}

time_sequence_raw::RoadBorderAvoidanceParams to_border_params(
  const trajectory_processor_params::Params::RoadBorderAvoidance & p)
{
  time_sequence_raw::RoadBorderAvoidanceParams out;
  out.enable = p.enable;
  out.footprint_margin_m = p.footprint_margin_m;
  out.search_radius_m = p.search_radius_m;
  out.shift_step_m = p.shift_step_m;
  out.max_lateral_shift_m = p.max_lateral_shift_m;
  out.propagate_shift = p.propagate_shift;
  return out;
}

bool is_ego_stopped(
  const TrajectoryProcessorData & data, const double stopped_velocity_threshold_mps)
{
  if (!data.current_odometry) {
    return false;
  }
  return std::abs(data.current_odometry->twist.twist.linear.x) <= stopped_velocity_threshold_mps;
}

bool is_stopped_reference_trajectory(
  const TrajectoryPoints & traj_points, const double stopped_velocity_threshold_mps,
  const double stopped_trajectory_max_length_m)
{
  if (traj_points.empty()) {
    return false;
  }

  if (
    utils::calculate_distance_to_last_point(traj_points, traj_points.front().pose) >
    stopped_trajectory_max_length_m) {
    return false;
  }

  const auto max_abs_velocity_mps = std::max_element(
    traj_points.begin(), traj_points.end(),
    [](const TrajectoryPoint & lhs, const TrajectoryPoint & rhs) {
      return std::abs(lhs.longitudinal_velocity_mps) < std::abs(rhs.longitudinal_velocity_mps);
    });
  return std::abs(max_abs_velocity_mps->longitudinal_velocity_mps) <=
         stopped_velocity_threshold_mps;
}

double duration_to_sec(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
}

/// Instant speed from pose spacing. Uses time_from_start when dt is valid, else 0.1 s.
double chord_longitudinal_speed_mps(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const size_t index)
{
  using time_sequence_raw::opt_dt_s;
  if (index + 1 >= trajectory.points.size()) {
    return 0.0;
  }
  const auto & a = trajectory.points[index];
  const auto & b = trajectory.points[index + 1];
  const double dist_m = std::hypot(
    b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y);
  double dt_s = duration_to_sec(b.time_from_start) - duration_to_sec(a.time_from_start);
  if (dt_s < 1e-6) {
    dt_s = opt_dt_s;
  }
  return dist_m / dt_s;
}

bool is_near_route_goal(
  const TrajectoryProcessorData & data, const double goal_steer_zero_distance_m)
{
  if (!data.route || !data.current_odometry) {
    return false;
  }

  const auto & goal = data.route->goal_pose.position;
  const auto & ego = data.current_odometry->pose.pose.position;
  const double dx = goal.x - ego.x;
  const double dy = goal.y - ego.y;
  return std::hypot(dx, dy) < goal_steer_zero_distance_m;
}
}  // namespace

void TrajectoryTimeSequenceRawOptimizer::set_params(const TrajectoryProcessorParams & params)
{
  enabled_ = params.use_time_sequence_raw_optimizer;
  opt_params_ = to_opt_params(params.time_sequence_raw_optimizer);
  border_params_ = to_border_params(params.road_border_avoidance);
  road_border_enable_ = border_params_.enable;
  publish_debug_topics_ = params.time_sequence_raw_optimizer.publish_debug_topics;
  use_stamped_ego_state_ = params.time_sequence_raw_optimizer.use_stamped_ego_state;
  ego_state_buffer_duration_s_ = params.time_sequence_raw_optimizer.ego_state_buffer_duration_s;
  max_ego_stamp_mismatch_s_ = params.time_sequence_raw_optimizer.max_ego_stamp_mismatch_s;
  ego_buffer_.set_duration(ego_state_buffer_duration_s_);
  stopped_velocity_threshold_mps_ =
    params.time_sequence_raw_optimizer.stopped_velocity_threshold_mps;
  stopped_trajectory_max_length_m_ =
    params.time_sequence_raw_optimizer.stopped_trajectory_max_length_m;
  goal_steer_zero_enable_ = params.time_sequence_raw_optimizer.goal_steer_zero_enable;
  goal_steer_zero_distance_m_ = params.time_sequence_raw_optimizer.goal_steer_zero_distance_m;
  goal_steer_zero_requires_stopped_ =
    params.time_sequence_raw_optimizer.goal_steer_zero_requires_stopped;
}

void TrajectoryTimeSequenceRawOptimizer::on_initialize(const TrajectoryProcessorParams & params)
{
  set_params(params);
  ensure_ego_subscriptions();
  ensure_debug_publishers();
  if (enabled_) {
    ensure_optimizer();
  }
}

void TrajectoryTimeSequenceRawOptimizer::ensure_ego_subscriptions()
{
  if (odom_sub_) {
    return;
  }
  auto * node = get_node_ptr();
  const auto qos = rclcpp::QoS{50};
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", qos,
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { ego_buffer_.push_odometry(*msg); });
  accel_sub_ = node->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "~/input/acceleration", qos,
    [this](const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg) {
      ego_buffer_.push_acceleration(*msg);
    });
  steer_sub_ = node->create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
    "~/input/steering_status", qos,
    [this](const autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr msg) {
      ego_buffer_.push_steering(*msg);
    });
}

void TrajectoryTimeSequenceRawOptimizer::ingest_live_ego(const TrajectoryProcessorData & data)
{
  if (data.current_odometry) {
    ego_buffer_.push_odometry(*data.current_odometry);
  }
  if (data.current_acceleration) {
    ego_buffer_.push_acceleration(*data.current_acceleration);
  }
  if (data.current_steering) {
    ego_buffer_.push_steering(*data.current_steering);
  }
}

nav_msgs::msg::Odometry TrajectoryTimeSequenceRawOptimizer::resolve_ocp_odometry(
  const TrajectoryProcessorData & data, double & accel_mps2, std::optional<double> & steering)
{
  last_lookup_dt_s_ = 0.0;
  last_live_lag_s_ = 0.0;
  accel_mps2 = data.current_acceleration->accel.accel.linear.x;
  if (data.current_steering) {
    steering = data.current_steering->steering_tire_angle;
  }

  if (!use_stamped_ego_state_) {
    return *data.current_odometry;
  }

  const rclcpp::Time query(data.candidate_header.stamp, RCL_ROS_TIME);
  if (query.nanoseconds() == 0) {
    return *data.current_odometry;
  }

  const auto looked = ego_buffer_.lookup(query, max_ego_stamp_mismatch_s_);
  if (!looked) {
    return *data.current_odometry;
  }

  last_lookup_dt_s_ = looked->lookup_dt_s;
  last_live_lag_s_ = looked->live_lag_s;
  if (looked->has_acceleration) {
    accel_mps2 = looked->acceleration.accel.accel.linear.x;
  }
  if (looked->has_steering) {
    steering = looked->steering.steering_tire_angle;
  }

  if (
    looked->fallback_latest &&
    std::abs(looked->lookup_dt_s) > max_ego_stamp_mismatch_s_) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 1000,
      "TS stamped ego fallback to latest odom (lookup_dt=%.3fs live_lag=%.3fs). "
      "Buffer may still be filling.",
      looked->lookup_dt_s, looked->live_lag_s);
  }
  return looked->odometry;
}

void TrajectoryTimeSequenceRawOptimizer::update_params(const TrajectoryProcessorParams & params)
{
  set_params(params);
  optimizer_.reset();
  road_border_avoidance_.reset();
  cached_map_ = nullptr;
  in_stopped_regime_ = false;
  in_goal_zero_regime_ = false;
  if (enabled_) {
    ensure_optimizer();
  }
}

void TrajectoryTimeSequenceRawOptimizer::ensure_optimizer()
{
  if (optimizer_ || !context_) {
    return;
  }
  optimizer_ = std::make_unique<time_sequence_raw::TrajectoryOptimizer>(
    opt_params_, context_->vehicle_info, 8);
  road_border_avoidance_ = std::make_unique<time_sequence_raw::RoadBorderAvoidance>(
    border_params_, context_->vehicle_info);
}

void TrajectoryTimeSequenceRawOptimizer::maybe_update_map(const TrajectoryProcessorData & data)
{
  if (!road_border_avoidance_ || !data.lanelet_map) {
    return;
  }
  if (cached_map_ == data.lanelet_map.get()) {
    return;
  }
  road_border_avoidance_->set_map(*data.lanelet_map);
  cached_map_ = data.lanelet_map.get();
}

void TrajectoryTimeSequenceRawOptimizer::ensure_debug_publishers()
{
  if (!publish_debug_topics_ || debug_raw_pub_) {
    return;
  }
  auto * node = get_node_ptr();
  debug_raw_pub_ = node->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/time_sequence_raw_optimizer/raw_trajectory", rclcpp::QoS{1});
  debug_adjusted_pub_ = node->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/time_sequence_raw_optimizer/adjusted_trajectory", rclcpp::QoS{1});
  debug_shifted_count_pub_ = node->create_publisher<std_msgs::msg::Int32>(
    "~/debug/time_sequence_raw_optimizer/shifted_point_count", rclcpp::QoS{1});
  debug_solver_status_pub_ = node->create_publisher<std_msgs::msg::Int32>(
    "~/debug/time_sequence_raw_optimizer/solver_status", rclcpp::QoS{1});
  debug_solve_time_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    "~/debug/time_sequence_raw_optimizer/solve_time_ms", rclcpp::QoS{1});
  debug_optimized_pub_ = node->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/time_sequence_raw_optimizer/optimized_trajectory", rclcpp::QoS{1});
  debug_geometry_velocity_pub_ = node->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "~/debug/time_sequence_raw_optimizer/geometry_velocity_trajectory", rclcpp::QoS{1});
  debug_velocity_profile_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/debug/time_sequence_raw_optimizer/velocity_profile", rclcpp::QoS{1});
}

autoware_planning_msgs::msg::Trajectory
TrajectoryTimeSequenceRawOptimizer::make_geometry_velocity_trajectory(
  const autoware_planning_msgs::msg::Trajectory & src) const
{
  auto out = src;
  if (out.points.size() < 2) {
    return out;
  }
  for (size_t i = 0; i + 1 < out.points.size(); ++i) {
    out.points[i].longitudinal_velocity_mps =
      static_cast<float>(chord_longitudinal_speed_mps(src, i));
  }
  out.points.back().longitudinal_velocity_mps =
    out.points[out.points.size() - 2].longitudinal_velocity_mps;
  for (size_t i = 0; i + 1 < out.points.size(); ++i) {
    double dt_s = duration_to_sec(out.points[i + 1].time_from_start) -
                  duration_to_sec(out.points[i].time_from_start);
    if (dt_s < 1e-6) {
      dt_s = time_sequence_raw::opt_dt_s;
    }
    out.points[i].acceleration_mps2 = static_cast<float>(
      (out.points[i + 1].longitudinal_velocity_mps - out.points[i].longitudinal_velocity_mps) /
      dt_s);
  }
  out.points.back().acceleration_mps2 = 0.0F;
  return out;
}

void TrajectoryTimeSequenceRawOptimizer::publish_velocity_diagnostics(
  const autoware_planning_msgs::msg::Trajectory & reference,
  const autoware_planning_msgs::msg::Trajectory & optimized,
  const time_sequence_raw::OptimizationResult & result, const nav_msgs::msg::Odometry & ocp_odom)
{
  if (!publish_debug_topics_) {
    return;
  }
  last_geometry_velocity_trajectory_ = make_geometry_velocity_trajectory(reference);

  using time_sequence_raw::opt_dt_s;
  constexpr size_t k_horizon = 80;
  const double ego_v = ocp_odom.twist.twist.linear.x;
  const auto & ego_pos = ocp_odom.pose.pose.position;
  const size_t n_steps = std::min(k_horizon, reference.points.size());

  double ego_to_p0_speed_mps = 0.0;
  if (!reference.points.empty()) {
    const auto & p0 = reference.points.front().pose.position;
    ego_to_p0_speed_mps =
      std::hypot(p0.x - ego_pos.x, p0.y - ego_pos.y) / opt_dt_s;
  }
  const double opt_a0 =
    optimized.points.empty() ? 0.0 : static_cast<double>(optimized.points.front().acceleration_mps2);
  const size_t a_idx = optimized.points.size() > 10 ? 10 : 0;
  const double opt_a1s =
    optimized.points.empty() ? 0.0
                             : static_cast<double>(optimized.points[a_idx].acceleration_mps2);

  std_msgs::msg::Float64MultiArray profile;
  profile.layout.dim.resize(1);
  profile.layout.dim[0].label =
    "v0_seed,a0_seed,ego_v,max_accel_limit,weight_jerk,ego_to_p0_speed,opt_a_t0,opt_a_t1s,"
    "lookup_dt_s,live_lag_s then per-step geom_v,msg_v,opt_v";
  profile.layout.dim[0].size = 10 + 3 * static_cast<uint32_t>(n_steps);
  profile.layout.data_offset = 0;
  profile.data = {
    result.initial_speed_mps, result.initial_accel_mps2, ego_v, opt_params_.max_acceleration_mps2,
    opt_params_.weight_jerk, ego_to_p0_speed_mps, opt_a0, opt_a1s, last_lookup_dt_s_,
    last_live_lag_s_};
  for (size_t k = 0; k < n_steps; ++k) {
    const double geom_v = chord_longitudinal_speed_mps(reference, k);
    const double msg_v = static_cast<double>(reference.points[k].longitudinal_velocity_mps);
    const double opt_v =
      (k < optimized.points.size())
        ? static_cast<double>(optimized.points[k].longitudinal_velocity_mps)
        : 0.0;
    profile.data.push_back(geom_v);
    profile.data.push_back(msg_v);
    profile.data.push_back(opt_v);
  }
  if (debug_velocity_profile_pub_) {
    debug_velocity_profile_pub_->publish(profile);
  }
  if (debug_geometry_velocity_pub_) {
    debug_geometry_velocity_pub_->publish(last_geometry_velocity_trajectory_);
  }

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_node_ptr()->get_clock(), 500,
    "TS velocity diag: v0_seed=%.3f a0_meas=%.3f ego_to_p0=%.3f lookup_dt=%.3fs live_lag=%.3fs | "
    "opt a[0]=%.3f a[1s]=%.3f | t0 geom=%.3f opt_v=%.3f | t1s geom=%.3f opt_v=%.3f",
    result.initial_speed_mps, result.initial_accel_mps2, ego_to_p0_speed_mps, last_lookup_dt_s_,
    last_live_lag_s_, opt_a0, opt_a1s, chord_longitudinal_speed_mps(reference, 0),
    optimized.points.empty() ? 0.0 : optimized.points.front().longitudinal_velocity_mps,
    chord_longitudinal_speed_mps(reference, std::min<size_t>(10, n_steps - 1)),
    optimized.points.size() > 10 ? optimized.points[10].longitudinal_velocity_mps : 0.0F);
}

void TrajectoryTimeSequenceRawOptimizer::publish_debug_data(const std::string & /*ns*/) const
{
  if (!publish_debug_topics_) {
    return;
  }
  if (debug_raw_pub_) {
    debug_raw_pub_->publish(last_raw_trajectory_);
  }
  if (debug_adjusted_pub_) {
    debug_adjusted_pub_->publish(last_adjusted_trajectory_);
  }
  if (debug_optimized_pub_) {
    debug_optimized_pub_->publish(last_optimized_trajectory_);
  }
  if (debug_geometry_velocity_pub_) {
    debug_geometry_velocity_pub_->publish(last_geometry_velocity_trajectory_);
  }
  if (debug_shifted_count_pub_) {
    std_msgs::msg::Int32 msg;
    msg.data = last_shifted_point_count_;
    debug_shifted_count_pub_->publish(msg);
  }
  if (debug_solver_status_pub_) {
    std_msgs::msg::Int32 msg;
    msg.data = last_solver_status_;
    debug_solver_status_pub_->publish(msg);
  }
  if (debug_solve_time_pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = last_solve_time_ms_;
    debug_solve_time_pub_->publish(msg);
  }
}

TrajectoryTimeSequenceRawOptimizer::SteerStopMode
TrajectoryTimeSequenceRawOptimizer::resolve_steer_stop_mode(
  const autoware_planning_msgs::msg::Trajectory & reference, const TrajectoryProcessorData & data)
{
  const bool ego_stopped = is_ego_stopped(data, stopped_velocity_threshold_mps_);
  const bool stopped_reference = is_stopped_reference_trajectory(
    reference.points, stopped_velocity_threshold_mps_, stopped_trajectory_max_length_m_);
  const bool near_goal = is_near_route_goal(data, goal_steer_zero_distance_m_);

  if (goal_steer_zero_enable_ && near_goal) {
    if (ego_stopped) {
      in_goal_zero_regime_ = true;
    }
    if (in_goal_zero_regime_) {
      return SteerStopMode::Zero;
    }
  } else {
    in_goal_zero_regime_ = false;
  }

  if (ego_stopped && stopped_reference) {
    return SteerStopMode::Hold;
  }

  return SteerStopMode::Track;
}

void TrajectoryTimeSequenceRawOptimizer::apply_stopped_reference(
  TrajectoryPoints & traj_points, const autoware_planning_msgs::msg::Trajectory & reference,
  const double steer_rad) const
{
  const size_t n_out = std::min(traj_points.size(), reference.points.size());
  for (size_t i = 0; i < n_out; ++i) {
    traj_points[i] = reference.points[i];
    traj_points[i].longitudinal_velocity_mps = 0.0F;
    traj_points[i].lateral_velocity_mps = 0.0F;
    traj_points[i].acceleration_mps2 = 0.0F;
    traj_points[i].heading_rate_rps = 0.0F;
    traj_points[i].front_wheel_angle_rad = static_cast<float>(steer_rad);
    traj_points[i].rear_wheel_angle_rad = 0.0F;
  }

  if (reference.points.size() > traj_points.size()) {
    traj_points.insert(
      traj_points.end(), reference.points.begin() + static_cast<std::ptrdiff_t>(n_out),
      reference.points.end());
    for (size_t i = n_out; i < traj_points.size(); ++i) {
      traj_points[i].longitudinal_velocity_mps = 0.0F;
      traj_points[i].lateral_velocity_mps = 0.0F;
      traj_points[i].acceleration_mps2 = 0.0F;
      traj_points[i].heading_rate_rps = 0.0F;
      traj_points[i].front_wheel_angle_rad = static_cast<float>(steer_rad);
      traj_points[i].rear_wheel_angle_rad = 0.0F;
    }
  }
}

ProcessingResult TrajectoryTimeSequenceRawOptimizer::process(
  TrajectoryPoints & traj_points, TrajectoryProcessorData & data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *get_time_keeper());
  if (!enabled_ || !data.current_odometry || !data.current_acceleration) {
    return ProcessingResult::Unchanged;
  }

  ensure_optimizer();
  if (!optimizer_) {
    return ProcessingResult::Unchanged;
  }
  ingest_live_ego(data);
  maybe_update_map(data);
  ensure_debug_publishers();

  double accel_mps2 = 0.0;
  std::optional<double> steering;
  const auto ocp_odom = resolve_ocp_odometry(data, accel_mps2, steering);

  const auto original = to_trajectory_msg(traj_points, data.candidate_header);
  last_raw_trajectory_ = original;

  autoware_planning_msgs::msg::Trajectory reference = original;
  last_shifted_point_count_ = 0;
  if (road_border_enable_ && road_border_avoidance_) {
    const auto border_result =
      road_border_avoidance_->adjust(original, ocp_odom.pose.pose);
    reference = border_result.trajectory;
    last_shifted_point_count_ =
      static_cast<int>(border_result.num_shifted_points + border_result.num_unresolved_points);
  }
  last_adjusted_trajectory_ = reference;

  const auto steer_stop_mode = resolve_steer_stop_mode(reference, data);
  if (steer_stop_mode != SteerStopMode::Track) {
    optimizer_->clear_warm_start(data.candidate_index);

    if (steer_stop_mode == SteerStopMode::Zero) {
      latched_steering_rad_ = 0.002;
    } else if (steer_stop_mode == SteerStopMode::Hold && !in_stopped_regime_) {
      latched_steering_rad_ =
        data.current_steering ? data.current_steering->steering_tire_angle : 0.001;
    }

    const double output_steer_rad =
      steer_stop_mode == SteerStopMode::Zero ? 0.002 : latched_steering_rad_;
    apply_stopped_reference(traj_points, reference, output_steer_rad);

    in_stopped_regime_ = true;
    last_solver_status_ = steer_stop_mode == SteerStopMode::Zero ? -2 : -1;
    last_solve_time_ms_ = 0.0;
    return ProcessingResult::Modified;
  }

  if (in_stopped_regime_) {
    optimizer_->clear_warm_start(data.candidate_index);
  }
  in_stopped_regime_ = false;

  const auto result =
    optimizer_->optimize(reference, ocp_odom, steering, accel_mps2, data.candidate_index);
  last_solver_status_ = result.solver_status;
  last_solve_time_ms_ = result.solve_time_ms;

  if (!result.optimized) {
    if (result.solver_status != 0) {
      RCLCPP_WARN(
        get_node_ptr()->get_logger(),
        "Time-sequence raw optimizer acados solve failed with status %d; leaving input unchanged",
        result.solver_status);
    }
    return ProcessingResult::Unchanged;
  }

  last_optimized_trajectory_ = result.trajectory;
  publish_velocity_diagnostics(reference, result.trajectory, result, ocp_odom);
  if (debug_optimized_pub_) {
    debug_optimized_pub_->publish(result.trajectory);
  }

  const size_t n_out = std::min(traj_points.size(), result.trajectory.points.size());
  for (size_t i = 0; i < n_out; ++i) {
    traj_points[i] = result.trajectory.points[i];
  }
  if (result.trajectory.points.size() > traj_points.size()) {
    traj_points.insert(
      traj_points.end(), result.trajectory.points.begin() + static_cast<std::ptrdiff_t>(n_out),
      result.trajectory.points.end());
  }
  return ProcessingResult::Modified;
}

}  // namespace autoware::trajectory_processor::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_processor::plugin::TrajectoryTimeSequenceRawOptimizer,
  autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase)
