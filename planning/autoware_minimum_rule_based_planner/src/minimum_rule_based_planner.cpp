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

#include "minimum_rule_based_planner.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware/velocity_smoother/resample.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

namespace
{
trajectory_optimizer::TrajectoryOptimizerData make_optimizer_data(
  const MinimumRuleBasedPlannerNode::InputData & input_data)
{
  trajectory_optimizer::TrajectoryOptimizerData data;
  data.current_odometry = *input_data.odometry_ptr;
  if (input_data.acceleration_ptr) {
    data.current_acceleration = *input_data.acceleration_ptr;
  }
  return data;
}

minimum_rule_based_planner::plugin::ModifierData make_modifier_data(
  const MinimumRuleBasedPlannerNode::InputData & input_data)
{
  minimum_rule_based_planner::plugin::ModifierData data;
  data.odometry_ptr = input_data.odometry_ptr;
  data.acceleration_ptr = input_data.acceleration_ptr;
  data.predicted_objects_ptr = input_data.predicted_objects_ptr;
  data.obstacle_pointcloud_ptr = input_data.obstacle_pointcloud_ptr;
  return data;
}

void assign_time_from_start(
  TrajectoryPoints & traj_points, const geometry_msgs::msg::Point & ego_point)
{
  constexpr double k_min_velocity = 1.0;  // [m/s] velocity floor for dt = ds / v
  constexpr double k_min_dt = 1.0e-3;     // [s] keeps the time base strictly increasing

  if (traj_points.size() < 2) {
    return;
  }

  std::vector<double> times{0.0};
  times.reserve(traj_points.size());
  for (size_t i = 1; i < traj_points.size(); ++i) {
    const double ds = autoware_utils::calc_distance2d(traj_points.at(i - 1), traj_points.at(i));
    const double v =
      std::max<double>(std::abs(traj_points.at(i - 1).longitudinal_velocity_mps), k_min_velocity);
    times.push_back(times.back() + std::max(ds / v, k_min_dt));
  }

  const double ego_time =
    times.at(autoware::motion_utils::findNearestSegmentIndex(traj_points, ego_point));
  for (size_t i = 0; i < traj_points.size(); ++i) {
    traj_points.at(i).time_from_start = rclcpp::Duration::from_seconds(times.at(i) - ego_time);
  }
}
}  // namespace

MinimumRuleBasedPlannerNode::MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("minimum_rule_based_planner_node", options),
  go_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  stop_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  modifier_plugin_loader_(
    "autoware_minimum_rule_based_planner",
    "autoware::minimum_rule_based_planner::plugin::PluginInterface"),
  modifier_context_(std::make_shared<plugin::ModifierContext>(this))
{
  param_listener_ =
    std::make_shared<::minimum_rule_based_planner::ParamListener>(get_node_parameters_interface());

  pub_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_stop_lines_marker_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/stop_lines", 1);
  pub_debug_path_ = this->create_publisher<PathWithLaneId>("~/debug/path_with_lane_id", 1);
  pub_debug_trajectory_ = this->create_publisher<Trajectory>("~/debug/trajectory", 1);
  pub_debug_stop_trajectory_ = this->create_publisher<Trajectory>("~/debug/stop_trajectory", 1);
  pub_debug_shifted_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/shifted_trajectory", 1);
  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  debug_processing_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "~/debug/processing_time_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  params_ = param_listener_->get_params();

  load_optimizer_plugins();
  load_modifier_plugins();

  path_planner_ =
    std::make_unique<PathPlanner>(get_logger(), get_clock(), time_keeper_, params_, vehicle_info_);
  map_based_stop_planner_ = std::make_unique<MapBasedStopPlanner>(get_logger(), time_keeper_);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&MinimumRuleBasedPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "Minimum Rule Based Planner Node has been started.");
}

void MinimumRuleBasedPlannerNode::load_optimizer_plugins()
{
  // Create plugin loader for autoware_trajectory_optimizer
  plugin_loader_ = std::make_unique<OptimizerPluginLoader>(
    "autoware_trajectory_optimizer",
    "autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase");

  auto try_load_optimizer_plugin = [&](const std::string & plugin_path, const std::string & name)
    -> std::shared_ptr<OptimizerPluginInterface> {
    try {
      auto plugin = plugin_loader_->createSharedInstance(plugin_path);
      plugin->initialize(name, this, time_keeper_);
      pub_debug_optimizer_module_trajectories_[plugin->get_name()] =
        this->create_publisher<Trajectory>(
          "~/debug/optimizer/" + plugin->get_name() + "/trajectory", 1);
      RCLCPP_INFO(get_logger(), "Loaded trajectory %s plugin", name.c_str());
      return plugin;
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to load trajectory %s plugin: %s", name.c_str(), ex.what());
      return nullptr;
    }
  };

  path_smoother_ = try_load_optimizer_plugin(
    "autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer", "eb_smoother");

  // Set up velocity optimizer
  // NOTE(odashima):
  // The velocity_optimizer modifier plugin used in diffusion_planner has different processing,
  // so a separate implementation is provided here.
  {
    VelocitySmootherParams vel_params;
    vel_params.nearest_dist_threshold_m =
      declare_parameter<double>("velocity_smoother.nearest_dist_threshold_m");
    vel_params.nearest_yaw_threshold_deg =
      declare_parameter<double>("velocity_smoother.nearest_yaw_threshold_deg");
    vel_params.target_pull_out_speed_mps =
      declare_parameter<double>("velocity_smoother.target_pull_out_speed_mps");
    vel_params.target_pull_out_acc_mps2 =
      declare_parameter<double>("velocity_smoother.target_pull_out_acc_mps2");
    vel_params.max_speed_mps = declare_parameter<double>("velocity_smoother.max_speed_mps");
    vel_params.max_lateral_accel_mps2 =
      declare_parameter<double>("velocity_smoother.max_lateral_accel_mps2");
    vel_params.stop_dist_to_prohibit_engage =
      declare_parameter<double>("velocity_smoother.stop_dist_to_prohibit_engage");
    vel_params.set_engage_speed = declare_parameter<bool>("velocity_smoother.set_engage_speed");
    vel_params.limit_speed = declare_parameter<bool>("velocity_smoother.limit_speed");
    vel_params.limit_lateral_acceleration =
      declare_parameter<bool>("velocity_smoother.limit_lateral_acceleration");
    vel_params.smooth_velocities = declare_parameter<bool>("velocity_smoother.smooth_velocities");

    const auto vehicle_info =
      autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
    auto jerk_filtered_smoother =
      std::make_shared<autoware::velocity_smoother::JerkFilteredSmoother>(*this, time_keeper_);

    velocity_smoother_ = std::make_shared<VelocitySmoother>(
      vel_params, get_logger(), get_clock(), vehicle_info, std::move(jerk_filtered_smoother));
  }

  // Joint shape / speed optimiser. When it owns the output it replaces both of the above, which
  // stay loaded as its fallback: a solved NLP is not necessarily a feasible one, and it must never
  // be the only option in a backup trajectory generator.
  if (params_.acados_mpt.enable) {
    // Same engage behaviour, and the same parameters, as the velocity smoother: the NLP pins its
    // initial speed to the ego's, so without this a trajectory planned from standstill starts at
    // exactly zero and the controller never leaves its stopped state.
    EngageParams engage;
    engage.enable = get_parameter("velocity_smoother.set_engage_speed").as_bool();
    engage.speed = get_parameter("velocity_smoother.target_pull_out_speed_mps").as_double();
    engage.acceleration = get_parameter("velocity_smoother.target_pull_out_acc_mps2").as_double();
    engage.stop_dist_to_prohibit_engage =
      get_parameter("velocity_smoother.stop_dist_to_prohibit_engage").as_double();

    // The velocity smoother is handed over as the NLP's speed guess: the solve time is dominated by
    // the quality of that guess (12 ms and 4 SQP iterations from the reference, 0.2 ms from its own
    // solution, measured on a bag) and the smoother costs about 1 ms.
    mpt_optimizer_ = std::make_unique<MptOptimizer>(
      make_mpt_optimizer_params(params_, vehicle_info_), engage, get_logger(), get_clock(),
      params_.acados_mpt.solver.use_velocity_smoother_guess ? velocity_smoother_ : nullptr);
    pub_debug_mpt_trajectory_ =
      this->create_publisher<Trajectory>("~/debug/optimizer/acados_mpt/trajectory", 1);
    pub_debug_mpt_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/debug/optimizer/acados_mpt/markers", 1);
    pub_debug_mpt_info_ = this->create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
      "~/debug/optimizer/acados_mpt/info", 1);
    RCLCPP_INFO(
      get_logger(), "acados MPT optimizer running in %s mode",
      params_.acados_mpt.shadow ? "shadow (debug topic only)"
                                : "active (replaces eb_smoother + velocity_smoother)");
  }
}

bool MinimumRuleBasedPlannerNode::mpt_owns_output() const
{
  return mpt_optimizer_ && !params_.acados_mpt.shadow;
}

void MinimumRuleBasedPlannerNode::load_modifier_plugins()
{
  for (const auto & name : params_.plugin_names) {
    if (name.empty()) continue;
    load_plugin(name);
  }

  initialized_modifiers_ = true;
  RCLCPP_INFO(
    get_logger(), "Trajectory modifier plugins initialized: %zu plugins", modifier_plugins_.size());
}

void MinimumRuleBasedPlannerNode::load_plugin(const std::string & name)
{
  // Check if the plugin is already instantiated
  auto it = std::find_if(modifier_plugins_.begin(), modifier_plugins_.end(), [&](const auto & p) {
    return p->get_name() == name;
  });
  if (it != modifier_plugins_.end()) {
    RCLCPP_WARN(
      this->get_logger(), "The plugin '%s' is already in the plugins list.", name.c_str());
    return;
  }

  if (modifier_plugin_loader_.isClassAvailable(name)) {
    const auto plugin = modifier_plugin_loader_.createSharedInstance(name);
    plugin->initialize(name, this, time_keeper_, modifier_context_, params_);

    // Convert "autoware::...::ObstacleStop" to "obstacle_stop"
    const auto short_name = [](const std::string & plugin_name) {
      const std::string class_name = plugin_name.find("::") != std::string::npos
                                       ? plugin_name.substr(plugin_name.rfind("::") + 2)
                                       : plugin_name;
      std::string short_name;
      for (size_t i = 0; i < class_name.size(); ++i) {
        if (std::isupper(class_name[i])) {
          if (i > 0) short_name += '_';
          short_name += static_cast<char>(std::tolower(class_name[i]));
        } else {
          short_name += class_name[i];
        }
      }

      return short_name;
    }(name);

    pub_debug_modifier_module_trajectories_[plugin->get_name()] =
      this->create_publisher<Trajectory>("~/debug/modifier/" + short_name + "/trajectory", 1);
    modifier_plugins_.push_back(plugin);
    RCLCPP_DEBUG(this->get_logger(), "The plugin '%s' has been loaded", name.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "The plugin '%s' is not available", name.c_str());
  }
}

void MinimumRuleBasedPlannerNode::unload_plugin(const std::string & name)
{
  auto it = std::remove_if(
    modifier_plugins_.begin(), modifier_plugins_.end(),
    [&](const auto plugin) { return plugin->get_name() == name; });

  if (it == modifier_plugins_.end()) {
    RCLCPP_WARN(
      this->get_logger(), "The plugin '%s' is not in the registered modules", name.c_str());
  } else {
    modifier_plugins_.erase(it, modifier_plugins_.end());
    RCLCPP_INFO(this->get_logger(), "The scene plugin '%s' has been unloaded", name.c_str());
  }
}

void MinimumRuleBasedPlannerNode::publish_debug_trajectory(
  const std::string & plugin_name, const TrajectoryPoints & traj_points) const
{
  Trajectory traj;
  traj.points = traj_points;
  if (const auto it = pub_debug_modifier_module_trajectories_.find(plugin_name);
      it != pub_debug_modifier_module_trajectories_.end()) {
    it->second->publish(traj);
  } else if (const auto opt_it = pub_debug_optimizer_module_trajectories_.find(plugin_name);
             opt_it != pub_debug_optimizer_module_trajectories_.end()) {
    opt_it->second->publish(traj);
  } else {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "No debug trajectory publisher registered for plugin '%s'",
      plugin_name.c_str());
  }
}

void MinimumRuleBasedPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  stop_watch_ptr_ = std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing_time");

  // 1. Check data availability
  const auto input_data = take_data();
  path_planner_->set_planner_data(input_data.lanelet_map_bin_ptr, input_data.route_ptr);
  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for necessary data to plan trajectories.");
    return;
  }

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  // 2. Get path
  const auto path = plan_path(input_data);

  if (!path) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to plan path.");
    return;
  }

  // 3. Convert path to trajectory
  auto trajectory =
    path_planner_->convert_path_to_trajectory(*path, params_.path_planning.output.delta_arc_length);
  trajectory.header = path->header;

  // 4. Shift trajectory to ego position
  trajectory = shift_trajectory_to_ego(trajectory, input_data);

  // 5. Smooth path
  trajectory = smooth_trajectory(trajectory, input_data);

  // 6. Apply trajectory modifiers
  apply_modifiers(trajectory, input_data);

  // 7. Plan the go/stop trajectories with map-defined stop points
  map_based_stop_planner_->set_planner_data(
    input_data.lanelet_map_bin_ptr, input_data.route_ptr, path_planner_->route_context());
  const auto stop_result = map_based_stop_planner_->plan(
    trajectory, input_data.odometry_ptr->pose.pose, input_data.odometry_ptr->twist.twist.linear.x,
    input_data.acceleration_ptr->accel.accel.linear.x, make_map_based_stop_params());
  pub_stop_lines_marker_->publish(stop_result.stop_line_markers);

  // 8. Velocity optimization
  // NOTE(odashima): the stop trajectory must not update the smoother's prev-output state: it
  // would drag the go trajectory's initial speed down to the stop profile on the next cycle.
  const auto go_trajectory =
    optimize_velocity(stop_result.go_trajectory, input_data, /*update_smoother_state=*/true);
  const auto stop_trajectory =
    stop_result.stop_trajectory
      ? std::make_optional(optimize_velocity(
          *stop_result.stop_trajectory, input_data, /*update_smoother_state=*/false))
      : std::nullopt;

  // 9. Create and publish CandidateTrajectories message
  publish_candidate_trajectories(go_trajectory, stop_trajectory);

  // 10. Publish debug information
  publish_debug_outputs(*path, go_trajectory, stop_trajectory);
  publish_processing_time();
}

StopSelectionParams MinimumRuleBasedPlannerNode::make_map_based_stop_params() const
{
  StopSelectionParams params;
  params.max_deceleration = params_.map_based_stop.max_deceleration;
  params.max_jerk = params_.map_based_stop.max_jerk;
  params.stop_margin_distance = params_.map_based_stop.stop_margin_distance;
  params.stop_distance_from_crosswalk = params_.map_based_stop.stop_distance_from_crosswalk;
  params.stop_distance_from_private_area = params_.map_based_stop.stop_distance_from_private_area;
  params.stop_distance_from_intersection = params_.map_based_stop.stop_distance_from_intersection;
  params.base_link_to_front = vehicle_info_.max_longitudinal_offset_m;
  params.stop_point_diff_threshold = params_.map_based_stop.stop_point_diff_threshold;
  return params;
}

void MinimumRuleBasedPlannerNode::publish_debug_outputs(
  const PathWithLaneId & path, const Trajectory & go_trajectory,
  const std::optional<Trajectory> & stop_trajectory) const
{
  if (params_.debug.enable_path) {
    pub_debug_path_->publish(path);
  }
  if (params_.debug.enable_output_trajectory) {
    pub_debug_trajectory_->publish(go_trajectory);
    if (stop_trajectory) {
      pub_debug_stop_trajectory_->publish(*stop_trajectory);
    }
  }
}

void MinimumRuleBasedPlannerNode::publish_processing_time()
{
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch_ptr_->toc("processing_time", true);
  debug_processing_time_pub_->publish(processing_time_msg);
}

std::optional<PathWithLaneId> MinimumRuleBasedPlannerNode::plan_path(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (input_data.test_path_with_lane_id_ptr) {
    return *input_data.test_path_with_lane_id_ptr;
  }
  return path_planner_->plan_path(
    input_data.odometry_ptr->pose.pose, input_data.odometry_ptr->twist.twist.linear.x,
    input_data.odometry_ptr->header.stamp);
}

Trajectory MinimumRuleBasedPlannerNode::shift_trajectory_to_ego(
  const Trajectory & trajectory, const InputData & input_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!params_.path_planning.path_shift.enable) return trajectory;

  TrajectoryShiftParams shift_params;
  shift_params.minimum_shift_length = params_.path_planning.path_shift.minimum_shift_length;
  shift_params.minimum_shift_yaw = params_.path_planning.path_shift.minimum_shift_yaw;
  shift_params.minimum_shift_distance = params_.path_planning.path_shift.minimum_shift_distance;
  shift_params.min_speed_for_curvature = params_.path_planning.path_shift.min_speed_for_curvature;
  shift_params.lateral_accel_limit = params_.path_planning.path_shift.lateral_accel_limit;

  const double ego_velocity = input_data.odometry_ptr->twist.twist.linear.x;
  const double ego_yaw_rate = input_data.odometry_ptr->twist.twist.angular.z;
  const auto shifted_trajectory = path_planner_->shift_trajectory_to_ego(
    trajectory, input_data.odometry_ptr->pose.pose, ego_velocity, ego_yaw_rate, shift_params,
    params_.path_planning.output.delta_arc_length);

  if (params_.debug.enable_shifted_trajectory) {
    Trajectory shifted_traj;
    shifted_traj.header = trajectory.header;
    shifted_traj.points = shifted_trajectory.points;
    pub_debug_shifted_trajectory_->publish(shifted_traj);
  }
  return shifted_trajectory;
}

void MinimumRuleBasedPlannerNode::apply_path_smoother(
  TrajectoryPoints & traj_points, const InputData & input_data) const
{
  if (!path_smoother_) return;

  autoware_utils_debug::ScopedTimeTrack st(path_smoother_->get_name(), *time_keeper_);
  auto optimizer_data = make_optimizer_data(input_data);
  trajectory_optimizer::TrajectoryOptimizerParams optimizer_params;
  optimizer_params.use_eb_smoother = true;
  path_smoother_->optimize_trajectory(traj_points, optimizer_params, optimizer_data);
  if (params_.debug.enable_optimizer_trajectory) {
    publish_debug_trajectory(path_smoother_->get_name(), traj_points);
  }
}

Trajectory MinimumRuleBasedPlannerNode::smooth_trajectory(
  const Trajectory & trajectory, const InputData & input_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto trajectory_points = trajectory.points;
  // The MPT optimiser decides the shape and the speed profile in one problem, so smoothing here
  // would only pre-shape its input. It still runs on the fallback path (see optimize_velocity),
  // and in shadow mode, where the published trajectory must stay exactly what it was.
  if (!mpt_owns_output()) {
    apply_path_smoother(trajectory_points, input_data);
  }

  Trajectory traj;
  traj.header = trajectory.header;
  traj.points = trajectory_points;
  return traj;
}

void MinimumRuleBasedPlannerNode::apply_modifiers(
  Trajectory & trajectory, const InputData & input_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto modifier_data = make_modifier_data(input_data);
  for (auto & modifier : modifier_plugins_) {
    autoware_utils_debug::ScopedTimeTrack st_modifier(modifier->get_name(), *time_keeper_);
    modifier->run(trajectory.points, modifier_data);
    modifier->publish_planning_factor();
    if (params_.debug.enable_modifier_trajectory) {
      publish_debug_trajectory(modifier->get_name(), trajectory.points);
    }
  }
}

Trajectory MinimumRuleBasedPlannerNode::optimize_velocity(
  const Trajectory & trajectory, const InputData & input_data,
  const bool update_smoother_state) const
{
  // Distinguish the two calls per cycle (go updates the smoother state, stop does not) in the
  // processing-time tree.
  autoware_utils_debug::ScopedTimeTrack st(
    update_smoother_state ? "optimize_velocity(go)" : "optimize_velocity(stop)", *time_keeper_);

  auto trajectory_points = trajectory.points;
  const double current_acceleration = input_data.acceleration_ptr->accel.accel.linear.x;

  bool optimized_by_mpt = false;
  // The measured curvature is not optional here. The NLP pins its initial curvature as a *hard*
  // constraint, and the only alternative - estimating it from the input path around the ego - spans
  // barely a metre, so the few centimetres of lateral offset an ego always has become a curvature
  // of the wrong sign and magnitude that the trajectory then has to bulge out to unwind: measured
  // on a straight path, a 5 cm offset bulges 0.76 m and a 10 cm one 3.95 m, at over 10 m/s^2 of
  // lateral acceleration. Solving without it is worse than not solving, so the cycle falls back
  // instead.
  const auto ego_curvature =
    input_data.steering_ptr
      ? std::make_optional(
          std::tan(input_data.steering_ptr->steering_tire_angle) / vehicle_info_.wheel_base_m)
      : std::nullopt;
  if (mpt_optimizer_ && !ego_curvature) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *throttle_clock_, 5000,
      "[mpt] no steering report on ~/input/steering; the initial curvature would have to be "
      "guessed "
      "from the path, which bends the trajectory metres off a straight lane. Not solving.");
  }
  // In shadow mode only the go trajectory is solved: the result is published for comparison, and
  // a second solve would double the cost for something nobody reads.
  if (mpt_optimizer_ && ego_curvature && (mpt_owns_output() || update_smoother_state)) {
    autoware_utils_debug::ScopedTimeTrack st_mpt("acados_mpt", *time_keeper_);
    auto points = mpt_optimizer_->optimize(
      trajectory_points, *input_data.odometry_ptr, current_acceleration, ego_curvature);
    // Outside the `if`: the markers describe the solve whether or not its result was usable, and
    // the failed ones are the interesting ones. Only the go solve publishes, so the two solves of a
    // cycle do not overwrite each other on screen.
    if (update_smoother_state && params_.debug.enable_optimizer_trajectory) {
      pub_debug_mpt_markers_->publish(mpt_optimizer_->debug_markers());
      autoware_internal_debug_msgs::msg::StringStamped info;
      info.stamp = throttle_clock_->now();
      info.data = mpt_optimizer_->debug_info();
      pub_debug_mpt_info_->publish(info);
    }
    if (points) {
      if (update_smoother_state && params_.debug.enable_optimizer_trajectory) {
        Trajectory mpt_traj;
        mpt_traj.header = trajectory.header;
        mpt_traj.points = *points;
        pub_debug_mpt_trajectory_->publish(mpt_traj);
      }
      if (mpt_owns_output()) {
        trajectory_points = std::move(*points);
        optimized_by_mpt = true;
      }
    }
  }

  if (!optimized_by_mpt) {
    if (mpt_owns_output()) {
      // The MPT optimiser took the elastic band's place in the pipeline, so the shape reaching
      // here is unsmoothed; the velocity smoother below takes the shape as given.
      apply_path_smoother(trajectory_points, input_data);
    }
    autoware_utils_debug::ScopedTimeTrack st_smoother("velocity_smoother", *time_keeper_);
    velocity_smoother_->optimize(
      trajectory_points, *input_data.odometry_ptr, current_acceleration, update_smoother_state);
  }

  // Post-optimization resample
  {
    autoware_utils_debug::ScopedTimeTrack st_resample("post_resample", *time_keeper_);
    autoware::velocity_smoother::resampling::ResampleParam post_resample_param;
    post_resample_param.max_trajectory_length = params_.post_resample.max_trajectory_length;
    post_resample_param.min_trajectory_length = params_.post_resample.min_trajectory_length;
    post_resample_param.resample_time = params_.post_resample.resample_time;
    post_resample_param.dense_resample_dt = params_.post_resample.dense_resample_dt;
    post_resample_param.dense_min_interval_distance =
      params_.post_resample.dense_min_interval_distance;
    post_resample_param.sparse_resample_dt = params_.post_resample.sparse_resample_dt;
    post_resample_param.sparse_min_interval_distance =
      params_.post_resample.sparse_min_interval_distance;

    const auto & ego_pose = input_data.odometry_ptr->pose.pose;
    const double v_current = input_data.odometry_ptr->twist.twist.linear.x;

    trajectory_points = autoware::velocity_smoother::resampling::resampleTrajectory(
      trajectory_points, v_current, ego_pose,
      params_.path_planning.ego_nearest_lanelet.dist_threshold,
      params_.path_planning.ego_nearest_lanelet.yaw_threshold, post_resample_param, false);

    if (!trajectory_points.empty()) {
      trajectory_points.back().longitudinal_velocity_mps = 0.0;
    }
  }

  // NOTE(odashima): replaces calculate_time_from_start(), whose time base is not strictly
  // increasing. This function will be removed once MPPI is implemented.
  assign_time_from_start(trajectory_points, input_data.odometry_ptr->pose.pose.position);

  Trajectory traj;
  traj.header = trajectory.header;
  traj.points = trajectory_points;
  return traj;
}

void MinimumRuleBasedPlannerNode::publish_candidate_trajectories(
  const Trajectory & go_trajectory, const std::optional<Trajectory> & stop_trajectory) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  CandidateTrajectories msg;

  const auto add_generator_info = [&msg](
                                    const UUID & generator_id, const std::string & generator_name) {
    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = generator_id;
    generator_info.generator_name.data = generator_name;
    msg.generator_info.push_back(generator_info);
  };

  const auto add_candidate = [&msg, &add_generator_info](
                               const UUID & generator_id, const std::string & generator_name,
                               const Trajectory & trajectory) {
    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_traj;
    candidate_traj.header = trajectory.header;
    candidate_traj.generator_id = generator_id;
    candidate_traj.points = trajectory.points;
    msg.candidate_trajectories.push_back(candidate_traj);

    add_generator_info(generator_id, generator_name);
  };

  add_candidate(go_generator_uuid_, "MinimumRuleBasedPlanner_Go", go_trajectory);

  // NOTE(odashima): the stop generator info must be published even when no distinct stop
  // trajectory exists this cycle. The downstream concatenator buffers candidates per generator id
  // and only overwrites entries whose generator info is present in the message, so the empty
  // entry is what invalidates the previously published stop trajectory; without it the stale stop
  // trajectory would stay buffered and selectable.
  if (stop_trajectory) {
    add_candidate(stop_generator_uuid_, "MinimumRuleBasedPlanner_Stop", *stop_trajectory);
  } else {
    add_generator_info(stop_generator_uuid_, "MinimumRuleBasedPlanner_Stop");
  }

  pub_trajectories_->publish(msg);
}

MinimumRuleBasedPlannerNode::InputData MinimumRuleBasedPlannerNode::take_data()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  InputData input_data;

  if (const auto msg = route_subscriber_.take_data()) {
    if (!msg->segments.empty()) {
      route_ptr_ = msg;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "input route is empty, ignoring...");
    }
  }
  input_data.route_ptr = route_ptr_;

  if (const auto msg = vector_map_subscriber_.take_data()) {
    lanelet_map_bin_ptr_ = msg;
  }
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;

  if (const auto msg = odometry_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }
  input_data.odometry_ptr = odometry_ptr_;

  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }
  input_data.acceleration_ptr = acceleration_ptr_;

  if (const auto msg = objects_subscriber_.take_data()) {
    predicted_objects_ptr_ = msg;
  }
  input_data.predicted_objects_ptr = predicted_objects_ptr_;

  if (const auto msg = steering_subscriber_.take_data()) {
    steering_ptr_ = msg;
  }
  input_data.steering_ptr = steering_ptr_;

  if (const auto msg = pointcloud_subscriber_.take_data()) {
    obstacle_pointcloud_ptr_ = msg;
  }
  input_data.obstacle_pointcloud_ptr = obstacle_pointcloud_ptr_;

  if (const auto msg = test_path_with_lane_id_subscriber_.take_data()) {
    test_path_with_lane_id_ptr = msg;
  }
  input_data.test_path_with_lane_id_ptr = test_path_with_lane_id_ptr;

  return input_data;
}

bool MinimumRuleBasedPlannerNode::is_data_ready(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  // NOTE(odashima): on test mode, minimum rule based planner receives path_with_lane_id topic
  if (!input_data.route_ptr && !input_data.test_path_with_lane_id_ptr) {
    notify_waiting("route");
    return false;
  }
  if (!input_data.lanelet_map_bin_ptr) {
    notify_waiting("lanelet map");
    return false;
  }
  if (!input_data.odometry_ptr) {
    notify_waiting("odometry");
    return false;
  }
  if (!input_data.acceleration_ptr) {
    notify_waiting("acceleration");
    return false;
  }
  return true;
}

void MinimumRuleBasedPlannerNode::update_params()
{
  params_ = param_listener_->get_params();
  path_planner_->update_params(params_);
  if (mpt_optimizer_) {
    mpt_optimizer_->update_params(make_mpt_optimizer_params(params_, vehicle_info_));
  }

  for (auto & modifier : modifier_plugins_) {
    modifier->update_params(params_);
  }
}

}  // namespace autoware::minimum_rule_based_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimum_rule_based_planner::MinimumRuleBasedPlannerNode)
