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
#include <autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp>
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
  stop_planner_ = std::make_unique<StopPlanner>(get_logger(), time_keeper_);
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

    velocity_smoother_ = std::make_unique<VelocitySmoother>(
      vel_params, get_logger(), get_clock(), vehicle_info, std::move(jerk_filtered_smoother));
  }
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
  auto traj_from_path =
    path_planner_->convert_path_to_trajectory(*path, params_.path_planning.output.delta_arc_length);
  traj_from_path.header = path->header;

  // 4. Shift trajectory to ego position
  const auto shifted_trajectory = shift_trajectory_to_ego(traj_from_path, input_data);

  // 5. Smooth path
  auto smoothed_trajectory = smooth_trajectory(shifted_trajectory, input_data);

  // 6. Apply trajectory modifiers
  apply_modifiers(smoothed_trajectory, input_data);

  // 停止計画: insert a stop point at map-defined stop lines when needed.
  // Stop line collection scans the whole lanelet layer and depends only on the map and the route,
  // so the result is cached and re-collected only when either changes.
  if (
    input_data.route_ptr != stop_lines_cache_route_ptr_ ||
    input_data.lanelet_map_bin_ptr != stop_lines_cache_map_ptr_) {
    stop_lines_cache_ = stop_planner_->collect_stop_lines(path_planner_->route_context());
    stop_lines_cache_route_ptr_ = input_data.route_ptr;
    stop_lines_cache_map_ptr_ = input_data.lanelet_map_bin_ptr;
  }

  // The "Go" trajectory stops only at mandatory targets (e.g. stop lines); the "Stop" trajectory
  // additionally stops at possibility targets (e.g. signals).
  const auto go_stop =
    plan_stop(smoothed_trajectory, stop_lines_cache_, input_data, /*include_possibility=*/false);
  const auto stop_stop =
    plan_stop(smoothed_trajectory, stop_lines_cache_, input_data, /*include_possibility=*/true);

  // 7. Velocity optimization
  // The "Go" trajectory is always produced (with the mandatory stop embedded when present).
  const auto & go_base = go_stop ? go_stop->trajectory : smoothed_trajectory;
  const auto go_trajectory = optimize_velocity(go_base, input_data, /*update_smoother_state=*/true);

  // The "Stop" trajectory is published only when it embeds a stop point different from the go
  // trajectory's (要件: 異なる停止点が埋め込まれる場合のみpublish).
  const bool stop_differs_from_go =
    stop_stop &&
    (!go_stop || std::abs(stop_stop->stop_point_arc_length - go_stop->stop_point_arc_length) >
                   params_.stop_planning.stop_point_diff_threshold);
  // The stop trajectory must not update the smoother's prev-output state: it would drag the go
  // trajectory's initial speed down to the stop profile on the next cycle (velocity continuity).
  const auto stop_trajectory =
    stop_differs_from_go ? std::make_optional(optimize_velocity(
                             stop_stop->trajectory, input_data, /*update_smoother_state=*/false))
                         : std::nullopt;

  // 8. Create and publish CandidateTrajectories message
  publish_candidate_trajectories(go_trajectory, stop_trajectory);

  // 9. Publish debug information if enabled
  if (params_.debug.enable_path) {
    pub_debug_path_->publish(*path);
  }
  if (params_.debug.enable_output_trajectory) {
    pub_debug_trajectory_->publish(go_trajectory);
    if (stop_trajectory) {
      pub_debug_stop_trajectory_->publish(*stop_trajectory);
    }
  }

  // Publish processing time
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
    input_data.odometry_ptr->pose.pose, input_data.odometry_ptr->twist.twist.linear.x);
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

Trajectory MinimumRuleBasedPlannerNode::smooth_trajectory(
  const Trajectory & trajectory, const InputData & input_data) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  auto optimizer_data = make_optimizer_data(input_data);

  trajectory_optimizer::TrajectoryOptimizerParams optimizer_params;
  optimizer_params.use_eb_smoother = true;

  auto trajectory_points = trajectory.points;
  if (path_smoother_) {
    autoware_utils_debug::ScopedTimeTrack st_path_smoother(
      path_smoother_->get_name(), *time_keeper_);
    path_smoother_->optimize_trajectory(trajectory_points, optimizer_params, optimizer_data);
    if (params_.debug.enable_optimizer_trajectory) {
      publish_debug_trajectory(path_smoother_->get_name(), trajectory_points);
    }
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

std::optional<MinimumRuleBasedPlannerNode::StopResult> MinimumRuleBasedPlannerNode::plan_stop(
  const Trajectory & trajectory, const std::vector<StopLine> & candidate_stop_lines,
  const InputData & input_data, const bool include_possibility_targets) const
{
  // Distinguish the two calls per cycle in the processing-time tree.
  autoware_utils_debug::ScopedTimeTrack st(
    include_possibility_targets ? "plan_stop(stop)" : "plan_stop(go)", *time_keeper_);

  if (trajectory.points.size() < 2) return std::nullopt;

  // Keep only the pre-collected stop lines crossed by the trajectory.
  const auto stop_lines =
    stop_planner_->filter_stop_lines_on_trajectory(candidate_stop_lines, trajectory.points);

  // Visualize the stop lines currently under consideration. Publish only for the superset call
  // (all targets) to avoid publishing twice per cycle.
  if (include_possibility_targets) {
    pub_stop_lines_marker_->publish(stop_planner_->create_stop_line_marker_array(stop_lines));
  }

  if (stop_lines.empty()) return std::nullopt;

  // Select the nearest reachable stop point (already adjusted for the vehicle front offset and the
  // stop margin) among the allowed stop line types.
  StopSelectionParams selection_params;
  selection_params.max_deceleration = params_.stop_planning.max_deceleration;
  selection_params.max_jerk = params_.stop_planning.max_jerk;
  selection_params.stop_margin_distance = params_.stop_planning.stop_margin_distance;
  selection_params.stop_distance_from_crosswalk =
    params_.stop_planning.stop_distance_from_crosswalk;
  selection_params.base_link_to_front = vehicle_info_.max_longitudinal_offset_m;

  const double ego_velocity = input_data.odometry_ptr->twist.twist.linear.x;
  const double ego_acceleration = input_data.acceleration_ptr->accel.accel.linear.x;
  const auto stop_point_arc_length = stop_planner_->select_stop_arc_length(
    stop_lines, trajectory.points, input_data.odometry_ptr->pose.pose, ego_velocity,
    ego_acceleration, selection_params, include_possibility_targets);
  if (!stop_point_arc_length) return std::nullopt;

  // If an earlier stage (e.g. the ObstacleStop modifier) already stops the vehicle before this
  // stop point, there is nothing to add.
  if (autoware::trajectory_modifier::utils::stop_point_exists(
        trajectory.points, *stop_point_arc_length)) {
    return std::nullopt;
  }

  autoware_utils_debug::ScopedTimeTrack st_insert("insert_stop_point", *time_keeper_);

  const double trajectory_length = autoware::motion_utils::calcArcLength(trajectory.points);

  // Insert the stop point on a copy; the downstream velocity optimizer builds the deceleration
  // profile down to it. Return the copy only when the stop point was actually inserted.
  Trajectory stop_trajectory = trajectory;
  if (!autoware::trajectory_modifier::utils::insert_stop_point(
        stop_trajectory.points, *stop_point_arc_length, trajectory_length)) {
    return std::nullopt;
  }

  return StopResult{*stop_point_arc_length, std::move(stop_trajectory)};
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

  {
    autoware_utils_debug::ScopedTimeTrack st_smoother("velocity_smoother", *time_keeper_);
    velocity_smoother_->optimize(
      trajectory_points, *input_data.odometry_ptr,
      input_data.acceleration_ptr->accel.accel.linear.x, update_smoother_state);
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

  autoware::motion_utils::calculate_time_from_start(
    trajectory_points, input_data.odometry_ptr->pose.pose.position);

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

  // The "Go" trajectory is always published.
  add_candidate(go_generator_uuid_, "MinimumRuleBasedPlanner_Go", go_trajectory);

  // The "Stop" generator info is always published even when no distinct stop trajectory exists
  // this cycle: the downstream concatenator buffers candidates per generator id and only
  // overwrites entries whose generator info is present in the message, so the empty entry is what
  // invalidates the previously published stop trajectory (要件「停止線超過後は無効な経路を出力
  // する」). Without it the stale stop trajectory would stay buffered and selectable.
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

  for (auto & modifier : modifier_plugins_) {
    modifier->update_params(params_);
  }
}

}  // namespace autoware::minimum_rule_based_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimum_rule_based_planner::MinimumRuleBasedPlannerNode)
