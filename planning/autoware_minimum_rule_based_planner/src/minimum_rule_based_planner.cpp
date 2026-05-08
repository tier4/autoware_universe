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
visualization_msgs::msg::Marker make_line_strip_marker(
  const lanelet::ConstLineString3d & line_string, const std_msgs::msg::Header & header,
  const std::string & ns, int id, float r, float g, float b)
{
  visualization_msgs::msg::Marker m;
  m.header = header;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.1;
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = 0.8f;
  m.pose.orientation.w = 1.0;
  m.points.reserve(line_string.size());
  for (const auto & p : line_string) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    m.points.push_back(pt);
  }
  return m;
}
}  // namespace

MinimumRuleBasedPlannerNode::MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("minimum_rule_based_planner_node", options),
  generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  modifier_plugin_loader_(
    "autoware_minimum_rule_based_planner",
    "autoware::minimum_rule_based_planner::plugin::PluginInterface"),
  modifier_data_(std::make_shared<plugin::ModifierData>(this))
{
  param_listener_ =
    std::make_shared<::minimum_rule_based_planner::ParamListener>(get_node_parameters_interface());

  pub_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_debug_path_ = this->create_publisher<PathWithLaneId>("~/debug/path_with_lane_id", 1);
  pub_debug_trajectory_ = this->create_publisher<Trajectory>("~/debug/trajectory", 1);
  pub_debug_shifted_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/shifted_trajectory", 1);
  pub_debug_lane_boundaries_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/lane_boundaries", 1);
  pub_debug_uncrossable_boundaries_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/uncrossable_boundaries", 1);
  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  load_optimizer_plugins();
  load_modifier_plugins();

  params_ = param_listener_->get_params();

  path_planner_ =
    std::make_unique<PathPlanner>(get_logger(), get_clock(), time_keeper_, params_, vehicle_info_);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&MinimumRuleBasedPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "Minimum Rule Based Planner Node has been started.");
}

void MinimumRuleBasedPlannerNode::load_optimizer_plugins()
{
  {
    const std::string name = "min_rule_based_path_optimizer";
    path_smoother_ = std::make_unique<PathOptimizer>(name, this);
    pub_debug_optimizer_module_trajectories_[path_smoother_->get_name()] =
      this->create_publisher<Trajectory>(
        "~/debug/optimizer/" + path_smoother_->get_name() + "/trajectory", 1);
    RCLCPP_INFO(get_logger(), "Loaded %s", name.c_str());
  }

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
  for (const auto & name : this->declare_parameter<std::vector<std::string>>(
         "modifier_launch_modules", std::vector<std::string>{})) {
    if (name.empty()) continue;
    load_plugin(name);
  }

  initialized_modifiers_ = true;
  RCLCPP_INFO(
    get_logger(), "Trajectory modifier plugins initialized: %zu plugins", modifier_plugins_.size());
}

void MinimumRuleBasedPlannerNode::load_plugin(const std::string & name)
{
  // Check if the plugin is already loaded.
  if (modifier_plugin_loader_.isClassLoaded(name)) {
    RCLCPP_WARN(this->get_logger(), "The plugin '%s' is already loaded.", name.c_str());
    return;
  }
  if (modifier_plugin_loader_.isClassAvailable(name)) {
    const auto plugin = modifier_plugin_loader_.createSharedInstance(name);
    plugin->initialize(name, this, time_keeper_, modifier_data_, vehicle_info_, params_);

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

void MinimumRuleBasedPlannerNode::set_modifier_data(
  const MinimumRuleBasedPlannerNode::InputData & input_data)
{
  modifier_data_->odometry_ptr = input_data.odometry_ptr;
  modifier_data_->acceleration_ptr = input_data.acceleration_ptr;
  modifier_data_->predicted_objects_ptr = input_data.predicted_objects_ptr;
  modifier_data_->obstacle_pointcloud_ptr = input_data.obstacle_pointcloud_ptr;
}

void MinimumRuleBasedPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  auto publish_debug_trajectory = [](
                                    const auto & publisher_map, const auto & plugin,
                                    const TrajectoryPoints & points,
                                    const std_msgs::msg::Header & header) {
    Trajectory traj;
    traj.header = header;
    traj.points = points;
    publisher_map.at(plugin->get_name())->publish(traj);
  };

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
  const auto path = [&]() -> std::optional<PathWithLaneId> {
    autoware_utils_debug::ScopedTimeTrack st("plan_path", *time_keeper_);
    if (input_data.test_path_with_lane_id_ptr) {
      return *input_data.test_path_with_lane_id_ptr;
    }
    return path_planner_->plan_path(
      input_data.odometry_ptr->pose.pose, input_data.odometry_ptr->twist.twist.linear.x);
  }();

  if (!path) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to plan path.");
    return;
  }

  // 3. Convert path to trajectory
  auto traj_from_path =
    path_planner_->convert_path_to_trajectory(*path, params_.path_planning.output.delta_arc_length);

  // 4. Shift trajectory to ego position
  if (params_.path_planning.path_shift.enable) {
    autoware_utils_debug::ScopedTimeTrack st("shift_trajectory_to_ego", *time_keeper_);
    TrajectoryShiftParams shift_params;
    shift_params.minimum_shift_length = params_.path_planning.path_shift.minimum_shift_length;
    shift_params.minimum_shift_yaw = params_.path_planning.path_shift.minimum_shift_yaw;
    shift_params.minimum_shift_distance = params_.path_planning.path_shift.minimum_shift_distance;
    shift_params.min_speed_for_curvature = params_.path_planning.path_shift.min_speed_for_curvature;
    shift_params.lateral_accel_limit = params_.path_planning.path_shift.lateral_accel_limit;

    const double ego_velocity = input_data.odometry_ptr->twist.twist.linear.x;
    const double ego_yaw_rate = input_data.odometry_ptr->twist.twist.angular.z;
    traj_from_path = path_planner_->shift_trajectory_to_ego(
      traj_from_path, input_data.odometry_ptr->pose.pose, ego_velocity, ego_yaw_rate, shift_params,
      params_.path_planning.output.delta_arc_length);

    if (params_.debug.enable_shifted_trajectory) {
      Trajectory shifted_traj;
      shifted_traj.header = path->header;
      shifted_traj.points = traj_from_path.points;
      pub_debug_shifted_trajectory_->publish(shifted_traj);
    }
  }

  // 5. Smooth path
  auto smoothed_path = [&]() {
    autoware_utils_debug::ScopedTimeTrack st("smoothing_path", *time_keeper_);

    // Layer 1 + 2 of the uncrossable_boundary active-set pipeline: cache hit
    // (rebuilt on map change) + AABB filter against the trajectory bounding
    // box. The conservative `uncrossable_boundary_near_path_radius_m` already
    // covers the per-stage radius applied later in PathOptimizer, so we
    // reuse the same param for both visualization and optimisation feed.
    const auto uncrossable_segments = extract_uncrossable_segments_for_optimization(
      traj_from_path, params_.debug.uncrossable_boundary_near_path_radius_m);

    auto trajectory_points = traj_from_path.points;
    if (path_smoother_) {
      autoware_utils_debug::ScopedTimeTrack st(path_smoother_->get_name(), *time_keeper_);
      path_smoother_->optimize_trajectory(
        trajectory_points, *input_data.odometry_ptr, uncrossable_segments);
      if (params_.debug.enable_optimizer_trajectory) {
        publish_debug_trajectory(
          pub_debug_optimizer_module_trajectories_, path_smoother_, trajectory_points,
          traj_from_path.header);
      }
    }

    Trajectory traj = traj_from_path;
    traj.points = trajectory_points;
    return traj;
  }();

  // 6. Apply trajectory modifiers
  {
    set_modifier_data(input_data);
    for (auto & modifier : modifier_plugins_) {
      autoware_utils_debug::ScopedTimeTrack st(modifier->get_name(), *time_keeper_);
      modifier->run(smoothed_path.points);
      modifier->publish_planning_factor();
      if (params_.debug.enable_modifier_trajectory) {
        publish_debug_trajectory(
          pub_debug_modifier_module_trajectories_, modifier, smoothed_path.points,
          smoothed_path.header);
      }
    }
  }

  // 7. Velocity optimization
  const auto smoothed_traj = [&]() {
    autoware_utils_debug::ScopedTimeTrack st("velocity_optimization", *time_keeper_);

    auto trajectory_points = smoothed_path.points;

    velocity_smoother_->optimize(
      trajectory_points, *input_data.odometry_ptr,
      input_data.acceleration_ptr->accel.accel.linear.x);

    // Post-optimization resample
    {
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

    Trajectory traj = traj_from_path;
    traj.points = trajectory_points;
    return traj;
  }();

  // 8. Create and publish CandidateTrajectories message
  {
    CandidateTrajectories msg;

    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_traj;
    candidate_traj.header = path->header;
    candidate_traj.generator_id = generator_uuid_;
    candidate_traj.points = smoothed_traj.points;
    msg.candidate_trajectories.push_back(candidate_traj);

    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = generator_uuid_;
    generator_info.generator_name.data = "MinimumRuleBasedPlanner";
    msg.generator_info.push_back(generator_info);

    pub_trajectories_->publish(msg);
  }

  // 9. Publish debug information if enabled
  if (params_.debug.enable_path) {
    pub_debug_path_->publish(*path);
  }
  if (params_.debug.enable_output_trajectory) {
    pub_debug_trajectory_->publish(smoothed_traj);
  }
  if (params_.debug.enable_lane_boundaries) {
    publish_lane_boundaries_marker();
  }
  if (params_.debug.enable_uncrossable_boundaries) {
    publish_uncrossable_boundaries_marker(traj_from_path);
  }
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

void MinimumRuleBasedPlannerNode::publish_lane_boundaries_marker() const
{
  const auto & route_context = path_planner_->route_context();
  if (route_context.route_lanelets.empty()) {
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id =
    route_context.route_frame_id.empty() ? std::string{"map"} : route_context.route_frame_id;
  header.stamp = this->now();

  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker clear_m;
  clear_m.header = header;
  clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.push_back(clear_m);

  int id = 0;
  for (const auto & lanelet : route_context.route_lanelets) {
    msg.markers.push_back(make_line_strip_marker(
      lanelet.leftBound(), header, "lane_boundary_left", id, 1.0f, 0.4f, 0.4f));
    msg.markers.push_back(make_line_strip_marker(
      lanelet.rightBound(), header, "lane_boundary_right", id, 0.4f, 0.4f, 1.0f));
    ++id;
  }

  pub_debug_lane_boundaries_->publish(msg);
}

void MinimumRuleBasedPlannerNode::update_uncrossable_cache_if_needed()
{
  const auto & route_context = path_planner_->route_context();
  const auto & types_to_detect = params_.debug.uncrossable_boundary_types;

  // Cache key: (map bin pointer, type list). Rebuild only when either changes.
  if (
    cached_uncrossable_map_bin_ == lanelet_map_bin_ptr_ &&
    cached_uncrossable_types_ == types_to_detect) {
    return;
  }

  cached_uncrossable_lines_.clear();
  cached_uncrossable_map_bin_ = lanelet_map_bin_ptr_;
  cached_uncrossable_types_ = types_to_detect;

  if (!route_context.lanelet_map_ptr || types_to_detect.empty()) {
    return;
  }

  for (const auto & ls : route_context.lanelet_map_ptr->lineStringLayer) {
    const auto type = ls.attributeOr(lanelet::AttributeName::Type, std::string{});
    if (
      type.empty() ||
      std::find(types_to_detect.begin(), types_to_detect.end(), type) == types_to_detect.end()) {
      continue;
    }
    const auto basic_ls = ls.basicLineString();
    if (basic_ls.empty()) {
      continue;
    }
    CachedUncrossableLine entry;
    entry.line_string = ls;
    entry.min_x = entry.max_x = basic_ls.front().x();
    entry.min_y = entry.max_y = basic_ls.front().y();
    for (const auto & p : basic_ls) {
      entry.min_x = std::min(entry.min_x, p.x());
      entry.max_x = std::max(entry.max_x, p.x());
      entry.min_y = std::min(entry.min_y, p.y());
      entry.max_y = std::max(entry.max_y, p.y());
    }
    cached_uncrossable_lines_.push_back(std::move(entry));
  }
}

std::vector<UncrossableSegment>
MinimumRuleBasedPlannerNode::extract_uncrossable_segments_for_optimization(
  const Trajectory & ref_trajectory, double radius_m)
{
  std::vector<UncrossableSegment> result;
  if (ref_trajectory.points.empty()) {
    return result;
  }
  update_uncrossable_cache_if_needed();
  if (cached_uncrossable_lines_.empty()) {
    return result;
  }

  // Trajectory AABB expanded by radius_m. Mirrors the publish-side filter so
  // visualization and optimization consider the same neighbourhood when the
  // radii match.
  double tx_min = ref_trajectory.points.front().pose.position.x;
  double tx_max = tx_min;
  double ty_min = ref_trajectory.points.front().pose.position.y;
  double ty_max = ty_min;
  for (const auto & pt : ref_trajectory.points) {
    tx_min = std::min(tx_min, pt.pose.position.x);
    tx_max = std::max(tx_max, pt.pose.position.x);
    ty_min = std::min(ty_min, pt.pose.position.y);
    ty_max = std::max(ty_max, pt.pose.position.y);
  }
  const double bx_min = tx_min - radius_m;
  const double bx_max = tx_max + radius_m;
  const double by_min = ty_min - radius_m;
  const double by_max = ty_max + radius_m;

  for (const auto & e : cached_uncrossable_lines_) {
    if (e.max_x < bx_min || e.min_x > bx_max) continue;
    if (e.max_y < by_min || e.min_y > by_max) continue;
    const auto & ls = e.line_string;
    if (ls.size() < 2) continue;
    for (size_t i = 0; i + 1 < ls.size(); ++i) {
      UncrossableSegment seg;
      seg.p_a.x = ls[i].x();
      seg.p_a.y = ls[i].y();
      seg.p_a.z = 0.0;
      seg.p_b.x = ls[i + 1].x();
      seg.p_b.y = ls[i + 1].y();
      seg.p_b.z = 0.0;
      result.push_back(seg);
    }
  }
  return result;
}

void MinimumRuleBasedPlannerNode::publish_uncrossable_boundaries_marker(
  const Trajectory & ref_trajectory)
{
  // Skip the work entirely when no one is listening — typical for headless runs
  // and avoids the AABB sweep when rviz isn't even subscribed.
  if (pub_debug_uncrossable_boundaries_->get_subscription_count() == 0) {
    return;
  }

  if (ref_trajectory.points.empty()) {
    return;
  }

  update_uncrossable_cache_if_needed();
  if (cached_uncrossable_lines_.empty()) {
    return;
  }

  const auto & route_context = path_planner_->route_context();
  std_msgs::msg::Header header;
  header.frame_id =
    route_context.route_frame_id.empty() ? std::string{"map"} : route_context.route_frame_id;
  header.stamp = this->now();

  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker clear_m;
  clear_m.header = header;
  clear_m.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.push_back(clear_m);

  // Expand the path AABB by the configured radius. Any uncrossable boundary
  // whose AABB intersects this rectangle is guaranteed to include every
  // boundary within `r` of any path point (the converse — that everything
  // selected is within `r` — is intentionally not guaranteed; expanded-bbox
  // is the cheap conservative filter).
  double traj_min_x = ref_trajectory.points.front().pose.position.x;
  double traj_max_x = traj_min_x;
  double traj_min_y = ref_trajectory.points.front().pose.position.y;
  double traj_max_y = traj_min_y;
  for (const auto & pt : ref_trajectory.points) {
    traj_min_x = std::min(traj_min_x, pt.pose.position.x);
    traj_max_x = std::max(traj_max_x, pt.pose.position.x);
    traj_min_y = std::min(traj_min_y, pt.pose.position.y);
    traj_max_y = std::max(traj_max_y, pt.pose.position.y);
  }
  const double r = params_.debug.uncrossable_boundary_near_path_radius_m;
  const double box_min_x = traj_min_x - r;
  const double box_max_x = traj_max_x + r;
  const double box_min_y = traj_min_y - r;
  const double box_max_y = traj_max_y + r;

  int id = 0;
  for (const auto & e : cached_uncrossable_lines_) {
    if (e.max_x < box_min_x || e.min_x > box_max_x) continue;
    if (e.max_y < box_min_y || e.min_y > box_max_y) continue;
    msg.markers.push_back(make_line_strip_marker(
      e.line_string, header, "uncrossable_boundary", id++, 1.0f, 0.6f, 0.0f));
  }

  pub_debug_uncrossable_boundaries_->publish(msg);
}

}  // namespace autoware::minimum_rule_based_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimum_rule_based_planner::MinimumRuleBasedPlannerNode)
