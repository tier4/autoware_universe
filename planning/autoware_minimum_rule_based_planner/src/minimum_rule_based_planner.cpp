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

#include "path_shift_to_ego.hpp"
#include "plugins/obstacle_stop_modifier.hpp"
#include "utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware/velocity_smoother/resample.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

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

trajectory_modifier::TrajectoryModifierData make_modifier_data(
  const MinimumRuleBasedPlannerNode::InputData & input_data)
{
  trajectory_modifier::TrajectoryModifierData data;
  data.current_odometry = *input_data.odometry_ptr;
  if (input_data.acceleration_ptr) {
    data.current_acceleration = *input_data.acceleration_ptr;
  }
  return data;
}
}  // namespace

MinimumRuleBasedPlannerNode::MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("minimum_rule_based_planner_node", options),
  generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  param_listener_ =
    std::make_shared<::minimum_rule_based_planner::ParamListener>(get_node_parameters_interface());

  pub_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_debug_path_ = this->create_publisher<PathWithLaneId>("~/debug/path_with_lane_id", 1);
  pub_debug_trajectory_ = this->create_publisher<Trajectory>("~/debug/trajectory", 1);
  pub_debug_shifted_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/shifted_trajectory", 1);
  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  load_optimizer_plugins();
  load_modifier_plugins();

  const auto params = param_listener_->get_params();
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_frequency_hz).period(),
    std::bind(&MinimumRuleBasedPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "Minimum Rule Based Planner Node has been started.");
}

void MinimumRuleBasedPlannerNode::load_optimizer_plugins()
{
  const auto params = param_listener_->get_params();

  // Create plugin loader for autoware_trajectory_optimizer
  plugin_loader_ = std::make_unique<PluginLoader>(
    "autoware_trajectory_optimizer",
    "autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase");

  auto try_load_optimizer_plugin = [&](
                                     const std::string & plugin_path,
                                     const std::string & name) -> std::shared_ptr<PluginInterface> {
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

  if (params.eb_smoother.enable) {
    path_smoother_ = try_load_optimizer_plugin(
      "autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer", "eb_smoother");
  }

  if (params.velocity_smoother.enable) {
    velocity_optimizer_ = try_load_optimizer_plugin(
      "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer", "velocity_optimizer");
  }
}

void MinimumRuleBasedPlannerNode::load_modifier_plugins()
{
  auto modifier = std::make_shared<plugin::ObstacleStopModifier>(
    "obstacle_stop", this, time_keeper_, modifier_params_);
  pub_debug_modifier_module_trajectories_[modifier->get_name()] =
    this->create_publisher<Trajectory>(
      "~/debug/modifier/" + modifier->get_name() + "/trajectory", 1);
  trajectory_modifiers_.push_back(std::move(modifier));
}

void MinimumRuleBasedPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto params = param_listener_->get_params();

  auto publish_debug_trajectory =
    [](const auto & publisher_map, const auto & plugin, const TrajectoryPoints & points) {
      Trajectory traj;
      traj.points = points;
      publisher_map.at(plugin->get_name())->publish(traj);
    };

  // 1. Check data availability
  const auto input_data = take_data();
  set_planner_data(input_data);
  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for necessary data to plan trajectories.");
    return;
  }

  // 2. Get path
  const auto planned_path = [&]() -> std::optional<PathWithLaneId> {
    autoware_utils_debug::ScopedTimeTrack st("plan_path", *time_keeper_);
    if (input_data.test_path_with_lane_id_ptr) {
      return *input_data.test_path_with_lane_id_ptr;
    }
    return plan_path(input_data);
  }();

  if (!planned_path) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to plan path.");
    return;
  }

  // 3. Convert path to trajectory
  auto traj_from_path =
    utils::convert_path_to_trajectory(*planned_path, params.output_delta_arc_length);

  // 4. Shift trajectory to ego position
  if (params.path_shift.enable) {
    autoware_utils_debug::ScopedTimeTrack st("shift_trajectory_to_ego", *time_keeper_);
    const TrajectoryShiftParams shift_params{
      params.path_shift.minimum_shift_length, params.path_shift.minimum_shift_distance,
      params.path_shift.shift_length_to_distance_ratio};

    traj_from_path = shift_trajectory_to_ego(
      traj_from_path, input_data.odometry_ptr->pose.pose, shift_params,
      params.output_delta_arc_length);

    // デバッグ用にpublishする
    Trajectory shifted_traj;
    shifted_traj.header = planned_path->header;
    shifted_traj.points = traj_from_path.points;
    pub_debug_shifted_trajectory_->publish(shifted_traj);
  }

  // 5. Smooth path
  auto smoothed_path = [&]() {
    autoware_utils_debug::ScopedTimeTrack st("smoothing_path", *time_keeper_);
    const auto optimizer_data = make_optimizer_data(input_data);

    trajectory_optimizer::TrajectoryOptimizerParams optimizer_params;
    optimizer_params.use_eb_smoother = params.eb_smoother.enable;

    auto trajectory_points = traj_from_path.points;
    if (path_smoother_) {
      autoware_utils_debug::ScopedTimeTrack st(path_smoother_->get_name(), *time_keeper_);
      path_smoother_->optimize_trajectory(trajectory_points, optimizer_params, optimizer_data);
      publish_debug_trajectory(
        pub_debug_optimizer_module_trajectories_, path_smoother_, trajectory_points);
    }

    Trajectory traj = traj_from_path;
    traj.points = trajectory_points;
    return traj;
  }();

  // 6. Apply trajectory modifiers
  {
    const auto modifier_data = make_modifier_data(input_data);
    for (auto & modifier : trajectory_modifiers_) {
      // TODO(odashima): add predicted_objects_ptr (and pointcloud ptr) into modifier input_data
      if (auto * obs = dynamic_cast<plugin::ObstacleStopModifier *>(modifier.get())) {
        obs->set_predicted_objects(input_data.predicted_objects_ptr);
      }
      autoware_utils_debug::ScopedTimeTrack st(modifier->get_name(), *time_keeper_);
      modifier->modify_trajectory(smoothed_path.points, modifier_params_, modifier_data);
      modifier->publish_planning_factor();
      publish_debug_trajectory(
        pub_debug_modifier_module_trajectories_, modifier, smoothed_path.points);
    }
  }

  // 7. Velocity optimization
  const auto smoothed_traj = [&]() {
    autoware_utils_debug::ScopedTimeTrack st("velocity_optimization", *time_keeper_);
    const auto optimizer_data = make_optimizer_data(input_data);

    trajectory_optimizer::TrajectoryOptimizerParams optimizer_params;
    optimizer_params.use_velocity_optimizer = params.velocity_smoother.enable;

    auto trajectory_points = smoothed_path.points;
    if (velocity_optimizer_) {
      velocity_optimizer_->optimize_trajectory(trajectory_points, optimizer_params, optimizer_data);
    }

    // Post-optimization resample (same as autoware_velocity_smoother)
    {
      autoware::velocity_smoother::resampling::ResampleParam post_resample_param;
      post_resample_param.max_trajectory_length = params.post_resample.max_trajectory_length;
      post_resample_param.min_trajectory_length = params.post_resample.min_trajectory_length;
      post_resample_param.resample_time = params.post_resample.resample_time;
      post_resample_param.dense_resample_dt = params.post_resample.dense_resample_dt;
      post_resample_param.dense_min_interval_distance =
        params.post_resample.dense_min_interval_distance;
      post_resample_param.sparse_resample_dt = params.post_resample.sparse_resample_dt;
      post_resample_param.sparse_min_interval_distance =
        params.post_resample.sparse_min_interval_distance;

      const auto & ego_pose = input_data.odometry_ptr->pose.pose;
      const double v_current = input_data.odometry_ptr->twist.twist.linear.x;

      trajectory_points = autoware::velocity_smoother::resampling::resampleTrajectory(
        trajectory_points, v_current, ego_pose, params.lanelet_ego_nearest_dist_threshold,
        params.lanelet_ego_nearest_yaw_threshold, post_resample_param, false);

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
    candidate_traj.header = planned_path->header;
    candidate_traj.generator_id = generator_uuid_;
    candidate_traj.points = smoothed_traj.points;
    msg.candidate_trajectories.push_back(candidate_traj);

    autoware_internal_planning_msgs::msg::GeneratorInfo generator_info;
    generator_info.generator_id = generator_uuid_;
    generator_info.generator_name.data = "minimum_rule_based_planner";
    msg.generator_info.push_back(generator_info);

    pub_trajectories_->publish(msg);
  }

  // 9. Publish debug information if enabled
  if (params.enable_debug_path) {
    pub_debug_path_->publish(*planned_path);
  }
  if (params.enable_debug_trajectory) {
    pub_debug_trajectory_->publish(smoothed_traj);
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

void MinimumRuleBasedPlannerNode::set_planner_data(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (input_data.lanelet_map_bin_ptr && !route_context_.lanelet_map_ptr) {
    route_context_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, route_context_.lanelet_map_ptr,
      &route_context_.traffic_rules_ptr, &route_context_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    set_route(input_data.route_ptr);
  }
}

void MinimumRuleBasedPlannerNode::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  route_context_.route_frame_id = route_ptr->header.frame_id;
  route_context_.goal_pose = route_ptr->goal_pose;

  route_context_.route_lanelets.clear();
  route_context_.preferred_lanelets.clear();
  route_context_.start_lanelets.clear();
  route_context_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  route_context_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & lanelet = route_context_.lanelet_map_ptr->laneletLayer.get(id);
      route_context_.route_lanelets.push_back(lanelet);
      if (id == route_section.preferred_primitive.id) {
        route_context_.preferred_lanelets.push_back(lanelet);
      }
    }
  }

  const auto set_lanelets_from_segment =
    [&](
      const autoware_planning_msgs::msg::LaneletSegment & segment,
      lanelet::ConstLanelets & lanelets) {
      lanelets.reserve(segment.primitives.size());
      for (const auto & primitive : segment.primitives) {
        const auto & lanelet = route_context_.lanelet_map_ptr->laneletLayer.get(primitive.id);
        lanelets.push_back(lanelet);
      }
    };
  set_lanelets_from_segment(route_ptr->segments.front(), route_context_.start_lanelets);
  set_lanelets_from_segment(route_ptr->segments.back(), route_context_.goal_lanelets);
}

bool MinimumRuleBasedPlannerNode::update_current_lanelet(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!current_lanelet_) {
    lanelet::ConstLanelet current_lanelet;
    if (lanelet::utils::query::getClosestLanelet(
          route_context_.route_lanelets, current_pose, &current_lanelet)) {
      current_lanelet_ = current_lanelet;
      return true;
    }
    return false;
  }

  lanelet::ConstLanelets candidates;
  if (
    const auto previous_lanelet =
      utils::get_previous_lanelet_within_route(*current_lanelet_, route_context_)) {
    candidates.push_back(*previous_lanelet);
  }
  candidates.push_back(*current_lanelet_);
  if (
    const auto next_lanelet =
      utils::get_next_lanelet_within_route(*current_lanelet_, route_context_)) {
    candidates.push_back(*next_lanelet);
  }

  // Include adjacent route lanelets so that ego transitions to the correct lane
  // during lane changes (e.g., 2→4) instead of being picked up by a longitudinally
  // adjacent lanelet on the wrong lane (e.g., 3)
  for (const auto & beside : route_context_.routing_graph_ptr->besides(*current_lanelet_)) {
    if (
      beside.id() != current_lanelet_->id() &&
      std::any_of(
        route_context_.route_lanelets.begin(), route_context_.route_lanelets.end(),
        [&](const auto & rl) { return rl.id() == beside.id(); })) {
      candidates.push_back(beside);
    }
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        candidates, current_pose, &*current_lanelet_, params.lanelet_ego_nearest_dist_threshold,
        params.lanelet_ego_nearest_yaw_threshold)) {
    return true;
  }

  if (lanelet::utils::query::getClosestLanelet(
        route_context_.route_lanelets, current_pose, &*current_lanelet_)) {
    return true;
  }

  return false;
}

}  // namespace autoware::minimum_rule_based_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimum_rule_based_planner::MinimumRuleBasedPlannerNode)
