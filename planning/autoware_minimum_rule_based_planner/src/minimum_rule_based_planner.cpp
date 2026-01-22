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

#include "minimum_rule_based_planner/minimum_rule_based_planner.hpp"

#include "minimum_rule_based_planner/utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner
{

MinimumRuleBasedPlannerNode::MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("minimum_rule_based_planner_node", options),
  generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  RCLCPP_INFO(get_logger(), "Minimum Rule Based Planner Node has been started.");

  // Initialize parameter listener
  param_listener_ =
    std::make_shared<::minimum_rule_based_planner::ParamListener>(get_node_parameters_interface());

  RCLCPP_INFO(get_logger(), "Parameter listener initialized.");

  // Initialize the node
  pub_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_debug_path_ = this->create_publisher<PathWithLaneId>("~/debug/path_with_lane_id", 1);
  pub_debug_trajectory_ = this->create_publisher<Trajectory>("~/debug/trajectory", 1);
  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  RCLCPP_INFO(get_logger(), "Publishers and TimeKeeper initialized.");

  // Load optimizer plugins
  load_optimizer_plugins();

  RCLCPP_INFO(get_logger(), "Optimizer plugins loaded.");

  const auto params = param_listener_->get_params();

  RCLCPP_INFO(get_logger(), "Creating timer with frequency: %f Hz", params.planning_frequency_hz);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_frequency_hz).period(),
    std::bind(&MinimumRuleBasedPlannerNode::on_timer, this));
}

void MinimumRuleBasedPlannerNode::load_optimizer_plugins()
{
  const auto params = param_listener_->get_params();

  // Create plugin loader for autoware_trajectory_optimizer
  plugin_loader_ = std::make_unique<PluginLoader>(
    "autoware_trajectory_optimizer",
    "autoware::trajectory_optimizer::plugin::TrajectoryOptimizerPluginBase");

  // Load plugins based on parameters (order matters: EB smoother first, then velocity optimizer)

  // 1. Elastic Band Smoother - path smoothing
  if (params.eb_smoother.enable) {
    try {
      auto plugin = plugin_loader_->createSharedInstance(
        "autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer");
      plugin->initialize("eb_smoother", this, time_keeper_);
      optimizer_plugins_.push_back(plugin);
      RCLCPP_INFO(get_logger(), "Loaded trajectory EB smoother optimizer plugin");
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load trajectory EB smoother plugin: %s", ex.what());
    }
  }

  // 2. Velocity Optimizer - velocity profile optimization
  if (params.velocity_smoother.enable) {
    try {
      auto plugin = plugin_loader_->createSharedInstance(
        "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer");
      plugin->initialize("velocity_optimizer", this, time_keeper_);
      optimizer_plugins_.push_back(plugin);
      RCLCPP_INFO(get_logger(), "Loaded trajectory velocity optimizer plugin");
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to load trajectory velocity optimizer plugin: %s", ex.what());
    }
  }
}

void MinimumRuleBasedPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack time_keeper_scope(__func__, *time_keeper_);
  const auto params = param_listener_->get_params();

  // Check data availability
  const auto input_data = take_data();
  set_planner_data(input_data);
  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for necessary data to plan trajectories.");
    return;
  }

  // Plan path
  const auto planned_path = plan_path(input_data);
  if (!planned_path) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to plan path.");
    return;
  }

  // TODO(odashima): path_smoother?

  // Convert path to trajectory (instead of path optimizer)
  const auto traj_from_path =
    convert_path_to_trajectory(planned_path.value(), params.output_delta_arc_length);

  // TODO(odashima): Apply each planner module (e.g. obstacle stop)

  // Apply optimizer plugins
  trajectory_optimizer::TrajectoryOptimizerData optimizer_data;
  optimizer_data.current_odometry = *input_data.odometry_ptr;
  if (input_data.acceleration_ptr) {
    optimizer_data.current_acceleration = *input_data.acceleration_ptr;
  }

  trajectory_optimizer::TrajectoryOptimizerParams optimizer_params;
  optimizer_params.use_eb_smoother = params.eb_smoother.enable;
  optimizer_params.use_velocity_optimizer = params.velocity_smoother.enable;

  auto trajectory_points = traj_from_path.points;

  // Set terminal velocity to 0 (TrajectoryVelocityOptimizer expects this to be done externally)
  if (!trajectory_points.empty()) {
    trajectory_points.back().longitudinal_velocity_mps = 0.0;
  }

  // Apply optimizer plugins (in-place modification)
  for (const auto & plugin : optimizer_plugins_) {
    plugin->optimize_trajectory(trajectory_points, optimizer_params, optimizer_data);
  }

  // Calculate time_from_start (TrajectoryVelocityOptimizer doesn't calculate this)
  if (!trajectory_points.empty()) {
    autoware::motion_utils::calculate_time_from_start(
      trajectory_points, trajectory_points.front().pose.position);
  }

  Trajectory smoothed_traj = traj_from_path;
  smoothed_traj.points = trajectory_points;

  // Create & publish CandidateTrajectories message
  const auto candidate_trajectories = [&]() {
    CandidateTrajectories msg;
    autoware_internal_planning_msgs::msg::CandidateTrajectory candidate_traj;
    candidate_traj.header = planned_path->header;
    candidate_traj.generator_id = generator_uuid_;
    candidate_traj.points = smoothed_traj.points;
    msg.candidate_trajectories.push_back(candidate_traj);
    return msg;
  }();
  pub_trajectories_->publish(candidate_trajectories);

  // Publish debug information if enabled
  if (params.enable_debug_path) {
    pub_debug_path_->publish(planned_path.value());
  }
  if (params.enable_debug_trajectory) {
    pub_debug_trajectory_->publish(smoothed_traj);
  }
}

MinimumRuleBasedPlannerNode::InputData MinimumRuleBasedPlannerNode::take_data()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  InputData input_data;

  // route - keep previous data if no new data
  if (const auto msg = route_subscriber_.take_data()) {
    if (!msg->segments.empty()) {
      route_ptr_ = msg;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "input route is empty, ignoring...");
    }
  }
  input_data.route_ptr = route_ptr_;

  // map - keep previous data if no new data
  if (const auto msg = vector_map_subscriber_.take_data()) {
    lanelet_map_bin_ptr_ = msg;
  }
  input_data.lanelet_map_bin_ptr = lanelet_map_bin_ptr_;

  // velocity - always use latest
  if (const auto msg = odometry_subscriber_.take_data()) {
    odometry_ptr_ = msg;
  }
  input_data.odometry_ptr = odometry_ptr_;

  // acceleration - always use latest
  if (const auto msg = acceleration_subscriber_.take_data()) {
    acceleration_ptr_ = msg;
  }
  input_data.acceleration_ptr = acceleration_ptr_;

  return input_data;
}

bool MinimumRuleBasedPlannerNode::is_data_ready(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  if (!input_data.route_ptr) {
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
  // acceleration is optional - don't require it
  return true;
}

void MinimumRuleBasedPlannerNode::set_planner_data(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (input_data.lanelet_map_bin_ptr && !planner_data_.lanelet_map_ptr) {
    planner_data_.lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(
      *input_data.lanelet_map_bin_ptr, planner_data_.lanelet_map_ptr,
      &planner_data_.traffic_rules_ptr, &planner_data_.routing_graph_ptr);
  }

  if (input_data.route_ptr) {
    set_route(input_data.route_ptr);
  }
}

void MinimumRuleBasedPlannerNode::set_route(const LaneletRoute::ConstSharedPtr & route_ptr)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  planner_data_.route_frame_id = route_ptr->header.frame_id;
  planner_data_.goal_pose = route_ptr->goal_pose;

  planner_data_.route_lanelets.clear();
  planner_data_.preferred_lanelets.clear();
  planner_data_.start_lanelets.clear();
  planner_data_.goal_lanelets.clear();

  size_t primitives_num = 0;
  for (const auto & route_section : route_ptr->segments) {
    primitives_num += route_section.primitives.size();
  }
  planner_data_.route_lanelets.reserve(primitives_num);

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(id);
      planner_data_.route_lanelets.push_back(lanelet);
      if (id == route_section.preferred_primitive.id) {
        planner_data_.preferred_lanelets.push_back(lanelet);
      }
    }
  }

  const auto set_lanelets_from_segment =
    [&](
      const autoware_planning_msgs::msg::LaneletSegment & segment,
      lanelet::ConstLanelets & lanelets) {
      lanelets.reserve(segment.primitives.size());
      for (const auto & primitive : segment.primitives) {
        const auto & lanelet = planner_data_.lanelet_map_ptr->laneletLayer.get(primitive.id);
        lanelets.push_back(lanelet);
      }
    };
  set_lanelets_from_segment(route_ptr->segments.front(), planner_data_.start_lanelets);
  set_lanelets_from_segment(route_ptr->segments.back(), planner_data_.goal_lanelets);
}

std::optional<PathWithLaneId> MinimumRuleBasedPlannerNode::plan_path(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto & current_pose = input_data.odometry_ptr->pose.pose;
  const auto params = param_listener_->get_params();
  const auto path_length_backward = params.path_length.backward;
  const auto path_length_forward = params.path_length.forward;

  if (!update_current_lanelet(current_pose, params)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to update current lanelet");
    return std::nullopt;
  }

  lanelet::ConstLanelets lanelets{*current_lanelet_};
  const auto s_on_current_lanelet =
    lanelet::utils::getArcCoordinates({*current_lanelet_}, current_pose).length;

  const auto backward_length = std::max(
    0., path_length_backward + vehicle_info_.max_longitudinal_offset_m - s_on_current_lanelet);
  const auto backward_lanelets_within_route =
    utils::get_lanelets_within_route_up_to(*current_lanelet_, planner_data_, backward_length);
  if (!backward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get backward lanelets within route for current lanelet (id: %ld)",
      current_lanelet_->id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.begin(), backward_lanelets_within_route->begin(),
    backward_lanelets_within_route->end());

  //  Extend lanelets by backward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the beginning of start lane
  auto backward_lanelets_length =
    lanelet::utils::getLaneletLength2d(*backward_lanelets_within_route);
  while (backward_lanelets_length < backward_length) {
    const auto prev_lanelets = planner_data_.routing_graph_ptr->previous(lanelets.front());
    if (prev_lanelets.empty()) {
      break;
    }
    lanelets.insert(lanelets.begin(), prev_lanelets.front());
    backward_lanelets_length += lanelet::geometry::length2d(prev_lanelets.front());
  }

  const auto forward_length = std::max(
    0., path_length_forward + vehicle_info_.max_longitudinal_offset_m -
          (lanelet::geometry::length2d(*current_lanelet_) - s_on_current_lanelet));
  const auto forward_lanelets_within_route =
    utils::get_lanelets_within_route_after(*current_lanelet_, planner_data_, forward_length);
  if (!forward_lanelets_within_route) {
    RCLCPP_ERROR(
      get_logger(), "Failed to get forward lanelets within route for current lanelet (id: %ld)",
      current_lanelet_->id());
    return std::nullopt;
  }
  lanelets.insert(
    lanelets.end(), forward_lanelets_within_route->begin(), forward_lanelets_within_route->end());

  //  Extend lanelets by forward_length even outside planned route to ensure
  //  ego footprint is inside lanelets if ego is at the end of goal lane
  auto forward_lanelets_length = lanelet::utils::getLaneletLength2d(*forward_lanelets_within_route);
  while (forward_lanelets_length < forward_length) {
    const auto next_lanelets = planner_data_.routing_graph_ptr->following(lanelets.back());
    if (next_lanelets.empty()) {
      break;
    }
    lanelets.insert(lanelets.end(), next_lanelets.front());
    forward_lanelets_length += lanelet::geometry::length2d(next_lanelets.front());
  }

  const auto s = s_on_current_lanelet + backward_lanelets_length;
  const auto s_start = std::max(0., s - path_length_backward);
  const auto s_end = [&]() {
    auto s_end_val = s + path_length_forward;

    if (!utils::get_next_lanelet_within_route(lanelets.back(), planner_data_)) {
      s_end_val = std::min(s_end_val, lanelet::utils::getLaneletLength2d(lanelets));
    }

    for (auto [it, goal_arc_length] = std::make_tuple(lanelets.begin(), 0.); it != lanelets.end();
         ++it) {
      if (std::any_of(
            planner_data_.goal_lanelets.begin(), planner_data_.goal_lanelets.end(),
            [&](const auto & goal_lanelet) { return it->id() == goal_lanelet.id(); })) {
        goal_arc_length += lanelet::utils::getArcCoordinates({*it}, planner_data_.goal_pose).length;
        s_end_val = std::min(s_end_val, goal_arc_length);
        break;
      }
      goal_arc_length += lanelet::geometry::length2d(*it);
    }

    const lanelet::LaneletSequence lanelet_seq(lanelets);
    if (
      const auto s_intersection = utils::get_first_intersection_arc_length(
        lanelet_seq, std::max(0., s_start - vehicle_info_.max_longitudinal_offset_m),
        s_end_val + vehicle_info_.max_longitudinal_offset_m, vehicle_info_.vehicle_length_m)) {
      s_end_val = std::min(
        s_end_val, std::max(0., *s_intersection - vehicle_info_.max_longitudinal_offset_m));
    }

    return s_end_val;
  }();

  return generate_path(lanelets, s_start, s_end, params);
}

std::optional<PathWithLaneId> MinimumRuleBasedPlannerNode::generate_path(
  const lanelet::LaneletSequence & lanelet_sequence, const double s_start, const double s_end,
  const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  if (lanelet_sequence.empty()) {
    RCLCPP_ERROR(get_logger(), "Lanelet sequence is empty");
    return std::nullopt;
  }

  std::vector<PathPointWithLaneId> path_points_with_lane_id{};

  const auto waypoint_groups = utils::get_waypoint_groups(
    lanelet_sequence, *planner_data_.lanelet_map_ptr, params.waypoint_group.separation_threshold,
    params.waypoint_group.interval_margin_ratio);

  auto extended_lanelets = lanelet_sequence.lanelets();
  auto extended_arc_length = 0.;
  for (const auto & [waypoints, interval] : waypoint_groups) {
    while (interval.start + extended_arc_length < 0.) {
      const auto prev_lanelets =
        planner_data_.routing_graph_ptr->previous(extended_lanelets.front());
      if (prev_lanelets.empty()) {
        break;
      }
      extended_lanelets.insert(extended_lanelets.begin(), prev_lanelets.front());
      extended_arc_length += lanelet::utils::getLaneletLength2d(prev_lanelets.front());
    }
  }

  const auto add_path_point =
    [&](const lanelet::ConstPoint3d & path_point, const lanelet::Id & lane_id) {
      PathPointWithLaneId path_point_with_lane_id{};
      path_point_with_lane_id.lane_ids.push_back(lane_id);
      path_point_with_lane_id.point.pose.position =
        lanelet::utils::conversion::toGeomMsgPt(path_point);
      path_point_with_lane_id.point.longitudinal_velocity_mps =
        planner_data_.traffic_rules_ptr
          ->speedLimit(planner_data_.lanelet_map_ptr->laneletLayer.get(lane_id))
          .speedLimit.value();
      path_points_with_lane_id.push_back(std::move(path_point_with_lane_id));
    };

  const lanelet::LaneletSequence extended_lanelet_sequence(extended_lanelets);
  std::optional<size_t> overlapping_waypoint_group_index = std::nullopt;

  for (auto [lanelet_it, s] = std::make_tuple(extended_lanelet_sequence.begin(), 0.);
       lanelet_it != extended_lanelet_sequence.end(); ++lanelet_it) {
    const auto & centerline = lanelet_it->centerline();

    for (auto point_it = centerline.begin(); point_it != centerline.end(); ++point_it) {
      if (point_it != centerline.begin()) {
        s += lanelet::geometry::distance2d(*std::prev(point_it), *point_it);
      } else if (lanelet_it != extended_lanelet_sequence.begin()) {
        continue;
      }

      if (overlapping_waypoint_group_index) {
        const auto & [waypoints, interval] = waypoint_groups[*overlapping_waypoint_group_index];
        if (s >= interval.start + extended_arc_length && s <= interval.end + extended_arc_length) {
          continue;
        }
        overlapping_waypoint_group_index = std::nullopt;
      }

      for (size_t i = 0; i < waypoint_groups.size(); ++i) {
        const auto & [waypoints, interval] = waypoint_groups[i];
        if (s < interval.start + extended_arc_length || s > interval.end + extended_arc_length) {
          continue;
        }
        for (const auto & waypoint : waypoints) {
          add_path_point(waypoint.point, waypoint.lane_id);
        }
        overlapping_waypoint_group_index = i;
        break;
      }
      if (overlapping_waypoint_group_index) {
        continue;
      }

      add_path_point(*point_it, lanelet_it->id());
      if (
        point_it == std::prev(centerline.end()) &&
        lanelet_it != std::prev(extended_lanelet_sequence.end())) {
        if (
          lanelet_it != extended_lanelet_sequence.begin() ||
          lanelet_it->id() == lanelet_sequence.begin()->id()) {
          path_points_with_lane_id.back().lane_ids.push_back(std::next(lanelet_it)->id());
        } else {
          path_points_with_lane_id.back().lane_ids = {std::next(lanelet_it)->id()};
        }
      }
    }
  }

  if (path_points_with_lane_id.empty()) {
    RCLCPP_ERROR(get_logger(), "No path points generated from lanelet sequence");
    return std::nullopt;
  }

  auto trajectory = autoware::experimental::trajectory::pretty_build(path_points_with_lane_id);
  if (!trajectory) {
    RCLCPP_ERROR(get_logger(), "Failed to build trajectory from path points");
    return std::nullopt;
  }

  // Attach orientation for all the points
  trajectory->align_orientation_with_trajectory_direction();

  const auto s_path_start = utils::get_arc_length_on_path(
    extended_lanelet_sequence, path_points_with_lane_id, extended_arc_length + s_start);
  const auto s_path_end = utils::get_arc_length_on_path(
    extended_lanelet_sequence, path_points_with_lane_id, extended_arc_length + s_end);

  // Refine the trajectory by cropping
  if (trajectory->length() - s_path_end > 0) {
    trajectory->crop(0., s_path_end);
  }

  // Check if the goal point is in the search range
  // Note: We only see if the goal is approaching the tail of the path.
  const auto distance_to_goal = autoware_utils::calc_distance2d(
    trajectory->compute(trajectory->length()), planner_data_.goal_pose);

  if (distance_to_goal < params.smooth_goal_connection.search_radius_range) {
    auto refined_path = utils::modify_path_for_smooth_goal_connection(
      *trajectory, planner_data_, params.smooth_goal_connection.search_radius_range,
      params.smooth_goal_connection.pre_goal_offset);

    if (refined_path) {
      refined_path->align_orientation_with_trajectory_direction();
      *trajectory = *refined_path;
    }
  }

  if (trajectory->length() - s_path_start > 0) {
    trajectory->crop(s_path_start, trajectory->length() - s_path_start);
  }

  // Compose the polished path
  PathWithLaneId finalized_path_with_lane_id{};
  finalized_path_with_lane_id.points = trajectory->restore();

  if (finalized_path_with_lane_id.points.empty()) {
    RCLCPP_ERROR(get_logger(), "Finalized path points are empty after cropping");
    return std::nullopt;
  }

  // Set header which is needed to engage
  finalized_path_with_lane_id.header.frame_id = planner_data_.route_frame_id;
  finalized_path_with_lane_id.header.stamp = now();

  const auto [left_bound, right_bound] = utils::get_path_bounds(
    extended_lanelet_sequence,
    std::max(0., extended_arc_length + s_start - vehicle_info_.max_longitudinal_offset_m),
    extended_arc_length + s_end + vehicle_info_.max_longitudinal_offset_m);
  finalized_path_with_lane_id.left_bound = left_bound;
  finalized_path_with_lane_id.right_bound = right_bound;

  return finalized_path_with_lane_id;
}

Trajectory MinimumRuleBasedPlannerNode::convert_path_to_trajectory(
  const PathWithLaneId & path, double resample_interval)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> traj_points;
  traj_points.reserve(path.points.size());
  for (const auto & path_point : path.points) {
    autoware_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose = path_point.point.pose;
    traj_point.longitudinal_velocity_mps = path_point.point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = path_point.point.lateral_velocity_mps;
    traj_point.heading_rate_rps = path_point.point.heading_rate_rps;
    traj_points.push_back(traj_point);
  }

  const auto traj_msg = autoware::motion_utils::convertToTrajectory(traj_points, path.header);
  return autoware::motion_utils::resampleTrajectory(
    traj_msg, resample_interval, false, true, true, true);
}

bool MinimumRuleBasedPlannerNode::update_current_lanelet(
  const geometry_msgs::msg::Pose & current_pose, const Params & params)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  if (!current_lanelet_) {
    lanelet::ConstLanelet current_lanelet;
    if (lanelet::utils::query::getClosestLanelet(
          planner_data_.route_lanelets, current_pose, &current_lanelet)) {
      current_lanelet_ = current_lanelet;
      return true;
    }
    return false;
  }

  lanelet::ConstLanelets candidates;
  if (
    const auto previous_lanelet =
      utils::get_previous_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*previous_lanelet);
  }
  candidates.push_back(*current_lanelet_);
  if (
    const auto next_lanelet =
      utils::get_next_lanelet_within_route(*current_lanelet_, planner_data_)) {
    candidates.push_back(*next_lanelet);
  }

  if (lanelet::utils::query::getClosestLaneletWithConstrains(
        candidates, current_pose, &*current_lanelet_, params.lanelet_ego_nearest_dist_threshold,
        params.lanelet_ego_nearest_yaw_threshold)) {
    return true;
  }

  if (lanelet::utils::query::getClosestLanelet(
        planner_data_.route_lanelets, current_pose, &*current_lanelet_)) {
    return true;
  }

  return false;
}

}  // namespace autoware::minimum_rule_based_planner

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::minimum_rule_based_planner::MinimumRuleBasedPlannerNode)
