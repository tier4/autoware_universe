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

#include "utils.hpp"

#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/pretty_build.hpp>
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

  // Load planner modules
  load_planner_modules();

  RCLCPP_INFO(get_logger(), "Planner modules loaded.");

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

  // Load plugins based on parameters
  // (order matters: EB smoother -> MPT optimizer -> velocity optimizer)
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

  if (params.mpt_optimizer.enable) {
    try {
      auto plugin = plugin_loader_->createSharedInstance(
        "autoware::trajectory_optimizer::plugin::TrajectoryMPTOptimizer");
      plugin->initialize("mpt_optimizer", this, time_keeper_);
      optimizer_plugins_.push_back(plugin);
      RCLCPP_INFO(get_logger(), "Loaded trajectory MPT optimizer plugin");
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load trajectory MPT optimizer plugin: %s", ex.what());
    }
  }

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

void MinimumRuleBasedPlannerNode::load_planner_modules()
{
  // Declare parameter for planner modules
  const std::vector<std::string> default_modules;
  const auto module_names =
    declare_parameter<std::vector<std::string>>("planner_modules", default_modules);

  for (const auto & name : module_names) {
    if (name.empty()) continue;
    planner_module_manager_.load_module_plugin(*this, name, time_keeper_);
  }
}

std::shared_ptr<rule_based_planner::PlannerData>
MinimumRuleBasedPlannerNode::create_module_planner_data(const InputData & input_data) const
{
  auto module_data = std::make_shared<rule_based_planner::PlannerData>();

  // Copy map and route data
  module_data->lanelet_map_ptr = planner_data_.lanelet_map_ptr;
  module_data->traffic_rules_ptr = planner_data_.traffic_rules_ptr;
  module_data->routing_graph_ptr = planner_data_.routing_graph_ptr;
  module_data->route_frame_id = planner_data_.route_frame_id;
  module_data->goal_pose = planner_data_.goal_pose;
  module_data->route_lanelets = planner_data_.route_lanelets;
  module_data->preferred_lanelets = planner_data_.preferred_lanelets;
  module_data->start_lanelets = planner_data_.start_lanelets;
  module_data->goal_lanelets = planner_data_.goal_lanelets;

  // Copy ego state
  module_data->current_odometry = input_data.odometry_ptr;
  module_data->current_acceleration = input_data.acceleration_ptr;

  // Set vehicle info
  module_data->vehicle_info = vehicle_info_;

  // Set perception data
  module_data->predicted_objects = input_data.predicted_objects_ptr;

  return module_data;
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

  // Convert path to trajectory
  auto traj_from_path =
    utils::convert_path_to_trajectory(planned_path.value(), params.output_delta_arc_length);

  // Apply planner modules (e.g. obstacle stop)
  const auto module_planner_data = create_module_planner_data(input_data);
  const auto planning_results =
    planner_module_manager_.plan(traj_from_path.points, module_planner_data);
  auto insert_stop = [&](
                       autoware_planning_msgs::msg::Trajectory & trajectory,
                       const geometry_msgs::msg::Point & stop_point) {
    // Prevent sudden yaw angle changes by using a larger overlap threshold
    // when inserting stop points that are very close to existing points
    const double overlap_threshold = 5e-2;
    const auto seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(trajectory.points, stop_point);
    const auto insert_idx = autoware::motion_utils::insertTargetPoint(
      seg_idx, stop_point, trajectory.points, overlap_threshold);
    if (insert_idx) {
      for (auto idx = *insert_idx; idx < trajectory.points.size(); ++idx)
        trajectory.points[idx].longitudinal_velocity_mps = 0.0;
    } else {
      RCLCPP_WARN(get_logger(), "Failed to insert stop point");
    }
  };
  for (const auto & planning_result : planning_results) {
    for (const auto & stop_point : planning_result.stop_points)
      insert_stop(traj_from_path, stop_point);
    // for (const auto & slowdown_interval : planning_result.slowdown_intervals)
    // insert_slowdown(output_trajectory_msg, slowdown_interval);
    // if (planning_result.velocity_limit) {
    //   velocity_limit_pub_->publish(*planning_result.velocity_limit);
    // }
    // if (planning_result.velocity_limit_clear_command) {
    //   clear_velocity_limit_pub_->publish(*planning_result.velocity_limit_clear_command);
    // }
  }

  // Apply optimizer plugins
  // TODO(odashima): eb smoother & mpt are must be applied before each planner module?
  const auto smoothed_traj = [&]() {
    trajectory_optimizer::TrajectoryOptimizerData optimizer_data;
    optimizer_data.current_odometry = *input_data.odometry_ptr;
    if (input_data.acceleration_ptr) {
      optimizer_data.current_acceleration = *input_data.acceleration_ptr;
    }

    trajectory_optimizer::TrajectoryOptimizerParams optimizer_params;
    optimizer_params.use_eb_smoother = params.eb_smoother.enable;
    optimizer_params.use_mpt_optimizer = params.mpt_optimizer.enable;
    optimizer_params.use_velocity_optimizer = params.velocity_smoother.enable;

    auto trajectory_points = traj_from_path.points;
    for (const auto & plugin : optimizer_plugins_) {
      plugin->optimize_trajectory(trajectory_points, optimizer_params, optimizer_data);
    }

    if (!trajectory_points.empty()) {
      autoware::motion_utils::calculate_time_from_start(
        trajectory_points, trajectory_points.front().pose.position);
      // autoware::motion_utils::calculate_time_from_start(
      //   trajectory_points, input_data.odometry_ptr->pose.pose.position);
    }

    Trajectory traj = traj_from_path;
    traj.points = trajectory_points;
    return traj;
  }();

  // Create & publish CandidateTrajectories message
  const auto candidate_trajectories = [&]() {
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
  if (!input_data.acceleration_ptr) {
    notify_waiting("acceleration");
    return false;
  }
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
