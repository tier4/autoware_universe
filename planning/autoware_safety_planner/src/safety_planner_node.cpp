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

#include "safety_planner_node.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

SafetyPlannerNode::SafetyPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("safety_planner_node", options),
  normal_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  cautious_generator_uuid_(autoware_utils_uuid::generate_uuid()),
  vehicle_info_(vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  param_listener_ =
    std::make_shared<::safety_planner::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  pub_debug_normal_trajectory_ = this->create_publisher<Trajectory>("~/debug/normal/trajectory", 1);
  pub_debug_cautious_trajectory_ =
    this->create_publisher<Trajectory>("~/debug/cautious/trajectory", 1);
  pub_candidate_trajectories_ =
    this->create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);
  pub_debug_marker_ = this->create_publisher<MarkerArray>("~/debug/debug_marker", 1);

  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  planner_ = std::make_unique<SafetyPlanner>(params_, time_keeper_);
  for (const auto & name : planner_->get_constraint_generator_plugin_names()) {
    constraint_debug_marker_publishers_[name] =
      this->create_publisher<MarkerArray>("~/debug/constraints/" + name, 1);
  }

  const auto planning_freq = rclcpp::Rate(10.0);
  timer_ = rclcpp::create_timer(
    this, get_clock(), planning_freq.period(), std::bind(&SafetyPlannerNode::on_timer, this));
}

bool SafetyPlannerNode::is_data_ready(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto notify_waiting = [this](const std::string & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for %s", name.c_str());
  };

  // TODO(odashima): check topic timeout

  if (!input_data.lanelet_map_bin_ptr) {
    notify_waiting("lanelet map");
    return false;
  }
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
  if (!input_data.steering_ptr) {
    notify_waiting("steering");
    return false;
  }

  return true;
}

SafetyPlannerNode::InputData SafetyPlannerNode::take_data()
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

  return input_data;
}

void SafetyPlannerNode::on_timer()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto input_data = take_data();
  if (!is_data_ready(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Waiting for necessary data to plan trajectories.");
    return;
  }

  if (!update_input(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to update input. Skipping planning cycle.");
    return;
  }

  const auto planned = planner_->plan(input_);
  if (!planned) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "%s. Skipping this cycle.",
      planned.error().c_str());

    // TODO(odashima): publish invalid(only 1point) trajectory to prevent selector use old
    // trajectory?

    return;
  }
  const auto & result = planned.value();

  publish_trajectories(result);

  publish_constraints_debug_markers(result.debug.constraint_generator_outputs);

  publish_planner_debug(result.debug);
  publish_debug_markers(result.debug);

  // TODO(odashima): publish planning factors?
  // publish_planning_factors();
}

bool SafetyPlannerNode::update_route_manager(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto & current_pose = input_data.odometry_ptr->pose.pose;

  // Rebuild it when the route or the map is replaced, since the old one no longer applies
  const bool needs_create = !input_.route_manager ||
                            route_uuid_of_route_manager_ != input_data.route_ptr->uuid ||
                            map_ptr_of_route_manager_ != input_data.lanelet_map_bin_ptr;

  if (!needs_create) {
    try {
      input_.route_manager = std::move(*input_.route_manager)
                               .update_current_pose(
                                 current_pose, params_.ego_nearest_lanelet.dist_threshold_m,
                                 params_.ego_nearest_lanelet.yaw_threshold_rad);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000, "update_current_pose threw: %s", e.what());
      input_.route_manager = std::nullopt;
    }
    if (input_.route_manager) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to track current pose on the route. Re-creating RouteManager.");
  }

  input_.route_manager =
    RouteManager::create(*input_data.lanelet_map_bin_ptr, *input_data.route_ptr, current_pose);

  if (!input_.route_manager) {
    route_uuid_of_route_manager_.reset();
    map_ptr_of_route_manager_.reset();
    return false;
  }

  route_uuid_of_route_manager_ = input_data.route_ptr->uuid;
  map_ptr_of_route_manager_ = input_data.lanelet_map_bin_ptr;
  return true;
}

bool SafetyPlannerNode::update_input(const InputData & input_data)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  input_.vehicle_info = vehicle_info_;

  if (!update_route_manager(input_data)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Failed to update RouteManager.");
    return false;
  }

  input_.odometry = *input_data.odometry_ptr;
  input_.acceleration = *input_data.acceleration_ptr;
  input_.steering = *input_data.steering_ptr;
  input_.goal_pose = input_data.route_ptr->goal_pose;
  input_.predicted_objects = input_data.predicted_objects_ptr;

  return true;
}

void SafetyPlannerNode::publish_trajectories(const SafetyPlannerResult & result) const
{
  CandidateTrajectories candidate_trajectories;
  const auto add = [&](
                     const std::optional<PlannedTrajectory> & planned, const UUID & generator_id,
                     const std::string & generator_name) {
    if (!planned) {
      return;
    }
    auto & candidate = candidate_trajectories.candidate_trajectories.emplace_back();
    candidate.header = planned->trajectory.header;
    candidate.generator_id = generator_id;
    candidate.points = planned->trajectory.points;
    candidate.turn_indicators_command = planned->turn_indicators;
    auto & generator_info = candidate_trajectories.generator_info.emplace_back();
    generator_info.generator_id = generator_id;
    generator_info.generator_name.data = generator_name;
  };
  add(result.normal_trajectory, normal_generator_uuid_, "SafetyPlanner_Normal");
  add(result.cautious_trajectory, cautious_generator_uuid_, "SafetyPlanner_Cautious");
  if (candidate_trajectories.candidate_trajectories.empty()) {
    return;
  }
  pub_candidate_trajectories_->publish(candidate_trajectories);

  // for debugging
  if (result.normal_trajectory) {
    pub_debug_normal_trajectory_->publish(result.normal_trajectory->trajectory);
  }
  if (result.cautious_trajectory) {
    pub_debug_cautious_trajectory_->publish(result.cautious_trajectory->trajectory);
  }
}

void SafetyPlannerNode::publish_planner_debug(const SafetyPlannerResult::Debug & debug)
{
  publish_planner_debug("normal", debug.normal);
  publish_planner_debug("cautious", debug.cautious);
}

void SafetyPlannerNode::publish_planner_debug(
  const std::string & side, const TrajectoryPlannerDebug & debug)
{
  for (const auto & [name, trajectory] : debug.trajectories) {
    auto & pub = planner_debug_trajectory_pubs_[side + "/" + name];
    if (!pub) {
      pub = this->create_publisher<Trajectory>("~/debug/" + side + "/" + name, 1);
    }
    pub->publish(trajectory);
  }
  for (const auto & [name, markers] : debug.markers) {
    auto & pub = planner_debug_marker_pubs_[side + "/" + name];
    if (!pub) {
      pub = this->create_publisher<MarkerArray>("~/debug/" + side + "/" + name, 1);
    }
    // The number of markers changes between cycles, which would leave the ones of the namespaces
    // that disappeared behind, so clear them first
    MarkerArray marker_array;
    Marker delete_all;
    delete_all.action = Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);
    marker_array.markers.insert(
      marker_array.markers.end(), markers.markers.begin(), markers.markers.end());
    pub->publish(marker_array);
  }
}

void SafetyPlannerNode::publish_debug_markers(const SafetyPlannerResult::Debug & debug) const
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto now = this->now();
  MarkerArray marker_array;

  // -------------------- the context --------------------
  // current_pose
  {
    auto marker = create_default_marker(
      "map", now, "current_pose", 0, Marker::ARROW, create_marker_scale(2.0, 0.5, 0.5),
      create_marker_color(0.0, 1.0, 0.0, 0.999));
    marker.pose = input_.odometry.pose.pose;
    marker_array.markers.push_back(marker);
  }

  // goal_pose
  {
    auto marker = create_default_marker(
      "map", now, "goal_pose", 0, Marker::ARROW, create_marker_scale(2.0, 0.5, 0.5),
      create_marker_color(1.0, 0.0, 0.0, 0.999));
    marker.pose = input_.goal_pose;
    marker_array.markers.push_back(marker);
  }

  // reference_path
  {
    auto marker = create_default_marker(
      "map", now, "reference_path", 0, Marker::LINE_STRIP, create_marker_scale(0.2, 0.0, 0.0),
      create_marker_color(0.0, 0.5, 1.0, 0.999));
    constexpr double MARKER_INTERVAL_M = 1.0;
    const auto & reference_path = debug.reference_path;
    for (double s = 0.0; s < reference_path.length(); s += MARKER_INTERVAL_M) {
      marker.points.push_back(reference_path.compute(s).point.pose.position);
    }
    if (reference_path.length() > 0.0) {
      marker.points.push_back(reference_path.compute(reference_path.length()).point.pose.position);
    }
    if (marker.points.size() >= 2) {
      marker_array.markers.push_back(marker);
    }
  }

  pub_debug_marker_->publish(marker_array);
}

void SafetyPlannerNode::publish_constraints_debug_markers(
  const std::map<std::string, ConstraintGeneratorOutput> & constraints) const
{
  for (const auto & [plugin_name, output] : constraints) {
    if (!output.debug_markers) {
      continue;
    }
    const auto it = constraint_debug_marker_publishers_.find(plugin_name);
    if (it == constraint_debug_marker_publishers_.end()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "no debug marker publisher for plugin '%s'", plugin_name.c_str());
      continue;
    }
    it->second->publish(*output.debug_markers);
  }
}

}  // namespace autoware::safety_planner

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::safety_planner::SafetyPlannerNode)
