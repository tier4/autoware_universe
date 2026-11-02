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

#include "autoware/trajectory_modifier/trajectory_modifier.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier
{

TrajectoryModifier::TrajectoryModifier(const rclcpp::NodeOptions & options)
: Node{"trajectory_modifier", options},
  param_listener_{
    std::make_unique<trajectory_modifier_params::ParamListener>(get_node_parameters_interface())},
  plugin_loader_(
    "autoware_trajectory_modifier",
    "autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase"),
  context_{std::make_shared<TrajectoryModifierContext>(this)}
{
  sub_map_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryModifier::on_map, this, std::placeholders::_1));

  trajectories_sub_ = create_subscription<CandidateTrajectories>(
    "~/input/candidate_trajectories", 1,
    std::bind(&TrajectoryModifier::on_traj, this, std::placeholders::_1));
  trajectories_pub_ = create_publisher<CandidateTrajectories>("~/output/candidate_trajectories", 1);

  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail", 1);

  pub_processing_time_ = std::make_shared<autoware_utils_debug::DebugPublisher>(this, "~/debug");

  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  params_ = param_listener_->get_params();

  // initialize plugins
  for (const auto & name : params_.plugin_names) {
    if (name.empty()) continue;
    load_plugin(name);
  }

  RCLCPP_INFO(get_logger(), "TrajectoryModifier initialized");
}

void TrajectoryModifier::on_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_bin_ptr_ = msg;
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
}

void TrajectoryModifier::on_traj(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic(__func__);

  if (!initialized_modifiers_) {
    RCLCPP_ERROR(get_logger(), "Modifiers not initialized");
    return;
  }

  const auto input = make_input_data();
  if (!input) {
    RCLCPP_ERROR(get_logger(), "%s", input.error().c_str());
    return;
  }
  CandidateTrajectories output_trajectories = *msg;

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  auto trajectory_count = 0;
  std::string modified_plugins_str;
  for (auto & trajectory : output_trajectories.candidate_trajectories) {
    auto input_data = input.value();
    input_data.candidate_header = trajectory.header;
    input_data.candidate_index = static_cast<std::size_t>(trajectory_count);
    input_data.candidate_count = output_trajectories.candidate_trajectories.size();
    for (auto & modifier : plugins_) {
      const auto modified = modifier->modify_trajectory(trajectory.points, input_data);
      const auto ns = "trajectory_" + std::to_string(trajectory_count);
      modifier->publish_debug_data(ns);
      if (!modified) continue;
      modifier->publish_planning_factor();
      if (!modified_plugins_str.empty()) modified_plugins_str += ", ";
      modified_plugins_str += modifier->get_short_name();
    }
    trajectory_count++;
  }
  if (!modified_plugins_str.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "[TM] Trajectory was modified by %s",
      modified_plugins_str.c_str());
  }

  trajectories_pub_->publish(output_trajectories);

  const auto processing_time_ms = stop_watch.toc(__func__);
  pub_processing_time_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "processing_time_ms", processing_time_ms);
}

tl::expected<plugin::InputData, std::string> TrajectoryModifier::make_input_data()
{
  plugin::InputData input;
  context_->refresh_external_velocity_limit();
  input.has_external_velocity_limit_snapshot = true;
  input.external_velocity_limit = context_->external_velocity_limit.active_limit();
  input.external_velocity_limit_clear_revision =
    context_->external_velocity_limit.clear_revision();
  input.current_odometry = sub_current_odometry_.take_data();
  input.current_acceleration = sub_current_acceleration_.take_data();
  input.predicted_objects = sub_objects_.take_data();
  input.obstacle_pointcloud = sub_pointcloud_.take_data();
  input.route = sub_route_.take_data();
  input.traffic_light_signals = sub_traffic_lights_.take_data();
  input.lanelet_map = lanelet_map_ptr_;
  input.lanelet_map_bin = lanelet_map_bin_ptr_;

  if (!input.current_odometry) {
    return tl::make_unexpected("Data is not ready: current_odometry is not set");
  }
  if (!input.current_acceleration) {
    return tl::make_unexpected("Data is not ready: current_acceleration is not set");
  }
  if (!input.predicted_objects) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: predicted_objects is not set");
  }
  if (!input.obstacle_pointcloud) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: obstacle_pointcloud is not set");
  }
  if (!input.route) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing data: route is not set");
  }
  if (!input.traffic_light_signals) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: traffic_light_signals is not set");
  }
  if (!input.lanelet_map) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing data: lanelet_map is not set");
  }
  return input;
}

void TrajectoryModifier::load_plugin(const std::string & name)
{
  // Check if the plugin is already instantiated
  auto it = std::find_if(
    plugins_.begin(), plugins_.end(), [&](const auto & p) { return p->get_name() == name; });
  if (it != plugins_.end()) {
    RCLCPP_WARN(
      this->get_logger(), "The plugin '%s' is already in the plugins list.", name.c_str());
    return;
  }

  if (plugin_loader_.isClassAvailable(name)) {
    const auto plugin = plugin_loader_.createSharedInstance(name);
    plugin->initialize(name, this, time_keeper_, context_, params_);
    // register
    plugins_.push_back(plugin);
    RCLCPP_INFO(this->get_logger(), "The modifier plugin '%s' has been loaded", name.c_str());
    initialized_modifiers_ = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "The modifier plugin '%s' is not available", name.c_str());
  }
}

void TrajectoryModifier::unload_plugin(const std::string & name)
{
  auto it = std::remove_if(plugins_.begin(), plugins_.end(), [&](const auto plugin) {
    return plugin->get_name() == name;
  });

  if (it == plugins_.end()) {
    RCLCPP_WARN(
      this->get_logger(), "The modifier plugin '%s' is not in the registered modules",
      name.c_str());
  } else {
    plugins_.erase(it, plugins_.end());
    RCLCPP_INFO(this->get_logger(), "The modifier plugin '%s' has been unloaded", name.c_str());
  }
}

void TrajectoryModifier::update_params()
{
  try {
    params_ = param_listener_->get_params();

    for (auto & plugin : plugins_) {
      plugin->update_params(params_);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Failed to update parameters: %s", e.what());
  }
}

tl::expected<TrajectoryModifierData, std::string> TrajectoryModifier::make_input_data()
{
  TrajectoryModifierData data;
  context_->refresh_external_velocity_limit();
  data.has_external_velocity_limit_snapshot = true;
  data.external_velocity_limit = context_->external_velocity_limit.active_limit();
  data.external_velocity_limit_clear_revision = context_->external_velocity_limit.clear_revision();
  context_->refresh_scenario_max_velocity();
  data.has_scenario_max_velocity_snapshot = true;
  data.scenario_max_velocity = context_->scenario_max_velocity();
  data.current_odometry = sub_current_odometry_.take_data();
  data.current_acceleration = sub_current_acceleration_.take_data();
  data.current_steering = sub_current_steering_.take_data();
  data.predicted_objects = sub_objects_.take_data();
  data.tracked_objects = sub_tracked_objects_.take_data();
  data.obstacle_pointcloud = sub_pointcloud_.take_data();
  data.route = sub_route_.take_data();
  data.traffic_light_signals = sub_traffic_lights_.take_data();
  data.lanelet_map = lanelet_map_ptr_;
  data.lanelet_map_bin = lanelet_map_bin_ptr_;

  if (!data.current_odometry || !data.current_acceleration) {
    return tl::make_unexpected("Data is not ready: odometry or acceleration is not set");
  }
  if (!data.predicted_objects) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: predicted_objects is not set");
  }
  if (!data.obstacle_pointcloud) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: obstacle_pointcloud is not set");
  }
  if (!data.route) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing data: route is not set");
  }
  if (!data.traffic_light_signals) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Missing data: traffic_light_signals is not set");
  }
  if (!data.lanelet_map) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Missing data: lanelet_map is not set");
  }
  return data;
}

void TrajectoryModifier::publish_processing_time(const double processing_time_ms)
{
  debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "processing_time_ms", processing_time_ms);
}

void TrajectoryModifier::on_trajectories(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic(__func__);

  if (param_listener_->is_old(params_)) {
    update_params();
  }

  const auto input = make_input_data();
  if (!input) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", input.error().c_str());
    publish_processing_time(stop_watch.toc(__func__));
    return;
  }

  CandidateTrajectories output = *msg;
  std::string modified_instances;
  for (std::size_t candidate_index = 0; candidate_index < output.candidate_trajectories.size();
       ++candidate_index) {
    auto & candidate = output.candidate_trajectories.at(candidate_index);
    auto data = input.value();
    data.candidate_header = candidate.header;
    data.candidate_index = candidate_index;
    data.candidate_count = output.candidate_trajectories.size();
    for (auto & modifier_plugin : plugins_) {
      const auto result = modifier_plugin->process(candidate.points, data);
      modifier_plugin->publish_debug_data("trajectory_" + std::to_string(candidate_index));
      // Always publish so inspection-only plugins (ModelPlanningFactorID) can emit factors
      // without mutating the trajectory. An empty publish also clears stale factors.
      modifier_plugin->publish_planning_factor();
      if (result != plugin::ProcessingResult::Modified) {
        continue;
      }
      if (!modified_instances.empty()) {
        modified_instances += ", ";
      }
      modified_instances += modifier_plugin->get_short_name();
    }
    if (candidate.points.size() < 3) {
      candidate.points =
        autoware::trajectory_modifier::utils::generate_three_point_stopped_trajectory(
          candidate.points, *data.current_odometry);
    }
  }

  if (!modified_instances.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "Trajectory was modified by %s",
      modified_instances.c_str());
  }

  trajectories_pub_->publish(output);
  if (output.candidate_trajectories.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received an empty candidate trajectory array; skipping selected trajectory output");
    publish_processing_time(stop_watch.toc(__func__));
    return;
  }

  Trajectory selected;
  selected.header = output.candidate_trajectories.front().header;
  selected.points = output.candidate_trajectories.front().points;
  trajectory_pub_->publish(selected);
  publish_processing_time(stop_watch.toc(__func__));
}

}  // namespace autoware::trajectory_modifier

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_modifier::TrajectoryModifier)
