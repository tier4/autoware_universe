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

#include "autoware/trajectory_validator/trajectory_validator_node.hpp"

#include "autoware/trajectory_validator/filter_context.hpp"
#include "autoware/trajectory_validator/status.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_internal_planning_msgs/msg/detail/candidate_trajectory__struct.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator
{
namespace
{
// for error diagnostic. Will be removed once node is combined.
std::unordered_map<std::string, std::string> get_generator_uuid_to_name_map(
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & candidate_trajectories)
{
  std::unordered_map<std::string, std::string> uuid_to_name;
  uuid_to_name.reserve(candidate_trajectories.generator_info.size());
  for (const auto & info : candidate_trajectories.generator_info) {
    uuid_to_name[autoware_utils_uuid::to_hex_string(info.generator_id)] = info.generator_name.data;
  }
  return uuid_to_name;
}
}  // namespace

TrajectoryValidator::TrajectoryValidator(const rclcpp::NodeOptions & options)
: Node{"trajectory_validator_node", options},
  listener_{std::make_unique<validator::ParamListener>(get_node_parameters_interface())},
  plugin_loader_(
    "autoware_trajectory_validator", "autoware::trajectory_validator::plugin::ValidatorInterface"),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  const auto filters = listener_->get_params().filter_names;
  for (const auto & filter : filters) {
    load_metric(filter);
  }

  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryValidator::map_callback, this, std::placeholders::_1));

  sub_trajectories_ = create_subscription<CandidateTrajectories>(
    "~/input/trajectories", 1,
    std::bind(&TrajectoryValidator::process, this, std::placeholders::_1));

  pub_trajectories_ = create_publisher<CandidateTrajectories>("~/output/trajectories", 1);

  debug_status_publisher_ = create_publisher<TrajectoryStatusArray>("~/debug/status", 1);
  debug_processing_time_detail_pub_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&TrajectoryValidator::on_parameter, this, std::placeholders::_1));
}

void TrajectoryValidator::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  // Prepare context for filters
  FilterContext context;

  context.odometry = sub_odometry_.take_data();
  if (!context.odometry) {
    return;
  }

  context.predicted_objects = sub_objects_.take_data();
  if (!context.predicted_objects) {
    return;
  }

  context.acceleration = sub_acceleration_.take_data();
  if (!context.acceleration) {
    return;
  }

  context.traffic_light_signals = sub_traffic_lights_.take_data();

  context.lanelet_map = lanelet_map_ptr_;
  if (!context.lanelet_map) {
    return;
  }

  diagnostics_interface_.clear();
  evaluation_tables_.clear();

  // Create output message for filtered trajectories
  auto filtered_msg = std::make_unique<CandidateTrajectories>();

  const auto uuid_to_name = get_generator_uuid_to_name_map(*msg);

  // Process and filter trajectories
  std::vector<TrajectoryStatus> trajectory_statuses;
  for (const auto & trajectory : msg->candidate_trajectories) {
    // Apply each filter to the trajectory

    EvaluationTable table;
    table.generator_id = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
    table.is_overall_feasible = true;

    // Hashmap of validation statuses for each category
    std::unordered_map<std::string, std::vector<TrajectoryValidationStatus>> validation_statuses;
    for (const auto & plugin : plugins_) {
      PluginEvaluation evaluation;
      evaluation.is_feasible = true;
      evaluation.plugin_name = plugin->get_name();

      const auto result = plugin->is_feasible(trajectory.points, context);
      if (!result) {
        // NOTE: Filter out the trajectory when exception occurred while validation
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "Got unexpected behavior: %s", result.error().c_str());

        diagnostics_interface_.add_key_value(plugin->get_name(), std::string("NG"));

        // Update evaluation table
        evaluation.is_feasible = false;
        evaluation.reason = result.error();
        if (!plugin->is_debug_mode()) {
          table.is_overall_feasible = false;
        }
      } else if (!check_validation_status(result.value())) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Not feasible: %s", result.value().name.c_str());

        diagnostics_interface_.add_key_value(plugin->get_name(), std::string("NG"));

        // Update evaluation table
        evaluation.is_feasible = false;
        evaluation.reason = result.value().name;
        if (!plugin->is_debug_mode()) {
          table.is_overall_feasible = false;
        }
      } else {
        diagnostics_interface_.add_key_value(plugin->get_name(), std::string("OK"));
      }

      table.evaluations[plugin->category()].push_back(evaluation);
      validation_statuses[plugin->category()].push_back(result.value());
    }

    evaluation_tables_.push_back(table);

    if (table.is_overall_feasible) filtered_msg->candidate_trajectories.push_back(trajectory);
    trajectory_statuses.push_back(
      to_trajectory_status(trajectory, validation_statuses, uuid_to_name));
  }

  // Also filter generator_info to match kept trajectories
  for (const auto & traj : filtered_msg->candidate_trajectories) {
    auto it = std::find_if(
      msg->generator_info.begin(), msg->generator_info.end(),
      [&](const auto & info) { return traj.generator_id.uuid == info.generator_id.uuid; });

    if (it != msg->generator_info.end()) {
      filtered_msg->generator_info.push_back(*it);
    }
  }

  update_diagnostic(*msg, *filtered_msg);
  pub_trajectories_->publish(*filtered_msg);

  debug_status_publisher_->publish(
    to_trajectory_status_array(get_clock()->now(), trajectory_statuses));
}

void TrajectoryValidator::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);
}

void TrajectoryValidator::load_metric(const std::string & name)
{
  if (name.empty()) return;

  try {
    auto plugin = plugin_loader_.createSharedInstance(name);

    for (const auto & p : plugins_) {
      if (plugin->get_name() == p->get_name()) {
        RCLCPP_WARN_STREAM(get_logger(), "The plugin '" << name << "' is already loaded.");
        return;
      }
    }

    plugin->set_vehicle_info(vehicle_info_);
    plugin->set_parameters(*this);
    std::string category;
    size_t pos = name.find("::");
    if (pos != std::string::npos) {
      category = name.substr(0, pos);
    }
    plugin->set_category(category);

    plugins_.push_back(plugin);

    RCLCPP_INFO_STREAM(
      get_logger(), "The scene plugin '" << name << "' is loaded and initialized.");
  } catch (const pluginlib::CreateClassException & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "[validator] createSharedInstance failed for '" << name << "': " << e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "[validator] unexpected exception for '" << name << "': " << e.what());
  }
}

void TrajectoryValidator::unload_metric(const std::string & name)
{
  auto it = std::remove_if(
    plugins_.begin(), plugins_.end(),
    [&](const std::shared_ptr<plugin::ValidatorInterface> & plugin) {
      return plugin->get_name() == name;
    });

  if (it == plugins_.end()) {
    RCLCPP_WARN_STREAM(
      get_logger(), "The scene plugin '" << name << "' is not found in the registered modules.");
  } else {
    plugins_.erase(it, plugins_.end());
    RCLCPP_INFO_STREAM(get_logger(), "The scene plugin '" << name << "' is unloaded.");
  }
}

void TrajectoryValidator::update_diagnostic(
  const CandidateTrajectories & input_trajectories,
  const CandidateTrajectories & filtered_trajectories)
{
  if (
    !input_trajectories.candidate_trajectories.empty() &&
    filtered_trajectories.candidate_trajectories.empty()) {
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No feasible trajectories found");
  } else if (
    input_trajectories.candidate_trajectories.size() !=
    filtered_trajectories.candidate_trajectories.size()) {
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Some trajectories are infeasible");
  } else {
    diagnostics_interface_.update_level_and_message(diagnostic_msgs::msg::DiagnosticStatus::OK, "");
  }

  diagnostics_interface_.publish(this->get_clock()->now());
}

rcl_interfaces::msg::SetParametersResult TrajectoryValidator::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    // Broadcast the changed parameters to all loaded plugins
    for (const auto & plugin : plugins_) {
      plugin->update_parameters(parameters);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    // Cleanly reject the parameter change if any plugin detects a type mismatch
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}
}  // namespace autoware::trajectory_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_validator::TrajectoryValidator)
