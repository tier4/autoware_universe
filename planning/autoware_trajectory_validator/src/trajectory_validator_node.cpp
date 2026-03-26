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

#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <autoware_internal_planning_msgs/msg/detail/candidate_trajectory__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

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
  listener_{get_node_parameters_interface()},
  params_(listener_.get_params()),
  plugin_loader_(
    "autoware_trajectory_validator", "autoware::trajectory_validator::plugin::ValidatorInterface"),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo())
{
  const auto filters = params_.filter_names;
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

  pub_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms/feasible_trajectory_filter", 1);
  time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(pub_processing_time_detail_);

  pub_processing_time_text_ = create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
    "~/debug/processing_time_text", rclcpp::QoS(1));

  pub_processing_time_ = std::make_shared<autoware_utils_debug::DebugPublisher>(this, "~/debug");
  pub_debug_markers_ = std::make_shared<autoware_utils_debug::DebugPublisher>(this, "~/debug");
  pub_debug_statuses_ = create_publisher<TrajectoryStatusArray>("~/debug/status", 1);
}

void TrajectoryValidator::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  autoware_utils_system::StopWatch<std::chrono::milliseconds> stop_watch;
  std::unordered_map<std::string, double> processing_time_ms;

  stop_watch.tic("Total");
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

  if (listener_.is_old(params_)) {
    params_ = listener_.get_params();

    for (const auto & plugin : plugins_) {
      plugin->update_parameters(params_);
    }
    RCLCPP_INFO(get_logger(), "Dynamic parameters updated successfully.");
  }

  diagnostics_interface_.clear();
  evaluation_tables_.clear();

  auto filtered_msg = std::make_unique<CandidateTrajectories>();

  const auto uuid_to_name = get_generator_uuid_to_name_map(*msg);

  // Process and filter trajectories
  std::vector<TrajectoryStatus> trajectory_statuses;
  diagnostics_interface_.clear();
  for (const auto & trajectory : msg->candidate_trajectories) {
    EvaluationTable table;
    table.generator_id = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
    table.is_overall_feasible = true;

    // Hashmap of validation statuses for each category
    std::unordered_map<std::string, std::vector<TrajectoryValidationStatus>> validation_statuses;
    for (const auto & plugin : plugins_) {
      PluginEvaluation evaluation;
      evaluation.is_feasible = true;
      evaluation.plugin_name = plugin->get_name();
      stop_watch.tic(evaluation.plugin_name);

      if (const auto result = plugin->is_feasible(trajectory.points, context); !result) {
        // NOTE: Filter out the trajectory when exception occurred while validation
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000, "Got unexpected behavior in %s: %s",
          plugin->get_name().c_str(), result.error().c_str());

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

        validation_statuses[plugin->category()].push_back(result.value());
      } else {
        diagnostics_interface_.add_key_value(plugin->get_name(), std::string("OK"));

        validation_statuses[plugin->category()].push_back(result.value());
      }
      processing_time_ms[evaluation.plugin_name] += stop_watch.toc(evaluation.plugin_name);

      table.evaluations[plugin->category()].push_back(evaluation);
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

  processing_time_ms["Total"] = stop_watch.toc("Total");

  for (const auto & plugin : plugins_) {
    auto msg_to_publish = plugin->take_debug_markers();
    pub_debug_markers_->publish<visualization_msgs::msg::MarkerArray>(
      plugin->get_name(), msg_to_publish);
  }
  publish_processing_time(processing_time_ms);
  publish_internal_state(processing_time_ms, evaluation_tables_, context.odometry->pose.pose);
  update_diagnostic(*msg, *filtered_msg);
  pub_trajectories_->publish(*filtered_msg);
  pub_debug_statuses_->publish(to_trajectory_status_array(get_clock()->now(), trajectory_statuses));
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
    plugin->update_parameters(params_);

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

void TrajectoryValidator::publish_processing_time(
  const std::unordered_map<std::string, double> & processing_time)
{
  for (const auto & [key, value] : processing_time) {
    if (key == "Total") {
      pub_processing_time_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "processing_time_ms", value);
      continue;
    }
    pub_processing_time_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      key + "/processing_time_ms", value);
  }
}

void TrajectoryValidator::publish_internal_state(
  const std::unordered_map<std::string, double> & processing_time,
  const std::vector<EvaluationTable> & evaluation_tables, const geometry_msgs::msg::Pose & ego_pose)
{
  std::unordered_map<std::string, std::vector<std::string>> plugin_filtered_paths;
  // 1. Group the filtered paths by plugin
  for (const auto & eval : evaluation_tables) {
    std::string short_uuid = eval.generator_id.substr(0, 8);  // Just using short UUID as requested
    for (const auto & [category, evaluations] : eval.evaluations) {
      for (const auto & plugin_eval : evaluations) {
        if (!plugin_eval.is_feasible) {
          plugin_filtered_paths[plugin_eval.plugin_name].push_back(short_uuid);
        }
      }
    }
  }

  // 2. Extract and sort the plugin names alphabetically
  std::vector<std::string> sorted_plugins;
  for (const auto & [plugin_name, _] : processing_time) {
    if (plugin_name != "Total") {
      sorted_plugins.push_back(plugin_name);
    }
  }
  std::sort(sorted_plugins.begin(), sorted_plugins.end());

  std::string report = "\n--- Trajectory Validator Processing Time Report ---\n";
  double total_time_ms = processing_time.count("Total") ? processing_time.at("Total") : 0.0;
  report += fmt::format("Total: {:.2f} ms\n", total_time_ms);

  for (const auto & plugin_name : sorted_plugins) {
    double time = processing_time.at(plugin_name);
    report += fmt::format("- {}: {:.2f} ms\n", plugin_name, time);
  }

  autoware_internal_debug_msgs::msg::StringStamped text_msg;
  text_msg.stamp = this->now();
  text_msg.data = report;
  pub_processing_time_text_->publish(text_msg);

  // --- Filtering Results Section ---
  fmt::memory_buffer out;
  fmt::format_to(std::back_inserter(out), "--- Trajectory Validator Filtering Result ---\n");

  for (const auto & plugin_name : sorted_plugins) {
    auto it = plugin_filtered_paths.find(plugin_name);
    if (it != plugin_filtered_paths.end() && !it->second.empty()) {
      fmt::format_to(
        std::back_inserter(out), "- {}: {} path(s) filtered\n", plugin_name, it->second.size());
    }
  }

  fmt::format_to(std::back_inserter(out), "-----------------------------------");

  constexpr int32_t id{0};
  auto text_marker = autoware_utils_visualization::create_default_marker(
    "map", get_clock()->now(), "plugin_report", id,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    autoware_utils_visualization::create_marker_scale(0.0, 0.0, 0.9),
    autoware_utils_visualization::create_marker_color(1., 1., 1., 0.999));
  text_marker.pose = ego_pose;
  text_marker.pose.position.z += vehicle_info_.vehicle_height_m + 2.0;
  text_marker.text = fmt::to_string(out);

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(text_marker);
  pub_debug_markers_->publish<visualization_msgs::msg::MarkerArray>("markers", marker_array);
}
}  // namespace autoware::trajectory_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_validator::TrajectoryValidator)
