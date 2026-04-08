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
#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware/motion_utils/distance/distance.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils_geometry/geometry.hpp>
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

bool has_trajectory_from_generator(
  const std::unordered_map<std::string, std::string> & uuid_to_generator_name_map,
  const autoware_internal_planning_msgs::msg::CandidateTrajectories & trajectories,
  const std::string & generator_name_prefix)
{
  return std::any_of(
    trajectories.candidate_trajectories.cbegin(), trajectories.candidate_trajectories.cend(),
    [&](const autoware_internal_planning_msgs::msg::CandidateTrajectory & trajectory) {
      const auto generator_id_str = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
      const auto generator_name_it = uuid_to_generator_name_map.find(generator_id_str);
      return generator_name_it != uuid_to_generator_name_map.end() &&
             generator_name_it->second.rfind(generator_name_prefix, 0) == 0;
    });
}

visualization_msgs::msg::MarkerArray create_internal_state_text(
  const std::unordered_map<std::string, std::vector<std::string>> & plugin_filtered_paths,
  const std::vector<std::string> & sorted_plugins, const geometry_msgs::msg::Pose & marker_pose,
  const rclcpp::Time & now, const double height_offset)
{
  fmt::memory_buffer out;
  auto out_it = std::back_inserter(out);

  fmt::format_to(out_it, "{:^50}\n", "--- Trajectory Validator Filtering Result ---");

  int skip_counter = 0;
  for (const auto & plugin_name : sorted_plugins) {
    auto it = plugin_filtered_paths.find(plugin_name);

    bool has_filtered_paths = (it != plugin_filtered_paths.end() && !it->second.empty());

    if (has_filtered_paths) {
      fmt::format_to(out_it, "- {}: {} path(s) filtered\n", plugin_name, it->second.size());
    } else {
      ++skip_counter;
    }
  }

  if (skip_counter > 0) {
    fmt::format_to(out_it, "{}", std::string(skip_counter, '\n'));
  }

  fmt::format_to(out_it, "-----------------------------------");

  constexpr int32_t id{0};
  auto text_marker = autoware_utils_visualization::create_default_marker(
    "map", now, "plugin_report", id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
    autoware_utils_visualization::create_marker_scale(0.0, 0.0, 0.4),
    autoware_utils_visualization::create_marker_color(1., 1., 1., 0.999));
  text_marker.pose = marker_pose;
  text_marker.pose.position.z += height_offset;
  text_marker.text = fmt::to_string(out);

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(text_marker);
  return marker_array;
}
}  // namespace

namespace autoware::trajectory_validator
{

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

  pseudo_emergency_stop_planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      this, "pseudo_emergency_stop");
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
  diagnostics_interface_.clear();
  for (const auto & trajectory : msg->candidate_trajectories) {
    EvaluationTable table;
    table.generator_id = autoware_utils_uuid::to_hex_string(trajectory.generator_id);
    table.is_overall_feasible = true;

    for (const auto & plugin : plugins_) {
      PluginEvaluation evaluation;
      evaluation.plugin_name = plugin->get_name();
      stop_watch.tic(evaluation.plugin_name);

      if (const auto res = plugin->is_feasible(trajectory.points, context); !res) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Not feasible: %s", res.error().c_str());
        diagnostics_interface_.add_key_value(plugin->get_name(), res.error());

        evaluation.is_feasible = false;
        evaluation.reason = res.error();

        if (!plugin->is_debug_mode()) {
          table.is_overall_feasible = false;
        }
      }
      processing_time_ms[evaluation.plugin_name] += stop_watch.toc(evaluation.plugin_name);

      table.evaluations[plugin->category()].push_back(evaluation);
    }

    evaluation_tables_.push_back(table);

    if (table.is_overall_feasible) filtered_msg->candidate_trajectories.push_back(trajectory);
  }

  if (params_.pseudo_emergency_stop.enable) {
    // NOTE(odashima): this fallback is ad-hoc and for evaluation only.
    stop_watch.tic("handle_pseudo_emergency_stop");
    handle_pseudo_emergency_stop(*msg, *filtered_msg, context);
    processing_time_ms["handle_pseudo_emergency_stop"] =
      stop_watch.toc("handle_pseudo_emergency_stop");
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
  if (params_.pseudo_emergency_stop.enable) {
    pseudo_emergency_stop_planning_factor_interface_->publish();
  }
}

void TrajectoryValidator::handle_pseudo_emergency_stop(
  const CandidateTrajectories & input_trajectories, CandidateTrajectories & filtered_trajectories,
  const FilterContext & context)
{
  const bool pseudo_emergency_stop_triggered =
    is_pseudo_emergency_stop_triggered(evaluation_tables_);
  update_pseudo_emergency_stop_state(
    pseudo_emergency_stop_triggered, context.odometry->twist.twist.linear.x);
  // Refresh the cached fallback shape from the latest clean MinimumRuleBasedPlanner trajectory.
  cache_fallback_trajectory(input_trajectories, evaluation_tables_);
  if (pseudo_emergency_stop_active_) {
    apply_pseudo_emergency_stop_fallback(filtered_trajectories, context);
  }
}

bool TrajectoryValidator::has_infeasible_evaluation(const EvaluationTable & table) const
{
  return std::any_of(
    table.evaluations.cbegin(), table.evaluations.cend(), [&](const auto & category_entry) {
      return std::any_of(
        category_entry.second.cbegin(), category_entry.second.cend(),
        [&](const PluginEvaluation & plugin_eval) { return !plugin_eval.is_feasible; });
    });
}

bool TrajectoryValidator::is_pseudo_emergency_stop_triggered(
  const std::vector<EvaluationTable> & evaluation_tables) const
{
  return std::any_of(
    evaluation_tables.cbegin(), evaluation_tables.cend(),
    [&](const EvaluationTable & table) { return has_infeasible_evaluation(table); });
}

void TrajectoryValidator::update_pseudo_emergency_stop_state(
  const bool triggered, const double ego_velocity_mps)
{
  if (triggered) {
    pseudo_emergency_stop_active_ = true;
    return;
  }
  if (params_.pseudo_emergency_stop.recovery_policy == "immediate") {
    // Release the emergency stop immediately when the trigger condition disappears.
    pseudo_emergency_stop_active_ = false;
    return;
  }
  if (
    pseudo_emergency_stop_active_ &&
    params_.pseudo_emergency_stop.recovery_policy == "until_stopped" &&
    std::abs(ego_velocity_mps) < params_.pseudo_emergency_stop.recovery_velocity_threshold_mps) {
    // Ego has come to a stop, release the latched emergency-stop state.
    pseudo_emergency_stop_active_ = false;
  }
}

void TrajectoryValidator::cache_fallback_trajectory(
  const CandidateTrajectories & input_trajectories,
  const std::vector<EvaluationTable> & evaluation_tables)
{
  constexpr const char * fallback_generator_name = "MinimumRuleBasedPlanner";

  // Find the generator_id whose name matches the fallback generator name.
  const auto generator_info_it = std::find_if(
    input_trajectories.generator_info.cbegin(), input_trajectories.generator_info.cend(),
    [&](const auto & info) { return info.generator_name.data == fallback_generator_name; });
  if (generator_info_it == input_trajectories.generator_info.cend()) {
    return;
  }
  const auto & fallback_generator_id = generator_info_it->generator_id;
  const auto fallback_it = std::find_if(
    input_trajectories.candidate_trajectories.cbegin(),
    input_trajectories.candidate_trajectories.cend(),
    [&](const autoware_internal_planning_msgs::msg::CandidateTrajectory & trajectory) {
      return trajectory.generator_id.uuid == fallback_generator_id.uuid;
    });
  if (fallback_it == input_trajectories.candidate_trajectories.cend()) {
    return;
  }

  // Only cache when no trigger filter has flagged this fallback trajectory as infeasible.
  const auto generator_id_str = autoware_utils_uuid::to_hex_string(fallback_it->generator_id);
  const auto table_it = std::find_if(
    evaluation_tables.cbegin(), evaluation_tables.cend(),
    [&](const EvaluationTable & table) { return table.generator_id == generator_id_str; });
  if (table_it != evaluation_tables.cend() && has_infeasible_evaluation(*table_it)) {
    return;
  }

  cached_fallback_trajectory_ = *fallback_it;
}

void TrajectoryValidator::apply_pseudo_emergency_stop_fallback(
  CandidateTrajectories & filtered_trajectories, const FilterContext & context)
{
  if (!cached_fallback_trajectory_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Emergency stop active but no cached fallback trajectory is available yet.");
    return;
  }

  auto stopping_trajectory = *cached_fallback_trajectory_;
  auto & points = stopping_trajectory.points;
  if (points.size() < 2) {
    return;
  }

  const double min_decel = params_.pseudo_emergency_stop.deceleration_mps2;
  const double min_jerk = params_.pseudo_emergency_stop.jerk_mps3;

  // Drop points behind ego so that the resulting trajectory starts at the nearest point to the
  // current ego position.
  const size_t ego_nearest_idx =
    autoware::motion_utils::findNearestIndex(points, context.odometry->pose.pose.position);
  points.erase(points.begin(), points.begin() + ego_nearest_idx);
  if (points.size() < 2) {
    return;
  }

  // Overwrite the velocity/acceleration profile in place with a jerk/decel-limited stopping
  // profile starting from the current ego state.
  double v = std::max(0.0, context.odometry->twist.twist.linear.x);
  double a = std::min(0.0, context.acceleration->accel.accel.linear.x);
  points.front().longitudinal_velocity_mps = v;
  points.front().acceleration_mps2 = a;
  for (size_t i = 1; i < points.size(); ++i) {
    const double ds = autoware_utils_geometry::calc_distance2d(points[i - 1].pose, points[i].pose);
    if (v <= 1e-3) {
      v = 0.0;
      a = 0.0;
    } else {
      const double dt = ds / v;
      a = std::max(min_decel, a + min_jerk * dt);
      v = std::max(0.0, v + a * dt);
    }
    points[i].longitudinal_velocity_mps = v;
    points[i].acceleration_mps2 = a;
  }

  autoware::motion_utils::calculate_time_from_start(
    stopping_trajectory.points, context.odometry->pose.pose.position);
  filtered_trajectories.candidate_trajectories.clear();
  filtered_trajectories.candidate_trajectories.push_back(stopping_trajectory);

  const auto stop_distance = autoware::motion_utils::calculate_stop_distance(
    context.odometry->twist.twist.linear.x, context.acceleration->accel.accel.linear.x, min_decel,
    min_jerk);
  const auto stop_pose_opt = stop_distance ? autoware::motion_utils::calcLongitudinalOffsetPose(
                                               stopping_trajectory.points, 0, *stop_distance)
                                           : std::nullopt;
  const auto & stop_pose = stop_pose_opt ? *stop_pose_opt : stopping_trajectory.points.back().pose;
  pseudo_emergency_stop_planning_factor_interface_->add(
    stopping_trajectory.points, context.odometry->pose.pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/,
    0.0 /*velocity*/, 0.0 /*shift_length*/);

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Emergency-stop trigger filter fired; falling back to cached MinimumRuleBasedPlanner "
    "trajectory with emergency stop.");
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
  const auto uuid_to_name_map = get_generator_uuid_to_name_map(input_trajectories);
  const auto input_has_diffusion_trajectories =
    has_trajectory_from_generator(uuid_to_name_map, input_trajectories, "Diffusion");
  const auto filtered_has_diffusion_trajectories =
    has_trajectory_from_generator(uuid_to_name_map, filtered_trajectories, "Diffusion");
  if (
    !input_trajectories.candidate_trajectories.empty() &&
    filtered_trajectories.candidate_trajectories.empty()) {
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No feasible trajectories found");
  } else if (input_has_diffusion_trajectories && !filtered_has_diffusion_trajectories) {
    diagnostics_interface_.update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "All diffusion planner trajectories are infeasible");
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

  constexpr double offset = 1.0;
  auto internal_state_text = create_internal_state_text(
    plugin_filtered_paths, sorted_plugins, ego_pose, get_clock()->now(),
    vehicle_info_.vehicle_height_m + offset);
  pub_debug_markers_->publish<visualization_msgs::msg::MarkerArray>("markers", internal_state_text);
}
}  // namespace autoware::trajectory_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::trajectory_validator::TrajectoryValidator)
