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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/virtual_traffic_light_stop.hpp"

#include "autoware/trajectory_modifier/trajectory_modifier_utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>

namespace autoware::trajectory_modifier::plugin
{
namespace
{
using Trajectory =
  autoware::experimental::trajectory::Trajectory<autoware_planning_msgs::msg::TrajectoryPoint>;
using VirtualTrafficLight = lanelet::autoware::VirtualTrafficLight;

tier4_v2x_msgs::msg::KeyValue create_key_value(const std::string & key, const std::string & value)
{
  tier4_v2x_msgs::msg::KeyValue key_value;
  key_value.key = key;
  key_value.value = value;
  return key_value;
}

std::optional<double> find_last_collision_before_line(
  const Trajectory & path, const double end_line_s, const lanelet::ConstLineString3d & line)
{
  auto cropped_path = path;
  cropped_path.crop(0.0, std::min(end_line_s, cropped_path.length()));
  const auto collisions = autoware::experimental::trajectory::crossed(cropped_path, line);
  if (collisions.empty()) {
    return std::nullopt;
  }
  return collisions.back();
}

std::optional<double> find_first_collision(
  const Trajectory & path, const lanelet::ConstLineStrings3d & lines)
{
  std::optional<double> first_collision;
  for (const auto & line : lines) {
    const auto collisions = autoware::experimental::trajectory::crossed(path, line);
    if (!collisions.empty() && (!first_collision || collisions.front() < *first_collision)) {
      first_collision = collisions.front();
    }
  }
  return first_collision;
}

template <class T>
std::optional<double> calc_arc_length_from_collision(
  const Trajectory & path, const double end_line_s, const T & line,
  const geometry_msgs::msg::Pose & ego_pose, const double front_offset,
  const double max_yaw_deviation_rad)
{
  const auto collision = find_last_collision_before_line(path, end_line_s, line);
  if (!collision) {
    return std::nullopt;
  }

  const auto ego_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, ego_pose, 5.0, max_yaw_deviation_rad);
  if (!ego_s) {
    return std::nullopt;
  }
  return *collision - *ego_s - front_offset;
}

std::optional<double> calc_arc_length_from_collision(
  const Trajectory & path, const double end_line_s,
  const lanelet::ConstLineStrings3d & lines,
  const geometry_msgs::msg::Pose & ego_pose, const double front_offset,
  const double max_yaw_deviation_rad)
{
  std::optional<double> last_collision;
  for (const auto & line : lines) {
    const auto collision = find_last_collision_before_line(path, end_line_s, line);
    if (collision && (!last_collision || *collision > *last_collision)) {
      last_collision = collision;
    }
  }
  if (!last_collision) {
    return std::nullopt;
  }
  const auto ego_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, ego_pose, 5.0, max_yaw_deviation_rad);
  if (!ego_s) {
    return std::nullopt;
  }
  return *last_collision - *ego_s - front_offset;
}

bool trajectory_has_enough_points(const TrajectoryPoints & points)
{
  return points.size() >= 2;
}
}  // namespace

void VirtualTrafficLightStop::on_initialize(const TrajectoryModifierParams & params)
{
  const auto node_ptr = get_node_ptr();
  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      node_ptr, "modifier_virtual_traffic_light_stop");
  pub_infrastructure_commands_ =
    node_ptr->create_publisher<tier4_v2x_msgs::msg::InfrastructureCommandArray>(
      "~/output/infrastructure_commands", 1);

  enabled_ = params.use_virtual_traffic_light_stop;
  planner_param_.max_delay_sec = params.virtual_traffic_light.max_delay_sec;
  planner_param_.near_line_distance = params.virtual_traffic_light.near_line_distance;
  planner_param_.dead_line_margin = params.virtual_traffic_light.dead_line_margin;
  planner_param_.max_yaw_deviation_rad =
    autoware_utils::deg2rad(params.virtual_traffic_light.max_yaw_deviation_deg);
  planner_param_.check_timeout_after_stop_line =
    params.virtual_traffic_light.check_timeout_after_stop_line;
  planner_param_.hold_stop_margin_distance =
    params.virtual_traffic_light.hold_stop_margin_distance;
}

void VirtualTrafficLightStop::update_params(const TrajectoryModifierParams & params)
{
  enabled_ = params.use_virtual_traffic_light_stop;
  planner_param_.max_delay_sec = params.virtual_traffic_light.max_delay_sec;
  planner_param_.near_line_distance = params.virtual_traffic_light.near_line_distance;
  planner_param_.dead_line_margin = params.virtual_traffic_light.dead_line_margin;
  planner_param_.max_yaw_deviation_rad =
    autoware_utils::deg2rad(params.virtual_traffic_light.max_yaw_deviation_deg);
  planner_param_.check_timeout_after_stop_line =
    params.virtual_traffic_light.check_timeout_after_stop_line;
  planner_param_.hold_stop_margin_distance =
    params.virtual_traffic_light.hold_stop_margin_distance;
}

void VirtualTrafficLightStop::begin_cycle(const InputData & input)
{
  first_candidate_in_cycle_ = true;
  if (!enabled_ || !input.current_odometry || !input.lanelet_map || !input.route) {
    modules_.clear();
    route_lanelet_ids_.clear();
    last_lanelet_map_.reset();
    return;
  }

  rebuild_modules(input);
  update_module_states(input);
}

void VirtualTrafficLightStop::end_cycle()
{
  if (!enabled_ || !pub_infrastructure_commands_) {
    return;
  }

  tier4_v2x_msgs::msg::InfrastructureCommandArray output;
  output.stamp = get_clock()->now();
  for (const auto & module : modules_) {
    if (module.infrastructure_command) {
      output.commands.push_back(*module.infrastructure_command);
    }
  }
  pub_infrastructure_commands_->publish(output);
}

bool VirtualTrafficLightStop::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const InputData & input)
{
  if (!enabled_ || !trajectory_has_enough_points(traj_points) || modules_.empty()) {
    return false;
  }

  auto copy = traj_points;
  return process_trajectory(copy, input, false, false);
}

bool VirtualTrafficLightStop::modify_trajectory(
  TrajectoryPoints & traj_points, const InputData & input)
{
  if (!enabled_ || !trajectory_has_enough_points(traj_points) || modules_.empty()) {
    return false;
  }

  const bool update_state = first_candidate_in_cycle_;
  const auto modified = process_trajectory(traj_points, input, update_state, true);
  first_candidate_in_cycle_ = false;
  return modified;
}

void VirtualTrafficLightStop::rebuild_modules(const InputData & input)
{
  std::vector<lanelet::Id> route_ids;
  route_ids.reserve(input.route->segments.size());
  for (const auto & segment : input.route->segments) {
    route_ids.push_back(segment.preferred_primitive.id);
  }

  if (route_ids == route_lanelet_ids_ && input.lanelet_map == last_lanelet_map_) {
    return;
  }

  std::unordered_map<std::string, Module> previous_modules;
  for (auto & module : modules_) {
    previous_modules.emplace(module_key(module.lane_id, module.regulatory_element->id()),
                             std::move(module));
  }

  modules_.clear();
  route_lanelet_ids_ = route_ids;
  last_lanelet_map_ = input.lanelet_map;
  std::unordered_set<std::string> registered_keys;

  for (const auto & segment : input.route->segments) {
    std::optional<lanelet::ConstLanelet> lane;
    try {
      lane = input.lanelet_map->laneletLayer.get(segment.preferred_primitive.id);
    } catch (const std::exception &) {
      RCLCPP_WARN_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 5000,
        "[TM VirtualTrafficLightStop] Route lanelet %ld is not in the map",
        segment.preferred_primitive.id);
      continue;
    }

    for (const auto & reg_elem : lane->regulatoryElementsAs<VirtualTrafficLight>()) {
      const auto key = module_key(lane->id(), reg_elem->id());
      if (!registered_keys.insert(key).second) {
        continue;
      }

      Module module;
      module.lane_id = lane->id();
      module.regulatory_element = reg_elem;
      module.lane = *lane;

      const auto instrument = reg_elem->getVirtualTrafficLight();
      const auto type = instrument.attribute("type").as<std::string>();
      module.instrument_type = type ? *type : "virtual_traffic_light";
      module.instrument_id = std::to_string(instrument.id());
      module.custom_tags.reserve(instrument.attributes().size() + 2);
      for (const auto & attribute : instrument.attributes()) {
        if (attribute.first == "type") {
          continue;
        }
        const auto value = attribute.second.as<std::string>();
        if (value) {
          module.custom_tags.push_back(create_key_value(attribute.first, *value));
        }
      }
      module.custom_tags.push_back(create_key_value("lane_id", std::to_string(lane->id())));
      module.custom_tags.push_back(
        create_key_value("turn_direction", lane->attributeOr("turn_direction", "straight")));

      if (auto previous = previous_modules.find(key); previous != previous_modules.end()) {
        module.state = previous->second.state;
        module.virtual_traffic_light_state = previous->second.virtual_traffic_light_state;
      }
      modules_.push_back(std::move(module));
    }
  }
}

void VirtualTrafficLightStop::update_module_states(const InputData & input)
{
  if (!input.virtual_traffic_light_states) {
    return;
  }

  for (auto & module : modules_) {
    for (const auto & state : input.virtual_traffic_light_states->states) {
      if (state.id == module.instrument_id) {
        module.virtual_traffic_light_state = state;
        break;
      }
    }
  }
}

bool VirtualTrafficLightStop::process_trajectory(
  TrajectoryPoints & traj_points, const InputData & input, const bool update_state,
  const bool apply_modification)
{
  bool modified = false;
  for (auto & module : modules_) {
    modified = process_module(module, traj_points, input, update_state, apply_modification) ||
               modified;
  }
  return modified;
}

bool VirtualTrafficLightStop::process_module(
  Module & module, TrajectoryPoints & traj_points, const InputData & input,
  const bool update_state, const bool apply_modification)
{
  auto path_result = Trajectory::Builder{}.build(traj_points);
  if (!path_result) {
    return false;
  }
  auto & path = *path_result;

  const auto & reg_elem = *module.regulatory_element;
  const auto end_line_s = find_first_collision(path, reg_elem.getEndLines());
  if (!end_line_s) {
    return false;
  }

  const auto ego_pose = input.current_odometry->pose.pose;
  const auto front_offset = context_->vehicle_info.max_longitudinal_offset_m;
  const auto start_arc = calc_arc_length_from_collision(
    path, *end_line_s, reg_elem.getStartLine(), ego_pose, front_offset,
    planner_param_.max_yaw_deviation_rad);
  if (start_arc && *start_arc > 0.0) {
    if (update_state) set_state(module, ModuleState::NONE);
    update_command(module);
    return false;
  }

  if (module.state == ModuleState::FINALIZED) {
    update_command(module);
    return false;
  }

  const auto end_arc = calc_arc_length_from_collision(
    path, path.length(), reg_elem.getEndLines(), ego_pose, front_offset,
    planner_param_.max_yaw_deviation_rad);
  if (end_arc && *end_arc < -planner_param_.dead_line_margin) {
    if (update_state) set_state(module, ModuleState::FINALIZED, reg_elem.getEndLines().front().id());
    update_command(module);
    return false;
  }

  const auto stop_line = reg_elem.getStopLine();
  if (!stop_line) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 5000,
      "[TM VirtualTrafficLightStop] VTL %s has no stop line", module.instrument_id.c_str());
    update_command(module);
    return false;
  }

  const auto stop_arc = calc_arc_length_from_collision(
    path, *end_line_s, *stop_line, ego_pose, front_offset, planner_param_.max_yaw_deviation_rad);
  const bool before_stop_line = stop_arc && *stop_arc > -planner_param_.dead_line_margin;
  const bool timeout = is_state_timeout(module);
  const bool no_right_of_way = !module.virtual_traffic_light_state || !has_right_of_way(module);

  if (no_right_of_way) {
    if (update_state) set_state(module, ModuleState::REQUESTING);
    update_command(module);
    return apply_modification && insert_stop_velocity(
             traj_points, traj_points, stop_arc, input, module, true);
  }

  if (before_stop_line) {
    if (update_state) set_state(module, ModuleState::REQUESTING);
    update_command(module);
    if (timeout) {
      return apply_modification && insert_stop_velocity(
               traj_points, traj_points, stop_arc, input, module, true);
    }
    return false;
  }

  if (update_state && module.state == ModuleState::REQUESTING) {
    set_state(module, ModuleState::PASSING);
  }

  if (planner_param_.check_timeout_after_stop_line && timeout) {
    if (update_state) set_state(module, ModuleState::PASSING);
    update_command(module);
    return apply_modification && insert_stop_velocity(
             traj_points, traj_points, stop_arc, input, module, true);
  }

  if (!module.virtual_traffic_light_state->is_finalized) {
    const auto end_stop_arc = calc_arc_length_from_collision(
      path, path.length(), reg_elem.getEndLines(), ego_pose, front_offset,
      planner_param_.max_yaw_deviation_rad);
    const auto changed = apply_modification &&
                         insert_stop_velocity(traj_points, traj_points, end_stop_arc, input, module, false);
    const bool near_end = end_arc && std::abs(*end_arc) < planner_param_.near_line_distance;
    const bool stopped = std::abs(input.current_odometry->twist.twist.linear.x) < 1e-3;
    if (update_state && near_end && stopped) {
      set_state(module, ModuleState::FINALIZING, reg_elem.getEndLines().front().id());
    }
    update_command(module);
    return changed;
  }

  update_command(module);
  return false;
}

bool VirtualTrafficLightStop::insert_stop_velocity(
  TrajectoryPoints & traj_points, const TrajectoryPoints & path_points,
  const std::optional<double> & collision_s, const InputData & input, Module & module,
  const bool stop_line)
{
  auto path_result = Trajectory::Builder{}.build(path_points);
  if (!path_result) {
    return false;
  }
  auto modified_path = *path_result;
  geometry_msgs::msg::Pose stop_pose = input.current_odometry->pose.pose;
  const auto ego_s_opt = autoware::experimental::trajectory::find_first_nearest_index(
    modified_path, input.current_odometry->pose.pose, 5.0, planner_param_.max_yaw_deviation_rad);
  const auto ego_s = ego_s_opt.value_or(0.0);

  if (!collision_s) {
    modified_path.longitudinal_velocity_mps() = 0.0;
  } else {
    const auto stop_s = std::max(0.0, *collision_s - context_->vehicle_info.max_longitudinal_offset_m);
    const auto stop_distance = *collision_s - ego_s - context_->vehicle_info.max_longitudinal_offset_m;
    const bool is_stopped = std::abs(input.current_odometry->twist.twist.linear.x) < 1e-3;
    const auto start_s =
      is_stopped && stop_distance < planner_param_.hold_stop_margin_distance ? ego_s : stop_s;
    modified_path.longitudinal_velocity_mps().range(start_s, modified_path.length()).set(0.0);
    stop_pose = modified_path.compute(std::clamp(stop_s, 0.0, modified_path.length())).pose;
  }

  traj_points = modified_path.restore();
  const auto distance = std::max(
    0.0, autoware::motion_utils::calcSignedArcLength(
           traj_points, input.current_odometry->pose.pose.position, stop_pose.position));
  planning_factor_interface_->add(
    distance, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{});

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM VirtualTrafficLightStop] Inserted %s stop for VTL %s",
    stop_line ? "stop-line" : "end-line", module.instrument_id.c_str());
  return true;
}

void VirtualTrafficLightStop::update_command(Module & module)
{
  tier4_v2x_msgs::msg::InfrastructureCommand command;
  command.stamp = get_clock()->now();
  command.type = module.instrument_type;
  command.id = module.instrument_id;
  command.state = static_cast<uint8_t>(module.state);
  command.custom_tags = module.custom_tags;
  module.infrastructure_command = command;
}

void VirtualTrafficLightStop::set_state(
  Module & module, const ModuleState state, const std::optional<lanelet::Id> end_line_id)
{
  if (module.state == state) {
    return;
  }
  if (state == ModuleState::FINALIZING || state == ModuleState::FINALIZED) {
    RCLCPP_INFO(
      get_node_ptr()->get_logger(),
      "[TM VirtualTrafficLightStop] VTL %s state %s (line %ld)", module.instrument_id.c_str(),
      state_to_string(state).c_str(), end_line_id.value_or(0));
  } else {
    RCLCPP_INFO(
      get_node_ptr()->get_logger(), "[TM VirtualTrafficLightStop] VTL %s state %s",
      module.instrument_id.c_str(), state_to_string(state).c_str());
  }
  module.state = state;
}

bool VirtualTrafficLightStop::is_state_timeout(const Module & module) const
{
  if (!module.virtual_traffic_light_state) {
    return false;
  }
  const auto delay =
    (get_clock()->now() - rclcpp::Time(module.virtual_traffic_light_state->stamp)).seconds();
  return delay > planner_param_.max_delay_sec;
}

bool VirtualTrafficLightStop::has_right_of_way(const Module & module) const
{
  return module.virtual_traffic_light_state && module.virtual_traffic_light_state->approval;
}

std::string VirtualTrafficLightStop::module_key(
  const lanelet::Id lane_id, const lanelet::Id regulatory_element_id)
{
  return std::to_string(lane_id) + ":" + std::to_string(regulatory_element_id);
}

std::string VirtualTrafficLightStop::state_to_string(const ModuleState state)
{
  switch (state) {
    case ModuleState::NONE:
      return "NONE";
    case ModuleState::REQUESTING:
      return "REQUESTING";
    case ModuleState::PASSING:
      return "PASSING";
    case ModuleState::FINALIZING:
      return "FINALIZING";
    case ModuleState::FINALIZED:
      return "FINALIZED";
    default:
      return "UNKNOWN";
  }
}
}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::VirtualTrafficLightStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
