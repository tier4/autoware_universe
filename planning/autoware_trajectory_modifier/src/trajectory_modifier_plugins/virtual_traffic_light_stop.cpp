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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <rclcpp/duration.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

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
  constexpr double collision_search_epsilon = 1e-2;
  auto cropped_path = path;
  cropped_path.crop(0.0, std::min(end_line_s + collision_search_epsilon, cropped_path.length()));
  const auto collisions = autoware::experimental::trajectory::crossed(cropped_path, line);
  if (collisions.empty()) {
    return std::nullopt;
  }
  return collisions.back();
}

std::optional<double> calc_arc_length_from_collision(
  const Trajectory & path, const double end_line_s, const lanelet::ConstLineString3d & line,
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

constexpr size_t k_min_control_points = 3;
constexpr double k_control_resample_ds = 0.1;
// Must stay above searchZeroVelocityIndex epsilon (1e-3). A zero first point makes
// PID calcStopDistance ≈ 0, so the vehicle never leaves STOPPED.
constexpr float k_min_start_velocity_mps = 0.3F;

size_t count_control_resampled_points(const TrajectoryPoints & points)
{
  const auto length = autoware::motion_utils::calcArcLength(points);
  if (points.empty()) {
    return 0;
  }
  if (length <= 0.0) {
    return 1;
  }
  size_t count = 0;
  for (double s = 0.0; s < length; s += k_control_resample_ds) {
    ++count;
  }
  return count;
}

bool needs_control_start_resample(const TrajectoryPoints & points)
{
  return count_control_resampled_points(points) < k_min_control_points;
}

void retime_stationary_trajectory(TrajectoryPoints & points, const double time_step)
{
  const auto safe_time_step = std::max(time_step, 1e-3);
  for (size_t i = 0; i < points.size(); ++i) {
    points.at(i).time_from_start =
      rclcpp::Duration::from_seconds(static_cast<double>(i) * safe_time_step);
  }
}

std::optional<double> calc_arc_length_on_centerline(
  const lanelet::ConstLineString3d & centerline, const geometry_msgs::msg::Point & point)
{
  if (centerline.size() < 2) {
    return std::nullopt;
  }

  double accumulated_length = 0.0;
  double nearest_arc_length = 0.0;
  double nearest_distance_sq = std::numeric_limits<double>::max();
  bool found = false;

  for (size_t i = 1; i < centerline.size(); ++i) {
    const auto & p0 = centerline[i - 1];
    const auto & p1 = centerline[i];
    const auto dx = p1.x() - p0.x();
    const auto dy = p1.y() - p0.y();
    const auto segment_length_sq = dx * dx + dy * dy;
    if (segment_length_sq <= std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const auto segment_length = std::sqrt(segment_length_sq);
    const auto raw_ratio = ((point.x - p0.x()) * dx + (point.y - p0.y()) * dy) / segment_length_sq;
    const auto ratio = std::clamp(raw_ratio, 0.0, 1.0);
    const auto projected_x = p0.x() + ratio * dx;
    const auto projected_y = p0.y() + ratio * dy;
    const auto distance_x = point.x - projected_x;
    const auto distance_y = point.y - projected_y;
    const auto distance_sq = distance_x * distance_x + distance_y * distance_y;

    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_arc_length = accumulated_length + ratio * segment_length;
      found = true;
    }
    accumulated_length += segment_length;
  }

  if (!found) {
    return std::nullopt;
  }
  return nearest_arc_length;
}

geometry_msgs::msg::Point calc_line_center(const lanelet::ConstLineString3d & line)
{
  const auto center = (line.front().basicPoint() + line.back().basicPoint()) / 2;
  geometry_msgs::msg::Point point;
  point.x = center.x();
  point.y = center.y();
  point.z = center.z();
  return point;
}

std::optional<double> calc_arc_length_from_lanelet_centerline(
  const lanelet::ConstLanelet & lane, const lanelet::ConstLineString3d & line,
  const geometry_msgs::msg::Pose & ego_pose, const double front_offset)
{
  if (line.empty()) {
    return std::nullopt;
  }

  const auto centerline = lane.centerline();
  const auto ego_s = calc_arc_length_on_centerline(centerline, ego_pose.position);
  const auto line_s = calc_arc_length_on_centerline(centerline, calc_line_center(line));
  if (!ego_s || !line_s) {
    return std::nullopt;
  }

  return *line_s - *ego_s - front_offset;
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
  debug_marker_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/virtual_traffic_light_stop/debug/marker", 1);
  debug_text_pub_ = node_ptr->create_publisher<autoware_internal_debug_msgs::msg::StringStamped>(
    "~/virtual_traffic_light_stop/debug/text", 1);

  enabled_ = params.use_virtual_traffic_light_stop;
  planner_param_.max_delay_sec = params.virtual_traffic_light.max_delay_sec;
  planner_param_.near_line_distance = params.virtual_traffic_light.near_line_distance;
  planner_param_.dead_line_margin = params.virtual_traffic_light.dead_line_margin;
  planner_param_.max_yaw_deviation_rad =
    autoware_utils::deg2rad(params.virtual_traffic_light.max_yaw_deviation_deg);
  planner_param_.check_timeout_after_stop_line =
    params.virtual_traffic_light.check_timeout_after_stop_line;
  planner_param_.min_hold_trajectory_length =
    params.virtual_traffic_light.min_hold_trajectory_length;
  stopping_params_ = params.stopping_constraints;
  trajectory_time_step_ = params.trajectory_time_step;
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
  planner_param_.min_hold_trajectory_length =
    params.virtual_traffic_light.min_hold_trajectory_length;
  stopping_params_ = params.stopping_constraints;
  trajectory_time_step_ = params.trajectory_time_step;
}

void VirtualTrafficLightStop::begin_cycle(const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::begin_cycle", *get_time_keeper());
  if (!enabled_ || !input.current_odometry || !input.lanelet_map || !input.route) {
    modules_.clear();
    route_lanelet_ids_.clear();
    last_lanelet_map_.reset();
    return;
  }

  rebuild_modules(input);
  update_module_states(input);
  update_module_lifecycle(input);
}

void VirtualTrafficLightStop::end_cycle()
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::end_cycle", *get_time_keeper());
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

void VirtualTrafficLightStop::publish_debug_data(const std::string & ns) const
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::publish_debug_data", *get_time_keeper());
  using autoware::experimental::lanelet2_utils::to_ros;
  using autoware_utils::create_default_marker;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;
  using Marker = visualization_msgs::msg::Marker;

  if (!debug_marker_pub_ || !debug_text_pub_) {
    return;
  }

  const auto now = get_clock()->now();
  const auto lifetime = rclcpp::Duration::from_seconds(0.5);
  constexpr double line_z_offset = 0.5;
  constexpr double label_z_offset = 1.2;

  visualization_msgs::msg::MarkerArray marker_array;
  const auto start_color = create_marker_color(0.1f, 0.4f, 1.0f, 0.999f);
  const auto stop_color = create_marker_color(1.0f, 0.0f, 0.0f, 0.999f);
  const auto instrument_color = create_marker_color(1.0f, 0.8f, 0.0f, 0.999f);
  const auto end_color = create_marker_color(0.0f, 1.0f, 0.2f, 0.999f);

  const auto add_line = [&marker_array, &now, &lifetime, line_z_offset, label_z_offset](
                          const lanelet::ConstLineString3d & line, const std::string & marker_ns,
                          const int32_t id, const std_msgs::msg::ColorRGBA & color,
                          const std::string & label) {
    auto line_marker = create_default_marker(
      "map", now, marker_ns + "/line", id, Marker::LINE_STRIP, create_marker_scale(0.35, 0.0, 0.0),
      color);
    line_marker.lifetime = lifetime;
    for (const auto & p : line) {
      auto point = to_ros(p);
      point.z += line_z_offset;
      line_marker.points.push_back(point);
    }
    marker_array.markers.push_back(line_marker);

    auto text_marker = create_default_marker(
      "map", now, marker_ns + "/label", id, Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.0, 0.0, 1.2), color);
    text_marker.lifetime = lifetime;
    text_marker.pose.position = calc_line_center(line);
    text_marker.pose.position.z += line_z_offset + label_z_offset;
    text_marker.text = label;
    marker_array.markers.push_back(text_marker);
  };

  for (const auto & module : modules_) {
    const auto & reg_elem = *module.regulatory_element;
    const auto marker_ns =
      ns + "/vtl_" + std::to_string(module.lane_id) + "_" + std::to_string(reg_elem.id());
    int32_t marker_id = 0;

    {
      auto marker = create_default_marker(
        "map", now, marker_ns + "/instrument_status", marker_id++, Marker::TEXT_VIEW_FACING,
        create_marker_scale(0.0, 0.0, 1.0), create_marker_color(1.0f, 1.0f, 1.0f, 0.999f));
      marker.lifetime = lifetime;
      marker.pose.position = module.instrument_center;
      marker.pose.position.z += line_z_offset + label_z_offset;
      marker.text = "VTL " + module.instrument_id + " [" + state_to_string(module.state) + "]";
      marker_array.markers.push_back(marker);
    }

    {
      auto marker = create_default_marker(
        "map", now, marker_ns + "/instrument_center", marker_id++, Marker::SPHERE,
        create_marker_scale(0.3, 0.3, 0.3), instrument_color);
      marker.lifetime = lifetime;
      marker.pose.position = module.instrument_center;
      marker.pose.position.z += line_z_offset;
      marker_array.markers.push_back(marker);
    }

    add_line(
      reg_elem.getStartLine(), marker_ns + "/start_line", marker_id++, start_color, "start_line");
    if (const auto stop_line = reg_elem.getStopLine()) {
      add_line(*stop_line, marker_ns + "/stop_line", marker_id++, stop_color, "stop_line");
    }
    add_line(
      reg_elem.getVirtualTrafficLight(), marker_ns + "/instrument_line", marker_id++,
      instrument_color, "instrument_line");
    if (module.active_end_line) {
      add_line(
        module.active_end_line->line, marker_ns + "/active_end_line", marker_id++, end_color,
        "active_end_line[" + std::to_string(module.active_end_line->id) + "]");
    }
  }

  debug_marker_pub_->publish(marker_array);
  publish_debug_string(ns);
}

void VirtualTrafficLightStop::publish_debug_string(const std::string & ns) const
{
  const auto format_optional = [](const std::optional<double> & value) {
    if (!value) {
      return std::string{"n/a"};
    }
    std::ostringstream value_stream;
    value_stream << std::fixed << std::setprecision(2) << *value;
    return value_stream.str();
  };

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2) << std::boolalpha;
  ss << "VIRTUAL TRAFFIC LIGHT STOP MODIFIER:\n";
  ss << "  CANDIDATE: " << ns << "\n";
  ss << "  ENABLED: " << enabled_ << "\n";
  ss << "  MODULES: " << modules_.size() << "\n";
  for (const auto & module : modules_) {
    const auto & debug = module.debug_data;
    ss << "  VTL[" << module.instrument_id << "]:\n";
    ss << "    LANELET_ID: " << module.lane_id << "\n";
    ss << "    REGULATORY_ELEMENT_ID: " << module.regulatory_element->id() << "\n";
    ss << "    INSTRUMENT_TYPE: " << module.instrument_type << "\n";
    ss << "    STATE: " << state_to_string(module.state) << "\n";
    ss << "    ACTIVE_END_LINE_ID: "
       << (module.active_end_line ? std::to_string(module.active_end_line->id) : "INVALID") << "\n";
    ss << "    V2X: received=" << debug.has_vtl_state << " approval=" << debug.approval
       << " finalized=" << debug.finalized << " age_sec=" << format_optional(debug.message_age_sec)
       << " timeout=" << debug.timeout << "\n";
    ss << "    DECISION: " << decision_to_string(debug.decision)
       << " target=" << stop_target_to_string(debug.stop_target)
       << " reason=" << stop_reason_to_string(debug.stop_reason) << " modified=" << debug.modified
       << "\n";
    ss << "    DISTANCE_PATH_M: start=" << format_optional(debug.start_arc_path)
       << " stop=" << format_optional(debug.stop_arc_path)
       << " end=" << format_optional(debug.end_arc_path) << "\n";
    ss << "    DISTANCE_CENTERLINE_M: start=" << format_optional(debug.start_arc_centerline)
       << " stop=" << format_optional(debug.stop_arc_centerline)
       << " end=" << format_optional(debug.end_arc_centerline) << "\n";
    ss << "    STOP_DISTANCE_M: " << format_optional(debug.stop_distance) << "\n";
    ss << "    FLAGS: ego_in_lane=" << debug.ego_is_in_module_lane
       << " stop_line_relevant=" << debug.stop_line_relevant
       << " end_hold_active=" << module.end_hold_active << "\n";
  }

  autoware_internal_debug_msgs::msg::StringStamped msg;
  msg.stamp = get_clock()->now();
  msg.data = ss.str();
  debug_text_pub_->publish(msg);
}

bool VirtualTrafficLightStop::is_trajectory_modification_required(
  const TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::is_trajectory_modification_required", *get_time_keeper());
  if (!enabled_ || traj_points.size() < 2 || modules_.empty()) {
    return false;
  }

  auto copy = traj_points;
  return process_trajectory(copy, input, false);
}

bool VirtualTrafficLightStop::modify_trajectory(
  TrajectoryPoints & traj_points, const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::modify_trajectory", *get_time_keeper());
  for (auto & module : modules_) {
    module.debug_data = DebugData{};
  }
  if (!enabled_ || traj_points.size() < 2 || modules_.empty()) {
    return false;
  }

  return process_trajectory(traj_points, input, true);
}

void VirtualTrafficLightStop::rebuild_modules(const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::rebuild_modules", *get_time_keeper());
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
    previous_modules.emplace(
      module_key(module.lane_id, module.regulatory_element->id()), std::move(module));
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

      const auto stop_line = reg_elem->getStopLine();
      const auto centerline = lane->centerline();
      const auto stop_line_arc = stop_line && !stop_line->empty()
                                   ? calc_arc_length_on_centerline(
                                       centerline, calc_line_center(*stop_line))
                                   : std::optional<double>{};
      if (stop_line_arc) {
        constexpr double arc_epsilon = 1e-3;
        for (const auto & end_line : reg_elem->getEndLines()) {
          if (end_line.empty()) {
            continue;
          }
          const auto end_line_center = calc_line_center(end_line);
          if (!lanelet::geometry::inside(
                *lane, lanelet::BasicPoint2d{end_line_center.x, end_line_center.y})) {
            continue;
          }
          const auto end_line_arc =
            calc_arc_length_on_centerline(centerline, end_line_center);
          if (!end_line_arc || *end_line_arc <= *stop_line_arc + arc_epsilon) {
            continue;
          }
          if (
            !module.active_end_line ||
            *end_line_arc < module.active_end_line->centerline_arc - arc_epsilon ||
            (std::abs(*end_line_arc - module.active_end_line->centerline_arc) <= arc_epsilon &&
             end_line.id() < module.active_end_line->id)) {
            module.active_end_line = ActiveEndLine{end_line.id(), end_line, *end_line_arc};
          }
        }
      }

      const auto instrument = reg_elem->getVirtualTrafficLight();
      const auto type = instrument.attribute("type").as<std::string>();
      module.instrument_type = type ? *type : "virtual_traffic_light";
      module.instrument_id = std::to_string(instrument.id());
      const lanelet::BasicPoint3d instrument_center =
        (instrument.front().basicPoint() + instrument.back().basicPoint()) / 2;
      module.instrument_center = autoware::experimental::lanelet2_utils::to_ros(instrument_center);
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
        module.virtual_traffic_light_state = previous->second.virtual_traffic_light_state;
        const bool same_active_end_line =
          module.active_end_line && previous->second.active_end_line &&
          module.active_end_line->id == previous->second.active_end_line->id;
        if (same_active_end_line) {
          module.state = previous->second.state;
          module.end_hold_active = previous->second.end_hold_active;
        }
      }
      if (!module.active_end_line) {
        RCLCPP_ERROR_THROTTLE(
          get_node_ptr()->get_logger(), *get_clock(), 5000,
          "[TM VirtualTrafficLightStop] VTL %s has no valid end line downstream of its stop line",
          module.instrument_id.c_str());
      }
      modules_.push_back(std::move(module));
    }
  }
}

void VirtualTrafficLightStop::update_module_states(const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::update_module_states", *get_time_keeper());
  for (auto & module : modules_) {
    const auto previous_state = module.virtual_traffic_light_state;
    module.virtual_traffic_light_state.reset();
    if (!input.virtual_traffic_light_states) {
      continue;
    }
    for (const auto & state : input.virtual_traffic_light_states->states) {
      if (state.id == module.instrument_id) {
        const bool approval_changed =
          !previous_state || previous_state->approval != state.approval;
        const bool finalized_changed =
          !previous_state || previous_state->is_finalized != state.is_finalized;
        if (approval_changed || finalized_changed) {
          RCLCPP_INFO(
            get_node_ptr()->get_logger(),
            "[TM VirtualTrafficLightStop] Received VTL %s state: approval=%s finalized=%s "
            "stamp=%d.%09u",
            state.id.c_str(), state.approval ? "true" : "false",
            state.is_finalized ? "true" : "false", state.stamp.sec, state.stamp.nanosec);
        }
        module.virtual_traffic_light_state = state;
        break;
      }
    }
  }
}

void VirtualTrafficLightStop::update_module_lifecycle(const InputData & input)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::update_module_lifecycle", *get_time_keeper());
  const auto & ego_pose = input.current_odometry->pose.pose;
  const auto front_offset = context_->vehicle_info.max_longitudinal_offset_m;
  const bool stopped = std::abs(input.current_odometry->twist.twist.linear.x) < 1e-3;

  for (auto & module : modules_) {
    const auto & reg_elem = *module.regulatory_element;
    const auto start_arc = calc_arc_length_from_lanelet_centerline(
      module.lane, reg_elem.getStartLine(), ego_pose, front_offset);
    const auto stop_line = reg_elem.getStopLine();
    const auto stop_arc = stop_line ? calc_arc_length_from_lanelet_centerline(
                                      module.lane, *stop_line, ego_pose, front_offset)
                                    : std::optional<double>{};
    const auto end_arc = module.active_end_line
                           ? calc_arc_length_from_lanelet_centerline(
                               module.lane, module.active_end_line->line, ego_pose, front_offset)
                           : std::optional<double>{};

    if (start_arc && *start_arc > 0.0) {
      set_state(module, ModuleState::NONE);
      update_command(module);
      continue;
    }
    if (!stop_line || !module.active_end_line || !stop_arc || !end_arc) {
      module.end_hold_active = false;
      set_state(module, ModuleState::REQUESTING);
      update_command(module);
      continue;
    }
    if (*end_arc < -planner_param_.dead_line_margin) {
      module.end_hold_active = true;
      set_state(module, ModuleState::FINALIZED, module.active_end_line->id);
      update_command(module);
      continue;
    }

    const bool before_stop_line = *stop_arc > -planner_param_.dead_line_margin;
    const bool timeout = is_state_timeout(module);
    const bool approved = has_right_of_way(module);
    if (!approved || (before_stop_line && timeout)) {
      set_state(module, ModuleState::REQUESTING);
      update_command(module);
      continue;
    }
    if (before_stop_line) {
      module.end_hold_active = false;
      set_state(module, ModuleState::REQUESTING);
      update_command(module);
      continue;
    }

    if (
      *end_arc >= -planner_param_.dead_line_margin &&
      *end_arc < planner_param_.min_hold_trajectory_length) {
      module.end_hold_active = true;
    }
    const bool externally_finalized =
      module.virtual_traffic_light_state && module.virtual_traffic_light_state->is_finalized;
    if (!externally_finalized && std::abs(*end_arc) < planner_param_.near_line_distance && stopped) {
      set_state(module, ModuleState::FINALIZING, module.active_end_line->id);
    } else {
      set_state(module, ModuleState::PASSING);
    }
    update_command(module);
  }
}

bool VirtualTrafficLightStop::process_trajectory(
  TrajectoryPoints & traj_points, const InputData & input, const bool apply_modification)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::process_trajectory", *get_time_keeper());
  bool modified = false;
  for (auto & module : modules_) {
    modified = process_module(module, traj_points, input, apply_modification) || modified;
  }
  return modified;
}

bool VirtualTrafficLightStop::process_module(
  Module & module, TrajectoryPoints & traj_points, const InputData & input,
  const bool apply_modification)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::process_module", *get_time_keeper());
  if (apply_modification) {
    module.debug_data = DebugData{};
    module.debug_data.has_vtl_state = module.virtual_traffic_light_state.has_value();
    if (module.virtual_traffic_light_state) {
      module.debug_data.approval = module.virtual_traffic_light_state->approval;
      module.debug_data.finalized = module.virtual_traffic_light_state->is_finalized;
      module.debug_data.message_age_sec =
        (get_clock()->now() - rclcpp::Time(module.virtual_traffic_light_state->stamp)).seconds();
      module.debug_data.timeout = *module.debug_data.message_age_sec > planner_param_.max_delay_sec;
    }
  }

  auto path_result = Trajectory::Builder{}.build(traj_points);
  if (!path_result) {
    return false;
  }
  auto & path = *path_result;

  const auto & reg_elem = *module.regulatory_element;
  const auto end_collision = module.active_end_line
                               ? find_last_collision_before_line(
                                   path, path.length(), module.active_end_line->line)
                               : std::optional<double>{};
  const auto collision_search_limit = end_collision.value_or(path.length());

  const auto ego_pose = input.current_odometry->pose.pose;
  const auto front_offset = context_->vehicle_info.max_longitudinal_offset_m;
  const auto start_arc = calc_arc_length_from_collision(
    path, collision_search_limit, reg_elem.getStartLine(), ego_pose, front_offset,
    planner_param_.max_yaw_deviation_rad);
  const auto start_centerline_arc = calc_arc_length_from_lanelet_centerline(
    module.lane, reg_elem.getStartLine(), ego_pose, front_offset);
  const auto stop_line = reg_elem.getStopLine();
  const auto stop_arc = stop_line ? calc_arc_length_from_collision(
                                      path, collision_search_limit, *stop_line, ego_pose,
                                      front_offset, planner_param_.max_yaw_deviation_rad)
                                  : std::optional<double>{};
  const auto stop_collision =
    stop_line ? find_last_collision_before_line(path, collision_search_limit, *stop_line)
              : std::optional<double>{};
  const auto stop_centerline_arc = stop_line ? calc_arc_length_from_lanelet_centerline(
                                                 module.lane, *stop_line, ego_pose, front_offset)
                                             : std::optional<double>{};
  const auto end_arc = module.active_end_line
                         ? calc_arc_length_from_collision(
                             path, path.length(), module.active_end_line->line, ego_pose,
                             front_offset, planner_param_.max_yaw_deviation_rad)
                         : std::optional<double>{};
  const auto end_centerline_arc = module.active_end_line
                                    ? calc_arc_length_from_lanelet_centerline(
                                        module.lane, module.active_end_line->line, ego_pose,
                                        front_offset)
                                    : std::optional<double>{};

  if (apply_modification) {
    auto & debug = module.debug_data;
    debug.start_arc_path = start_arc;
    debug.start_arc_centerline = start_centerline_arc;
    debug.stop_arc_path = stop_arc;
    debug.stop_arc_centerline = stop_centerline_arc;
    debug.end_arc_path = end_arc;
    debug.end_arc_centerline = end_centerline_arc;
  }

  if (start_arc && *start_arc > 0.0) {
    return false;
  }

  if (!stop_line) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 5000,
      "[TM VirtualTrafficLightStop] VTL %s has no stop line", module.instrument_id.c_str());
    return false;
  }

  const bool before_stop_line_by_map =
    stop_centerline_arc && *stop_centerline_arc > -planner_param_.dead_line_margin;
  const bool before_stop_line = (stop_arc && *stop_arc > -planner_param_.dead_line_margin) ||
                                (!stop_arc && before_stop_line_by_map);
  const bool still_approaching_stopline =
    (stop_arc && *stop_arc > 0.0) ||
    (!stop_arc && stop_centerline_arc && *stop_centerline_arc > 0.0) ||
    (!stop_arc && !stop_centerline_arc);
  const auto ego_point = lanelet::BasicPoint2d{ego_pose.position.x, ego_pose.position.y};
  const bool ego_is_in_module_lane = lanelet::geometry::inside(module.lane, ego_point);
  const bool stop_line_relevant = stop_collision || (ego_is_in_module_lane && !before_stop_line);
  const bool timeout = is_state_timeout(module);
  const bool no_state = !module.virtual_traffic_light_state;
  const bool no_right_of_way = no_state || !has_right_of_way(module);
  if (apply_modification) {
    module.debug_data.ego_is_in_module_lane = ego_is_in_module_lane;
    module.debug_data.stop_line_relevant = stop_line_relevant;
    module.debug_data.timeout = timeout;
  }
  if (module.state == ModuleState::FINALIZED) {
    return false;
  }

  const auto stop_at_stop_line = [&](const StopReason reason) {
    if (!stop_line_relevant) {
      return false;
    }
    if (apply_modification) {
      RCLCPP_INFO_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 5000,
        "[TM VirtualTrafficLightStop] VTL %s requires a stop at its stop line: %s",
        module.instrument_id.c_str(), stop_reason_to_string(reason).c_str());
    }
    const auto modified = apply_modification && insert_stop_velocity(
                                                  traj_points, traj_points, stop_collision, input,
                                                  module, reason, StopTarget::STOP_LINE);
    return apply_modification ? modified : true;
  };

  if (!module.active_end_line) {
    return stop_at_stop_line(StopReason::INVALID_END_LINE);
  }

  if (no_right_of_way) {
    return stop_at_stop_line(no_state ? StopReason::NO_STATE : StopReason::NO_RIGHT_OF_WAY);
  }

  if (before_stop_line) {
    if (timeout) {
      return stop_at_stop_line(StopReason::STATE_TIMEOUT_BEFORE_STOP_LINE);
    }
    if (still_approaching_stopline) {
      const bool needs_start_resample = needs_control_start_resample(traj_points);
      if (!needs_start_resample) {
        return false;
      }
      if (!apply_modification) {
        return true;
      }
      return ensure_control_start_trajectory(traj_points, input, module);
    }
  }

  const bool near_or_past_end_line =
    module.end_hold_active ||
    (end_arc && *end_arc < planner_param_.min_hold_trajectory_length);
  if (planner_param_.check_timeout_after_stop_line && timeout && !near_or_past_end_line) {
    return stop_at_stop_line(StopReason::STATE_TIMEOUT_AFTER_STOP_LINE);
  }

  if (!module.virtual_traffic_light_state->is_finalized) {
    auto end_stop_collision = end_collision;
    if (!end_stop_collision && module.end_hold_active && end_centerline_arc) {
      const auto ego_s_opt = autoware::experimental::trajectory::find_first_nearest_index(
        path, ego_pose, 5.0, planner_param_.max_yaw_deviation_rad);
      end_stop_collision = ego_s_opt.value_or(0.0) + *end_centerline_arc + front_offset;
    }
    const bool skip_end_stop = !end_stop_collision && !module.end_hold_active;
    const auto changed = apply_modification && !skip_end_stop &&
                         insert_stop_velocity(
                           traj_points, traj_points, end_stop_collision, input, module,
                           StopReason::WAITING_FINALIZATION, StopTarget::END_LINE);
    if (apply_modification && !skip_end_stop) {
      RCLCPP_INFO_THROTTLE(
        get_node_ptr()->get_logger(), *get_clock(), 5000,
        "[TM VirtualTrafficLightStop] VTL %s is waiting for finalization; stop at end line",
        module.instrument_id.c_str());
    }
    return apply_modification ? changed : !skip_end_stop;
  }

  return false;
}

bool VirtualTrafficLightStop::insert_stop_velocity(
  TrajectoryPoints & traj_points, const TrajectoryPoints & path_points,
  const std::optional<double> & collision_s, const InputData & input, Module & module,
  const StopReason reason, const StopTarget target)
{
  autoware_utils_debug::ScopedTimeTrack st(
    "VirtualTrafficLightStop::insert_stop_velocity", *get_time_keeper());
  auto & debug = module.debug_data;
  debug.decision = Decision::STOP;
  debug.stop_reason = reason;
  debug.stop_target = target;

  auto path_result = Trajectory::Builder{}.build(path_points);
  if (!path_result) {
    return false;
  }
  auto path = *path_result;
  const auto & ego_pose = input.current_odometry->pose.pose;
  geometry_msgs::msg::Pose stop_pose = ego_pose;
  const auto ego_s = autoware::experimental::trajectory::find_first_nearest_index(
                       path, ego_pose, 5.0, planner_param_.max_yaw_deviation_rad)
                       .value_or(0.0);
  const auto ego_vel = input.current_odometry->twist.twist.linear.x;
  const auto ego_accel =
    input.current_acceleration ? input.current_acceleration->accel.accel.linear.x : 0.0;

  std::optional<double> stop_s;
  if (collision_s) {
    const auto geometric_stop_s =
      std::max(0.0, *collision_s - context_->vehicle_info.max_longitudinal_offset_m);
    stop_s = utils::clamp_stop_point_arc_length(
      geometric_stop_s, path.length(), ego_vel, ego_accel, stopping_params_.maximum_deceleration,
      stopping_params_.jerk_limit);
    stop_pose = path.compute(std::clamp(*stop_s, 0.0, path.length())).pose;
  }

  const auto target_s = stop_s.value_or(ego_s);
  if (utils::stop_point_exists(traj_points, target_s)) {
    RCLCPP_WARN_THROTTLE(
      get_node_ptr()->get_logger(), *get_clock(), 1000,
      "[TM VirtualTrafficLightStop] Preceding (or duplicate) stop point exists, skip inserting "
      "%s stop for VTL %s",
      stop_target_to_string(target).c_str(), module.instrument_id.c_str());
    return false;
  }

  const auto remaining = stop_s ? (*stop_s - ego_s) : 0.0;
  if (
    !stop_s || remaining < stopping_params_.arrived_distance_threshold ||
    !utils::insert_stop_point(traj_points, *stop_s, trajectory_time_step_)) {
    utils::replace_trajectory_with_stop_point(traj_points, ego_pose, trajectory_time_step_);
    stop_pose = ego_pose;
  }

  const auto raw_distance = autoware::motion_utils::calcSignedArcLength(
    traj_points, ego_pose.position, stop_pose.position);
  const auto distance = std::isnan(raw_distance) ? 0.0 : std::max(0.0, raw_distance);
  const auto detail = "VTL " + module.instrument_id + ": " + stop_reason_to_string(reason) +
                      " -> " + stop_target_to_string(target);
  planning_factor_interface_->add(
    distance, stop_pose, PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true, 0.0, 0.0, detail);

  debug.modified = true;
  debug.stop_distance = distance;

  RCLCPP_WARN_THROTTLE(
    get_node_ptr()->get_logger(), *get_clock(), 1000,
    "[TM VirtualTrafficLightStop] Inserted %s stop for VTL %s",
    stop_target_to_string(target).c_str(), module.instrument_id.c_str());
  return true;
}

bool VirtualTrafficLightStop::ensure_control_start_trajectory(
  TrajectoryPoints & traj_points, const InputData & input, const Module & /*module*/) const
{
  if (traj_points.empty() || !needs_control_start_resample(traj_points)) {
    return false;
  }

  const auto original = traj_points;
  const auto min_length = std::max(
    planner_param_.min_hold_trajectory_length,
    k_control_resample_ds * static_cast<double>(k_min_control_points - 1) + 1e-3);
  const double point_interval = k_control_resample_ds;
  constexpr float start_acceleration_mps2 = 1.0F;

  TrajectoryPoint seed = original.front();
  seed.pose = input.current_odometry->pose.pose;
  const auto ego_v = static_cast<float>(input.current_odometry->twist.twist.linear.x);
  seed.longitudinal_velocity_mps = std::max(std::max(0.0F, ego_v), k_min_start_velocity_mps);
  seed.lateral_velocity_mps = 0.0F;
  seed.acceleration_mps2 = start_acceleration_mps2;
  seed.heading_rate_rps = 0.0F;
  seed.time_from_start = rclcpp::Duration::from_seconds(0.0);

  TrajectoryPoints rebuilt;
  rebuilt.push_back(seed);

  while (count_control_resampled_points(rebuilt) < k_min_control_points ||
         autoware::motion_utils::calcArcLength(rebuilt) < min_length) {
    TrajectoryPoint point = rebuilt.back();
    const auto pose = autoware_utils::calc_offset_pose(point.pose, point_interval, 0.0, 0.0);
    const auto ds = std::hypot(
      pose.position.x - point.pose.position.x, pose.position.y - point.pose.position.y);
    point.pose = pose;
    const auto v0 = std::max(0.0F, point.longitudinal_velocity_mps);
    point.longitudinal_velocity_mps =
      std::sqrt(v0 * v0 + 2.0F * start_acceleration_mps2 * static_cast<float>(ds));
    point.acceleration_mps2 = start_acceleration_mps2;
    rebuilt.push_back(point);
    if (rebuilt.size() > 80) {
      break;
    }
  }

  retime_stationary_trajectory(rebuilt, trajectory_time_step_);
  traj_points = std::move(rebuilt);

  return count_control_resampled_points(traj_points) >= k_min_control_points;
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
      get_node_ptr()->get_logger(), "[TM VirtualTrafficLightStop] VTL %s state %s (line %ld)",
      module.instrument_id.c_str(), state_to_string(state).c_str(), end_line_id.value_or(0));
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

std::string VirtualTrafficLightStop::decision_to_string(const Decision decision)
{
  switch (decision) {
    case Decision::NONE:
      return "NONE";
    case Decision::STOP:
      return "STOP";
    default:
      return "UNKNOWN";
  }
}

std::string VirtualTrafficLightStop::stop_target_to_string(const StopTarget target)
{
  switch (target) {
    case StopTarget::NONE:
      return "NONE";
    case StopTarget::STOP_LINE:
      return "STOP_LINE";
    case StopTarget::END_LINE:
      return "END_LINE";
    default:
      return "UNKNOWN";
  }
}

std::string VirtualTrafficLightStop::stop_reason_to_string(const StopReason reason)
{
  switch (reason) {
    case StopReason::NONE:
      return "NONE";
    case StopReason::NO_STATE:
      return "NO_STATE";
    case StopReason::NO_RIGHT_OF_WAY:
      return "NO_RIGHT_OF_WAY";
    case StopReason::STATE_TIMEOUT_BEFORE_STOP_LINE:
      return "STATE_TIMEOUT_BEFORE_STOP_LINE";
    case StopReason::STATE_TIMEOUT_AFTER_STOP_LINE:
      return "STATE_TIMEOUT_AFTER_STOP_LINE";
    case StopReason::WAITING_FINALIZATION:
      return "WAITING_FINALIZATION";
    case StopReason::INVALID_END_LINE:
      return "INVALID_END_LINE";
    default:
      return "UNKNOWN";
  }
}
}  // namespace autoware::trajectory_modifier::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_modifier::plugin::VirtualTrafficLightStop,
  autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase)
