
// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_

#include <autoware/universe_utils/ros/parameter.hpp>
#include <autoware/universe_utils/ros/update_param.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{
/// @brief parameters for the filtering of the dynamic objects
struct ObjectParameters
{
  bool ignore_if_stopped;
  double stopped_velocity_threshold;
  bool ignore_if_on_ego_trajectory;
  bool ignore_if_behind_ego;
  bool ignore_if_on_crosswalk;
  std::vector<std::string> cut_linestring_types;
  std::vector<std::string> cut_polygon_types;
  bool cut_if_crossing_ego_from_behind;
  double confidence_filtering_threshold;
  bool confidence_filtering_only_use_highest;
};
/// @brief Parameters of the run_out module
struct Parameters
{
  double
    max_history_duration;  // [s]  calculated as the maximum duration among all buffer parameters
  double ego_time_interval_expansion;  // [s] time to add to the ego time interval (on both the
                                       // start and end times)
  double stop_on_time_buffer;   // [s] successive collision detection time required to start the
                                // stopping decision
  double stop_off_time_buffer;  // [s] successive non-collision detection time required to remove a
                                // stopping decision
  double stop_distance_buffer;  // [m] longitudinal safety distance to keep between ego and the
                                // collision position
  bool
    stop_calculate_earliest_within_history;  // if true, use the earliest stop position within the
                                             // history, otherwise the stop position is calculated
                                             // only from the currently detected collision
  bool keep_stop_until_object_is_gone;  // if true, once ego stopped for an object we keep the stop
                                        // decision until the object is gone

  double preventive_slowdown_on_time_buffer;  // [s] successive collision detection time required to
                                              // start the slowdown decision
  double preventive_slowdown_off_time_buffer;     // [s] successive non-collision detection time
                                                  // required to remove the slowdown decision
  double preventive_slowdown_distance_buffer;     // [m] longitudinal distance between the collision
                                                  // and the slowdown positions
  double preventive_slowdown_deceleration_limit;  // [m/s²] minimum deceleration that can be applied
                                                  // by the preventive slowdown

  bool enable_passing_collisions;         // If true, a collision where ego arrives first is ignored
  double passing_collisions_time_margin;  // [s] required time margin to decide a passing_collision
  double passing_max_overlap_duration;    // [s] the collision is not ignored if ego is predicted to
                                          // stay on the object's path for longer than this duration
  bool enable_deceleration_limit;  // if true, collisions are not ignored if happening within the
                                   // minimum duration required to stop ego comfortably
  double ego_lateral_margin;       // [m] ego footprint lateral margin
  double ego_longitudinal_margin;  // [m] ego footprint longitudinal margin
  double collision_time_margin;    // [s] extra time margin to determine collisions
  double time_overlap_tolerance;   // [s] when calculating overlap time intervals, intervals are
                                  // grouped if they are separated by less than this tolerance value
  // object parameters
  std::vector<std::string> objects_target_labels;
  std::vector<ObjectParameters> object_parameters_per_label;

  /// @brief Get the parameter defined for a specific object label, or the default value if it was
  /// not specified
  template <class T>
  auto get_object_parameter(
    rclcpp::Node & node, const std::string & ns, const uint8_t object_label,
    const std::string & param)
  {
    using universe_utils::getOrDeclareParameter;
    const auto label_str = ".objects." + label_to_string(object_label);
    try {
      return getOrDeclareParameter<T>(node, ns + label_str + param);
    } catch (const std::exception &) {
      return getOrDeclareParameter<T>(node, ns + ".objects.DEFAULT" + param);
    }
  }
  /// @brief Initialize the parameters
  void initialize(rclcpp::Node & node, const std::string & ns)
  {
    using universe_utils::getOrDeclareParameter;
    enable_passing_collisions =
      getOrDeclareParameter<bool>(node, ns + ".passing.enable_passing_margin");
    passing_collisions_time_margin =
      getOrDeclareParameter<double>(node, ns + ".passing.time_margin");
    passing_max_overlap_duration =
      getOrDeclareParameter<double>(node, ns + ".passing.max_overlap_duration");
    enable_deceleration_limit =
      getOrDeclareParameter<bool>(node, ns + ".passing.enable_deceleration_limit");
    stop_off_time_buffer = getOrDeclareParameter<double>(node, ns + ".stop.off_time_buffer");
    stop_on_time_buffer = getOrDeclareParameter<double>(node, ns + ".stop.on_time_buffer");
    stop_distance_buffer = getOrDeclareParameter<double>(node, ns + ".stop.distance_buffer");
    preventive_slowdown_off_time_buffer =
      getOrDeclareParameter<double>(node, ns + ".preventive_slowdown.off_time_buffer");
    preventive_slowdown_on_time_buffer =
      getOrDeclareParameter<double>(node, ns + ".preventive_slowdown.on_time_buffer");
    preventive_slowdown_distance_buffer =
      getOrDeclareParameter<double>(node, ns + ".preventive_slowdown.distance_buffer");
    preventive_slowdown_deceleration_limit =
      getOrDeclareParameter<double>(node, ns + ".preventive_slowdown.deceleration_limit");
    stop_calculate_earliest_within_history =
      getOrDeclareParameter<bool>(node, ns + ".stop.calculate_earliest_position_within_history");
    keep_stop_until_object_is_gone =
      getOrDeclareParameter<bool>(node, ns + ".stop.keep_until_object_is_gone");
    ego_lateral_margin = getOrDeclareParameter<double>(node, ns + ".ego.lateral_margin");
    ego_longitudinal_margin = getOrDeclareParameter<double>(node, ns + ".ego.longitudinal_margin");
    collision_time_margin = getOrDeclareParameter<double>(node, ns + ".collision_time_margin");
    time_overlap_tolerance = getOrDeclareParameter<double>(node, ns + ".time_overlap_tolerance");

    const auto all_object_labels = all_labels();
    object_parameters_per_label.resize(
      *std::max_element(all_object_labels.begin(), all_object_labels.end()) + 1);
    objects_target_labels =
      getOrDeclareParameter<std::vector<std::string>>(node, ns + ".objects.target_labels");
    for (const auto label : all_labels()) {
      object_parameters_per_label[label].ignore_if_on_ego_trajectory =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_on_ego_trajectory");
      object_parameters_per_label[label].ignore_if_behind_ego =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_behind_ego");
      object_parameters_per_label[label].ignore_if_on_crosswalk =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_on_crosswalk");
      object_parameters_per_label[label].ignore_if_stopped =
        get_object_parameter<bool>(node, ns, label, ".ignore.if_stopped");
      object_parameters_per_label[label].stopped_velocity_threshold =
        get_object_parameter<double>(node, ns, label, ".ignore.stopped_velocity_threshold");
      object_parameters_per_label[label].confidence_filtering_threshold =
        get_object_parameter<double>(node, ns, label, ".confidence_filtering.threshold");
      object_parameters_per_label[label].confidence_filtering_only_use_highest =
        get_object_parameter<bool>(node, ns, label, ".confidence_filtering.only_use_highest");
      object_parameters_per_label[label].cut_polygon_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.polygon_types");
      object_parameters_per_label[label].cut_linestring_types =
        get_object_parameter<std::vector<std::string>>(
          node, ns, label, ".cut_predicted_paths.linestring_types");
      object_parameters_per_label[label].cut_if_crossing_ego_from_behind =
        get_object_parameter<bool>(
          node, ns, label, ".cut_predicted_paths.if_crossing_ego_from_behind");
    }

    max_history_duration = std::max(stop_off_time_buffer, stop_on_time_buffer);
  }
  /// @brief Update the parameters
  void update(const std::vector<rclcpp::Parameter> & params, const std::string & ns)
  {
    using universe_utils::updateParam;
    updateParam(params, ns + ".passing.enable_passing_margin", enable_passing_collisions);
    updateParam(params, ns + ".passing.time_margin", passing_collisions_time_margin);
    updateParam(params, ns + ".passing.max_overlap_duration", passing_max_overlap_duration);
    updateParam(params, ns + ".passing.enable_deceleration_limit", enable_deceleration_limit);
    updateParam(
      params, ns + ".preventive_slowdown.on_time_buffer", preventive_slowdown_on_time_buffer);
    updateParam(
      params, ns + ".preventive_slowdown.off_time_buffer", preventive_slowdown_off_time_buffer);
    updateParam(
      params, ns + ".preventive_slowdown.distance_buffer", preventive_slowdown_distance_buffer);
    updateParam(
      params, ns + ".preventive_slowdown.deceleration_limit",
      preventive_slowdown_deceleration_limit);
    updateParam(params, ns + ".stop.on_time_buffer", stop_on_time_buffer);
    updateParam(params, ns + ".stop.off_time_buffer", stop_off_time_buffer);
    updateParam(params, ns + ".stop.distance_buffer", stop_distance_buffer);
    updateParam(
      params, ns + ".stop.calculate_earliest_position_within_history",
      stop_calculate_earliest_within_history);
    updateParam(params, ns + ".stop.keep_until_object_is_gone", keep_stop_until_object_is_gone);
    updateParam(params, ns + ".ego.lateral_margin", ego_lateral_margin);
    updateParam(params, ns + ".ego.longitudinal_margin", ego_longitudinal_margin);
    updateParam(params, ns + ".collision_time_margin", collision_time_margin);
    updateParam(params, ns + ".time_overlap_tolerance", time_overlap_tolerance);

    updateParam(params, ns + ".objects.target_labels", objects_target_labels);

    for (const auto label : all_labels()) {
      const auto str = ".objects." + label_to_string(label);
      updateParam(
        params, ns + str + ".ignore.if_stopped",
        object_parameters_per_label[label].ignore_if_stopped);
      updateParam(
        params, ns + str + ".ignore.stopped_velocity_threshold",
        object_parameters_per_label[label].stopped_velocity_threshold);
      updateParam(
        params, ns + str + ".ignore.if_on_ego_trajectory",
        object_parameters_per_label[label].ignore_if_on_ego_trajectory);
      updateParam(
        params, ns + ".ignore.if_behind_ego",
        object_parameters_per_label[label].ignore_if_behind_ego);
      updateParam(
        params, ns + str + ".ignore.if_on_crosswalk",
        object_parameters_per_label[label].ignore_if_on_crosswalk);
      updateParam(
        params, ns + str + ".confidence_filtering.threshold",
        object_parameters_per_label[label].confidence_filtering_threshold);
      updateParam(
        params, ns + str + ".confidence_filtering.only_use_highest",
        object_parameters_per_label[label].confidence_filtering_only_use_highest);
      updateParam(
        params, ns + str + ".cut_predicted_paths.if_crossing_ego_from_behind",
        object_parameters_per_label[label].cut_if_crossing_ego_from_behind);
      updateParam(
        params, ns + str + ".cut_predicted_paths.polygon_types",
        object_parameters_per_label[label].cut_polygon_types);
      updateParam(
        params, ns + str + ".cut_predicted_paths.linestring_types",
        object_parameters_per_label[label].cut_linestring_types);
    }

    max_history_duration = std::max(stop_off_time_buffer, stop_on_time_buffer);
  }
  /// @brief get a string representation of the given classification label
  static std::string label_to_string(
    const autoware_perception_msgs::msg::ObjectClassification::_label_type & label)
  {
    using autoware_perception_msgs::msg::ObjectClassification;
    switch (label) {
      case ObjectClassification::CAR:
        return "CAR";
      case ObjectClassification::TRUCK:
        return "TRUCK";
      case ObjectClassification::BICYCLE:
        return "BICYCLE";
      case ObjectClassification::BUS:
        return "BUS";
      case ObjectClassification::MOTORCYCLE:
        return "MOTORCYCLE";
      case ObjectClassification::PEDESTRIAN:
        return "PEDESTRIAN";
      case ObjectClassification::TRAILER:
        return "TRAILER";
      case ObjectClassification::UNKNOWN:
        return "UNKNOWN";
      default:
        return "DEFAULT";
    }
  }
  /// @brief get all possible classification labels
  static std::vector<uint8_t> all_labels()
  {
    using autoware_perception_msgs::msg::ObjectClassification;
    return {ObjectClassification::CAR,        ObjectClassification::TRUCK,
            ObjectClassification::BICYCLE,    ObjectClassification::BUS,
            ObjectClassification::MOTORCYCLE, ObjectClassification::PEDESTRIAN,
            ObjectClassification::TRAILER,    ObjectClassification::UNKNOWN};
  }
};
}  // namespace autoware::motion_velocity_planner::run_out

#endif  // PARAMETERS_HPP_
