// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <autoware/freespace_planning_algorithms/abstract_algorithm.hpp>
#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
namespace autoware::behavior_path_planner
{

using geometry_msgs::msg::Pose;

using autoware::behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using autoware::behavior_path_planner::utils::path_safety_checker::TargetObjectsOnLane;
using autoware_perception_msgs::msg::PredictedObjects;

using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStarParam;

enum class PlannerType {
  NONE = 0,
  SHIFT = 1,
  GEOMETRIC = 2,
  CLOTHOID = 3,
  STOP = 4,
  FREESPACE = 5,
};

struct PlannerDebugData
{
public:
  PlannerType planner_type;
  double backward_distance{0.0};
  double required_margin{0.0};
  std::vector<std::string> conditions_evaluation;

  static std::string double_to_str(double value, int precision = 1)
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  static std::string to_planner_type_name(PlannerType pt)
  {
    // Adding whitespace for column width alignment in RViz display
    switch (pt) {
      case PlannerType::NONE:
        return "NONE                  ";
      case PlannerType::SHIFT:
        return "SHIFT               ";
      case PlannerType::GEOMETRIC:
        return "GEOMETRIC   ";
      case PlannerType::CLOTHOID:
        return "CLOTHOID    ";
      case PlannerType::STOP:
        return "STOP                  ";
      case PlannerType::FREESPACE:
        return "FREESPACE   ";
      default:
        return "UNKNOWN";
    }
  }
};

/**
 * @brief Structure representing a pose-based arc segment
 */
struct ArcSegment
{
  // Geometric parameters of the arc
  geometry_msgs::msg::Point center;     // Center point of the arc
  double radius;                        // Radius [m]
  geometry_msgs::msg::Pose start_pose;  // Start pose
  geometry_msgs::msg::Pose end_pose;    // End pose
  bool is_clockwise;                    // Whether clockwise or not

  ArcSegment() : radius(0.0), is_clockwise(true) { center.x = center.y = center.z = 0.0; }

  /**
   * @brief Calculate start angle
   * @return Start angle [rad]
   */
  double calculateStartAngle() const
  {
    return std::atan2(start_pose.position.y - center.y, start_pose.position.x - center.x);
  }

  /**
   * @brief Calculate end angle
   * @return End angle [rad]
   */
  double calculateEndAngle() const
  {
    return std::atan2(end_pose.position.y - center.y, end_pose.position.x - center.x);
  }

  /**
   * @brief Calculate arc length
   * @return Arc length [m]
   */
  double calculateArcLength() const
  {
    double start_angle = calculateStartAngle();
    double end_angle = calculateEndAngle();
    double angle_diff = std::abs(end_angle - start_angle);

    // Adjust angle difference if it exceeds 2π
    if (angle_diff > 2.0 * M_PI) {
      angle_diff = 2.0 * M_PI - std::fmod(angle_diff, 2.0 * M_PI);
    }
    return radius * angle_diff;
  }

  /**
   * @brief Get curvature (constant for arc)
   * @return Curvature [1/m]
   */
  double getCurvature() const { return (radius > 0.0) ? (1.0 / radius) : 0.0; }

  /**
   * @brief Calculate position at specified angle
   * @param angle Angle [rad]
   * @return Position
   */
  geometry_msgs::msg::Point getPointAtAngle(double angle) const
  {
    geometry_msgs::msg::Point point;
    point.x = center.x + radius * std::cos(angle);
    point.y = center.y + radius * std::sin(angle);
    point.z = center.z;
    return point;
  }

  /**
   * @brief Get start position
   * @return Start position
   */
  geometry_msgs::msg::Point getStartPoint() const { return start_pose.position; }

  /**
   * @brief Get end position
   * @return End position
   */
  geometry_msgs::msg::Point getEndPoint() const { return end_pose.position; }

  /**
   * @brief Calculate pose at specified angle
   * @param angle Angle [rad]
   * @return Pose
   */
  geometry_msgs::msg::Pose getPoseAtAngle(double angle) const
  {
    geometry_msgs::msg::Pose pose;

    // Calculate position
    pose.position = getPointAtAngle(angle);

    // Calculate tangent direction (arc progression direction)
    double tangent_angle = angle + (is_clockwise ? -M_PI / 2 : M_PI / 2);

    // Set quaternion
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(tangent_angle / 2.0);
    pose.orientation.w = std::cos(tangent_angle / 2.0);

    return pose;
  }

  /**
   * @brief Get start pose
   * @return Start pose
   */
  geometry_msgs::msg::Pose getStartPose() const { return start_pose; }

  /**
   * @brief Get end pose
   * @return End pose
   */
  geometry_msgs::msg::Pose getEndPose() const { return end_pose; }
};

/**
 * @brief Composite arc path consisting of multiple arc segments
 */
struct CompositeArcPath
{
  std::vector<ArcSegment> segments;  // Array of arc segments

  CompositeArcPath() = default;
};

/**
 * @brief Structure to hold relative pose information in vehicle coordinate system
 */
struct RelativePoseInfo
{
  double longitudinal_distance_vehicle;  // Longitudinal distance in vehicle coordinate [m]
  double lateral_distance_vehicle;  // Lateral distance in vehicle coordinate [m] (positive: left,
                                    // negative: right)
  double angle_diff;  // Angle difference [rad] (positive: counter-clockwise/left turn, negative:
                      // clockwise/right turn)
};

/**
 * @brief Clothoid segment structure for smooth path transitions
 */
struct ClothoidSegment
{
  enum Type { CLOTHOID_ENTRY, CIRCULAR_ARC, CLOTHOID_EXIT };

  Type type;
  double A;           // Clothoid parameter
  double L;           // Arc length
  double radius;      // Radius (for circular arc segment)
  double angle;       // Angle (for circular arc segment)
  bool is_clockwise;  // Rotation direction
  std::string description;

  explicit ClothoidSegment(Type t, double a = 0.0, double l = 0.0)
  : type(t), A(a), L(l), radius(0.0), angle(0.0), is_clockwise(true)
  {
  }
};

struct StartPlannerDebugData
{
  // filtered objects
  PredictedObjects filtered_objects;
  TargetObjectsOnLane target_objects_on_lane;
  std::vector<PoseWithVelocityStamped> ego_predicted_path;
  // collision check debug map
  CollisionCheckDebugMap collision_check;
  lanelet::ConstLanelets departure_check_lanes;

  Pose refined_start_pose;
  std::vector<Pose> start_pose_candidates;
  size_t selected_start_pose_candidate_index;
  double margin_for_start_pose_candidate;

  // for isPreventingRearVehicleFromPassingThrough
  std::optional<Pose> estimated_stop_pose;
};

struct StartPlannerParameters
{
  template <typename NodeT>
  static StartPlannerParameters init(NodeT & node);
  double th_arrived_distance{0.0};
  double th_stopped_velocity{0.0};
  double th_stopped_time{0.0};
  double prepare_time_before_start{0.0};
  double th_distance_to_middle_of_the_road{0.0};
  bool skip_rear_vehicle_check{false};
  double extra_width_margin_for_rear_obstacle{0.0};
  std::vector<double> collision_check_margins{};
  double collision_check_margin_from_front_object{0.0};
  double th_moving_object_velocity{0.0};
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck
    object_types_to_check_for_path_generation{};
  double center_line_path_interval{0.0};
  double lane_departure_check_expansion_margin{0.0};

  // shift pull out
  bool check_shift_path_lane_departure{false};
  bool allow_check_shift_path_lane_departure_override{false};
  double shift_collision_check_distance_from_end{0.0};
  double minimum_shift_pull_out_distance{0.0};
  int lateral_acceleration_sampling_num{0};
  double lateral_jerk{0.0};
  double maximum_lateral_acc{0.0};
  double minimum_lateral_acc{0.0};
  double maximum_curvature{0.0};  // maximum curvature considered in the path generation
  double end_pose_curvature_threshold{0.0};
  double maximum_longitudinal_deviation{0.0};
  // geometric pull out
  double geometric_collision_check_distance_from_end{0.0};
  bool divide_pull_out_path{false};
  // Enable clothoid path search when no path is found with collision margins
  ParallelParkingParameters parallel_parking_parameters{};

  // clothoid pull out
  double clothoid_initial_velocity{0.0};
  double clothoid_acceleration{0.0};
  std::vector<double> clothoid_max_steer_angles_deg{};
  double clothoid_max_steer_angle_rate_deg_per_sec{0.0};
  double clothoid_collision_check_distance_from_end{0.0};
  bool check_clothoid_path_lane_departure{true};  // enable lane departure check for clothoid path

  // List of planner types in priority order (e.g., ["SHIFT", "GEOMETRIC", "CLOTHOID"])
  std::vector<std::string> search_priority{};
  // Search policy: "planner_priority" or "distance_priority"
  std::string search_policy{};
  bool enable_back{false};
  double backward_velocity{0.0};
  double max_back_distance{0.0};  // max backward distance to search start pose
  double backward_search_resolution{0.0};
  double backward_path_update_duration{0.0};
  double ignore_distance_from_lane_end{0.0};
  // freespace planner
  bool enable_freespace_planner{false};
  std::string freespace_planner_algorithm;
  double end_pose_search_start_distance{0.0};
  double end_pose_search_end_distance{0.0};
  double end_pose_search_interval{0.0};
  double freespace_planner_velocity{0.0};
  double vehicle_shape_margin{0.0};
  PlannerCommonParam freespace_planner_common_parameters;
  AstarParam astar_parameters;
  RRTStarParam rrt_star_parameters;

  // stop condition
  double maximum_deceleration_for_stop{0.0};
  double maximum_jerk_for_stop{0.0};

  // hysteresis parameters
  double hysteresis_factor_expand_rate{0.0};

  // path safety checker
  utils::path_safety_checker::EgoPredictedPathParams ego_predicted_path_params{};
  utils::path_safety_checker::ObjectsFilteringParams objects_filtering_params{};
  utils::path_safety_checker::SafetyCheckParams safety_check_params{};

  // surround moving obstacle check
  double search_radius{0.0};
  double th_moving_obstacle_velocity{0.0};
  autoware::behavior_path_planner::utils::path_safety_checker::ObjectTypesToCheck
    surround_moving_obstacles_type_to_check{};

  bool print_debug_info{false};
};

template <typename NodeT>
StartPlannerParameters StartPlannerParameters::init(NodeT & node)
{
  using autoware_utils::get_or_declare_parameter;
  StartPlannerParameters p;
  {
    const std::string ns = "start_planner.";

    p.th_arrived_distance = get_or_declare_parameter<double>(node, ns + "th_arrived_distance");
    p.th_stopped_velocity = get_or_declare_parameter<double>(node, ns + "th_stopped_velocity");
    p.th_stopped_time = get_or_declare_parameter<double>(node, ns + "th_stopped_time");
    p.prepare_time_before_start =
      get_or_declare_parameter<double>(node, ns + "prepare_time_before_start");
    p.th_distance_to_middle_of_the_road =
      get_or_declare_parameter<double>(node, ns + "th_distance_to_middle_of_the_road");
    p.skip_rear_vehicle_check =
      get_or_declare_parameter<bool>(node, ns + "skip_rear_vehicle_check");
    p.extra_width_margin_for_rear_obstacle =
      get_or_declare_parameter<double>(node, ns + "extra_width_margin_for_rear_obstacle");
    p.collision_check_margins =
      get_or_declare_parameter<std::vector<double>>(node, ns + "collision_check_margins");
    p.collision_check_margin_from_front_object =
      get_or_declare_parameter<double>(node, ns + "collision_check_margin_from_front_object");
    p.th_moving_object_velocity =
      get_or_declare_parameter<double>(node, ns + "th_moving_object_velocity");
    p.center_line_path_interval =
      get_or_declare_parameter<double>(node, ns + "center_line_path_interval");
    // shift pull out
    p.check_shift_path_lane_departure =
      get_or_declare_parameter<bool>(node, ns + "check_shift_path_lane_departure");
    p.allow_check_shift_path_lane_departure_override =
      get_or_declare_parameter<bool>(node, ns + "allow_check_shift_path_lane_departure_override");
    p.shift_collision_check_distance_from_end =
      get_or_declare_parameter<double>(node, ns + "shift_collision_check_distance_from_end");
    p.minimum_shift_pull_out_distance =
      get_or_declare_parameter<double>(node, ns + "minimum_shift_pull_out_distance");
    p.lateral_acceleration_sampling_num =
      get_or_declare_parameter<int>(node, ns + "lateral_acceleration_sampling_num");
    p.lateral_jerk = get_or_declare_parameter<double>(node, ns + "lateral_jerk");
    p.maximum_lateral_acc = get_or_declare_parameter<double>(node, ns + "maximum_lateral_acc");
    p.minimum_lateral_acc = get_or_declare_parameter<double>(node, ns + "minimum_lateral_acc");
    p.maximum_curvature = get_or_declare_parameter<double>(node, ns + "maximum_curvature");
    p.end_pose_curvature_threshold =
      get_or_declare_parameter<double>(node, ns + "end_pose_curvature_threshold");
    p.maximum_longitudinal_deviation =
      get_or_declare_parameter<double>(node, ns + "maximum_longitudinal_deviation");
    // geometric pull out
    p.geometric_collision_check_distance_from_end =
      get_or_declare_parameter<double>(node, ns + "geometric_collision_check_distance_from_end");
    p.divide_pull_out_path = get_or_declare_parameter<bool>(node, ns + "divide_pull_out_path");
    p.parallel_parking_parameters.pull_out_velocity =
      get_or_declare_parameter<double>(node, ns + "geometric_pull_out_velocity");
    p.parallel_parking_parameters.pull_out_arc_path_interval =
      get_or_declare_parameter<double>(node, ns + "arc_path_interval");
    p.parallel_parking_parameters.pull_out_lane_departure_margin =
      get_or_declare_parameter<double>(node, ns + "lane_departure_margin");
    p.lane_departure_check_expansion_margin =
      get_or_declare_parameter<double>(node, ns + "lane_departure_check_expansion_margin");
    p.parallel_parking_parameters.geometric_pull_out_max_steer_angle_margin_scale =
      get_or_declare_parameter<double>(
        node, ns + "geometric_pull_out_max_steer_angle_margin_scale");
    p.parallel_parking_parameters.center_line_path_interval =
      p.center_line_path_interval;  // for geometric parallel parking

    // clothoid pull out
    p.clothoid_initial_velocity =
      get_or_declare_parameter<double>(node, ns + "clothoid_initial_velocity");
    p.clothoid_acceleration = get_or_declare_parameter<double>(node, ns + "clothoid_acceleration");
    p.clothoid_max_steer_angles_deg =
      get_or_declare_parameter<std::vector<double>>(node, ns + "clothoid_max_steer_angles_deg");
    p.clothoid_max_steer_angle_rate_deg_per_sec =
      get_or_declare_parameter<double>(node, ns + "clothoid_max_steer_angle_rate_deg_per_sec");
    p.clothoid_collision_check_distance_from_end =
      get_or_declare_parameter<double>(node, ns + "clothoid_collision_check_distance_from_end");
    p.check_clothoid_path_lane_departure =
      get_or_declare_parameter<bool>(node, ns + "check_clothoid_path_lane_departure");

    // search start pose backward
    p.search_priority =
      get_or_declare_parameter<std::vector<std::string>>(node, ns + "search_priority");
    if (p.search_priority.empty()) {
      RCLCPP_ERROR(node.get_logger(), "search_priority cannot be empty");
      throw std::runtime_error("search_priority cannot be empty");
    }

    // Validate each planner type in search_priority
    for (const auto & planner_type_str : p.search_priority) {
      const auto planner_type =
        magic_enum::enum_cast<autoware::behavior_path_planner::PlannerType>(planner_type_str);
      if (
        !planner_type.has_value() ||
        planner_type.value() == autoware::behavior_path_planner::PlannerType::NONE) {
        RCLCPP_ERROR(
          node.get_logger(), "Invalid planner type in search_priority: %s",
          planner_type_str.c_str());
        throw std::runtime_error("Invalid planner type in search_priority: " + planner_type_str);
      }
    }

    p.search_policy = get_or_declare_parameter<std::string>(node, ns + "search_policy");
    if (p.search_policy != "distance_priority" && p.search_policy != "planner_priority") {
      RCLCPP_ERROR(node.get_logger(), "Invalid search_policy: %s", p.search_policy.c_str());
      throw std::runtime_error("Invalid search_policy: " + p.search_policy);
    }
    p.enable_back = get_or_declare_parameter<bool>(node, ns + "enable_back");
    p.backward_velocity = get_or_declare_parameter<double>(node, ns + "backward_velocity");
    p.max_back_distance = get_or_declare_parameter<double>(node, ns + "max_back_distance");
    p.backward_search_resolution =
      get_or_declare_parameter<double>(node, ns + "backward_search_resolution");
    p.backward_path_update_duration =
      get_or_declare_parameter<double>(node, ns + "backward_path_update_duration");
    p.ignore_distance_from_lane_end =
      get_or_declare_parameter<double>(node, ns + "ignore_distance_from_lane_end");
    // stop condition
    p.maximum_deceleration_for_stop =
      get_or_declare_parameter<double>(node, ns + "stop_condition.maximum_deceleration_for_stop");
    p.maximum_jerk_for_stop =
      get_or_declare_parameter<double>(node, ns + "stop_condition.maximum_jerk_for_stop");
  }
  {
    const std::string ns = "start_planner.object_types_to_check_for_path_generation.";
    p.object_types_to_check_for_path_generation.check_car =
      get_or_declare_parameter<bool>(node, ns + "check_car");
    p.object_types_to_check_for_path_generation.check_truck =
      get_or_declare_parameter<bool>(node, ns + "check_truck");
    p.object_types_to_check_for_path_generation.check_bus =
      get_or_declare_parameter<bool>(node, ns + "check_bus");
    p.object_types_to_check_for_path_generation.check_trailer =
      get_or_declare_parameter<bool>(node, ns + "check_trailer");
    p.object_types_to_check_for_path_generation.check_unknown =
      get_or_declare_parameter<bool>(node, ns + "check_unknown");
    p.object_types_to_check_for_path_generation.check_bicycle =
      get_or_declare_parameter<bool>(node, ns + "check_bicycle");
    p.object_types_to_check_for_path_generation.check_motorcycle =
      get_or_declare_parameter<bool>(node, ns + "check_motorcycle");
    p.object_types_to_check_for_path_generation.check_pedestrian =
      get_or_declare_parameter<bool>(node, ns + "check_pedestrian");
  }
  // freespace planner general params
  {
    const std::string ns = "start_planner.freespace_planner.";
    p.enable_freespace_planner =
      get_or_declare_parameter<bool>(node, ns + "enable_freespace_planner");
    p.freespace_planner_algorithm =
      get_or_declare_parameter<std::string>(node, ns + "freespace_planner_algorithm");
    p.end_pose_search_start_distance =
      get_or_declare_parameter<double>(node, ns + "end_pose_search_start_distance");
    p.end_pose_search_end_distance =
      get_or_declare_parameter<double>(node, ns + "end_pose_search_end_distance");
    p.end_pose_search_interval =
      get_or_declare_parameter<double>(node, ns + "end_pose_search_interval");
    p.freespace_planner_velocity = get_or_declare_parameter<double>(node, ns + "velocity");
    p.vehicle_shape_margin = get_or_declare_parameter<double>(node, ns + "vehicle_shape_margin");
    p.freespace_planner_common_parameters.time_limit =
      get_or_declare_parameter<double>(node, ns + "time_limit");
    p.freespace_planner_common_parameters.max_turning_ratio =
      get_or_declare_parameter<double>(node, ns + "max_turning_ratio");
    p.freespace_planner_common_parameters.turning_steps =
      get_or_declare_parameter<int>(node, ns + "turning_steps");
  }
  //  freespace planner search config
  {
    const std::string ns = "start_planner.freespace_planner.search_configs.";
    p.freespace_planner_common_parameters.theta_size =
      get_or_declare_parameter<int>(node, ns + "theta_size");
    p.freespace_planner_common_parameters.angle_goal_range =
      get_or_declare_parameter<double>(node, ns + "angle_goal_range");
    p.freespace_planner_common_parameters.curve_weight =
      get_or_declare_parameter<double>(node, ns + "curve_weight");
    p.freespace_planner_common_parameters.reverse_weight =
      get_or_declare_parameter<double>(node, ns + "reverse_weight");
    p.freespace_planner_common_parameters.lateral_goal_range =
      get_or_declare_parameter<double>(node, ns + "lateral_goal_range");
    p.freespace_planner_common_parameters.longitudinal_goal_range =
      get_or_declare_parameter<double>(node, ns + "longitudinal_goal_range");
  }
  //  freespace planner costmap configs
  {
    const std::string ns = "start_planner.freespace_planner.costmap_configs.";
    p.freespace_planner_common_parameters.obstacle_threshold =
      get_or_declare_parameter<int>(node, ns + "obstacle_threshold");
  }
  //  freespace planner astar
  {
    const std::string ns = "start_planner.freespace_planner.astar.";
    p.astar_parameters.search_method =
      get_or_declare_parameter<std::string>(node, ns + "search_method");
    p.astar_parameters.only_behind_solutions =
      get_or_declare_parameter<bool>(node, ns + "only_behind_solutions");
    p.astar_parameters.use_back = get_or_declare_parameter<bool>(node, ns + "use_back");
    p.astar_parameters.distance_heuristic_weight =
      get_or_declare_parameter<double>(node, ns + "distance_heuristic_weight");
  }
  //   freespace planner rrtstar
  {
    const std::string ns = "start_planner.freespace_planner.rrtstar.";
    p.rrt_star_parameters.enable_update =
      get_or_declare_parameter<bool>(node, ns + "enable_update");
    p.rrt_star_parameters.use_informed_sampling =
      get_or_declare_parameter<bool>(node, ns + "use_informed_sampling");
    p.rrt_star_parameters.max_planning_time =
      get_or_declare_parameter<double>(node, ns + "max_planning_time");
    p.rrt_star_parameters.neighbor_radius =
      get_or_declare_parameter<double>(node, ns + "neighbor_radius");
    p.rrt_star_parameters.margin = get_or_declare_parameter<double>(node, ns + "margin");
  }

  const std::string base_ns = "start_planner.path_safety_check.";
  // EgoPredictedPath
  {
    const std::string ego_path_ns = base_ns + "ego_predicted_path.";
    p.ego_predicted_path_params.min_velocity =
      get_or_declare_parameter<double>(node, ego_path_ns + "min_velocity");
    p.ego_predicted_path_params.acceleration =
      get_or_declare_parameter<double>(node, ego_path_ns + "min_acceleration");
    p.ego_predicted_path_params.time_horizon_for_front_object =
      get_or_declare_parameter<double>(node, ego_path_ns + "time_horizon_for_front_object");
    p.ego_predicted_path_params.time_horizon_for_rear_object =
      get_or_declare_parameter<double>(node, ego_path_ns + "time_horizon_for_rear_object");
    p.ego_predicted_path_params.time_resolution =
      get_or_declare_parameter<double>(node, ego_path_ns + "time_resolution");
    p.ego_predicted_path_params.delay_until_departure =
      get_or_declare_parameter<double>(node, ego_path_ns + "delay_until_departure");
  }
  // ObjectFilteringParams
  const std::string obj_filter_ns = base_ns + "target_filtering.";
  {
    p.objects_filtering_params.safety_check_time_horizon =
      get_or_declare_parameter<double>(node, obj_filter_ns + "safety_check_time_horizon");
    p.objects_filtering_params.safety_check_time_resolution =
      get_or_declare_parameter<double>(node, obj_filter_ns + "safety_check_time_resolution");
    p.objects_filtering_params.object_check_forward_distance =
      get_or_declare_parameter<double>(node, obj_filter_ns + "object_check_forward_distance");
    p.objects_filtering_params.object_check_backward_distance =
      get_or_declare_parameter<double>(node, obj_filter_ns + "object_check_backward_distance");
    p.objects_filtering_params.ignore_object_velocity_threshold =
      get_or_declare_parameter<double>(node, obj_filter_ns + "ignore_object_velocity_threshold");
    p.objects_filtering_params.include_opposite_lane =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "include_opposite_lane");
    p.objects_filtering_params.invert_opposite_lane =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "invert_opposite_lane");
    p.objects_filtering_params.check_all_predicted_path =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "check_all_predicted_path");
    p.objects_filtering_params.use_all_predicted_path =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "use_all_predicted_path");
    p.objects_filtering_params.use_predicted_path_outside_lanelet =
      get_or_declare_parameter<bool>(node, obj_filter_ns + "use_predicted_path_outside_lanelet");
  }
  // ObjectTypesToCheck
  {
    const std::string obj_types_ns = obj_filter_ns + "object_types_to_check.";
    p.objects_filtering_params.object_types_to_check.check_car =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_car");
    p.objects_filtering_params.object_types_to_check.check_truck =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_truck");
    p.objects_filtering_params.object_types_to_check.check_bus =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_bus");
    p.objects_filtering_params.object_types_to_check.check_trailer =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_trailer");
    p.objects_filtering_params.object_types_to_check.check_unknown =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_unknown");
    p.objects_filtering_params.object_types_to_check.check_bicycle =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_bicycle");
    p.objects_filtering_params.object_types_to_check.check_motorcycle =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_motorcycle");
    p.objects_filtering_params.object_types_to_check.check_pedestrian =
      get_or_declare_parameter<bool>(node, obj_types_ns + "check_pedestrian");
  }
  // ObjectLaneConfiguration
  {
    const std::string obj_lane_ns = obj_filter_ns + "object_lane_configuration.";
    p.objects_filtering_params.object_lane_configuration.check_current_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_current_lane");
    p.objects_filtering_params.object_lane_configuration.check_right_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_right_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_left_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_left_side_lane");
    p.objects_filtering_params.object_lane_configuration.check_shoulder_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_shoulder_lane");
    p.objects_filtering_params.object_lane_configuration.check_other_lane =
      get_or_declare_parameter<bool>(node, obj_lane_ns + "check_other_lane");
  }
  // SafetyCheckParams
  const std::string safety_check_ns = base_ns + "safety_check_params.";
  {
    p.safety_check_params.enable_safety_check =
      get_or_declare_parameter<bool>(node, safety_check_ns + "enable_safety_check");
    p.safety_check_params.hysteresis_factor_expand_rate =
      get_or_declare_parameter<double>(node, safety_check_ns + "hysteresis_factor_expand_rate");
    p.safety_check_params.backward_path_length =
      get_or_declare_parameter<double>(node, safety_check_ns + "backward_path_length");
    p.safety_check_params.forward_path_length =
      get_or_declare_parameter<double>(node, safety_check_ns + "forward_path_length");
    p.safety_check_params.publish_debug_marker =
      get_or_declare_parameter<bool>(node, safety_check_ns + "publish_debug_marker");
    p.safety_check_params.collision_check_yaw_diff_threshold = get_or_declare_parameter<double>(
      node, safety_check_ns + "collision_check_yaw_diff_threshold");
  }
  // RSSparams
  {
    const std::string rss_ns = safety_check_ns + "rss_params.";
    p.safety_check_params.rss_params.rear_vehicle_reaction_time =
      get_or_declare_parameter<double>(node, rss_ns + "rear_vehicle_reaction_time");
    p.safety_check_params.rss_params.rear_vehicle_safety_time_margin =
      get_or_declare_parameter<double>(node, rss_ns + "rear_vehicle_safety_time_margin");
    p.safety_check_params.rss_params.lateral_distance_max_threshold =
      get_or_declare_parameter<double>(node, rss_ns + "lateral_distance_max_threshold");
    p.safety_check_params.rss_params.longitudinal_distance_min_threshold =
      get_or_declare_parameter<double>(node, rss_ns + "longitudinal_distance_min_threshold");
    p.safety_check_params.rss_params.longitudinal_velocity_delta_time =
      get_or_declare_parameter<double>(node, rss_ns + "longitudinal_velocity_delta_time");
    p.safety_check_params.rss_params.extended_polygon_policy =
      get_or_declare_parameter<std::string>(node, rss_ns + "extended_polygon_policy");
  }
  // surround moving obstacle check
  {
    const std::string surround_moving_obstacle_check_ns =
      "start_planner.surround_moving_obstacle_check.";
    p.search_radius =
      get_or_declare_parameter<double>(node, surround_moving_obstacle_check_ns + "search_radius");
    p.th_moving_obstacle_velocity = get_or_declare_parameter<double>(
      node, surround_moving_obstacle_check_ns + "th_moving_obstacle_velocity");
    // ObjectTypesToCheck
    {
      const std::string obj_types_ns =
        surround_moving_obstacle_check_ns + "object_types_to_check.";
      p.surround_moving_obstacles_type_to_check.check_car =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_car");
      p.surround_moving_obstacles_type_to_check.check_truck =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_truck");
      p.surround_moving_obstacles_type_to_check.check_bus =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_bus");
      p.surround_moving_obstacles_type_to_check.check_trailer =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_trailer");
      p.surround_moving_obstacles_type_to_check.check_unknown =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_unknown");
      p.surround_moving_obstacles_type_to_check.check_bicycle =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_bicycle");
      p.surround_moving_obstacles_type_to_check.check_motorcycle =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_motorcycle");
      p.surround_moving_obstacles_type_to_check.check_pedestrian =
        get_or_declare_parameter<bool>(node, obj_types_ns + "check_pedestrian");
    }
  }

  // debug
  {
    const std::string debug_ns = "start_planner.debug.";
    p.print_debug_info = get_or_declare_parameter<bool>(node, debug_ns + "print_debug_info");
  }

  return p;
}

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_START_PLANNER_MODULE__DATA_STRUCTS_HPP_
