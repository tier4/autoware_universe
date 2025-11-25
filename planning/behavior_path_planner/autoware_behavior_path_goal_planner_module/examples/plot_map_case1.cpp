// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_goal_planner_module/goal_searcher.hpp"
#include "autoware/behavior_path_goal_planner_module/util.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/behavior_path_goal_planner_module/manager.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_planner/bezier_pull_over.hpp>
#include <autoware/behavior_path_goal_planner_module/pull_over_planner/shift_pull_over.hpp>
#include <autoware/behavior_path_planner/behavior_path_planner_node.hpp>
#include <autoware/behavior_path_planner_common/data_manager.hpp>
#include <autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp>
#include <autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp>
#include <autoware/behavior_path_planner_common/utils/path_utils.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_lanelet2_extension/io/autoware_osm_parser.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_test_utils/mock_data_parser.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <lanelet2_io/Io.h>
#include <matplotlibcpp17/pyplot.h>
#include <pybind11/pytypes.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;  // NOLINT

using autoware::behavior_path_planner::BehaviorModuleOutput;
using autoware::behavior_path_planner::GoalCandidate;
using autoware::behavior_path_planner::GoalCandidates;
using autoware::behavior_path_planner::GoalPlannerParameters;
using autoware::behavior_path_planner::PlannerData;
using autoware::behavior_path_planner::PullOverPath;
using autoware::behavior_path_planner::utils::parking_departure::calcFeasibleDecelDistance;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_planning_msgs::msg::LaneletRoute;

std::vector<std::string> g_colors = {
  "#F0F8FF", "#FAEBD7", "#00FFFF", "#7FFFD4", "#F0FFFF", "#F5F5DC", "#FFE4C4", "#000000", "#FFEBCD",
  "#0000FF", "#8A2BE2", "#A52A2A", "#DEB887", "#5F9EA0", "#7FFF00", "#D2691E", "#FF7F50", "#6495ED",
  "#FFF8DC", "#DC143C", "#00FFFF", "#00008B", "#008B8B", "#B8860B", "#A9A9A9", "#006400", "#A9A9A9",
  "#BDB76B", "#8B008B", "#556B2F", "#FF8C00", "#9932CC", "#8B0000", "#E9967A", "#8FBC8F", "#483D8B",
  "#2F4F4F", "#2F4F4F", "#00CED1", "#9400D3", "#FF1493", "#00BFFF", "#696969", "#696969", "#1E90FF",
  "#B22222", "#FFFAF0", "#228B22", "#FF00FF", "#DCDCDC", "#F8F8FF", "#FFD700", "#DAA520", "#808080",
  "#008000", "#ADFF2F", "#808080", "#F0FFF0", "#FF69B4", "#CD5C5C", "#4B0082", "#FFFFF0", "#F0E68C",
  "#E6E6FA", "#FFF0F5", "#7CFC00", "#FFFACD", "#ADD8E6", "#F08080", "#E0FFFF", "#FAFAD2", "#D3D3D3",
  "#90EE90", "#D3D3D3", "#FFB6C1", "#FFA07A", "#20B2AA", "#87CEFA", "#778899", "#778899", "#B0C4DE",
  "#FFFFE0", "#00FF00", "#32CD32", "#FAF0E6", "#FF00FF", "#800000", "#66CDAA", "#0000CD", "#BA55D3",
  "#9370DB", "#3CB371", "#7B68EE", "#00FA9A", "#48D1CC", "#C71585", "#191970", "#F5FFFA", "#FFE4E1",
  "#FFE4B5", "#FFDEAD", "#000080", "#FDF5E6", "#808000", "#6B8E23", "#FFA500", "#FF4500", "#DA70D6",
  "#EEE8AA", "#98FB98", "#AFEEEE", "#DB7093", "#FFEFD5", "#FFDAB9", "#CD853F", "#FFC0CB", "#DDA0DD",
  "#B0E0E6", "#800080", "#663399", "#FF0000", "#BC8F8F", "#4169E1", "#8B4513", "#FA8072", "#F4A460",
  "#2E8B57", "#FFF5EE", "#A0522D", "#C0C0C0", "#87CEEB", "#6A5ACD", "#708090", "#708090", "#FFFAFA",
  "#00FF7F", "#4682B4", "#D2B48C", "#008080", "#D8BFD8", "#FF6347", "#40E0D0", "#EE82EE", "#F5DEB3",
  "#FFFFFF", "#F5F5F5", "#FFFF00", "#9ACD32"};

void plot_footprint(
  matplotlibcpp17::axes::Axes & axes, const autoware_utils::LinearRing2d & footprint,
  const std::string & color)
{
  std::vector<double> xs, ys;
  for (const auto & pt : footprint) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  xs.push_back(xs.front());
  ys.push_back(ys.front());
  axes.plot(Args(xs, ys), Kwargs("color"_a = color, "linestyle"_a = "dotted"));
}

void plot_lanelet_polygon(matplotlibcpp17::axes::Axes & axes, const lanelet::BasicPolygon2d polygon)
{
  std::vector<double> xs, ys;
  for (const auto & p : polygon) {
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  xs.push_back(xs.front());
  ys.push_back(ys.front());
  axes.fill(Args(xs, ys), Kwargs("color"_a = "grey", "alpha"_a = 0.5));
}

void plot_goal_candidate(
  matplotlibcpp17::axes::Axes & axes, const GoalCandidate & goal, const size_t prio,
  const autoware_utils::LinearRing2d & local_footprint, const std::string & color)
{
  std::vector<double> xs, ys;
  std::vector<double> yaw_cos, yaw_sin;
  const auto goal_footprint = autoware_utils_geometry::transform_vector(
    local_footprint, autoware_utils::pose2transform(goal.goal_pose));
  plot_footprint(axes, goal_footprint, color);
  xs.push_back(goal.goal_pose.position.x);
  ys.push_back(goal.goal_pose.position.y);
  axes.text(Args(xs.back(), ys.back(), std::to_string(prio)));
  const double yaw = autoware_utils::get_rpy(goal.goal_pose).z;
  yaw_cos.push_back(std::cos(yaw));
  yaw_sin.push_back(std::sin(yaw));
  axes.scatter(Args(xs, ys), Kwargs("color"_a = color));
  axes.quiver(
    Args(xs, ys, yaw_cos, yaw_sin),
    Kwargs("angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 2.0));
}

void plot_goal_candidates(
  matplotlibcpp17::axes::Axes & axes, const GoalCandidates & goals,
  const std::map<size_t, size_t> & goal_id2prio,
  const autoware_utils::LinearRing2d & local_footprint, const std::string & color = "green")
{
  for (const auto & goal : goals) {
    const auto it = goal_id2prio.find(goal.id);
    if (it != goal_id2prio.end()) {
      plot_goal_candidate(axes, goal, it->second, local_footprint, color);
    }
  }
}

void plot_path_with_lane_id(
  matplotlibcpp17::axes::Axes & axes, const PathWithLaneId & path,
  const std::string & color = "red", const std::string & label = "", const double linewidth = 1.0)
{
  std::vector<double> xs, ys;
  std::vector<double> yaw_cos, yaw_sin;
  for (const auto & point : path.points) {
    xs.push_back(point.point.pose.position.x);
    ys.push_back(point.point.pose.position.y);
    const double yaw = autoware_utils::get_rpy(point.point.pose).z;
    yaw_cos.push_back(std::cos(yaw));
    yaw_sin.push_back(std::sin(yaw));
    axes.scatter(
      Args(xs.back(), ys.back()), Kwargs("marker"_a = "o", "color"_a = "blue", "s"_a = 10));
  }
  axes.quiver(
    Args(xs, ys, yaw_cos, yaw_sin),
    Kwargs("angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 2.0));

  if (label == "") {
    axes.plot(Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  } else {
    axes.plot(
      Args(xs, ys), Kwargs("color"_a = color, "linewidth"_a = linewidth, "label"_a = label));
  }
}

void plot_lanelet(
  matplotlibcpp17::axes::Axes & axes, lanelet::ConstLanelet lanelet,
  const std::string & color = "blue", const double linewidth = 0.5)
{
  const auto lefts = lanelet.leftBound();
  const auto rights = lanelet.rightBound();
  std::vector<double> xs_left, ys_left;
  for (const auto & point : lefts) {
    xs_left.push_back(point.x());
    ys_left.push_back(point.y());
  }

  std::vector<double> xs_right, ys_right;
  for (const auto & point : rights) {
    xs_right.push_back(point.x());
    ys_right.push_back(point.y());
  }

  std::vector<double> xs_center, ys_center;
  for (const auto & point : lanelet.centerline()) {
    xs_center.push_back(point.x());
    ys_center.push_back(point.y());
  }

  axes.plot(Args(xs_left, ys_left), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  axes.plot(Args(xs_right, ys_right), Kwargs("color"_a = color, "linewidth"_a = linewidth));
  axes.plot(
    Args(xs_center, ys_center),
    Kwargs("color"_a = "black", "linewidth"_a = linewidth, "linestyle"_a = "dashed"));
}

std::shared_ptr<PlannerData> instantiate_planner_data(
  rclcpp::Node::SharedPtr node, const std::string & map_path,
  const std::string & sample_planner_data_yaml_path)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector{};
  const lanelet::LaneletMapPtr lanelet_map_ptr = lanelet::load(map_path, projector, &errors);
  if (!errors.empty()) {
    for (const auto & error : errors) {
      std::cout << error << std::endl;
    }
    return nullptr;
  }
  autoware_map_msgs::msg::LaneletMapBin map_bin;
  lanelet::utils::conversion::toBinMsg(
    lanelet_map_ptr, &map_bin);  // TODO(soblin): pass lanelet_map_ptr to RouteHandler

  YAML::Node config = YAML::LoadFile(sample_planner_data_yaml_path);

  auto planner_data = std::make_shared<PlannerData>();
  planner_data->init_parameters(*node);
  planner_data->route_handler->setMap(map_bin);
  planner_data->route_handler->setRoute(
    autoware::test_utils::parse<autoware_planning_msgs::msg::LaneletRoute>(config["route"]));

  auto odometry = autoware::test_utils::create_const_shared_ptr(
    autoware::test_utils::parse<nav_msgs::msg::Odometry>(config["self_odometry"]));
  planner_data->self_odometry = odometry;

  auto accel_ptr = autoware::test_utils::create_const_shared_ptr(
    autoware::test_utils::parse<geometry_msgs::msg::AccelWithCovarianceStamped>(
      config["self_acceleration"]));
  planner_data->self_acceleration = accel_ptr;
  return planner_data;
}

bool hasEnoughDistance(
  const PullOverPath & pull_over_path, const PathWithLaneId & long_tail_reference_path,
  const std::shared_ptr<const PlannerData> planner_data, const GoalPlannerParameters & parameters)
{
  using autoware::motion_utils::calcSignedArcLength;

  const Pose & current_pose = planner_data->self_odometry->pose.pose;
  const double current_vel = planner_data->self_odometry->twist.twist.linear.x;

  // when the path is separated and start_pose is close,
  // once stopped, the vehicle cannot start again.
  // so need enough distance to restart.
  // distance to restart should be less than decide_path_distance.
  // otherwise, the goal would change immediately after departure.
  const bool is_separated_path = pull_over_path.partial_paths().size() > 1;
  const double distance_to_start = calcSignedArcLength(
    long_tail_reference_path.points, current_pose.position, pull_over_path.start_pose().position);
  const double distance_to_restart = parameters.decide_path_distance / 2;
  const double eps_vel = 0.01;
  const bool is_stopped = std::abs(current_vel) < eps_vel;
  if (is_separated_path && is_stopped && distance_to_start < distance_to_restart) {
    return false;
  }

  const auto current_to_stop_distance = calcFeasibleDecelDistance(
    planner_data, parameters.maximum_deceleration, parameters.maximum_jerk, 0.0);
  if (!current_to_stop_distance) {
    return false;
  }

  /*
  // If the stop line is subtly exceeded, it is assumed that there is not enough distance to the
  // starting point of parking, so to prevent this, once the vehicle has stopped, it also has a
  // stop_distance_buffer to allow for the amount exceeded.
  const double buffer = is_stopped ? stop_distance_buffer_ : 0.0;
  if (distance_to_start + buffer < *current_to_stop_distance) {
    return false;
    }*/

  return true;
}

std::vector<PullOverPath> selectPullOverPaths(
  const std::vector<PullOverPath> & pull_over_path_candidates,
  const GoalCandidates & goal_candidates, const std::shared_ptr<const PlannerData> planner_data,
  const GoalPlannerParameters & parameters, const BehaviorModuleOutput & upstream_module_output,
  const size_t max_paths = SIZE_MAX)
{
  using autoware::behavior_path_planner::utils::getExtendedCurrentLanesFromPath;
  using autoware::motion_utils::calcSignedArcLength;

  const auto & goal_pose = planner_data->route_handler->getOriginalGoalPose();
  const double backward_length =
    parameters.backward_goal_search_length + parameters.decide_path_distance;

  std::vector<size_t> sorted_path_indices;
  sorted_path_indices.reserve(pull_over_path_candidates.size());

  std::unordered_map<int, GoalCandidate> goal_candidate_map;
  for (const auto & goal_candidate : goal_candidates) {
    goal_candidate_map[goal_candidate.id] = goal_candidate;
  }
  for (size_t i = 0; i < pull_over_path_candidates.size(); ++i) {
    const auto & path = pull_over_path_candidates[i];
    const auto goal_candidate_it = goal_candidate_map.find(path.goal_id());
    if (goal_candidate_it != goal_candidate_map.end() && goal_candidate_it->second.is_safe) {
      sorted_path_indices.push_back(i);
    }
  }

  const double prev_path_front_to_goal_dist = calcSignedArcLength(
    upstream_module_output.path.points,
    upstream_module_output.path.points.front().point.pose.position, goal_pose.position);
  const auto & long_tail_reference_path = [&]() {
    if (prev_path_front_to_goal_dist > backward_length) {
      return upstream_module_output.path;
    }
    // get road lanes which is at least backward_length[m] behind the goal
    const auto road_lanes = getExtendedCurrentLanesFromPath(
      upstream_module_output.path, planner_data, backward_length, 0.0, false);
    const auto goal_pose_length = lanelet::utils::getArcCoordinates(road_lanes, goal_pose).length;
    return planner_data->route_handler->getCenterLinePath(
      road_lanes, std::max(0.0, goal_pose_length - backward_length),
      goal_pose_length + parameters.forward_goal_search_length);
  }();

  sorted_path_indices.erase(
    std::remove_if(
      sorted_path_indices.begin(), sorted_path_indices.end(),
      [&](const size_t i) {
        return !hasEnoughDistance(
          pull_over_path_candidates[i], long_tail_reference_path, planner_data, parameters);
      }),
    sorted_path_indices.end());

  const auto & soft_margins = parameters.object_recognition_collision_check_soft_margins;
  const auto & hard_margins = parameters.object_recognition_collision_check_hard_margins;

  const auto [margins, margins_with_zero] =
    std::invoke([&]() -> std::tuple<std::vector<double>, std::vector<double>> {
      std::vector<double> margins = soft_margins;
      margins.insert(margins.end(), hard_margins.begin(), hard_margins.end());
      std::vector<double> margins_with_zero = margins;
      margins_with_zero.push_back(0.0);
      return std::make_tuple(margins, margins_with_zero);
    });

  // Create a map of PullOverPath pointer to largest collision check margin
  std::map<size_t, double> path_id_to_rough_margin_map;
  const auto & target_objects = autoware_perception_msgs::msg::PredictedObjects{};
  for (const size_t i : sorted_path_indices) {
    const auto & path = pull_over_path_candidates[i];  // cppcheck-suppress containerOutOfBounds
    const double distance =
      autoware::behavior_path_planner::utils::path_safety_checker::calculateRoughDistanceToObjects(
        path.parking_path(), target_objects, planner_data->parameters, false, "max");
    auto it = std::lower_bound(
      margins_with_zero.begin(), margins_with_zero.end(), distance, std::greater<double>());
    if (it == margins_with_zero.end()) {
      path_id_to_rough_margin_map[path.id()] = margins_with_zero.back();
    } else {
      path_id_to_rough_margin_map[path.id()] = *it;
    }
  }

  // sorts in descending order so the item with larger margin comes first
  std::stable_sort(
    sorted_path_indices.begin(), sorted_path_indices.end(),
    [&](const size_t a_i, const size_t b_i) {
      const auto & a = pull_over_path_candidates[a_i];
      const auto & b = pull_over_path_candidates[b_i];
      if (
        std::abs(path_id_to_rough_margin_map[a.id()] - path_id_to_rough_margin_map[b.id()]) <
        0.01) {
        return false;
      }
      return path_id_to_rough_margin_map[a.id()] > path_id_to_rough_margin_map[b.id()];
    });

  // STEP2-3: Sort by curvature
  // If the curvature is less than the threshold, prioritize the path.
  const auto isHighCurvature = [&](const PullOverPath & path) -> bool {
    return path.parking_path_max_curvature() >= parameters.high_curvature_threshold;
  };

  const auto isSoftMargin = [&](const PullOverPath & path) -> bool {
    const double margin = path_id_to_rough_margin_map[path.id()];
    return std::any_of(
      soft_margins.begin(), soft_margins.end(),
      [margin](const double soft_margin) { return std::abs(margin - soft_margin) < 0.01; });
  };
  const auto isSameHardMargin = [&](const PullOverPath & a, const PullOverPath & b) -> bool {
    return !isSoftMargin(a) && !isSoftMargin(b) &&
           std::abs(path_id_to_rough_margin_map[a.id()] - path_id_to_rough_margin_map[b.id()]) <
             0.01;
  };

  // NOTE: this is just partition sort based on curvature threshold within each sub partitions
  std::stable_sort(
    sorted_path_indices.begin(), sorted_path_indices.end(),
    [&](const size_t a_i, const size_t b_i) {
      const auto & a = pull_over_path_candidates[a_i];
      const auto & b = pull_over_path_candidates[b_i];
      // if both are soft margin or both are same hard margin, prioritize the path with lower
      // curvature.
      if ((isSoftMargin(a) && isSoftMargin(b)) || isSameHardMargin(a, b)) {
        return !isHighCurvature(a) && isHighCurvature(b);
      }
      // otherwise, keep the order based on the margin.
      return false;
    });

  // STEP2-4: Sort pull_over_path_candidates based on the order in efficient_path_order keeping
  // the collision check margin and curvature priority.
  if (parameters.path_priority == "efficient_path") {
    std::stable_sort(
      sorted_path_indices.begin(), sorted_path_indices.end(),
      [&](const size_t a_i, const size_t b_i) {
        // if any of following conditions are met, sort by path type priority
        // - both are soft margin
        // - both are same hard margin
        const auto & a = pull_over_path_candidates[a_i];
        const auto & b = pull_over_path_candidates[b_i];
        // otherwise, keep the order.
        return false;
      });
  }

  // Limit to top max_paths
  const size_t num_paths_to_select = std::min(max_paths, sorted_path_indices.size());

  std::vector<PullOverPath> selected;
  selected.reserve(num_paths_to_select);
  for (size_t i = 0; i < num_paths_to_select; ++i) {
    selected.push_back(pull_over_path_candidates.at(sorted_path_indices[i]));
  }

  if (max_paths == SIZE_MAX) {
    std::cout << "selectPullOverPaths: Selected all " << selected.size() << " paths (no filter)"
              << std::endl;
  } else {
    std::cout << "selectPullOverPaths: Selected " << selected.size() << " out of "
              << sorted_path_indices.size() << " paths (limited to top " << max_paths << ")"
              << std::endl;
  }

  return selected;
}

std::optional<PathWithLaneId> calculate_centerline_path(
  const std::shared_ptr<PlannerData> planner_data, const GoalPlannerParameters & parameters)
{
  const auto original_goal_pose = planner_data->route_handler->getOriginalGoalPose();
  const auto refined_goal_opt =
    autoware::behavior_path_planner::goal_planner_utils::calcRefinedGoal(
      original_goal_pose, planner_data->route_handler, true,
      planner_data->parameters.vehicle_length, planner_data->parameters.base_link2front,
      planner_data->parameters.base_link2front, parameters);
  if (!refined_goal_opt) {
    return std::nullopt;
  }
  const auto & refined_goal = refined_goal_opt.value();

  const auto & route_handler = planner_data->route_handler;
  const double forward_length = parameters.forward_goal_search_length;
  const double backward_length = parameters.backward_goal_search_length;
  const bool use_bus_stop_area = parameters.bus_stop_area.use_bus_stop_area;
  /*
  const double margin_from_boundary = parameters.margin_from_boundary;
  const double lateral_offset_interval = use_bus_stop_area
                                           ? parameters.bus_stop_area.lateral_offset_interval
                                           : parameters.lateral_offset_interval;
  const double max_lateral_offset = parameters.max_lateral_offset;
  const double ignore_distance_from_lane_start = parameters.ignore_distance_from_lane_start;
  */

  const auto pull_over_lanes =
    autoware::behavior_path_planner::goal_planner_utils::getPullOverLanes(
      *route_handler, true, parameters.backward_goal_search_length,
      parameters.forward_goal_search_length);
  const auto departure_check_lane =
    autoware::behavior_path_planner::goal_planner_utils::createDepartureCheckLanelet(
      pull_over_lanes, *route_handler, true);
  const auto goal_arc_coords = lanelet::utils::getArcCoordinates(pull_over_lanes, refined_goal);
  const double s_start = std::max(0.0, goal_arc_coords.length - backward_length);
  const double s_end = goal_arc_coords.length + forward_length;
  const double longitudinal_interval = use_bus_stop_area
                                         ? parameters.bus_stop_area.goal_search_interval
                                         : parameters.goal_search_interval;
  auto center_line_path = autoware::behavior_path_planner::utils::resamplePathWithSpline(
    route_handler->getCenterLinePath(pull_over_lanes, s_start, s_end), longitudinal_interval);
  return center_line_path;
}

std::vector<lanelet::BasicPolygon2d> getBusStopAreaPolygons(
  const std::shared_ptr<PlannerData> planner_data, const GoalPlannerParameters & parameters)
{
  const auto pull_over_lanes =
    autoware::behavior_path_planner::goal_planner_utils::getPullOverLanes(
      *(planner_data->route_handler), true, parameters.backward_goal_search_length,
      parameters.forward_goal_search_length);
  return autoware::behavior_path_planner::goal_planner_utils::getBusStopAreaPolygons(
    pull_over_lanes);
}

struct SortByWeightedDistance
{
  double lateral_cost{0.0};
  bool prioritize_goals_before_objects{false};

  SortByWeightedDistance(double cost, bool prioritize_goals_before_objects)
  : lateral_cost(cost), prioritize_goals_before_objects(prioritize_goals_before_objects)
  {
  }

  bool operator()(const GoalCandidate & a, const GoalCandidate & b) const noexcept
  {
    if (prioritize_goals_before_objects) {
      if (a.num_objects_to_avoid != b.num_objects_to_avoid) {
        return a.num_objects_to_avoid < b.num_objects_to_avoid;
      }
    }

    return a.distance_from_original_goal + lateral_cost * a.lateral_offset <
           b.distance_from_original_goal + lateral_cost * b.lateral_offset;
  }
};

int main(int argc, char ** argv)
{
  using autoware::behavior_path_planner::utils::getReferencePath;

  // Parse command line arguments with named arguments support
  // Usage: plot_map_case1 [--distance_threshold=5.0] [--yaw_threshold=0.1] [--lateral_threshold=0.1] [--max_paths=unlimited] [--mode=yaw] [--filter_type=positive] [--max_display=unlimited]
  // Example: plot_map_case1 --distance_threshold=5.0 --yaw_threshold=0.2 --mode=yaw --max_paths=100
  // Example: plot_map_case1 --distance_threshold=5.0 --lateral_threshold=0.5 --mode=lateral --filter_type=negative
  // Example: plot_map_case1 --distance_threshold=5.0 --yaw_threshold=0.2 --lateral_threshold=0.5 --mode=both --max_display=50
  double distance_threshold = 5.0;     // default: 5.0m (for yaw/lateral check distance)
  double yaw_threshold = -1.0;         // default: -1.0 (negative = show all)
  double lateral_threshold = -1.0;     // default: -1.0 (negative = show all)
  size_t max_paths = SIZE_MAX;         // default: unlimited (no filter in selectPullOverPaths)
  std::string filter_mode = "yaw";     // default: "yaw" ("yaw", "lateral", or "both")
  std::string filter_type = "positive"; // default: "positive" ("positive" or "negative")
  size_t max_display = SIZE_MAX;       // default: unlimited (display all filtered paths)

  // Helper to parse named arguments
  auto parseArg = [argc, argv](const std::string & name) -> std::string {
    std::string prefix = "--" + name + "=";
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if (arg.find(prefix) == 0) {
        return arg.substr(prefix.length());
      }
    }
    return "";
  };

  // Parse named arguments only (no positional arguments)
  std::string dist_str = parseArg("distance_threshold");
  std::string yaw_thresh_str = parseArg("yaw_threshold");
  std::string lat_thresh_str = parseArg("lateral_threshold");
  std::string max_paths_str = parseArg("max_paths");
  std::string mode_str = parseArg("mode");
  std::string filter_type_str = parseArg("filter_type");
  std::string max_display_str = parseArg("max_display");

  if (!dist_str.empty()) {
    try {
      distance_threshold = std::stod(dist_str);
      std::cout << "Using distance_threshold: " << distance_threshold << " meters" << std::endl;
    } catch (const std::exception & e) {
      std::cerr << "Invalid --distance_threshold value" << std::endl;
    }
  }

  if (!yaw_thresh_str.empty()) {
    try {
      yaw_threshold = std::stod(yaw_thresh_str);
      std::cout << "Using yaw_threshold: " << yaw_threshold << " radians" << std::endl;
    } catch (const std::exception & e) {
      std::cerr << "Invalid --yaw_threshold value" << std::endl;
    }
  }

  if (!lat_thresh_str.empty()) {
    try {
      lateral_threshold = std::stod(lat_thresh_str);
      std::cout << "Using lateral_threshold: " << lateral_threshold << " meters" << std::endl;
    } catch (const std::exception & e) {
      std::cerr << "Invalid --lateral_threshold value" << std::endl;
    }
  }

  if (!max_paths_str.empty()) {
    if (max_paths_str == "unlimited" || max_paths_str == "inf") {
      max_paths = SIZE_MAX;
      std::cout << "Using max_paths: unlimited (no filter)" << std::endl;
    } else {
      try {
        max_paths = std::stoull(max_paths_str);
        std::cout << "Using max_paths (selectPullOverPaths filter): " << max_paths << std::endl;
      } catch (const std::exception & e) {
        std::cerr << "Invalid --max_paths value" << std::endl;
      }
    }
  }

  if (!mode_str.empty()) {
    filter_mode = mode_str;
    if (filter_mode != "yaw" && filter_mode != "lateral" && filter_mode != "both") {
      std::cerr << "Invalid --mode value (must be 'yaw', 'lateral', or 'both'), using default 'yaw'"
                << std::endl;
      filter_mode = "yaw";
    } else {
      std::cout << "Using filter mode: " << filter_mode << std::endl;
    }
  }

  if (!filter_type_str.empty()) {
    filter_type = filter_type_str;
    if (filter_type != "positive" && filter_type != "negative") {
      std::cerr << "Invalid --filter_type value (must be 'positive' or 'negative'), using default 'positive'"
                << std::endl;
      filter_type = "positive";
    } else {
      std::cout << "Using filter type: " << filter_type << std::endl;
    }
  }

  if (!max_display_str.empty()) {
    if (max_display_str == "unlimited" || max_display_str == "inf") {
      max_display = SIZE_MAX;
      std::cout << "Using max_display: unlimited (display all filtered paths)" << std::endl;
    } else {
      try {
        max_display = std::stoull(max_display_str);
        std::cout << "Using max_display (plot display limit): " << max_display << std::endl;
      } catch (const std::exception & e) {
        std::cerr << "Invalid --max_display value" << std::endl;
      }
    }
  }

  std::cout << "\n=== Configuration ===" << std::endl;
  std::cout << "distance_threshold: " << distance_threshold << " m" << std::endl;
  std::cout << "yaw_threshold: " << (yaw_threshold >= 0.0 ? std::to_string(yaw_threshold) + " rad" : "disabled (show all)") << std::endl;
  std::cout << "lateral_threshold: " << (lateral_threshold >= 0.0 ? std::to_string(lateral_threshold) + " m" : "disabled (show all)") << std::endl;
  std::cout << "max_paths: " << (max_paths == SIZE_MAX ? "unlimited" : std::to_string(max_paths))
            << std::endl;
  std::cout << "mode: " << filter_mode << std::endl;
  std::cout << "filter_type: " << filter_type << std::endl;
  std::cout << "max_display: " << (max_display == SIZE_MAX ? "unlimited" : std::to_string(max_display))
            << std::endl;
  std::cout << "=====================\n" << std::endl;

  rclcpp::init(argc, argv);

  auto node_options = rclcpp::NodeOptions{};
  node_options.parameter_overrides(
    std::vector<rclcpp::Parameter>{{"launch_modules", std::vector<std::string>{}}});
  node_options.arguments(std::vector<std::string>{
    "--ros-args", "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner") +
      "/config/behavior_path_planner.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner") +
      "/config/drivable_area_expansion.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_planner") +
      "/config/scene_module_manager.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_test_utils") +
      "/config/test_common.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_test_utils") +
      "/config/test_nearest_search.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_test_utils") +
      "/config/test_vehicle_info.param.yaml",
    "--params-file",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_goal_planner_module") +
      "/config/goal_planner.param.yaml"});
  auto node = rclcpp::Node::make_shared("plot_map", node_options);

  auto planner_data = instantiate_planner_data(
    node,
    ament_index_cpp::get_package_share_directory("autoware_test_utils") +
      "/test_map/road_shoulder/lanelet2_map.osm",
    ament_index_cpp::get_package_share_directory("autoware_behavior_path_goal_planner_module") +
      "/config/sample_planner_data_case1.yaml");

  lanelet::ConstLanelet current_route_lanelet;
  planner_data->route_handler->getClosestLaneletWithinRoute(
    planner_data->self_odometry->pose.pose, &current_route_lanelet);
  const auto reference_path = getReferencePath(current_route_lanelet, planner_data);
  auto goal_planner_parameter =
    autoware::behavior_path_planner::GoalPlannerModuleManager::initGoalPlannerParameters(
      node.get(), "goal_planner.");
  goal_planner_parameter.bus_stop_area.use_bus_stop_area = true;
  goal_planner_parameter.lane_departure_check_expansion_margin = 0.2;
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo();
  autoware::boundary_departure_checker::Param boundary_departure_checker_params;
  boundary_departure_checker_params.footprint_extra_margin =
    goal_planner_parameter.lane_departure_check_expansion_margin;
  autoware::boundary_departure_checker::BoundaryDepartureChecker boundary_departure_checker(
    boundary_departure_checker_params, vehicle_info);

  const auto footprint = vehicle_info.createFootprint();
  auto goal_searcher = autoware::behavior_path_planner::GoalSearcher::create(
    goal_planner_parameter, footprint, planner_data);
  auto goal_candidates =
    goal_searcher.search(planner_data, goal_planner_parameter.bus_stop_area.use_bus_stop_area);
  std::sort(
    goal_candidates.begin(), goal_candidates.end(),
    SortByWeightedDistance(
      goal_planner_parameter.minimum_weighted_distance_lateral_weight,
      goal_planner_parameter.prioritize_goals_before_objects));
  std::map<size_t, size_t> goal_id2prio{};
  for (auto i = 0; i < goal_candidates.size(); ++i) {
    goal_id2prio[goal_candidates.at(i).id] = i;
  }

  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  // Set figure size for large display (16:9 aspect ratio)
  auto [fig, axes] = plt.subplots(1, 1, Kwargs("figsize"_a = pybind11::make_tuple(20, 11.25)));
  auto & ax2 = axes[0];
  const std::vector<lanelet::Id> ids{/*15213, 15214, */ 15225,
                                     15226,
                                     15224,
                                     15227,
                                     15228,
                                     15229,
                                     15230,
                                     15231,
                                     15232};
  for (const auto & id : ids) {
    const auto lanelet = planner_data->route_handler->getLaneletMapPtr()->laneletLayer.get(id);
    // plot_lanelet(ax1, lanelet);  // Temporarily disabled
    plot_lanelet(ax2, lanelet);
    // plot_lanelet(ax3, lanelet);
  }

  // plot_goal_candidates(ax1, goal_candidates, footprint);

  // Plot reference path
  plot_path_with_lane_id(ax2, reference_path.path, "green", "reference_path", 1.5);

  const auto start = std::chrono::steady_clock::now();
  std::vector<PullOverPath> candidates;
  for (auto i = 0; i < goal_candidates.size(); ++i) {
    const auto & goal_candidate = goal_candidates.at(i);
    auto shift_pull_over_planner =
      autoware::behavior_path_planner::BezierPullOver(*node, goal_planner_parameter);
    auto pull_over_paths =
      shift_pull_over_planner.plans(goal_candidate, 0, planner_data, reference_path);
    if (!pull_over_paths.empty()) {
      std::copy(
        std::make_move_iterator(pull_over_paths.begin()),
        std::make_move_iterator(pull_over_paths.end()), std::back_inserter(candidates));
    }
  }

  const auto filtered_paths = selectPullOverPaths(
    candidates, goal_candidates, planner_data, goal_planner_parameter, reference_path, max_paths);
  std::cout << filtered_paths.size() << std::endl;
  const auto end = std::chrono::steady_clock::now();
  std::cout << "computed candidate bezier paths in "
            << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() * 1.0 /
                 1000000
            << "msecs" << std::endl;
  std::cout << "filtered " << filtered_paths.size() << "/" << candidates.size() << " paths"
            << std::endl;
  /*
  for (auto i = 0; i < filtered_paths.size(); ++i) {
    const auto & filtered_path = filtered_paths.at(i);
    const auto goal_id = filtered_path.goal_id();
    const auto prio = goal_id2prio[goal_id];
    const auto & color = (i == 0) ? "red" : g_colors.at(i % g_colors.size());
    const auto max_parking_curvature = filtered_path.parking_path_max_curvature();
    plot_goal_candidate(ax1, filtered_path.modified_goal(), prio, footprint, color);
    if (i == 0) {
      plot_path_with_lane_id(ax1, filtered_path.parking_path(), color, "most prio", 2.0);
    } else {
      plot_path_with_lane_id(
        ax1, filtered_path.full_path(), color,
        std::to_string(prio) + "-th goal(id=" + std::to_string(goal_id) +
          "): k_max=" + std::to_string(max_parking_curvature),
        0.5);
    }
  }
  */
  const auto original_goal_pos = planner_data->route_handler->getOriginalGoalPose().position;
  // ax1.plot(  // Temporarily disabled
  //   Args(original_goal_pos.x, original_goal_pos.y),
  //   Kwargs("marker"_a = "x", "label"_a = "goal", "markersize"_a = 20, "color"_a = "red"));
  // if (goal_planner_parameter.bus_stop_area.use_bus_stop_area) {
  //   const auto bus_stop_area_polygons =
  //     getBusStopAreaPolygons(planner_data, goal_planner_parameter);
  //   for (const auto & bus_stop_area_polygon : bus_stop_area_polygons) {
  //     plot_lanelet_polygon(ax1, bus_stop_area_polygon);
  //   }
  // }

  // Normalize angle to [-pi, pi]
  auto normalizeAngle = [](double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  };

  // Filter paths with high curvature change near start
  auto calculateStartYawChange = [&normalizeAngle](
                                   const PullOverPath & path, double distance_threshold = 5.0) {
    const auto & points = path.full_path().points;
    if (points.size() < 2) return 0.0;

    const auto start_pose = path.start_pose();
    double start_yaw = autoware_utils::get_rpy(start_pose).z;
    double max_relative_yaw = 0.0;  // Maximum relative yaw from start

    // Find start_pose index in the path
    size_t start_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
      double dist = std::hypot(
        points[i].point.pose.position.x - start_pose.position.x,
        points[i].point.pose.position.y - start_pose.position.y);
      if (dist < min_dist) {
        min_dist = dist;
        start_idx = i;
      }
    }

    // Calculate max relative yaw within distance_threshold from start_pose
    double accumulated_distance = 0.0;
    for (size_t i = start_idx + 1; i < points.size(); ++i) {
      const auto & prev_point = points[i - 1].point.pose.position;
      const auto & curr_point = points[i].point.pose.position;
      accumulated_distance += std::hypot(curr_point.x - prev_point.x, curr_point.y - prev_point.y);

      if (accumulated_distance > distance_threshold) {
        break;
      }

      double yaw = autoware_utils::get_rpy(points[i].point.pose).z;
      double relative_yaw = normalizeAngle(yaw - start_yaw);  // Relative to start_yaw
      max_relative_yaw = std::max(max_relative_yaw, std::abs(relative_yaw));
    }

    return max_relative_yaw;
  };

  // Calculate maximum lateral distance deviation from start_pose within distance_threshold
  auto calculateMaxLateralDistance = [](const PullOverPath & path, double distance_threshold = 5.0) {
    const auto & points = path.full_path().points;
    if (points.size() < 2) return 0.0;

    const auto start_pose = path.start_pose();
    const double start_x = start_pose.position.x;
    const double start_y = start_pose.position.y;
    const double start_yaw = autoware_utils::get_rpy(start_pose).z;

    // Unit vector in the direction of start_pose
    const double cos_yaw = std::cos(start_yaw);
    const double sin_yaw = std::sin(start_yaw);

    // Find start_pose index in the path
    size_t start_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < points.size(); ++i) {
      double dist = std::hypot(
        points[i].point.pose.position.x - start_x,
        points[i].point.pose.position.y - start_y);
      if (dist < min_dist) {
        min_dist = dist;
        start_idx = i;
      }
    }

    // Calculate max lateral distance within distance_threshold from start_pose
    double max_lateral_distance = 0.0;
    double accumulated_distance = 0.0;

    for (size_t i = start_idx + 1; i < points.size(); ++i) {
      const auto & prev_point = points[i - 1].point.pose.position;
      const auto & curr_point = points[i].point.pose.position;
      accumulated_distance += std::hypot(curr_point.x - prev_point.x, curr_point.y - prev_point.y);

      if (accumulated_distance > distance_threshold) {
        break;
      }

      // Vector from start_pose to current point
      const double dx = curr_point.x - start_x;
      const double dy = curr_point.y - start_y;

      // Lateral distance (perpendicular to start_pose direction)
      // lateral = dx * sin(yaw) - dy * cos(yaw) for right-hand coordinate system
      const double lateral_distance = std::abs(dx * sin_yaw - dy * cos_yaw);
      max_lateral_distance = std::max(max_lateral_distance, lateral_distance);
    }

    return max_lateral_distance;
  };

  // Calculate metrics for all paths
  std::vector<std::tuple<size_t, double, double>> path_metrics;  // <idx, yaw_change, lateral_dist>

  for (size_t i = 0; i < filtered_paths.size(); ++i) {
    double yaw_change = calculateStartYawChange(filtered_paths[i], distance_threshold);
    double lateral_dist = calculateMaxLateralDistance(filtered_paths[i], distance_threshold);
    path_metrics.push_back({i, yaw_change, lateral_dist});
  }

  // Filter paths based on thresholds
  std::vector<size_t> filtered_indices;

  if (filter_mode == "yaw") {
    // Filter by yaw threshold only
    for (const auto & [idx, yaw_change, lateral_dist] : path_metrics) {
      bool condition_met = (yaw_threshold < 0.0) ||
                           (filter_type == "positive" ? yaw_change >= yaw_threshold : yaw_change < yaw_threshold);
      if (condition_met) {
        filtered_indices.push_back(idx);
      }
    }
    // Sort by yaw change (descending order for positive, ascending for negative)
    if (filter_type == "positive") {
      std::sort(filtered_indices.begin(), filtered_indices.end(), [&path_metrics](size_t a, size_t b) {
        return std::get<1>(path_metrics[a]) > std::get<1>(path_metrics[b]);
      });
    } else {
      std::sort(filtered_indices.begin(), filtered_indices.end(), [&path_metrics](size_t a, size_t b) {
        return std::get<1>(path_metrics[a]) < std::get<1>(path_metrics[b]);
      });
    }

    std::cout << "\n=== Paths filtered by yaw threshold (" << filter_type << ") ===" << std::endl;
    if (yaw_threshold < 0.0) {
      std::cout << "No yaw threshold applied (showing all " << filtered_indices.size() << " paths)" << std::endl;
    } else if (filter_type == "positive") {
      std::cout << "Filtered " << filtered_indices.size() << " paths with yaw change >= "
                << yaw_threshold << " rad (" << (yaw_threshold * 180.0 / M_PI) << " deg)" << std::endl;
    } else {
      std::cout << "Filtered " << filtered_indices.size() << " paths with yaw change < "
                << yaw_threshold << " rad (" << (yaw_threshold * 180.0 / M_PI) << " deg)" << std::endl;
    }

    // Show top 10 paths
    if (filter_type == "positive") {
      std::cout << "\n=== Top 10 paths with highest yaw change ===" << std::endl;
    } else {
      std::cout << "\n=== Top 10 paths with lowest yaw change ===" << std::endl;
    }
    for (size_t rank = 0; rank < std::min<size_t>(10, filtered_indices.size()); ++rank) {
      const auto idx = filtered_indices[rank];
      const auto yaw_change = std::get<1>(path_metrics[idx]);
      std::cout << "Rank " << rank + 1 << ": Path " << idx << " - Yaw change: " << yaw_change
                << " rad (" << (yaw_change * 180.0 / M_PI) << " deg)" << std::endl;
    }

  } else if (filter_mode == "lateral") {
    // Filter by lateral threshold only
    for (const auto & [idx, yaw_change, lateral_dist] : path_metrics) {
      bool condition_met = (lateral_threshold < 0.0) ||
                           (filter_type == "positive" ? lateral_dist >= lateral_threshold : lateral_dist < lateral_threshold);
      if (condition_met) {
        filtered_indices.push_back(idx);
      }
    }
    // Sort by lateral distance (descending order for positive, ascending for negative)
    if (filter_type == "positive") {
      std::sort(filtered_indices.begin(), filtered_indices.end(), [&path_metrics](size_t a, size_t b) {
        return std::get<2>(path_metrics[a]) > std::get<2>(path_metrics[b]);
      });
    } else {
      std::sort(filtered_indices.begin(), filtered_indices.end(), [&path_metrics](size_t a, size_t b) {
        return std::get<2>(path_metrics[a]) < std::get<2>(path_metrics[b]);
      });
    }

    std::cout << "\n=== Paths filtered by lateral threshold (" << filter_type << ") ===" << std::endl;
    if (lateral_threshold < 0.0) {
      std::cout << "No lateral threshold applied (showing all " << filtered_indices.size() << " paths)" << std::endl;
    } else if (filter_type == "positive") {
      std::cout << "Filtered " << filtered_indices.size() << " paths with lateral distance >= "
                << lateral_threshold << " m" << std::endl;
    } else {
      std::cout << "Filtered " << filtered_indices.size() << " paths with lateral distance < "
                << lateral_threshold << " m" << std::endl;
    }

    // Show top 10 paths
    if (filter_type == "positive") {
      std::cout << "\n=== Top 10 paths with highest lateral distance ===" << std::endl;
    } else {
      std::cout << "\n=== Top 10 paths with lowest lateral distance ===" << std::endl;
    }
    for (size_t rank = 0; rank < std::min<size_t>(10, filtered_indices.size()); ++rank) {
      const auto idx = filtered_indices[rank];
      const auto lateral_dist = std::get<2>(path_metrics[idx]);
      std::cout << "Rank " << rank + 1 << ": Path " << idx << " - Lateral distance: "
                << lateral_dist << " m" << std::endl;
    }

  } else if (filter_mode == "both") {
    // Filter by both yaw OR lateral thresholds (either condition satisfies)
    for (const auto & [idx, yaw_change, lateral_dist] : path_metrics) {
      bool yaw_ok, lat_ok;

      if (filter_type == "positive") {
        yaw_ok = (yaw_threshold >= 0.0) && (yaw_change >= yaw_threshold);
        lat_ok = (lateral_threshold >= 0.0) && (lateral_dist >= lateral_threshold);
      } else {
        yaw_ok = (yaw_threshold >= 0.0) && (yaw_change < yaw_threshold);
        lat_ok = (lateral_threshold >= 0.0) && (lateral_dist < lateral_threshold);
      }

      // If both thresholds are negative (not specified), show all paths
      if (yaw_threshold < 0.0 && lateral_threshold < 0.0) {
        filtered_indices.push_back(idx);
      }
      // If either threshold is met (OR condition)
      else if (yaw_ok || lat_ok) {
        filtered_indices.push_back(idx);
      }
    }
    // Sort by yaw change (primary), then lateral distance (secondary)
    if (filter_type == "positive") {
      std::sort(filtered_indices.begin(), filtered_indices.end(), [&path_metrics](size_t a, size_t b) {
        auto yaw_a = std::get<1>(path_metrics[a]);
        auto yaw_b = std::get<1>(path_metrics[b]);
        if (std::abs(yaw_a - yaw_b) > 1e-6) {
          return yaw_a > yaw_b;
        }
        return std::get<2>(path_metrics[a]) > std::get<2>(path_metrics[b]);
      });
    } else {
      std::sort(filtered_indices.begin(), filtered_indices.end(), [&path_metrics](size_t a, size_t b) {
        auto yaw_a = std::get<1>(path_metrics[a]);
        auto yaw_b = std::get<1>(path_metrics[b]);
        if (std::abs(yaw_a - yaw_b) > 1e-6) {
          return yaw_a < yaw_b;
        }
        return std::get<2>(path_metrics[a]) < std::get<2>(path_metrics[b]);
      });
    }

    std::cout << "\n=== Paths filtered by yaw OR lateral thresholds (" << filter_type << ") ===" << std::endl;
    if (yaw_threshold < 0.0 && lateral_threshold < 0.0) {
      std::cout << "No thresholds applied (showing all " << filtered_indices.size() << " paths)" << std::endl;
    } else {
      std::cout << "Filtered " << filtered_indices.size() << " paths with ";
      if (yaw_threshold >= 0.0) {
        if (filter_type == "positive") {
          std::cout << "yaw change >= " << yaw_threshold << " rad";
        } else {
          std::cout << "yaw change < " << yaw_threshold << " rad";
        }
      }
      if (yaw_threshold >= 0.0 && lateral_threshold >= 0.0) {
        std::cout << " OR ";
      }
      if (lateral_threshold >= 0.0) {
        if (filter_type == "positive") {
          std::cout << "lateral distance >= " << lateral_threshold << " m";
        } else {
          std::cout << "lateral distance < " << lateral_threshold << " m";
        }
      }
      std::cout << std::endl;
    }

    // Show top 10 paths
    std::cout << "\n=== Top 10 paths (sorted by yaw, then lateral) ===" << std::endl;
    for (size_t rank = 0; rank < std::min<size_t>(10, filtered_indices.size()); ++rank) {
      const auto idx = filtered_indices[rank];
      const auto yaw_change = std::get<1>(path_metrics[idx]);
      const auto lateral_dist = std::get<2>(path_metrics[idx]);
      std::cout << "Rank " << rank + 1 << ": Path " << idx
                << " - Yaw: " << yaw_change << " rad (" << (yaw_change * 180.0 / M_PI) << " deg)"
                << ", Lateral: " << lateral_dist << " m" << std::endl;
    }
  }

  for (auto i = 0; i < filtered_paths.size(); ++i) {
    const auto & filtered_path = filtered_paths.at(i);
    const auto goal_id = filtered_path.goal_id();
    const auto prio = goal_id2prio[goal_id];
    const auto & color = (i == 0) ? "red" : g_colors.at(i % g_colors.size());
    const auto max_parking_curvature = filtered_path.parking_path_max_curvature();
    if (i == 0) {
      // plot_goal_candidate(ax1, filtered_path.modified_goal(), prio, footprint, color);  //
      // Temporarily disabled
      plot_path_with_lane_id(ax2, filtered_path.full_path(), color, "most prio", 2.0);

      // Visualize most prio start pose (arrow only, red)
      const auto & most_prio_start_pose = filtered_path.start_pose();
      std::vector<double> most_prio_start_x = {most_prio_start_pose.position.x};
      std::vector<double> most_prio_start_y = {most_prio_start_pose.position.y};
      const double most_prio_start_yaw = autoware_utils::get_rpy(most_prio_start_pose).z;
      std::vector<double> most_prio_start_yaw_cos = {std::cos(most_prio_start_yaw)};
      std::vector<double> most_prio_start_yaw_sin = {std::sin(most_prio_start_yaw)};

      ax2.quiver(
        Args(
          most_prio_start_x, most_prio_start_y, most_prio_start_yaw_cos, most_prio_start_yaw_sin),
        Kwargs(
          "angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 1.5, "color"_a = "red",
          "width"_a = 0.01, "zorder"_a = 15));

      // Find yaw_check_end_pose (5m from start_pose)
      const auto & points = filtered_path.full_path().points;
      double accumulated_distance = 0.0;
      geometry_msgs::msg::Pose yaw_check_end_pose;
      bool found_end_pose = false;

      // Find start_pose index in the path
      size_t start_idx = 0;
      double min_dist = std::numeric_limits<double>::max();
      for (size_t j = 0; j < points.size(); ++j) {
        double dist = std::hypot(
          points[j].point.pose.position.x - most_prio_start_pose.position.x,
          points[j].point.pose.position.y - most_prio_start_pose.position.y);
        if (dist < min_dist) {
          min_dist = dist;
          start_idx = j;
        }
      }

      // Calculate 5m from start_pose
      for (size_t j = start_idx + 1; j < points.size(); ++j) {
        const auto & prev_point = points[j - 1].point.pose.position;
        const auto & curr_point = points[j].point.pose.position;
        accumulated_distance +=
          std::hypot(curr_point.x - prev_point.x, curr_point.y - prev_point.y);

        if (accumulated_distance >= distance_threshold && !found_end_pose) {
          yaw_check_end_pose = points[j].point.pose;
          found_end_pose = true;
          break;
        }
      }

      // Draw yaw_check_end_pose (arrow only, blue)
      if (found_end_pose) {
        std::vector<double> end_x = {yaw_check_end_pose.position.x};
        std::vector<double> end_y = {yaw_check_end_pose.position.y};
        const double end_yaw = autoware_utils::get_rpy(yaw_check_end_pose).z;
        std::vector<double> end_yaw_cos = {std::cos(end_yaw)};
        std::vector<double> end_yaw_sin = {std::sin(end_yaw)};

        ax2.quiver(
          Args(end_x, end_y, end_yaw_cos, end_yaw_sin),
          Kwargs(
            "angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 1.5, "color"_a = "blue",
            "width"_a = 0.01, "zorder"_a = 14));
      }
    } else if (i % 50 == 0) {
      // std::cout << "plotting " << i << "-th filtered path" << std::endl;
      // plot_goal_candidate(ax1, filtered_path.modified_goal(), prio, footprint, color);  //
      // Temporarily disabled plot_path_with_lane_id(ax1, filtered_path.full_path(), color,
      // "", 2.0);  // Temporarily disabled
    }
  }

  // Plot filtered paths on ax2
  for (size_t rank = 0; rank < filtered_indices.size(); ++rank) {
    const auto idx = filtered_indices[rank];
    const auto & path = filtered_paths[idx];
    const auto goal_id = path.goal_id();
    const auto prio = goal_id2prio[goal_id];
    const auto & color = g_colors.at(rank % g_colors.size());

    plot_goal_candidate(ax2, path.modified_goal(), prio, footprint, color);

    std::string label;
    const auto yaw_change = std::get<1>(path_metrics[idx]);
    const auto lateral_dist = std::get<2>(path_metrics[idx]);

    if (filter_mode == "yaw") {
      label = "Rank " + std::to_string(rank + 1) +
              " (yaw: " + std::to_string(yaw_change * 180.0 / M_PI) + " deg)";
    } else if (filter_mode == "lateral") {
      label = "Rank " + std::to_string(rank + 1) +
              " (lateral: " + std::to_string(lateral_dist) + " m)";
    } else if (filter_mode == "both") {
      label = "Rank " + std::to_string(rank + 1) +
              " (yaw: " + std::to_string(yaw_change * 180.0 / M_PI) + " deg" +
              ", lat: " + std::to_string(lateral_dist) + " m)";
    }

    plot_path_with_lane_id(ax2, path.full_path(), color, label, 1.5);

    // Visualize start pose (arrow only, red)
    const auto & start_pose = path.start_pose();
    std::vector<double> start_x = {start_pose.position.x};
    std::vector<double> start_y = {start_pose.position.y};
    const double start_yaw = autoware_utils::get_rpy(start_pose).z;
    std::vector<double> start_yaw_cos = {std::cos(start_yaw)};
    std::vector<double> start_yaw_sin = {std::sin(start_yaw)};

    ax2.quiver(
      Args(start_x, start_y, start_yaw_cos, start_yaw_sin),
      Kwargs(
        "angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 2.0, "color"_a = "red",
        "width"_a = 0.008, "zorder"_a = 10));

    // Find yaw_check_end_pose (5m from start_pose)
    const auto & points = path.full_path().points;
    double accumulated_distance = 0.0;
    geometry_msgs::msg::Pose yaw_check_end_pose;
    bool found_end_pose = false;

    // Find start_pose index in the path
    size_t start_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t j = 0; j < points.size(); ++j) {
      double dist = std::hypot(
        points[j].point.pose.position.x - start_pose.position.x,
        points[j].point.pose.position.y - start_pose.position.y);
      if (dist < min_dist) {
        min_dist = dist;
        start_idx = j;
      }
    }

    // Calculate 5m from start_pose
    for (size_t j = start_idx + 1; j < points.size(); ++j) {
      const auto & prev_point = points[j - 1].point.pose.position;
      const auto & curr_point = points[j].point.pose.position;
      accumulated_distance += std::hypot(curr_point.x - prev_point.x, curr_point.y - prev_point.y);

      if (accumulated_distance >= distance_threshold
        && !found_end_pose) {
        yaw_check_end_pose = points[j].point.pose;
        found_end_pose = true;
        break;
      }
    }

    // Draw yaw_check_end_pose (arrow only, blue)
    if (found_end_pose) {
      std::vector<double> end_x = {yaw_check_end_pose.position.x};
      std::vector<double> end_y = {yaw_check_end_pose.position.y};
      const double end_yaw = autoware_utils::get_rpy(yaw_check_end_pose).z;
      std::vector<double> end_yaw_cos = {std::cos(end_yaw)};
      std::vector<double> end_yaw_sin = {std::sin(end_yaw)};

      ax2.quiver(
        Args(end_x, end_y, end_yaw_cos, end_yaw_sin),
        Kwargs(
          "angles"_a = "xy", "scale_units"_a = "xy", "scale"_a = 2.0, "color"_a = "blue",
          "width"_a = 0.008, "zorder"_a = 11));
    }
  }
  ax2.plot(
    Args(original_goal_pos.x, original_goal_pos.y),
    Kwargs("marker"_a = "x", "label"_a = "goal", "markersize"_a = 20, "color"_a = "red"));

  const auto centerline_path = calculate_centerline_path(planner_data, goal_planner_parameter);
  if (centerline_path) {
    // plot_path_with_lane_id(ax2, centerline_path.value(), "red", "centerline_path");
  }

  // ax1.set_aspect(Args("equal"));  // Temporarily disabled
  ax2.set_aspect(Args("equal"));
  ax2.legend();
  plt.show();

  rclcpp::shutdown();
  return 0;
}
