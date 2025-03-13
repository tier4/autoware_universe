// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_

#include "autoware/behavior_path_planner_common/parameters.hpp"
#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/lateral_offset.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

struct TrafficSignalStamped
{
  builtin_interfaces::msg::Time stamp;
  autoware_perception_msgs::msg::TrafficLightGroup signal;
};

struct BoolStamped
{
  explicit BoolStamped(bool in_data) : data(in_data) {}
  bool data{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct ModuleNameStamped
{
  std::string module_name = "NONE";
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct DrivableLanes
{
  lanelet::ConstLanelet right_lane;
  lanelet::ConstLanelet left_lane;
  lanelet::ConstLanelets middle_lanes;
};

// NOTE: To deal with some policies about drivable area generation, currently DrivableAreaInfo is
// quite messy. Needs to be refactored.
struct DrivableAreaInfo
{
  struct Obstacle
  {
    geometry_msgs::msg::Pose pose;
    autoware_utils::Polygon2d poly;
    bool is_left{true};
  };
  std::vector<DrivableLanes> drivable_lanes;
  std::vector<Obstacle> obstacles;  // obstacles to extract from the drivable area
  bool enable_expanding_hatched_road_markings{false};
  bool enable_expanding_intersection_areas{false};
  bool enable_expanding_freespace_areas{false};

  // temporary only for pull over's freespace planning
  double drivable_margin{0.0};

  // temporary only for side shift
  bool is_already_expanded{false};
};

struct BehaviorModuleOutput
{
  BehaviorModuleOutput() = default;
  BehaviorModuleOutput & operator=(const BehaviorModuleOutput &) = default;
  BehaviorModuleOutput(
    PathWithLaneId path, PathWithLaneId reference_path, TurnSignalInfo turn_signal_info,
    std::optional<autoware_planning_msgs::msg::PoseWithUuidStamped> modified_goal,
    DrivableAreaInfo drivable_area_info)
  : path(std::move(path)),
    reference_path(std::move(reference_path)),
    turn_signal_info(turn_signal_info),
    modified_goal(std::move(modified_goal)),
    drivable_area_info(std::move(drivable_area_info))
  {
  }
  BehaviorModuleOutput(const BehaviorModuleOutput &) = default;

  // path planed by module
  PathWithLaneId path;

  // reference path planed by module
  PathWithLaneId reference_path;

  TurnSignalInfo turn_signal_info;

  std::optional<autoware_planning_msgs::msg::PoseWithUuidStamped> modified_goal;

  // drivable area info to create drivable area
  // NOTE: Drivable area in the path is generated at last from drivable_area_info.
  DrivableAreaInfo drivable_area_info;
};

struct CandidateOutput
{
  CandidateOutput() = default;
  explicit CandidateOutput(PathWithLaneId path) : path_candidate{std::move(path)} {}
  PathWithLaneId path_candidate;
  double lateral_shift{0.0};
  double start_distance_to_path_change{std::numeric_limits<double>::lowest()};
  double finish_distance_to_path_change{std::numeric_limits<double>::lowest()};
};

/**
 * @brief Adds detail text to stop/slow/dead_pose virtual walls.
 */
struct PoseWithDetail
{
  Pose pose;
  std::string detail;
  explicit PoseWithDetail(const Pose & p, std::string_view d = "") : pose(p), detail(d) {}
};
using PoseWithDetailOpt = std::optional<PoseWithDetail>;

struct PlannerData
{
  Odometry::ConstSharedPtr self_odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr self_acceleration;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr dynamic_object;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr costmap;
  tier4_planning_msgs::msg::LateralOffset::ConstSharedPtr lateral_offset;
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode;
  PathWithLaneId::SharedPtr prev_output_path{std::make_shared<PathWithLaneId>()};
  std::optional<autoware_planning_msgs::msg::PoseWithUuidStamped> prev_modified_goal;
  std::optional<UUID> prev_route_id;
  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};
  std::map<int64_t, TrafficSignalStamped> traffic_light_id_map;
  BehaviorPathPlannerParameters parameters{};
  autoware::behavior_path_planner::drivable_area_expansion::DrivableAreaExpansionParameters
    drivable_area_expansion_parameters;
  tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr external_limit_max_velocity;

  mutable std::vector<geometry_msgs::msg::Pose> drivable_area_expansion_prev_path_poses;
  mutable std::vector<double> drivable_area_expansion_prev_curvatures;
  mutable TurnSignalDecider turn_signal_decider;

  void init_parameters(rclcpp::Node & node);

  std::pair<TurnSignalInfo, bool> getBehaviorTurnSignalInfo(
    const PathWithLaneId & path, const size_t shift_start_idx, const size_t shift_end_idx,
    const lanelet::ConstLanelets & current_lanelets, const double current_shift_length,
    const bool is_driving_forward, const bool egos_lane_is_shifted,
    const bool override_ego_stopped_check = false, const bool is_pull_out = false,
    const bool is_lane_change = false, const bool is_pull_over = false) const;

  std::pair<TurnSignalInfo, bool> getBehaviorTurnSignalInfo(
    const ShiftedPath & path, const ShiftLine & shift_line,
    const lanelet::ConstLanelets & current_lanelets, const double current_shift_length,
    const bool is_driving_forward, const bool egos_lane_is_shifted,
    const bool override_ego_stopped_check = false, const bool is_pull_out = false) const;

  TurnIndicatorsCommand getTurnSignal(
    const PathWithLaneId & path, const TurnSignalInfo & turn_signal_info,
    TurnSignalDebugData & debug_data);

  std::optional<TrafficSignalStamped> getTrafficSignal(const int64_t id) const;

  template <class T>
  size_t findEgoIndex(const std::vector<T> & points) const
  {
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }

  template <class T>
  size_t findEgoSegmentIndex(const std::vector<T> & points) const
  {
    return autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, self_odometry->pose.pose, parameters.ego_nearest_dist_threshold,
      parameters.ego_nearest_yaw_threshold);
  }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__DATA_MANAGER_HPP_
