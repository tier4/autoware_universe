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

#include "autoware/behavior_path_planner_common/data_manager.hpp"

#include "autoware_lanelet2_extension/utility/utilities.hpp"

#include <utility>
#include <vector>
namespace autoware::behavior_path_planner
{

using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrafficLightGroup;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using tier4_planning_msgs::msg::LateralOffset;
using PlanResult = PathWithLaneId::SharedPtr;
using tier4_planning_msgs::msg::VelocityLimit;
using unique_identifier_msgs::msg::UUID;

void PlannerData::init_parameters(rclcpp::Node & node)
{
  parameters.traffic_light_signal_timeout =
    node.declare_parameter<double>("traffic_light_signal_timeout");

  // vehicle info
  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
  parameters.vehicle_info = vehicle_info;
  parameters.vehicle_width = vehicle_info.vehicle_width_m;
  parameters.vehicle_length = vehicle_info.vehicle_length_m;
  parameters.wheel_tread = vehicle_info.wheel_tread_m;
  parameters.wheel_base = vehicle_info.wheel_base_m;
  parameters.front_overhang = vehicle_info.front_overhang_m;
  parameters.rear_overhang = vehicle_info.rear_overhang_m;
  parameters.left_over_hang = vehicle_info.left_overhang_m;
  parameters.right_over_hang = vehicle_info.right_overhang_m;
  parameters.base_link2front = vehicle_info.max_longitudinal_offset_m;
  parameters.base_link2rear = parameters.rear_overhang;

  // NOTE: backward_path_length is used not only calculating path length but also calculating the
  // size of a drivable area.
  //       The drivable area has to cover not the base link but the vehicle itself. Therefore
  //       rear_overhang must be added to backward_path_length. In addition, because of the
  //       calculation of the drivable area in the autoware_path_optimizer package, the drivable
  //       area has to be a little longer than the backward_path_length parameter by adding
  //       min_backward_offset.
  constexpr double min_backward_offset = 1.0;
  const double backward_offset = vehicle_info.rear_overhang_m + min_backward_offset;

  // ROS parameters
  parameters.backward_path_length =
    node.declare_parameter<double>("backward_path_length") + backward_offset;
  parameters.forward_path_length = node.declare_parameter<double>("forward_path_length");

  // acceleration parameters
  parameters.min_acc = node.declare_parameter<double>("normal.min_acc");
  parameters.max_acc = node.declare_parameter<double>("normal.max_acc");

  parameters.max_vel = node.declare_parameter<double>("max_vel");
  parameters.backward_length_buffer_for_end_of_pull_over =
    node.declare_parameter<double>("backward_length_buffer_for_end_of_pull_over");
  parameters.backward_length_buffer_for_end_of_pull_out =
    node.declare_parameter<double>("backward_length_buffer_for_end_of_pull_out");

  parameters.minimum_pull_over_length = node.declare_parameter<double>("minimum_pull_over_length");
  parameters.refine_goal_search_radius_range =
    node.declare_parameter<double>("refine_goal_search_radius_range");
  parameters.turn_signal_intersection_search_distance =
    node.declare_parameter<double>("turn_signal_intersection_search_distance");
  parameters.turn_signal_intersection_angle_threshold_deg =
    node.declare_parameter<double>("turn_signal_intersection_angle_threshold_deg");
  parameters.turn_signal_minimum_search_distance =
    node.declare_parameter<double>("turn_signal_minimum_search_distance");
  parameters.turn_signal_search_time = node.declare_parameter<double>("turn_signal_search_time");
  parameters.turn_signal_shift_length_threshold =
    node.declare_parameter<double>("turn_signal_shift_length_threshold");
  parameters.turn_signal_remaining_shift_length_threshold =
    node.declare_parameter<double>("turn_signal_remaining_shift_length_threshold");
  parameters.turn_signal_on_swerving = node.declare_parameter<bool>("turn_signal_on_swerving");

  parameters.enable_akima_spline_first = node.declare_parameter<bool>("enable_akima_spline_first");
  parameters.enable_cog_on_centerline = node.declare_parameter<bool>("enable_cog_on_centerline");
  parameters.input_path_interval = node.declare_parameter<double>("input_path_interval");
  parameters.output_path_interval = node.declare_parameter<double>("output_path_interval");
  parameters.ego_nearest_dist_threshold =
    node.declare_parameter<double>("ego_nearest_dist_threshold");
  parameters.ego_nearest_yaw_threshold =
    node.declare_parameter<double>("ego_nearest_yaw_threshold");

  drivable_area_expansion_parameters.init(node);
}

std::pair<TurnSignalInfo, bool> PlannerData::getBehaviorTurnSignalInfo(
  const PathWithLaneId & path, const size_t shift_start_idx, const size_t shift_end_idx,
  const lanelet::ConstLanelets & current_lanelets, const double current_shift_length,
  const bool is_driving_forward, const bool egos_lane_is_shifted,
  const bool override_ego_stopped_check, const bool is_pull_out, const bool is_lane_change,
  const bool is_pull_over) const
{
  if (shift_start_idx + 1 > path.points.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return std::make_pair(TurnSignalInfo{}, true);
  }

  if (shift_end_idx + 1 > path.points.size()) {
    RCLCPP_WARN(rclcpp::get_logger(__func__), "index inconsistency.");
    return std::make_pair(TurnSignalInfo{}, true);
  }

  std::vector<double> lengths(path.points.size(), 0.0);
  ShiftedPath shifted_path{path, lengths};
  ShiftLine shift_line;

  {
    const auto start_pose = path.points.at(shift_start_idx).point.pose;
    const auto start_shift_length =
      lanelet::utils::getArcCoordinates(current_lanelets, start_pose).distance;
    const auto end_pose = path.points.at(shift_end_idx).point.pose;
    const auto end_shift_length =
      lanelet::utils::getArcCoordinates(current_lanelets, end_pose).distance;
    shifted_path.shift_length.at(shift_start_idx) = start_shift_length;
    shifted_path.shift_length.at(shift_end_idx) = end_shift_length;

    shift_line.start = start_pose;
    shift_line.end = end_pose;
    shift_line.start_shift_length = start_shift_length;
    shift_line.end_shift_length = end_shift_length;
    shift_line.start_idx = shift_start_idx;
    shift_line.end_idx = shift_end_idx;
  }

  return turn_signal_decider.getBehaviorTurnSignalInfo(
    shifted_path, shift_line, current_lanelets, route_handler, parameters, self_odometry,
    current_shift_length, is_driving_forward, egos_lane_is_shifted, override_ego_stopped_check,
    is_pull_out, is_lane_change, is_pull_over);
}

std::pair<TurnSignalInfo, bool> PlannerData::getBehaviorTurnSignalInfo(
  const ShiftedPath & path, const ShiftLine & shift_line,
  const lanelet::ConstLanelets & current_lanelets, const double current_shift_length,
  const bool is_driving_forward, const bool egos_lane_is_shifted,
  const bool override_ego_stopped_check, const bool is_pull_out) const
{
  return turn_signal_decider.getBehaviorTurnSignalInfo(
    path, shift_line, current_lanelets, route_handler, parameters, self_odometry,
    current_shift_length, is_driving_forward, egos_lane_is_shifted, override_ego_stopped_check,
    is_pull_out);
}

TurnIndicatorsCommand PlannerData::getTurnSignal(
  const PathWithLaneId & path, const TurnSignalInfo & turn_signal_info,
  TurnSignalDebugData & debug_data)
{
  const auto & current_pose = self_odometry->pose.pose;
  const auto & current_vel = self_odometry->twist.twist.linear.x;
  return turn_signal_decider.getTurnSignal(
    route_handler, path, turn_signal_info, current_pose, current_vel, parameters, debug_data);
}

std::optional<TrafficSignalStamped> PlannerData::getTrafficSignal(const int64_t id) const
{
  if (traffic_light_id_map.count(id) == 0) {
    return std::nullopt;
  }

  const auto elapsed_time =
    (rclcpp::Clock{RCL_ROS_TIME}.now() - traffic_light_id_map.at(id).stamp).seconds();
  if (elapsed_time > parameters.traffic_light_signal_timeout) {
    return std::nullopt;
  }

  return traffic_light_id_map.at(id);
}

}  // namespace autoware::behavior_path_planner
