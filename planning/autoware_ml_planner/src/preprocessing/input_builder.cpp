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

#include "autoware/ml_planner/preprocessing/input_builder.hpp"

#include "autoware/ml_planner/constants.hpp"
#include "autoware/ml_planner/dimensions.hpp"
#include "autoware/ml_planner/preprocessing/items/agent.hpp"
#include "autoware/ml_planner/preprocessing/items/ego_history.hpp"
#include "autoware/ml_planner/preprocessing/items/ego_shape.hpp"
#include "autoware/ml_planner/preprocessing/items/goal_pose.hpp"
#include "autoware/ml_planner/preprocessing/items/turn_indicators.hpp"
#include "autoware/ml_planner/utils/utils.hpp"

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{
std::unique_ptr<LaneSegmentContext> build_map_context(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr, double line_string_max_step_m)
{
  return std::make_unique<LaneSegmentContext>(lanelet_map_ptr, line_string_max_step_m);
}

InputBuilderResult create_input_data_map(
  const FrameInputs & frame_inputs, const LaneSegmentContext & map_context,
  const VehicleSpec & vehicle_spec, const InputBuilderParams & params)
{
  if (frame_inputs.ego_history.empty()) {
    return tl::unexpected(std::string{"Ego history is empty"});
  }
  if (frame_inputs.route.segments.empty()) {
    return tl::unexpected(std::string{"Route has no segments"});
  }

  TensorMap input_data_map;

  const nav_msgs::msg::Odometry & kinematic_state = frame_inputs.ego_history.back();
  const geometry_msgs::msg::Pose & pose_center = kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_center);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);
  const auto center_x = static_cast<float>(pose_center.position.x);
  const auto center_y = static_cast<float>(pose_center.position.y);
  const auto center_z = static_cast<float>(pose_center.position.z);

  // Ego history
  input_data_map["ego_agent_past"] = create_ego_history(
    frame_inputs.ego_history, EGO_HISTORY_SHAPE[1], map_to_ego_transform, frame_inputs.frame_time);

  // Neighbor agents on ego reference frame. Return this exact selection so
  // callers can preserve the tensor row-to-object correspondence.
  auto selected_agents = select_current_agents(
    frame_inputs.objects_history, frame_inputs.frame_time, map_to_ego_transform, MAX_NUM_NEIGHBORS);
  {
    input_data_map["neighbor_agents_past"] = create_neighbor_agent_sequence(
      frame_inputs.objects_history, selected_agents, frame_inputs.frame_time, map_to_ego_transform,
      MAX_NUM_NEIGHBORS, INPUT_T + 1, constants::PREDICTION_TIME_STEP_S,
      AgentSequenceDirection::Past);
    input_data_map["agent_shape"] = create_agent_shape(selected_agents, MAX_NUM_NEIGHBORS);
    input_data_map["agent_label"] = create_agent_label(selected_agents, MAX_NUM_NEIGHBORS);
  }

  // Map data on ego reference frame, with the traffic light state history of
  // each selected lane segment
  {
    const std::vector<int64_t> segment_indices = map_context.select_lane_segment_indices(
      map_to_ego_transform, center_x, center_y, NUM_SEGMENTS_IN_LANE);
    auto [lanes, lane_types, lanes_speed_limit] = map_context.create_tensor_data_from_indices(
      map_to_ego_transform, segment_indices, NUM_SEGMENTS_IN_LANE);
    input_data_map["lanes"] = std::move(lanes);
    input_data_map["lane_types"] = std::move(lane_types);
    input_data_map["lanes_speed_limit"] = std::move(lanes_speed_limit);

    input_data_map["lane_traffic_light_past"] = create_traffic_light_past(
      frame_inputs.traffic_signals_history, map_context.get_traffic_light_ids(segment_indices),
      NUM_SEGMENTS_IN_LANE, frame_inputs.frame_time, INPUT_T_WITH_CURRENT,
      constants::PREDICTION_TIME_STEP_S, params.traffic_light_group_msg_timeout_seconds);
    input_data_map["lane_traffic_light_future"] =
      infer_traffic_light_future(input_data_map.at("lane_traffic_light_past"));
  }

  // Route data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = map_context.select_route_segment_indices(
      frame_inputs.route, center_x, center_y, center_z, NUM_SEGMENTS_IN_ROUTE);
    auto [route_lanes, route_lane_types, route_lanes_speed_limit] =
      map_context.create_tensor_data_from_indices(
        map_to_ego_transform, segment_indices, NUM_SEGMENTS_IN_ROUTE);
    input_data_map["route_lanes"] = std::move(route_lanes);
    input_data_map["route_lane_types"] = std::move(route_lane_types);
    input_data_map["route_lanes_speed_limit"] = std::move(route_lanes_speed_limit);

    input_data_map["route_traffic_light_past"] = create_traffic_light_past(
      frame_inputs.traffic_signals_history, map_context.get_traffic_light_ids(segment_indices),
      NUM_SEGMENTS_IN_ROUTE, frame_inputs.frame_time, INPUT_T_WITH_CURRENT,
      constants::PREDICTION_TIME_STEP_S, params.traffic_light_group_msg_timeout_seconds);
    input_data_map["route_traffic_light_future"] =
      infer_traffic_light_future(input_data_map.at("route_traffic_light_past"));
  }

  // Intersection areas, stop lines, and road borders
  input_data_map["intersection_area"] =
    map_context.create_intersection_area_tensor(map_to_ego_transform, center_x, center_y);
  input_data_map["stop_lines"] =
    map_context.create_stop_line_tensor(map_to_ego_transform, center_x, center_y);
  input_data_map["road_borders"] =
    map_context.create_road_border_tensor(map_to_ego_transform, center_x, center_y);

  input_data_map["goal_pose"] =
    create_goal_pose(frame_inputs.route.goal_pose, map_to_ego_transform);

  // Ego shape
  input_data_map["ego_shape"] = create_ego_shape(
    vehicle_spec.base_link_to_front, vehicle_spec.vehicle_length, vehicle_spec.vehicle_width);

  input_data_map["turn_indicators"] = create_turn_indicators(
    frame_inputs.turn_indicators_history, frame_inputs.frame_time, INPUT_T_WITH_CURRENT,
    constants::PREDICTION_TIME_STEP_S);

  return InputBuilderOutput{std::move(input_data_map), std::move(selected_agents)};
}

}  // namespace autoware::ml_planner::preprocess
