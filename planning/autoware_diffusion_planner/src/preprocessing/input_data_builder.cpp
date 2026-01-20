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

#include "autoware/diffusion_planner/preprocessing/input_data_builder.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
HistoricalData::HistoricalData(
  bool ignore_unknown_neighbors_param, double traffic_light_group_msg_timeout_seconds_param)
: ignore_unknown_neighbors(ignore_unknown_neighbors_param),
  traffic_light_group_msg_timeout_seconds(traffic_light_group_msg_timeout_seconds_param)
{
}

void HistoricalData::update_params(
  bool ignore_unknown_neighbors_param, double traffic_light_group_msg_timeout_seconds_param)
{
  ignore_unknown_neighbors = ignore_unknown_neighbors_param;
  traffic_light_group_msg_timeout_seconds = traffic_light_group_msg_timeout_seconds_param;
}

void HistoricalData::update_from_sensor_msgs(
  const SensorMsgs & sensor_msgs, const rclcpp::Time & now)
{
  ego_history.push_back(*sensor_msgs.ego_kinematic_states.back());
  if (ego_history.size() > static_cast<size_t>(EGO_HISTORY_SHAPE[1])) {
    ego_history.pop_front();
  }

  turn_indicators_history.push_back(*sensor_msgs.turn_indicators.back());
  if (turn_indicators_history.size() > static_cast<size_t>(TURN_INDICATORS_SHAPE[1])) {
    turn_indicators_history.pop_front();
  }

  agent_data.update_histories(*sensor_msgs.tracked_objects.back(), ignore_unknown_neighbors);

  preprocess::process_traffic_signals(
    sensor_msgs.traffic_signals, traffic_light_id_map, now,
    traffic_light_group_msg_timeout_seconds);
}

InputDataMap create_input_data(
  const FrameContext & frame_context, const preprocess::LaneSegmentContext & lane_segment_context,
  const LaneletRoute::ConstSharedPtr & route_ptr, const VehicleSize & vehicle_size)
{
  InputDataMap input_data_map;

  Odometry kinematic_state = *(frame_context.sensor_msgs.ego_kinematic_states.back());
  const geometry_msgs::msg::Pose & pose_center = kinematic_state.pose.pose;
  const rclcpp::Time frame_time(kinematic_state.header.stamp);
  const Eigen::Matrix4d & map_to_ego_transform = frame_context.map_to_ego_transform;

  // Ego history
  {
    const std::vector<float> single_ego_agent_past = preprocess::create_ego_agent_past(
      frame_context.historical_data.ego_history, EGO_HISTORY_SHAPE[1], map_to_ego_transform,
      frame_time);
    input_data_map["ego_agent_past"] = single_ego_agent_past;
  }
  // Ego state
  {
    const std::vector<float> ego_current_state = preprocess::create_ego_current_state(
      kinematic_state, *(frame_context.sensor_msgs.ego_accelerations.back()),
      static_cast<float>(vehicle_size.wheel_base_m));
    input_data_map["ego_current_state"] = ego_current_state;
  }
  // Agent data on ego reference frame
  {
    const auto neighbor_agents_past = preprocess::create_neighbor_agents_past(
      frame_context.ego_centric_neighbor_histories, MAX_NUM_NEIGHBORS, INPUT_T + 1, frame_time);
    input_data_map["neighbor_agents_past"] = neighbor_agents_past;
  }
  // Static objects
  // TODO(Daniel): add static objects
  {
    std::vector<int64_t> single_batch_shape(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end());
    auto static_objects_data = utils::create_float_data(single_batch_shape, 0.0f);
    input_data_map["static_objects"] = static_objects_data;
  }

  // map data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = lane_segment_context.select_lane_segment_indices(
      map_to_ego_transform, pose_center.position, NUM_SEGMENTS_IN_LANE);
    const auto [lanes, lanes_speed_limit] = lane_segment_context.create_tensor_data_from_indices(
      map_to_ego_transform, frame_context.historical_data.traffic_light_id_map, segment_indices,
      NUM_SEGMENTS_IN_LANE);
    input_data_map["lanes"] = lanes;
    input_data_map["lanes_speed_limit"] = lanes_speed_limit;
  }

  // route data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = lane_segment_context.select_route_segment_indices(
      *route_ptr, pose_center.position, NUM_SEGMENTS_IN_ROUTE);
    const auto [route_lanes, route_lanes_speed_limit] =
      lane_segment_context.create_tensor_data_from_indices(
        map_to_ego_transform, frame_context.historical_data.traffic_light_id_map, segment_indices,
        NUM_SEGMENTS_IN_ROUTE);
    input_data_map["route_lanes"] = route_lanes;
    input_data_map["route_lanes_speed_limit"] = route_lanes_speed_limit;
  }

  // polygons
  {
    const auto & polygons =
      lane_segment_context.create_polygon_tensor(map_to_ego_transform, pose_center.position);
    input_data_map["polygons"] = polygons;
  }

  // line strings
  {
    const auto & line_strings =
      lane_segment_context.create_line_string_tensor(map_to_ego_transform, pose_center.position);
    input_data_map["line_strings"] = line_strings;
  }

  // goal pose
  {
    const auto & goal_pose = route_ptr->goal_pose;

    // Convert goal pose to 4x4 transformation matrix
    const Eigen::Matrix4d goal_pose_map_4x4 = utils::pose_to_matrix4f(goal_pose);

    // Transform to ego frame
    const Eigen::Matrix4d goal_pose_ego_4x4 = map_to_ego_transform * goal_pose_map_4x4;

    // Extract relative position
    const float x = goal_pose_ego_4x4(0, 3);
    const float y = goal_pose_ego_4x4(1, 3);

    // Extract heading as cos/sin from rotation matrix
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(goal_pose_ego_4x4.block<3, 3>(0, 0));

    std::vector<float> single_goal_pose = {x, y, cos_yaw, sin_yaw};
    input_data_map["goal_pose"] = single_goal_pose;
  }

  // ego shape
  {
    const float wheel_base = static_cast<float>(vehicle_size.wheel_base_m);
    const float vehicle_length = static_cast<float>(vehicle_size.length_m);
    const float vehicle_width = static_cast<float>(vehicle_size.width_m);
    std::vector<float> single_ego_shape = {wheel_base, vehicle_length, vehicle_width};
    input_data_map["ego_shape"] = single_ego_shape;
  }

  // turn indicators
  {
    const std::vector<float> single_turn_indicators = preprocess::create_turn_indicators_past(
      frame_context.historical_data.turn_indicators_history, INPUT_T + 1, frame_time);
    input_data_map["turn_indicators"] = single_turn_indicators;
  }

  return input_data_map;
}
}  // namespace autoware::diffusion_planner::preprocess
