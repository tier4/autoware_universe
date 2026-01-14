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
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <rclcpp/time.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
namespace
{
std::vector<float> replicate_for_batch(
  const std::vector<float> & single_data, const int batch_size)
{
  const size_t single_size = single_data.size();
  const size_t total_size = static_cast<size_t>(batch_size) * single_size;

  std::vector<float> batch_data;
  batch_data.reserve(total_size);

  for (int i = 0; i < batch_size; ++i) {
    batch_data.insert(batch_data.end(), single_data.begin(), single_data.end());
  }

  return batch_data;
}
}  // namespace

InputDataMap create_input_data(
  const FrameContext & frame_context, const preprocess::LaneSegmentContext & lane_segment_context,
  const LaneletRoute::ConstSharedPtr & route_ptr, const VehicleSize & vehicle_size,
  const std::vector<double> & temperature_list, int batch_size, bool shift_x)
{
  InputDataMap input_data_map;

  // random sample trajectories
  {
    for (int64_t b = 0; b < batch_size; b++) {
      const std::vector<float> sampled_trajectories =
        preprocess::create_sampled_trajectories(temperature_list[b]);
      input_data_map["sampled_trajectories"].insert(
        input_data_map["sampled_trajectories"].end(), sampled_trajectories.begin(),
        sampled_trajectories.end());
    }
  }

  Odometry kinematic_state = *(frame_context.sensor_msgs.ego_kinematic_states.back());
  if (shift_x) {
    kinematic_state.pose.pose =
      utils::shift_x(kinematic_state.pose.pose, vehicle_size.wheel_base_m / 2.0);
  }
  const geometry_msgs::msg::Pose & pose_center = kinematic_state.pose.pose;
  const rclcpp::Time frame_time(kinematic_state.header.stamp);
  const Eigen::Matrix4d & map_to_ego_transform = frame_context.map_to_ego_transform;

  // Ego history
  {
    const std::vector<float> single_ego_agent_past = preprocess::create_ego_agent_past(
      frame_context.historical_data.ego_history, EGO_HISTORY_SHAPE[1], map_to_ego_transform,
      frame_time);
    input_data_map["ego_agent_past"] = replicate_for_batch(single_ego_agent_past, batch_size);
  }
  // Ego state
  {
    const std::vector<float> ego_current_state = preprocess::create_ego_current_state(
      kinematic_state, *(frame_context.sensor_msgs.ego_accelerations.back()),
      static_cast<float>(vehicle_size.wheel_base_m));
    input_data_map["ego_current_state"] = replicate_for_batch(ego_current_state, batch_size);
  }
  // Agent data on ego reference frame
  {
    const auto neighbor_agents_past = preprocess::create_neighbor_agents_past(
      frame_context.ego_centric_neighbor_histories, MAX_NUM_NEIGHBORS, INPUT_T + 1, frame_time);
    input_data_map["neighbor_agents_past"] = replicate_for_batch(neighbor_agents_past, batch_size);
  }
  // Static objects
  // TODO(Daniel): add static objects
  {
    std::vector<int64_t> single_batch_shape(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end());
    auto static_objects_data = utils::create_float_data(single_batch_shape, 0.0f);
    input_data_map["static_objects"] = replicate_for_batch(static_objects_data, batch_size);
  }

  // map data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = lane_segment_context.select_lane_segment_indices(
      map_to_ego_transform, pose_center.position, NUM_SEGMENTS_IN_LANE);
    const auto [lanes, lanes_speed_limit] = lane_segment_context.create_tensor_data_from_indices(
      map_to_ego_transform, frame_context.historical_data.traffic_light_id_map, segment_indices,
      NUM_SEGMENTS_IN_LANE);
    input_data_map["lanes"] = replicate_for_batch(lanes, batch_size);
    input_data_map["lanes_speed_limit"] = replicate_for_batch(lanes_speed_limit, batch_size);
  }

  // route data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = lane_segment_context.select_route_segment_indices(
      *route_ptr, pose_center.position, NUM_SEGMENTS_IN_ROUTE);
    const auto [route_lanes, route_lanes_speed_limit] =
      lane_segment_context.create_tensor_data_from_indices(
        map_to_ego_transform, frame_context.historical_data.traffic_light_id_map, segment_indices,
        NUM_SEGMENTS_IN_ROUTE);
    input_data_map["route_lanes"] = replicate_for_batch(route_lanes, batch_size);
    input_data_map["route_lanes_speed_limit"] =
      replicate_for_batch(route_lanes_speed_limit, batch_size);
  }

  // polygons
  {
    const auto & polygons =
      lane_segment_context.create_polygon_tensor(map_to_ego_transform, pose_center.position);
    input_data_map["polygons"] = replicate_for_batch(polygons, batch_size);
  }

  // line strings
  {
    const auto & line_strings =
      lane_segment_context.create_line_string_tensor(map_to_ego_transform, pose_center.position);
    input_data_map["line_strings"] = replicate_for_batch(line_strings, batch_size);
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
    input_data_map["goal_pose"] = replicate_for_batch(single_goal_pose, batch_size);
  }

  // ego shape
  {
    const float wheel_base = static_cast<float>(vehicle_size.wheel_base_m);
    const float vehicle_length = static_cast<float>(vehicle_size.length_m);
    const float vehicle_width = static_cast<float>(vehicle_size.width_m);
    std::vector<float> single_ego_shape = {wheel_base, vehicle_length, vehicle_width};
    input_data_map["ego_shape"] = replicate_for_batch(single_ego_shape, batch_size);
  }

  // turn indicators
  {
    // copy from back to front, and use the front value for padding if not enough history
    std::vector<float> single_turn_indicators(INPUT_T + 1, 0.0f);
    for (int64_t t = 0; t < INPUT_T + 1; ++t) {
      const int64_t index = std::max(
        static_cast<int64_t>(frame_context.historical_data.turn_indicators_history.size()) - 1 - t,
        static_cast<int64_t>(0));
      single_turn_indicators[INPUT_T - t] =
        frame_context.historical_data.turn_indicators_history[index].report;
    }
    input_data_map["turn_indicators"] =
      replicate_for_batch(single_turn_indicators, batch_size);
  }

  return input_data_map;
}

}  // namespace autoware::diffusion_planner::preprocess
