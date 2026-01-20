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

#ifndef AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__INPUT_DATA_BUILDER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__INPUT_DATA_BUILDER_HPP_

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <Eigen/Dense>
#include <rclcpp/time.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <map>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{

using autoware_perception_msgs::msg::TrackedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using preprocess::TrafficSignalStamped;

struct SensorMsgs
{
  std::vector<TrackedObjects::ConstSharedPtr> tracked_objects;
  std::vector<Odometry::ConstSharedPtr> ego_kinematic_states;
  std::vector<AccelWithCovarianceStamped::ConstSharedPtr> ego_accelerations;
  std::vector<TrafficLightGroupArray::ConstSharedPtr> traffic_signals;
  std::vector<TurnIndicatorsReport::ConstSharedPtr> turn_indicators;
};

struct HistoricalData
{
  HistoricalData() = default;
  HistoricalData(bool ignore_unknown_neighbors, double traffic_light_group_msg_timeout_seconds);

  void update_params(bool ignore_unknown_neighbors, double traffic_light_group_msg_timeout_seconds);

  ::autoware::diffusion_planner::UpdateResult update_from_sensor_msgs(
    const SensorMsgs & sensor_msgs, const rclcpp::Time & now,
    double max_msg_time_gap_seconds);

  std::deque<nav_msgs::msg::Odometry> ego_history;
  std::deque<TurnIndicatorsReport> turn_indicators_history;
  AgentData agent_data;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map;
  bool ignore_unknown_neighbors{false};
  double traffic_light_group_msg_timeout_seconds{0.0};
};

struct FrameContext
{
  const SensorMsgs sensor_msgs;
  const HistoricalData & historical_data;
  const std::vector<AgentHistory> ego_centric_neighbor_histories;
  const Eigen::Matrix4d ego_to_map_transform;
  const Eigen::Matrix4d map_to_ego_transform;
};

struct VehicleSize
{
  double wheel_base_m;
  double length_m;
  double width_m;
};

InputDataMap create_input_data(
  const FrameContext & frame_context, const preprocess::LaneSegmentContext & lane_segment_context,
  const LaneletRoute::ConstSharedPtr & route_ptr, const VehicleSize & vehicle_size);

}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__INPUT_DATA_BUILDER_HPP_
