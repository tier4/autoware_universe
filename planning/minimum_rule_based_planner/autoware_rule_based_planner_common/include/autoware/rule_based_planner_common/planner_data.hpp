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

#ifndef AUTOWARE__RULE_BASED_PLANNER_COMMON__PLANNER_DATA_HPP_
#define AUTOWARE__RULE_BASED_PLANNER_COMMON__PLANNER_DATA_HPP_

#include <autoware/vehicle_info_utils/vehicle_info.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>
#include <string>

namespace autoware::rule_based_planner
{

/**
 * @brief Shared data structure passed to all planner modules
 *
 * Contains map data, route information, ego vehicle state, and sensor data
 * that modules may need for planning decisions.
 */
struct PlannerData
{
  // Map and route data
  lanelet::LaneletMapPtr lanelet_map_ptr{nullptr};
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr{nullptr};
  lanelet::routing::RoutingGraphPtr routing_graph_ptr{nullptr};

  std::string route_frame_id{};
  geometry_msgs::msg::Pose goal_pose{};

  lanelet::ConstLanelets route_lanelets{};
  lanelet::ConstLanelets preferred_lanelets{};
  lanelet::ConstLanelets start_lanelets{};
  lanelet::ConstLanelets goal_lanelets{};

  // Ego vehicle state
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry{nullptr};
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_acceleration{nullptr};

  // Vehicle info
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;

  // Perception data (optional - modules can check for nullptr)
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr predicted_objects{nullptr};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud{nullptr};
};

}  // namespace autoware::rule_based_planner

#endif  // AUTOWARE__RULE_BASED_PLANNER_COMMON__PLANNER_DATA_HPP_
