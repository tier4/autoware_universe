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

#ifndef GOAL_SELECTOR_NODE_HPP_
#define GOAL_SELECTOR_NODE_HPP_

#include "types.hpp"

#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace autoware::goal_selector
{

class GoalSelectorNode : public rclcpp::Node
{
public:
  explicit GoalSelectorNode(const rclcpp::NodeOptions & options);

private:
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;
  using RouteState = autoware_planning_msgs::msg::RouteState;

  geometry_msgs::msg::Pose declare_pose(const std::string & name);
  void load_triggers();

  void on_route_state(const RouteState::ConstSharedPtr msg);
  void on_objects(const PredictedObjects::ConstSharedPtr msg);
  void publish_debug_marker();
  void set_next_goal();
  const Trigger * find_trigger(const geometry_msgs::msg::Point & position) const;
  const GoalCandidate * select_goal(const Trigger & trigger) const;
  void publish_goal(const geometry_msgs::msg::Pose & goal);

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Parameter
  double goal_footprint_margin_;  // [m]
  std::vector<Trigger> triggers_;

  // State
  LaneletRoute::ConstSharedPtr route_;
  PredictedObjects::ConstSharedPtr objects_;
  bool is_arrived_{false};
  bool is_next_goal_set_{false};

  // Subscriber
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;

  // Publisher
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_;
};

}  // namespace autoware::goal_selector

#endif  // GOAL_SELECTOR_NODE_HPP_
