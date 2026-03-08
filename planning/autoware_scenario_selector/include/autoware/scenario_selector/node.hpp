// Copyright 2020 Tier IV, Inc.
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

#ifndef AUTOWARE__SCENARIO_SELECTOR__NODE_HPP_
#define AUTOWARE__SCENARIO_SELECTOR__NODE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_internal_planning_msgs/msg/scenario.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_utils/system/stop_watch.hpp>

#include <deque>
#include <memory>
#include <string>

namespace autoware::scenario_selector
{
class ScenarioSelectorNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit ScenarioSelectorNode(const rclcpp::NodeOptions & node_options);

  void onOdom(AUTOWARE_MESSAGE_SHARED_PTR(const nav_msgs::msg::Odometry) msg);
  bool isDataReady();
  void onTimer();
  void onMap(AUTOWARE_MESSAGE_UNIQUE_PTR(autoware_map_msgs::msg::LaneletMapBin) msg);
  void onRoute(AUTOWARE_MESSAGE_UNIQUE_PTR(autoware_planning_msgs::msg::LaneletRoute) msg);
  void onLaneDrivingTrajectory(
    AUTOWARE_MESSAGE_UNIQUE_PTR(autoware_planning_msgs::msg::Trajectory) msg);
  void onParkingTrajectory(
    AUTOWARE_MESSAGE_UNIQUE_PTR(autoware_planning_msgs::msg::Trajectory) msg);
  void publishTrajectory(
    const autoware_planning_msgs::msg::Trajectory & msg);

  void updateCurrentScenario();
  std::string selectScenarioByPosition();
  const autoware_planning_msgs::msg::Trajectory * getScenarioTrajectory(
    const std::string & scenario);

  void updateData();

private:
  bool isAutonomous() const;
  bool isEmptyParkingTrajectory() const;
  bool isSwitchToLaneDriving();
  bool isSwitchToParking(const bool is_stopped);

  inline bool isCurrentLaneDriving() const
  {
    return current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::LANEDRIVING;
  }

  inline bool isCurrentParking() const
  {
    return current_scenario_ == autoware_internal_planning_msgs::msg::Scenario::PARKING;
  }

  autoware::agnocast_wrapper::Timer::SharedPtr timer_;

  // subscribers
  AUTOWARE_SUBSCRIPTION_PTR(autoware_map_msgs::msg::LaneletMapBin) sub_lanelet_map_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_planning_msgs::msg::LaneletRoute) sub_route_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_planning_msgs::msg::Trajectory)
    sub_lane_driving_trajectory_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_planning_msgs::msg::Trajectory) sub_parking_trajectory_;
  AUTOWARE_ALL_POLLING_SUBSCRIBER_PTR(nav_msgs::msg::Odometry) sub_odom_;

  // publishers
  AUTOWARE_PUBLISHER_PTR(autoware_planning_msgs::msg::Trajectory) pub_trajectory_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_planning_msgs::msg::Scenario) pub_scenario_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped)
    pub_processing_time_;

  // polling subscribers
  AUTOWARE_POLLING_SUBSCRIBER_PTR(std_msgs::msg::Bool) sub_parking_state_;
  AUTOWARE_POLLING_SUBSCRIBER_PTR(autoware_adapi_v1_msgs::msg::OperationModeState)
    sub_operation_mode_state_;

  AUTOWARE_MESSAGE_SHARED_PTR(const autoware_adapi_v1_msgs::msg::OperationModeState)
    operation_mode_state_;
  AUTOWARE_MESSAGE_SHARED_PTR(const autoware_planning_msgs::msg::Trajectory)
    lane_driving_trajectory_;
  AUTOWARE_MESSAGE_SHARED_PTR(const autoware_planning_msgs::msg::Trajectory) parking_trajectory_;
  AUTOWARE_MESSAGE_SHARED_PTR(const autoware_planning_msgs::msg::LaneletRoute) route_;
  AUTOWARE_MESSAGE_SHARED_PTR(const nav_msgs::msg::Odometry) current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  std::string current_scenario_;
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer_;

  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;
  std::unique_ptr<
    autoware_utils::BasicPublishedTimePublisher<autoware::agnocast_wrapper::Node>>
    published_time_publisher_;

  // Parameters
  double update_rate_;
  double th_max_message_delay_sec_;
  double th_arrived_distance_m_;
  double th_stopped_time_sec_;
  double th_stopped_velocity_mps_;
  bool enable_mode_switching_;
  bool is_parking_completed_;

  boost::optional<rclcpp::Time> lane_driving_stop_time_;
  boost::optional<rclcpp::Time> empty_parking_trajectory_time_;

  static constexpr double lane_stopping_timeout_s = 5.0;
  static constexpr double empty_parking_trajectory_timeout_s = 3.0;

  // processing time
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
};
}  // namespace autoware::scenario_selector
#endif  // AUTOWARE__SCENARIO_SELECTOR__NODE_HPP_
