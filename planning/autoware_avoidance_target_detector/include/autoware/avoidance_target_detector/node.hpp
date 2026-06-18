// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__NODE_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__NODE_HPP_

#include "autoware/avoidance_target_detector/drivable_area.hpp"
#include "autoware/avoidance_target_detector/impl.hpp"
#include "autoware/avoidance_target_detector/traffic_rules.hpp"
#include "autoware/avoidance_target_detector/two_class_filter.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>

#include <lanelet2_routing/RoutingGraph.h>

#include <map>
#include <memory>
#include <optional>

namespace autoware::avoidance_target_detector
{

using autoware::route_handler::RouteHandler;
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;

/** ROS 2 node that detects and publishes avoidance target objects. */
class AvoidanceTargetDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the avoidance target detector node.
   * @param node_options Node options for component loading.
   */
  explicit AvoidanceTargetDetectorNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Callback for incoming predicted objects.
   * @param msg Predicted objects message.
   */
  void on_objects(const PredictedObjects::ConstSharedPtr msg);

  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  autoware_utils::InterProcessPollingSubscriber<Trajectory> sub_trajectory_{
    this, "~/input/trajectory"};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    sub_route_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    sub_lanelet_map_{this, "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local()};

  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_avoidance_targets_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_debug_avoidance_targets_;
  rclcpp::Publisher<Path>::SharedPtr pub_drivable_area_path_;

  std::shared_ptr<RouteHandler> route_handler_;
  lanelet::routing::RoutingGraphConstPtr goal_purpose_routing_graph_;
  VehicleInfo vehicle_info_;
  LongitudinalDistanceFilterParams longitudinal_filter_params_;
  LateralDistanceFilterParams lateral_filter_params_;

  FilterManagerMap object_filters_;

  Trajectory::ConstSharedPtr trajectory_;
  LaneletRoute::ConstSharedPtr route_;
  std::optional<DrivableAreaResult> cached_drivable_area_;
};

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__NODE_HPP_
