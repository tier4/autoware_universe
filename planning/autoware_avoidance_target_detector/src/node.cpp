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

#include "autoware/avoidance_target_detector/node.hpp"

#include <memory>
#include <utility>

namespace autoware::avoidance_target_detector
{

/**
 * @brief Construct the avoidance target detector node.
 * @param node_options Node options for component loading.
 */
AvoidanceTargetDetectorNode::AvoidanceTargetDetectorNode(const rclcpp::NodeOptions & node_options)
: Node{"avoidance_target_detector", node_options},
  sub_objects_{create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&AvoidanceTargetDetectorNode::on_objects, this, std::placeholders::_1))},
  pub_avoidance_targets_{create_publisher<PredictedObjects>("~/output/avoidance_targets", 1)},
  pub_drivable_area_path_{create_publisher<Path>("~/output/drivable_area", 1)},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()}
{
}

/**
 * @brief Callback for incoming predicted objects.
 * @param msg Predicted objects message.
 */
void AvoidanceTargetDetectorNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  bool map_or_route_updated = false;

  if (const auto route_msg = sub_route_.take_data()) {
    if (!route_msg->segments.empty()) {
      route_ = route_msg;
      map_or_route_updated = true;
      RCLCPP_INFO(get_logger(), "Received route");
    }
  }

  if (const auto map_msg = sub_lanelet_map_.take_data()) {
    map_bin_ = map_msg;
    map_or_route_updated = true;
    RCLCPP_INFO(get_logger(), "Received lanelet map");
  }

  if (map_or_route_updated && route_ && map_bin_) {
    create_map(*map_bin_, *route_, route_handler_, routing_graph_);
    cached_drivable_area_.reset();
  }

  if (!route_ || !route_handler_ || !routing_graph_) {
    return;
  }

  const auto current_time = get_clock()->now();

  trajectory_ = sub_trajectory_.take_data();
  const Trajectory trajectory_msg = trajectory_ ? *trajectory_ : Trajectory{};

  if (!trajectory_ || !route_handler_->isHandlerReady()) {
    RCLCPP_WARN(get_logger(), "Data is not ready");
    return;
  }

  if (
    auto new_drivable_area = create_drivable_area(
      *trajectory_, vehicle_info_, *route_, *route_handler_, *routing_graph_)) {
    cached_drivable_area_ = std::move(new_drivable_area);
  }
  if (cached_drivable_area_) {
    pub_drivable_area_path_->publish(to_path_msg(*cached_drivable_area_, *trajectory_));
  }

  const auto avoidance_targets = get_avoidance_targets(
    current_time, *msg, trajectory_msg, object_filters_, cached_drivable_area_);

  pub_avoidance_targets_->publish(avoidance_targets);
}

}  // namespace autoware::avoidance_target_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::avoidance_target_detector::AvoidanceTargetDetectorNode)
