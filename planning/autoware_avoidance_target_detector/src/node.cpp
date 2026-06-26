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
  pub_drivable_area_path_{create_publisher<Path>("~/output/drivable_area", 1)}
{
  declare_parameter<bool>("use_extended_route_bounds", true);
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
    }
  }

  if (const auto map_msg = sub_lanelet_map_.take_data()) {
    map_bin_ = map_msg;
    map_or_route_updated = true;
  }

  if (map_or_route_updated && route_ && map_bin_) {
    extended_route_handler_ = std::make_shared<ExtendedRouteHandler>(*map_bin_, *route_);
    extended_route_handler_->create_map();
    extended_route_handler_->export_debug_map();
  }

  if (!route_ || !extended_route_handler_) {
    return;
  }

  trajectory_ = sub_trajectory_.take_data();
  const Trajectory trajectory_msg = trajectory_ ? *trajectory_ : Trajectory{};

  if (!trajectory_ || !extended_route_handler_->getOriginalRouteHandler()->isHandlerReady()) {
    return;
  }

  const auto use_extended_bounds = get_parameter("use_extended_route_bounds").as_bool();
  const auto & route_bounds = use_extended_bounds
                                ? extended_route_handler_->get_extended_route_bounds()
                                : extended_route_handler_->get_original_route_bounds();
  pub_drivable_area_path_->publish(to_path_msg(route_bounds, *trajectory_));

  const auto avoidance_targets =
    object_selector_.get_avoidance_targets(get_clock()->now(), *msg, trajectory_msg, route_bounds);

  pub_avoidance_targets_->publish(avoidance_targets);
}

}  // namespace autoware::avoidance_target_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::avoidance_target_detector::AvoidanceTargetDetectorNode)
