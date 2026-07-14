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

#include "autoware/avoidance_target_detector/avoidance_target_detector_logic.hpp"
#include "autoware/avoidance_target_detector/node.hpp"

namespace autoware::avoidance_target_detector
{

AvoidanceTargetDetectorNode::AvoidanceTargetDetectorNode(const rclcpp::NodeOptions & node_options)
: Node{"avoidance_target_detector", node_options},
  pub_drivable_area_path_{create_publisher<Path>("~/output/drivable_area", 1)},
  pub_near_segment_polygon_{
    create_publisher<MarkerArray>("~/debug/near_segment_polygon", rclcpp::QoS{1}.transient_local())}
{
  declare_parameter<bool>("use_extended_route_bounds", true);
  declare_parameter<bool>("use_tracked_objects", false);
  use_tracked_objects_ = get_parameter("use_tracked_objects").as_bool();

  logic_ = std::make_unique<AvoidanceTargetDetectorLogic>(
    get_parameter("use_extended_route_bounds").as_bool());

  if (use_tracked_objects_) {
    sub_tracked_objects_ = create_subscription<TrackedObjects>(
      "~/input/tracked_objects", rclcpp::QoS{1},
      std::bind(&AvoidanceTargetDetectorNode::on_tracked_objects, this, std::placeholders::_1));
    pub_tracked_avoidance_targets_ =
      create_publisher<TrackedObjects>("~/output/tracked_avoidance_targets", 1);
    pub_tracked_driving_along_vehicles_ =
      create_publisher<TrackedObjects>("~/output/tracked_driving_along_vehicles", 1);
  } else {
    sub_objects_ = create_subscription<PredictedObjects>(
      "~/input/objects", rclcpp::QoS{1},
      std::bind(&AvoidanceTargetDetectorNode::on_objects, this, std::placeholders::_1));
    pub_avoidance_targets_ = create_publisher<PredictedObjects>("~/output/avoidance_targets", 1);
    pub_driving_along_vehicles_ =
      create_publisher<PredictedObjects>("~/output/driving_along_vehicles", 1);
  }
}

void AvoidanceTargetDetectorNode::update_map_route_from_subscribers()
{
  if (const auto route_msg = sub_route_.take_data()) {
    if (!route_msg->segments.empty()) {
      route_ = route_msg;
    }
  }

  if (const auto map_msg = sub_lanelet_map_.take_data()) {
    map_bin_ = map_msg;
  }

  if (route_ && map_bin_) {
    logic_->set_use_extended_route_bounds(get_parameter("use_extended_route_bounds").as_bool());
    logic_->update_map_and_route(*map_bin_, *route_);
  }
}

void AvoidanceTargetDetectorNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!msg || use_tracked_objects_) {
    return;
  }

  update_map_route_from_subscribers();

  if (!logic_->is_ready()) {
    return;
  }

  trajectory_ = sub_trajectory_.take_data();
  if (!trajectory_ || trajectory_->points.empty()) {
    return;
  }

  const auto output = logic_->process_predicted_objects(get_clock()->now(), *msg, *trajectory_);
  if (!output) {
    return;
  }

  pub_drivable_area_path_->publish(output->drivable_area);
  if (output->near_segment_polygon) {
    pub_near_segment_polygon_->publish(*output->near_segment_polygon);
  }
  pub_avoidance_targets_->publish(output->avoidance_targets);
  pub_driving_along_vehicles_->publish(output->driving_along_vehicles);
}

void AvoidanceTargetDetectorNode::on_tracked_objects(const TrackedObjects::ConstSharedPtr msg)
{
  if (!msg || !use_tracked_objects_) {
    return;
  }

  update_map_route_from_subscribers();

  if (!logic_->is_ready()) {
    return;
  }

  trajectory_ = sub_trajectory_.take_data();
  if (!trajectory_ || trajectory_->points.empty()) {
    return;
  }

  const auto output = logic_->process_tracked_objects(get_clock()->now(), *msg, *trajectory_);
  if (!output) {
    return;
  }

  pub_tracked_avoidance_targets_->publish(output->tracked_avoidance_targets);
  pub_tracked_driving_along_vehicles_->publish(output->tracked_driving_along_vehicles);
}

}  // namespace autoware::avoidance_target_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::avoidance_target_detector::AvoidanceTargetDetectorNode)
