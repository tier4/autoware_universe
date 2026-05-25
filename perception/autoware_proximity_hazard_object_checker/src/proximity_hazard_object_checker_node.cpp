// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "autoware/proximity_hazard_object_checker/proximity_hazard_object_checker_node.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>

namespace autoware::proximity_hazard_object_checker
{
ProximityHazardObjectCheckerNode::ProximityHazardObjectCheckerNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("proximity_hazard_object_checker", options)
{
  param_listener_ =
    std::make_shared<proximity_hazard_object::ParamListener>(get_node_parameters_interface());

  const auto node_ms = 1 / param_listener_->get_params().node_hz;
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(node_ms),
    std::bind(&ProximityHazardObjectCheckerNode::on_timer, this));

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  impl_ = std::make_unique<ProximityHazardObjectChecker>(
    param_listener_->get_params(), vehicle_info.createFootprint());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pub_hazards_ = create_publisher<ProximityHazardObjects>(
    "~/output/proximity_hazards", rclcpp::QoS{1}.reliable());
}

void ProximityHazardObjectCheckerNode::on_timer()
{
  const auto object_ptr = sub_objects_.take_data();

  if (!object_ptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to take predicted objects data");
    return;
  }
  const auto odometry_ptr = sub_odometry_.take_data();
  if (!object_ptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to take predicted odometry data");
    return;
  }

  geometry_msgs::msg::TransformStamped to_base_link;
  try {
    to_base_link = tf_buffer_->lookupTransform(
      odometry_ptr->child_frame_id, object_ptr->header.frame_id, object_ptr->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "TF lookup %s -> %s failed: %s",
      object_ptr->header.frame_id.c_str(), odometry_ptr->child_frame_id.c_str(), ex.what());
    return;
  }

  pub_hazards_->publish(impl_->process(*object_ptr, to_base_link));
}

}  // namespace autoware::proximity_hazard_object_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::proximity_hazard_object_checker::ProximityHazardObjectCheckerNode)
