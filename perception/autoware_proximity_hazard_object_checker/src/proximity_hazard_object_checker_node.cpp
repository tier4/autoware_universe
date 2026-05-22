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

namespace
{
// Vehicle reference frame for TF lookup and internal geometry. The output header's
// frame_id is propagated from the input PredictedObjects, not this frame.
constexpr auto kBaseFrame = "base_link";
}  // namespace

ProximityHazardObjectCheckerNode::ProximityHazardObjectCheckerNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("proximity_hazard_object_checker", options)
{
  param_listener_ =
    std::make_shared<proximity_hazard_object::ParamListener>(get_node_parameters_interface());

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  impl_ = std::make_unique<ProximityHazardObjectChecker>(
    param_listener_->get_params(), vehicle_info.createFootprint());

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&ProximityHazardObjectCheckerNode::on_objects, this, std::placeholders::_1));

  pub_hazards_ = create_publisher<ProximityHazardObjects>(
    "~/output/proximity_hazards", rclcpp::QoS{1}.reliable());
}

void ProximityHazardObjectCheckerNode::on_objects(PredictedObjects::ConstSharedPtr msg)
{
  geometry_msgs::msg::TransformStamped to_base_link;
  try {
    to_base_link = tf_buffer_->lookupTransform(
      kBaseFrame, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "TF lookup %s -> %s failed: %s",
      msg->header.frame_id.c_str(), kBaseFrame, ex.what());
    return;
  }

  pub_hazards_->publish(impl_->process(*msg, to_base_link));
}

}  // namespace autoware::proximity_hazard_object_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::proximity_hazard_object_checker::ProximityHazardObjectCheckerNode)
