// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_NODE_HPP_
#define AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_NODE_HPP_

#include "autoware/proximity_hazard_object_checker/proximity_hazard_object_checker.hpp"

#include <autoware_proximity_hazard_object_checker/msg/proximity_hazard_objects.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace autoware::proximity_hazard_object_checker
{

using autoware_perception_msgs::msg::PredictedObjects;
using autoware_proximity_hazard_object_checker::msg::ProximityHazardObjects;

class ProximityHazardObjectCheckerNode : public rclcpp::Node
{
public:
  explicit ProximityHazardObjectCheckerNode(const rclcpp::NodeOptions & options);

private:
  void on_objects(PredictedObjects::ConstSharedPtr msg);

  // I/O
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Publisher<ProximityHazardObjects>::SharedPtr pub_hazards_;

  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameter listener (generate_parameter_library).
  std::shared_ptr<proximity_hazard_object::ParamListener> param_listener_;

  // The actual logic.
  std::unique_ptr<ProximityHazardObjectChecker> impl_;
};

}  // namespace autoware::proximity_hazard_object_checker

#endif  // AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_NODE_HPP_
