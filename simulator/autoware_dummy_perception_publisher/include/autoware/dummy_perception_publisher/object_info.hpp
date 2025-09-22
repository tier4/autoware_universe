// Copyright 2025 Tier IV, Inc.
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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__OBJECT_INFO_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__OBJECT_INFO_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
namespace autoware::dummy_perception_publisher
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::TrackedObject;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TwistWithCovariance;
using tier4_simulation_msgs::msg::DummyObject;

struct ObjectInfo
{
  ObjectInfo(const DummyObject & object, const rclcpp::Time & current_time);
  ObjectInfo(
    const DummyObject & object, const PredictedObject & predicted_object,
    const rclcpp::Time & predicted_time, const rclcpp::Time & current_time,
    const double switch_time_threshold);

  // Position calculation methods
  static Pose calculateStraightLinePosition(
    const DummyObject & object, const rclcpp::Time & current_time);
  static Pose calculateTrajectoryBasedPosition(
    const DummyObject & object, const PredictedObject & predicted_object,
    const rclcpp::Time & predicted_time, const rclcpp::Time & current_time);
  static void stopAtZeroVelocity(double & current_vel, double initial_vel, double initial_acc);
  double length;
  double width;
  double height;
  double std_dev_x;
  double std_dev_y;
  double std_dev_z;
  double std_dev_yaw;
  tf2::Transform tf_map2moved_object;
  // pose and twist
  TwistWithCovariance twist_covariance_;
  PoseWithCovariance pose_covariance_;
  // convert to TrackedObject
  // (todo) currently need object input to get id and header information, but it should be removed
  [[nodiscard]] autoware_perception_msgs::msg::TrackedObject toTrackedObject(
    const DummyObject & object) const;
};
}  // namespace autoware::dummy_perception_publisher

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__OBJECT_INFO_HPP_
