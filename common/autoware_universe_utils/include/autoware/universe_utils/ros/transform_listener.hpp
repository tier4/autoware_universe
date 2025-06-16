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

#ifndef AUTOWARE__UNIVERSE_UTILS__ROS__TRANSFORM_LISTENER_HPP_
#define AUTOWARE__UNIVERSE_UTILS__ROS__TRANSFORM_LISTENER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>

namespace autoware::universe_utils
{
class TransformListener
{
public:
  explicit TransformListener(rclcpp::Node * node) : logger_(node->get_logger())
  {
    managed_tf_buffer_ = std::make_shared<managed_transform_buffer::ManagedTransformBuffer>(node);
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr getLatestTransform(
    const std::string & from, const std::string & to)
  {
    auto tf_opt = managed_tf_buffer_->getTransform<geometry_msgs::msg::TransformStamped>(
      from, to, tf2::TimePointZero, tf2::Duration::zero());

    if (!tf_opt) return {};

    return std::make_shared<const geometry_msgs::msg::TransformStamped>(*tf_opt);
  }

  geometry_msgs::msg::TransformStamped::ConstSharedPtr getTransform(
    const std::string & from, const std::string & to, const rclcpp::Time & time,
    const rclcpp::Duration & duration)
  {
    auto tf_opt = managed_tf_buffer_->getTransform<geometry_msgs::msg::TransformStamped>(
      from, to, time, duration);

    if (!tf_opt) return {};

    return std::make_shared<const geometry_msgs::msg::TransformStamped>(*tf_opt);
  }

  rclcpp::Logger getLogger() { return logger_; }

private:
  rclcpp::Logger logger_;
  std::shared_ptr<managed_transform_buffer::ManagedTransformBuffer> managed_tf_buffer_;
};
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__ROS__TRANSFORM_LISTENER_HPP_
