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

#ifndef COLLISION_CHECKER__NODE_HPP_
#define COLLISION_CHECKER__NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <utility>

namespace collision_checker
{

using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::Shape;
using vehicle_info_util::VehicleInfo;

using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

class CollisionCheckerNode : public rclcpp::Node
{
public:
  explicit CollisionCheckerNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    bool use_pointcloud;
    bool use_dynamic_object;
    double collision_distance;
  };

private:
  void checkCollision(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void onDynamicObjects(const PredictedObjects::ConstSharedPtr msg);

  // void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  boost::optional<Obstacle> getNearestObstacle() const;

  boost::optional<Obstacle> getNearestObstacleByPointCloud() const;

  boost::optional<Obstacle> getNearestObstacleByDynamicObject() const;

  boost::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  // ros
  mutable tf2_ros::Buffer tf_buffer_{get_clock()};
  mutable tf2_ros::TransformListener tf_listener_{tf_buffer_};
  rclcpp::TimerBase::SharedPtr timer_;

  // publisher and subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_dynamic_objects_;

  // parameter
  NodeParam node_param_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  // data
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr_;
  PredictedObjects::ConstSharedPtr object_ptr_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;
};
}  // namespace collision_checker

#endif  // COLLISION_CHECKER__NODE_HPP_
