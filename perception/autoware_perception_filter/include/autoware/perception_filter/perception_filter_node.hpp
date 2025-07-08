// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_NODE_HPP_
#define AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_NODE_HPP_

#include <autoware_utils/ros/published_time_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <memory>

namespace autoware::perception_filter
{

class PerceptionFilterNode : public rclcpp::Node
{
public:
  explicit PerceptionFilterNode(const rclcpp::NodeOptions & node_options);

private:
  // Callback functions
  void onObjects(const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onApproval(const std_msgs::msg::Bool::ConstSharedPtr msg);
  void onPredictedPath(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  // Filter functions
  autoware_perception_msgs::msg::PredictedObjects filterObjects(
    const autoware_perception_msgs::msg::PredictedObjects & input_objects);
  sensor_msgs::msg::PointCloud2 filterPointCloud(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud);

  // Utility functions
  bool isObjectNearPath(
    const autoware_perception_msgs::msg::PredictedObject & object,
    const autoware_planning_msgs::msg::Trajectory & path, double filter_distance);

  bool isPointNearPath(
    const geometry_msgs::msg::Point & point,
    const autoware_planning_msgs::msg::Trajectory & path,
    double filter_distance, double min_distance);

  // Subscribers
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr approval_sub_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr predicted_path_sub_;

  // Publishers
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    filtered_objects_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_;

  // Published time publisher
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_publisher_;

  // State variables
  bool approval_received_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_path_;

  // Parameters
  bool enable_object_filtering_;
  bool enable_pointcloud_filtering_;
  double filter_distance_;  // Distance from path to filter objects [m]
  double min_distance_;     // Minimum distance for pointcloud filtering [m]
};

}  // namespace autoware::perception_filter

#endif  // AUTOWARE__PERCEPTION_FILTER__PERCEPTION_FILTER_NODE_HPP_
