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

#include "autoware/perception_filter/perception_filter_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>

#include <memory>

namespace autoware::perception_filter
{

PerceptionFilterNode::PerceptionFilterNode(const rclcpp::NodeOptions & node_options)
: Node("perception_filter_node", node_options), approval_received_(false), predicted_path_(nullptr)
{
  // Declare parameters
  enable_object_filtering_ = declare_parameter<bool>("enable_object_filtering", true);
  enable_pointcloud_filtering_ = declare_parameter<bool>("enable_pointcloud_filtering", true);
  filter_distance_ = declare_parameter<double>("filter_distance", 3.0);  // Default 3m
  min_distance_ = declare_parameter<double>("min_distance", 1.0);       // Default 1m

  // Initialize subscribers
  objects_sub_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "input/objects", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onObjects, this, std::placeholders::_1));

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onPointCloud, this, std::placeholders::_1));

  approval_sub_ = create_subscription<std_msgs::msg::Bool>(
    "input/approval", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onApproval, this, std::placeholders::_1));

  predicted_path_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/predicted_path", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onPredictedPath, this, std::placeholders::_1));

  // Initialize publishers
  filtered_objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "output/filtered_objects", rclcpp::QoS{1});

  filtered_pointcloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("output/filtered_pointcloud", rclcpp::QoS{1});

  // Initialize published time publisher
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  RCLCPP_INFO(get_logger(), "PerceptionFilterNode initialized");
}

void PerceptionFilterNode::onObjects(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  if (!enable_object_filtering_) {
    filtered_objects_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_objects_pub_, msg->header.stamp);
    return;
  }

  auto filtered_objects = filterObjects(*msg);
  filtered_objects_pub_->publish(filtered_objects);
  published_time_publisher_->publish_if_subscribed(
    filtered_objects_pub_, filtered_objects.header.stamp);
}

void PerceptionFilterNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!enable_pointcloud_filtering_) {
    filtered_pointcloud_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_pointcloud_pub_, msg->header.stamp);
    return;
  }

  auto filtered_pointcloud = filterPointCloud(*msg);
  filtered_pointcloud_pub_->publish(filtered_pointcloud);
  published_time_publisher_->publish_if_subscribed(
    filtered_pointcloud_pub_, filtered_pointcloud.header.stamp);
}

void PerceptionFilterNode::onApproval(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  approval_received_ = msg->data;
  RCLCPP_INFO(get_logger(), "Approval status updated: %s", approval_received_ ? "true" : "false");
}

void PerceptionFilterNode::onPredictedPath(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  predicted_path_ = msg;
  RCLCPP_DEBUG(get_logger(), "Predicted path received with %zu points", msg->points.size());
}

autoware_perception_msgs::msg::PredictedObjects PerceptionFilterNode::filterObjects(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects)
{
  autoware_perception_msgs::msg::PredictedObjects filtered_objects;
  filtered_objects.header = input_objects.header;

  if (!approval_received_) {
    // If approval is not received, filter out all objects
    RCLCPP_DEBUG(get_logger(), "Filtering objects due to no approval");
    return filtered_objects;
  }

  if (!predicted_path_ || predicted_path_->points.empty()) {
    // If no predicted path is available, pass through all objects
    RCLCPP_DEBUG(get_logger(), "No predicted path available, passing through all objects");
    filtered_objects = input_objects;
    return filtered_objects;
  }

  // Filter objects based on distance from predicted path
  for (const auto & object : input_objects.objects) {
    if (!isObjectNearPath(object, *predicted_path_, filter_distance_)) {
      filtered_objects.objects.push_back(object);
    } else {
      RCLCPP_DEBUG(get_logger(), "Filtering out object (too close to path)");
    }
  }

  return filtered_objects;
}

sensor_msgs::msg::PointCloud2 PerceptionFilterNode::filterPointCloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud)
{
  sensor_msgs::msg::PointCloud2 filtered_pointcloud;
  filtered_pointcloud.header = input_pointcloud.header;

  if (!approval_received_) {
    // If approval is not received, return empty pointcloud
    filtered_pointcloud.height = 0;
    filtered_pointcloud.width = 0;
    filtered_pointcloud.fields = input_pointcloud.fields;
    filtered_pointcloud.is_bigendian = input_pointcloud.is_bigendian;
    filtered_pointcloud.point_step = input_pointcloud.point_step;
    filtered_pointcloud.row_step = 0;
    filtered_pointcloud.data.clear();
    filtered_pointcloud.is_dense = input_pointcloud.is_dense;

    RCLCPP_DEBUG(get_logger(), "Filtering pointcloud due to no approval");
    return filtered_pointcloud;
  }

  if (!predicted_path_ || predicted_path_->points.empty()) {
    // If no predicted path is available, pass through the pointcloud
    RCLCPP_DEBUG(get_logger(), "No predicted path available, passing through pointcloud");
    filtered_pointcloud = input_pointcloud;
    return filtered_pointcloud;
  }

  // Filter pointcloud based on distance from predicted path
  // Points closer than min_distance are always kept
  // Points between min_distance and filter_distance are filtered out
  // Points farther than filter_distance are kept

  // For now, we'll implement a simple approach by keeping all points
  // In a more sophisticated implementation, you would iterate through each point
  // and check its distance from the path
  filtered_pointcloud = input_pointcloud;
  RCLCPP_DEBUG(get_logger(), "Pointcloud filtering: keeping all points (sophisticated filtering not implemented yet)");

  return filtered_pointcloud;
}

bool PerceptionFilterNode::isObjectNearPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path, double filter_distance)
{
  // Get object position
  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;

  // Find the minimum distance from object to any point on the path
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & path_point : path.points) {
    const auto & path_pos = path_point.pose.position;

    // Calculate 2D distance between object and path point
    const double dx = object_pos.x - path_pos.x;
    const double dy = object_pos.y - path_pos.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    min_distance = std::min(min_distance, distance);
  }

  // Return true if object is within filter_distance of the path
  return min_distance <= filter_distance;
}

bool PerceptionFilterNode::isPointNearPath(
  const geometry_msgs::msg::Point & point,
  const autoware_planning_msgs::msg::Trajectory & path,
  double filter_distance, double min_distance)
{
  // Find the minimum distance from point to any point on the path
  double min_dist_to_path = std::numeric_limits<double>::max();

  for (const auto & path_point : path.points) {
    const auto & path_pos = path_point.pose.position;

    // Calculate 2D distance between point and path point
    const double dx = point.x - path_pos.x;
    const double dy = point.y - path_pos.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    min_dist_to_path = std::min(min_dist_to_path, distance);
  }

  // Return true if point should be filtered out:
  // - Distance is less than filter_distance (close to path)
  // - AND distance is greater than min_distance (not too close)
  return (min_dist_to_path <= filter_distance) && (min_dist_to_path > min_distance);
}

}  // namespace autoware::perception_filter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<autoware::perception_filter::PerceptionFilterNode>(rclcpp::NodeOptions{});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
