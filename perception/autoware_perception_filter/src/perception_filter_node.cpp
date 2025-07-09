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
#include <glog/logging.h>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace autoware::perception_filter
{

PerceptionFilterNode::PerceptionFilterNode(const rclcpp::NodeOptions & node_options)
: Node("perception_filter_node", node_options), predicted_path_(nullptr)
{
  // Initialize glog
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("perception_filter_node");
    google::InstallFailureSignalHandler();
  }

  // Declare parameters
  enable_object_filtering_ = declare_parameter<bool>("enable_object_filtering");
  enable_pointcloud_filtering_ = declare_parameter<bool>("enable_pointcloud_filtering");
  max_filter_distance_ = declare_parameter<double>("max_filter_distance");
  pointcloud_safety_distance_ = declare_parameter<double>("pointcloud_safety_distance");

  // Initialize RTC interface
  rtc_interface_ = std::make_unique<autoware::rtc_interface::RTCInterface>(this, "perception_filter");
  rtc_uuid_ = autoware::universe_utils::generateUUID();

  // Initialize subscribers
  objects_sub_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "input/objects", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onObjects, this, std::placeholders::_1));

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onPointCloud, this, std::placeholders::_1));

  predicted_path_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/predicted_path", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onPredictedPath, this, std::placeholders::_1));

  // Initialize publishers
  filtered_objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "output/filtered_objects", rclcpp::QoS{1});

  filtered_pointcloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("output/filtered_pointcloud", rclcpp::QoS{1});

  // Initialize debug visualization publishers
  debug_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "debug/filtering_markers", rclcpp::QoS{1});

  // Initialize published time publisher
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  RCLCPP_INFO(get_logger(), "PerceptionFilterNode initialized");
}

void PerceptionFilterNode::onObjects(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  // Update RTC status
  updateRTCStatus();

  if (!enable_object_filtering_) {
    filtered_objects_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_objects_pub_, msg->header.stamp);
    return;
  }

  auto filtered_objects = filterObjects(*msg);
  filtered_objects_pub_->publish(filtered_objects);
  published_time_publisher_->publish_if_subscribed(
    filtered_objects_pub_, filtered_objects.header.stamp);

  // Publish debug markers
  const bool rtc_activated = rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);
  publishDebugMarkers(*msg, filtered_objects, rtc_activated);
}

void PerceptionFilterNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Update RTC status
  updateRTCStatus();

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

void PerceptionFilterNode::onPredictedPath(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  predicted_path_ = msg;
  RCLCPP_DEBUG(get_logger(), "Predicted path received with %zu points", msg->points.size());
}

void PerceptionFilterNode::updateRTCStatus()
{
  if (!rtc_interface_) {
    return;
  }

  const bool safe = true;  // Always safe for perception filtering
  const auto state = tier4_rtc_msgs::msg::State::RUNNING;
  const double start_distance = 0.0;
  const double finish_distance = std::numeric_limits<double>::max();

  rtc_interface_->updateCooperateStatus(
    rtc_uuid_, safe, state, start_distance, finish_distance, this->now());
  rtc_interface_->publishCooperateStatus(this->now());
}

autoware_perception_msgs::msg::PredictedObjects PerceptionFilterNode::filterObjects(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects)
{
  autoware_perception_msgs::msg::PredictedObjects filtered_objects;
  filtered_objects.header = input_objects.header;

  // Check if RTC interface is activated
  if (!rtc_interface_ || !rtc_interface_->isActivated(rtc_uuid_)) {
    // If RTC is not activated, pass through all objects
    RCLCPP_DEBUG(get_logger(), "RTC not activated, passing through all objects");
    filtered_objects = input_objects;
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
    if (!isObjectNearPath(object, *predicted_path_, max_filter_distance_)) {
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

  // Check if RTC interface is activated
  if (!rtc_interface_ || !rtc_interface_->isActivated(rtc_uuid_)) {
    // If RTC is not activated, pass through the pointcloud
    RCLCPP_DEBUG(get_logger(), "RTC not activated, passing through pointcloud");
    filtered_pointcloud = input_pointcloud;
    return filtered_pointcloud;
  }

  if (!predicted_path_ || predicted_path_->points.empty()) {
    // If no predicted path is available, pass through the pointcloud
    RCLCPP_DEBUG(get_logger(), "No predicted path available, passing through pointcloud");
    filtered_pointcloud = input_pointcloud;
    return filtered_pointcloud;
  }

  // Filter pointcloud based on distance from predicted path
  // Points closer than pointcloud_safety_distance are always kept
  // Points between pointcloud_safety_distance and max_filter_distance are filtered out
  // Points farther than max_filter_distance are kept

  // For now, we'll implement a simple approach by keeping all points
  // In a more sophisticated implementation, you would iterate through each point
  // and check its distance from the path
  filtered_pointcloud = input_pointcloud;
  RCLCPP_DEBUG(get_logger(), "Pointcloud filtering: keeping all points (sophisticated filtering not implemented yet)");

  return filtered_pointcloud;
}

bool PerceptionFilterNode::isObjectNearPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path, double max_filter_distance)
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

  // Return true if object is within max_filter_distance of the path
  return min_distance <= max_filter_distance;
}

bool PerceptionFilterNode::isPointNearPath(
  const geometry_msgs::msg::Point & point,
  const autoware_planning_msgs::msg::Trajectory & path,
  double max_filter_distance, double pointcloud_safety_distance)
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
  // - Distance is less than max_filter_distance (close to path)
  // - AND distance is greater than pointcloud_safety_distance (not too close)
  return (min_dist_to_path <= max_filter_distance) && (min_dist_to_path > pointcloud_safety_distance);
}

void PerceptionFilterNode::publishDebugMarkers(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects,
  const autoware_perception_msgs::msg::PredictedObjects & filtered_objects,
  bool rtc_activated)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Find filtered out objects (objects in input but not in filtered)
  autoware_perception_msgs::msg::PredictedObjects filtered_out_objects;
  filtered_out_objects.header = input_objects.header;

  for (const auto & input_obj : input_objects.objects) {
    bool found_in_filtered = false;
    for (const auto & filtered_obj : filtered_objects.objects) {
      // Simple comparison based on position (in real implementation, use UUID)
      if (std::abs(input_obj.kinematics.initial_pose_with_covariance.pose.position.x -
                   filtered_obj.kinematics.initial_pose_with_covariance.pose.position.x) < 0.1 &&
          std::abs(input_obj.kinematics.initial_pose_with_covariance.pose.position.y -
                   filtered_obj.kinematics.initial_pose_with_covariance.pose.position.y) < 0.1) {
        found_in_filtered = true;
        break;
      }
    }
    if (!found_in_filtered) {
      filtered_out_objects.objects.push_back(input_obj);
    }
  }

  // Create marker for filtered out objects (red)
  if (!filtered_out_objects.objects.empty()) {
    auto filtered_out_marker = createObjectMarker(
      filtered_out_objects, "map", 0, {1.0, 0.0, 0.0, 0.8});
    marker_array.markers.push_back(filtered_out_marker);
  }

  // Create marker for passed through objects (green)
  if (!filtered_objects.objects.empty()) {
    auto passed_through_marker = createObjectMarker(
      filtered_objects, "map", 1, {0.0, 1.0, 0.0, 0.8});
    marker_array.markers.push_back(passed_through_marker);
  }

  // Create status text marker
  visualization_msgs::msg::Marker status_marker;
  status_marker.header.frame_id = "map";
  status_marker.header.stamp = this->now();
  status_marker.ns = "rtc_status";
  status_marker.id = 2;
  status_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  status_marker.action = visualization_msgs::msg::Marker::ADD;
  status_marker.pose.position.x = 0.0;
  status_marker.pose.position.y = 0.0;
  status_marker.pose.position.z = 5.0;
  status_marker.scale.z = 1.0;
  status_marker.color.a = 1.0;
  status_marker.color.r = rtc_activated ? 0.0 : 1.0;
  status_marker.color.g = rtc_activated ? 1.0 : 0.0;
  status_marker.color.b = 0.0;

  std::string status_text = rtc_activated ? "RTC: ACTIVATED" : "RTC: NOT ACTIVATED";
  status_text += "\nFiltered: " + std::to_string(filtered_out_objects.objects.size());
  status_text += " | Passed: " + std::to_string(filtered_objects.objects.size());
  status_marker.text = status_text;

  marker_array.markers.push_back(status_marker);

  debug_markers_pub_->publish(marker_array);

  RCLCPP_DEBUG(get_logger(),
    "Debug markers published - RTC: %s, Filtered: %zu, Passed: %zu",
    rtc_activated ? "ACTIVATED" : "NOT ACTIVATED",
    filtered_out_objects.objects.size(),
    filtered_objects.objects.size());
}

visualization_msgs::msg::Marker PerceptionFilterNode::createObjectMarker(
  const autoware_perception_msgs::msg::PredictedObjects & objects,
  const std::string & frame_id, int id, const std::array<double, 4> & color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = this->now();
  marker.ns = "perception_filter_debug";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.a = color[3];
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];

  for (const auto & object : objects.objects) {
    geometry_msgs::msg::Point point;
    point.x = object.kinematics.initial_pose_with_covariance.pose.position.x;
    point.y = object.kinematics.initial_pose_with_covariance.pose.position.y;
    point.z = object.kinematics.initial_pose_with_covariance.pose.position.z;
    marker.points.push_back(point);
  }

  return marker;
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
