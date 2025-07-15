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
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <glog/logging.h>

// Add PCL headers for pointcloud processing
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Add autoware_universe_utils and boost geometry for object shape and distance calculation
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry.hpp>

#include <memory>
#include <set>

namespace autoware::perception_filter
{

PerceptionFilterNode::PerceptionFilterNode(const rclcpp::NodeOptions & node_options)
: Node("perception_filter_node", node_options),
  vehicle_stop_checker_(this),
  planning_trajectory_(nullptr),
  latest_pointcloud_(nullptr)
{
  // Initialize glog
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("perception_filter_node");
    google::InstallFailureSignalHandler();
  }

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize latest classification (empty)
  latest_classification_ = ObjectClassification{};

  // Declare parameters
  enable_object_filtering_ = declare_parameter<bool>("enable_object_filtering");
  enable_pointcloud_filtering_ = declare_parameter<bool>("enable_pointcloud_filtering");
  max_filter_distance_ = declare_parameter<double>("max_filter_distance");
  pointcloud_safety_distance_ = declare_parameter<double>("pointcloud_safety_distance");
  object_classification_radius_ = declare_parameter<double>("object_classification_radius", 50.0);

  // Add parameter for RTC recreation on stop (only stop velocity threshold needed)
  stop_velocity_threshold_ = declare_parameter<double>("stop_velocity_threshold", 0.001);

  // Initialize vehicle stopped state
  is_vehicle_stopped_ = false;

  // Initialize RTC transition and frozen filtering state management
  previous_rtc_activated_ = false;
  frozen_filter_object_ids_.clear();

  // Initialize RTC interface
  initializeRTCInterface();

  // Initialize subscribers
  objects_sub_ = create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
    "input/objects", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onObjects, this, std::placeholders::_1));

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/pointcloud", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onPointCloud, this, std::placeholders::_1));

  planning_trajectory_sub_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/planning_trajectory", rclcpp::QoS{1},
    std::bind(&PerceptionFilterNode::onPlanningTrajectory, this, std::placeholders::_1));

  // Initialize publishers
  filtered_objects_pub_ = create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
    "output/filtered_objects", rclcpp::QoS{1});

  filtered_pointcloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>("output/filtered_pointcloud", rclcpp::QoS{1});

  planning_factors_pub_ =
    create_publisher<autoware_internal_planning_msgs::msg::PlanningFactorArray>(
      "/planning/planning_factors/supervised_perception_filter", rclcpp::QoS{1});

  // Initialize debug visualization publishers
  debug_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "debug/filtering_markers", rclcpp::QoS{1});

  // Initialize published time publisher
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  RCLCPP_DEBUG(get_logger(), "PerceptionFilterNode initialized");
}

void PerceptionFilterNode::initializeRTCInterface()
{
  rtc_interface_ =
    std::make_unique<autoware::rtc_interface::RTCInterface>(this, "supervised_perception_filter");
  rtc_uuid_ = autoware::universe_utils::generateUUID();

  RCLCPP_INFO(get_logger(), "RTC interface initialized with new UUID");
}

void PerceptionFilterNode::handleRTCTransition(
  bool current_rtc_activated, const autoware_perception_msgs::msg::PredictedObjects & input_objects)
{
  // Detect transition from not activated to activated
  if (!previous_rtc_activated_ && current_rtc_activated) {
    RCLCPP_DEBUG(get_logger(), "Freezing filter target objects at RTC approval time...");
    RCLCPP_INFO(get_logger(), "RTC transition detected: NOT ACTIVATED -> ACTIVATED");


    // Clear previous frozen object IDs
    frozen_filter_object_ids_.clear();

    // Find objects that should be filtered at this moment
    for (const auto & object : input_objects.objects) {
      const double distance_from_ego = getDistanceFromEgo(object);

      // Only consider objects within classification radius
      if (distance_from_ego <= object_classification_radius_) {
        const double distance_to_path = getMinDistanceToPath(object, *planning_trajectory_);

        // If object is within filter distance, add its ID to frozen list
        if (distance_to_path <= max_filter_distance_) {
          std::array<uint8_t, 16> uuid_array;
          std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());
          frozen_filter_object_ids_.insert(uuid_array);

          // Convert UUID to string for logging
          std::string uuid_str = "";
          for (size_t i = 0; i < object.object_id.uuid.size(); ++i) {
            if (i > 0) uuid_str += "-";
            uuid_str += std::to_string(static_cast<int>(object.object_id.uuid[i]));
          }

          RCLCPP_DEBUG(
            get_logger(), "Frozen object for filtering - UUID: %s, Distance to path: %.2f m",
            uuid_str.c_str(), distance_to_path);
        }
      }
    }

    RCLCPP_DEBUG(get_logger(), "Frozen %zu objects for filtering", frozen_filter_object_ids_.size());
  }

  // Reset when RTC becomes not activated
  if (previous_rtc_activated_ && !current_rtc_activated) {
    RCLCPP_DEBUG(get_logger(), "RTC transition detected: ACTIVATED -> NOT ACTIVATED");
    RCLCPP_DEBUG(get_logger(), "Clearing frozen filter objects...");
    frozen_filter_object_ids_.clear();
  }

  // Update previous state
  previous_rtc_activated_ = current_rtc_activated;
}

void PerceptionFilterNode::checkVehicleStoppedState()
{
  const bool is_currently_stopped =
    vehicle_stop_checker_.isVehicleStopped(1.0);  // 1 second duration

  // Detect transition from moving to stopped
  if (!is_vehicle_stopped_ && is_currently_stopped) {
    RCLCPP_INFO(get_logger(), "Vehicle stopped detected. Recreating RTC interface...");

    // Store current frozen list size for logging
    const size_t frozen_list_size = frozen_filter_object_ids_.size();

    // Preserve the previous RTC activation state before recreating
    const bool previous_rtc_was_activated = rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);

    // Clear current RTC status before recreating
    if (rtc_interface_) {
      rtc_interface_->clearCooperateStatus();
    }

    // Recreate RTC interface with new UUID
    initializeRTCInterface();

    // Restore the previous RTC activation state to maintain consistency
    // This ensures frozen list behavior is preserved across RTC recreation
    previous_rtc_activated_ = previous_rtc_was_activated;

    RCLCPP_INFO(get_logger(), "New RTC interface created on vehicle stop");

    if (frozen_list_size > 0) {
      RCLCPP_DEBUG(
        get_logger(),
        "Frozen filter list preserved across RTC recreation (%zu objects maintained)",
        frozen_list_size);
    }
  }

  is_vehicle_stopped_ = is_currently_stopped;
}

void PerceptionFilterNode::onObjects(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  // Check if required data is ready
  if (!isDataReady()) {
    // If data is not ready, publish input objects as-is
    filtered_objects_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_objects_pub_, msg->header.stamp);

    // Also publish latest pointcloud if available
    if (latest_pointcloud_) {
      filtered_pointcloud_pub_->publish(*latest_pointcloud_);
      published_time_publisher_->publish_if_subscribed(filtered_pointcloud_pub_, latest_pointcloud_->header.stamp);
    }
    return;
  }

  // Check vehicle stopped state for potential RTC recreation
  checkVehicleStoppedState();

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

  // Check if RTC is activated and execute pointcloud filtering if enabled
  const bool rtc_activated = rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);
  if (rtc_activated && enable_pointcloud_filtering_ && latest_pointcloud_) {
    RCLCPP_DEBUG(get_logger(), "RTC activated: executing pointcloud filtering from onObjects");
    auto filtered_pointcloud = filterPointCloud(*latest_pointcloud_);
    filtered_pointcloud_pub_->publish(filtered_pointcloud);
    published_time_publisher_->publish_if_subscribed(
      filtered_pointcloud_pub_, filtered_pointcloud.header.stamp);
  }

  // Publish planning factors (include objects that pass through now but would be filtered if RTC
  // approved)
  auto planning_factors = createPlanningFactors();
  planning_factors_pub_->publish(planning_factors);

  // Publish debug markers
  publishDebugMarkers(*msg, rtc_activated);
}

void PerceptionFilterNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Store latest pointcloud for use in onObjects when RTC is activated
  latest_pointcloud_ = msg;

  // Check vehicle stopped state for potential RTC recreation
  checkVehicleStoppedState();

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

void PerceptionFilterNode::onPlanningTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  planning_trajectory_ = msg;
  RCLCPP_DEBUG(get_logger(), "Planning trajectory received with %zu points", msg->points.size());
}

void PerceptionFilterNode::updateRTCStatus()
{
  if (!rtc_interface_) {
    return;
  }

  // Only update RTC status when vehicle is stopped
  const bool is_currently_stopped =
    vehicle_stop_checker_.isVehicleStopped(1.0);  // 1 second duration
  if (!is_currently_stopped) {
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

  // Always classify objects to understand potential filtering behavior
  auto classification = classifyObjectsWithinRadius(input_objects);

  // Store the latest classification result
  latest_classification_ = classification;

  // Check if vehicle is currently stopped for more detailed logging
  const bool is_currently_stopped =
    vehicle_stop_checker_.isVehicleStopped(1.0);  // 1 second duration

  // Log classification results regardless of RTC status
  RCLCPP_DEBUG(
    get_logger(),
    "Object classification within %.1fm (Vehicle %s): Always pass=%zu, Would filter=%zu, Currently "
    "filtered=%zu",
    object_classification_radius_, is_currently_stopped ? "STOPPED" : "MOVING",
    classification.pass_through_always.size(), classification.pass_through_would_filter.size(),
    classification.currently_filtered.size());

  // Check if RTC interface is activated
  const bool rtc_activated = rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);

  if (!rtc_activated) {
    // If RTC is not activated, pass through all objects but log what would be filtered
    RCLCPP_DEBUG(get_logger(), "RTC not activated, passing through all objects");

    // Log objects that would be filtered if RTC were approved
    for (const auto & object : classification.pass_through_would_filter) {
      const double distance_to_path = getMinDistanceToPath(object, *planning_trajectory_);

      // Convert UUID to string for debug output
      std::string uuid_str = "";
      for (size_t i = 0; i < object.object_id.uuid.size(); ++i) {
        if (i > 0) uuid_str += "-";
        uuid_str += std::to_string(static_cast<int>(object.object_id.uuid[i]));
      }

      RCLCPP_DEBUG(
        get_logger(),
        "Object UUID: %s, Distance: %.2f m, Threshold: %.2f m, Would be filtered if RTC approved",
        uuid_str.c_str(), distance_to_path, max_filter_distance_);
    }

    filtered_objects = input_objects;
    return filtered_objects;
  }

  // When RTC is activated, use classification results to determine filtered objects
  // Pass through: pass_through_always + pass_through_would_filter (empty when RTC active)
  // Filter out: currently_filtered
  for (const auto & object : classification.pass_through_always) {
    filtered_objects.objects.push_back(object);
  }
  for (const auto & object : classification.pass_through_would_filter) {
    filtered_objects.objects.push_back(object);
  }

  // Log detailed filtering information for each filtered object
  for (const auto & object : classification.currently_filtered) {
    const double distance_to_path = getMinDistanceToPath(object, *planning_trajectory_);

    // Convert UUID to string for debug output
    std::string uuid_str = "";
    for (size_t i = 0; i < object.object_id.uuid.size(); ++i) {
      if (i > 0) uuid_str += "-";
      uuid_str += std::to_string(static_cast<int>(object.object_id.uuid[i]));
    }

    RCLCPP_DEBUG(
      get_logger(), "Object UUID: %s, Distance: %.2f m, Threshold: %.2f m, Filtered: YES",
      uuid_str.c_str(), distance_to_path, max_filter_distance_);
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

  // Convert ROS PointCloud2 to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(input_pointcloud, *input_cloud);

  // Filter points based on distance from planning trajectory
  for (const auto & point : input_cloud->points) {
    geometry_msgs::msg::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = point.z;

    // Check if point should be filtered out
    if (!isPointNearPath(
          ros_point, *planning_trajectory_, max_filter_distance_, pointcloud_safety_distance_)) {
      // Keep point if it's not near the path or if it's within safety distance
      filtered_cloud->points.push_back(point);
    }
  }

  // Update cloud properties
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;

  // Convert back to ROS PointCloud2
  pcl::toROSMsg(*filtered_cloud, filtered_pointcloud);
  filtered_pointcloud.header = input_pointcloud.header;

  RCLCPP_DEBUG(
    get_logger(), "Pointcloud filtering: input points=%zu, output points=%zu",
    input_cloud->points.size(), filtered_cloud->points.size());

  return filtered_pointcloud;
}

double PerceptionFilterNode::getMinDistanceToPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path)
{
  // Return default value if path is empty
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  // Get object shape polygon (same approach as other Autoware modules)
  const auto object_polygon = autoware::universe_utils::toPolygon2d(object);

  // Calculate minimum distance to each path segment
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto & current_pose = path.points[i].pose;
    const auto & next_pose = path.points[i + 1].pose;

    // Create path segment as line string
    autoware::universe_utils::LineString2d line_segment;
    line_segment.push_back(
      autoware::universe_utils::Point2d(current_pose.position.x, current_pose.position.y));
    line_segment.push_back(
      autoware::universe_utils::Point2d(next_pose.position.x, next_pose.position.y));

    // Handle case where segment is a point
    if (boost::geometry::distance(line_segment[0], line_segment[1]) < 1e-6) {
      // Calculate distance to point
      const double distance = boost::geometry::distance(
        object_polygon,
        autoware::universe_utils::Point2d(current_pose.position.x, current_pose.position.y));
      min_distance = std::min(min_distance, distance);
    } else {
      // Calculate distance to line segment
      const double distance = boost::geometry::distance(object_polygon, line_segment);
      min_distance = std::min(min_distance, distance);
    }
  }

  // Also check distance to the last point
  if (!path.points.empty()) {
    const auto & last_pose = path.points.back().pose;
    const double distance = boost::geometry::distance(
      object_polygon,
      autoware::universe_utils::Point2d(last_pose.position.x, last_pose.position.y));
    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

bool PerceptionFilterNode::isObjectNearPath(
  const autoware_perception_msgs::msg::PredictedObject & object,
  const autoware_planning_msgs::msg::Trajectory & path, double max_filter_distance)
{
  const double min_distance = getMinDistanceToPath(object, path);
  // Return true if object is within max_filter_distance of the path
  return min_distance <= max_filter_distance;
}

bool PerceptionFilterNode::isPointNearPath(
  const geometry_msgs::msg::Point & point, const autoware_planning_msgs::msg::Trajectory & path,
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
  return (min_dist_to_path <= max_filter_distance) &&
         (min_dist_to_path > pointcloud_safety_distance);
}

void PerceptionFilterNode::publishDebugMarkers(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects, bool rtc_activated)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // First, delete all previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = "map";
  delete_marker.header.stamp = this->now();
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // Get ego pose for marker positioning
  const auto ego_pose = getCurrentEgoPose();

  // Use the latest classification result stored in member variable
  const auto & classification = latest_classification_;

  // Create markers for different object categories
  int marker_id = 0;

  // Always pass through objects (blue)
  if (!classification.pass_through_always.empty()) {
    autoware_perception_msgs::msg::PredictedObjects always_pass_objects;
    always_pass_objects.header = input_objects.header;
    always_pass_objects.objects = classification.pass_through_always;

    auto always_pass_marker =
      createObjectMarker(always_pass_objects, "map", marker_id++, {0.0, 0.0, 1.0, 0.8});
    always_pass_marker.ns = "always_pass_objects";
    marker_array.markers.push_back(always_pass_marker);

    // Add text markers for always pass objects
    for (size_t i = 0; i < classification.pass_through_always.size(); ++i) {
      const auto & object = classification.pass_through_always[i];
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "map";
      text_marker.header.stamp = this->now();
      text_marker.ns = "always_pass_text";
      text_marker.id = static_cast<int>(i);
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = object.kinematics.initial_pose_with_covariance.pose.position;
      text_marker.pose.position.z += 2.0;  // Display above object
      text_marker.scale.z = 0.5;
      text_marker.color.a = 1.0;
      text_marker.color.r = 0.0;
      text_marker.color.g = 0.0;
      text_marker.color.b = 1.0;
      text_marker.text = "ALWAYS PASS";
      marker_array.markers.push_back(text_marker);
    }
  }

  // Pass through but would be filtered objects (yellow)
  if (!classification.pass_through_would_filter.empty()) {
    autoware_perception_msgs::msg::PredictedObjects would_filter_objects;
    would_filter_objects.header = input_objects.header;
    would_filter_objects.objects = classification.pass_through_would_filter;

    auto would_filter_marker =
      createObjectMarker(would_filter_objects, "map", marker_id++, {1.0, 1.0, 0.0, 0.8});
    would_filter_marker.ns = "would_filter_objects";
    marker_array.markers.push_back(would_filter_marker);

    // Add text markers for would filter objects
    for (size_t i = 0; i < classification.pass_through_would_filter.size(); ++i) {
      const auto & object = classification.pass_through_would_filter[i];
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "map";
      text_marker.header.stamp = this->now();
      text_marker.ns = "would_filter_text";
      text_marker.id = static_cast<int>(i);
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = object.kinematics.initial_pose_with_covariance.pose.position;
      text_marker.pose.position.z += 2.0;  // Display above object
      text_marker.scale.z = 0.5;
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 0.0;
      text_marker.text = "WOULD FILTER";
      marker_array.markers.push_back(text_marker);
    }
  }

  // Currently filtered objects (red)
  if (!classification.currently_filtered.empty()) {
    autoware_perception_msgs::msg::PredictedObjects filtered_out_objects;
    filtered_out_objects.header = input_objects.header;
    filtered_out_objects.objects = classification.currently_filtered;

    auto filtered_out_marker =
      createObjectMarker(filtered_out_objects, "map", marker_id++, {1.0, 0.0, 0.0, 0.8});
    filtered_out_marker.ns = "filtered_objects";
    marker_array.markers.push_back(filtered_out_marker);

    // Add text markers for filtered objects
    for (size_t i = 0; i < classification.currently_filtered.size(); ++i) {
      const auto & object = classification.currently_filtered[i];
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = "map";
      text_marker.header.stamp = this->now();
      text_marker.ns = "filtered_text";
      text_marker.id = static_cast<int>(i);
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.pose.position = object.kinematics.initial_pose_with_covariance.pose.position;
      text_marker.pose.position.z += 2.0;  // Display above object
      text_marker.scale.z = 0.5;
      text_marker.color.a = 1.0;
      text_marker.color.r = 1.0;
      text_marker.color.g = 0.0;
      text_marker.color.b = 0.0;
      text_marker.text = "FILTERED";
      marker_array.markers.push_back(text_marker);
    }
  }

  // Create status text marker above ego vehicle
  visualization_msgs::msg::Marker status_marker;
  status_marker.header.frame_id = "map";
  status_marker.header.stamp = this->now();
  status_marker.ns = "rtc_status";
  status_marker.id = 0;
  status_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  status_marker.action = visualization_msgs::msg::Marker::ADD;
  status_marker.pose.position.x = ego_pose.position.x;
  status_marker.pose.position.y = ego_pose.position.y;
  status_marker.pose.position.z = ego_pose.position.z + 3.0;  // 3m above ego vehicle
  status_marker.scale.z = 1.0;
  status_marker.color.a = 1.0;
  status_marker.color.r = rtc_activated ? 0.0 : 1.0;
  status_marker.color.g = rtc_activated ? 1.0 : 0.0;
  status_marker.color.b = 0.0;

  std::string status_text = rtc_activated ? "RTC: ACTIVATED" : "RTC: NOT ACTIVATED";
  status_text += "\nAlways Pass: " + std::to_string(classification.pass_through_always.size());
  status_text +=
    "\nWould Filter: " + std::to_string(classification.pass_through_would_filter.size());
  status_text += "\nFiltered: " + std::to_string(classification.currently_filtered.size());
  status_marker.text = status_text;

  marker_array.markers.push_back(status_marker);

  debug_markers_pub_->publish(marker_array);

  RCLCPP_DEBUG(
    get_logger(),
    "Debug markers published - RTC: %s, Always Pass: %zu, Would Filter: %zu, Filtered: %zu",
    rtc_activated ? "ACTIVATED" : "NOT ACTIVATED", classification.pass_through_always.size(),
    classification.pass_through_would_filter.size(), classification.currently_filtered.size());
}

visualization_msgs::msg::Marker PerceptionFilterNode::createObjectMarker(
  const autoware_perception_msgs::msg::PredictedObjects & objects, const std::string & frame_id,
  int id, const std::array<double, 4> & color)
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

autoware_internal_planning_msgs::msg::PlanningFactorArray
PerceptionFilterNode::createPlanningFactors()
{
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors;
  // Set header with current time and map frame
  planning_factors.header.stamp = this->now();
  planning_factors.header.frame_id = "map";

  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    return planning_factors;
  }

  // Use the latest classification result stored in member variable
  const auto & classification = latest_classification_;

  // Get UUIDs of objects that pass through now but would be filtered if RTC approved
  std::vector<unique_identifier_msgs::msg::UUID> objects_to_be_filtered;
  for (const auto & object : classification.pass_through_would_filter) {
    objects_to_be_filtered.push_back(object.object_id);
  }

  // Create PlanningFactor only when there are objects that would be filtered when RTC is approved
  if (!objects_to_be_filtered.empty()) {
    autoware_internal_planning_msgs::msg::PlanningFactor factor;
    factor.module = "supervised_perception_filter";
    factor.behavior = autoware_internal_planning_msgs::msg::PlanningFactor::STOP;
    factor.detail = "Objects that would be filtered when RTC is approved";

    // Add control point (nearest point on path)
    if (!planning_trajectory_->points.empty()) {
      autoware_internal_planning_msgs::msg::ControlPoint control_point;
      control_point.pose = planning_trajectory_->points.front().pose;
      control_point.distance = 0.0;
      factor.control_points.push_back(control_point);
    }

    // Add safety factors (include UUIDs of objects that would be filtered when RTC approved)
    factor.safety_factors.is_safe = false;
    for (const auto & uuid : objects_to_be_filtered) {
      autoware_internal_planning_msgs::msg::SafetyFactor safety_factor;
      safety_factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::OBJECT;
      safety_factor.is_safe = false;
      safety_factor.object_id = uuid;

      // Add object position
      for (const auto & object : classification.pass_through_would_filter) {
        if (object.object_id.uuid == uuid.uuid) {
          geometry_msgs::msg::Point point;
          point.x = object.kinematics.initial_pose_with_covariance.pose.position.x;
          point.y = object.kinematics.initial_pose_with_covariance.pose.position.y;
          point.z = object.kinematics.initial_pose_with_covariance.pose.position.z;
          safety_factor.points.push_back(point);
          break;
        }
      }

      factor.safety_factors.factors.push_back(safety_factor);
    }

    planning_factors.factors.push_back(factor);

    RCLCPP_DEBUG(
      get_logger(),
      "Planning factors published with %zu objects that would be filtered when RTC approved",
      objects_to_be_filtered.size());
  }

  return planning_factors;
}

PerceptionFilterNode::ObjectClassification PerceptionFilterNode::classifyObjectsWithinRadius(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects)
{
  ObjectClassification classification;
  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    // If no trajectory available, classify all objects as pass_through_always
    for (const auto & object : input_objects.objects) {
      const double distance_from_ego = getDistanceFromEgo(object);
      if (distance_from_ego <= object_classification_radius_) {
        classification.pass_through_always.push_back(object);
      }
    }
    return classification;
  }

  // Check if RTC interface exists and is activated
  const bool rtc_interface_exists = rtc_interface_ != nullptr;
  const bool rtc_activated = rtc_interface_exists && rtc_interface_->isActivated(rtc_uuid_);
  const bool is_currently_stopped =
    vehicle_stop_checker_.isVehicleStopped(1.0);  // 1 second duration

  // Handle RTC state transition within classification process
  handleRTCTransition(rtc_activated, input_objects);

  for (const auto & object : input_objects.objects) {
    const double distance_from_ego = getDistanceFromEgo(object);
    // Classify only objects within the specified radius
    if (distance_from_ego <= object_classification_radius_) {
      const double distance_to_path = getMinDistanceToPath(object, *planning_trajectory_);
      const bool would_be_filtered = distance_to_path <= max_filter_distance_;

      // Check if this object ID is in the frozen filter list
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());
      const bool is_frozen_for_filtering =
        frozen_filter_object_ids_.find(uuid_array) != frozen_filter_object_ids_.end();

      if (rtc_activated) {
        // RTC exists and is approved: filtering function is active
        // Only filter objects that were frozen at RTC approval time
        if (is_frozen_for_filtering) {
          classification.currently_filtered.push_back(object);

          // Log frozen object filtering
          std::string uuid_str = "";
          for (size_t i = 0; i < object.object_id.uuid.size(); ++i) {
            if (i > 0) uuid_str += "-";
            uuid_str += std::to_string(static_cast<int>(object.object_id.uuid[i]));
          }
          RCLCPP_DEBUG(
            get_logger(), "Filtering frozen object - UUID: %s, Distance: %.2f m", uuid_str.c_str(),
            distance_to_path);
        } else {
          // Not in frozen list, always pass through (even if it would meet filter criteria)
          classification.pass_through_always.push_back(object);

          if (would_be_filtered) {
            // Log that this new object would be filtered but isn't because it's not frozen
            std::string uuid_str = "";
            for (size_t i = 0; i < object.object_id.uuid.size(); ++i) {
              if (i > 0) uuid_str += "-";
              uuid_str += std::to_string(static_cast<int>(object.object_id.uuid[i]));
            }
            RCLCPP_DEBUG(
              get_logger(), "New object not filtered (not frozen) - UUID: %s, Distance: %.2f m",
              uuid_str.c_str(), distance_to_path);
          }
        }
      } else if (rtc_interface_exists && !rtc_activated && is_currently_stopped) {
        // RTC exists, not approved, and vehicle is stopped: filtering function is inactive
        if (would_be_filtered) {
          // Currently passing through, but would be filtered if RTC approved
          classification.pass_through_would_filter.push_back(object);
        } else {
          // Always pass through regardless of RTC status
          classification.pass_through_always.push_back(object);
        }
      } else {
        // No RTC interface or vehicle is moving: classify all objects as pass_through_always
        classification.pass_through_always.push_back(object);
      }
    }
  }

  return classification;
}

double PerceptionFilterNode::getDistanceFromEgo(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  // Get current ego pose using TF
  const auto ego_pose = getCurrentEgoPose();

  // Calculate distance from ego position to object position
  const auto & object_pos = object.kinematics.initial_pose_with_covariance.pose.position;
  const double dx = object_pos.x - ego_pose.position.x;
  const double dy = object_pos.y - ego_pose.position.y;

  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Pose PerceptionFilterNode::getCurrentEgoPose() const
{
  geometry_msgs::msg::Pose ego_pose;
  try {
    const auto transform = tf_buffer_->lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

    ego_pose.position.x = transform.transform.translation.x;
    ego_pose.position.y = transform.transform.translation.y;
    ego_pose.position.z = transform.transform.translation.z;
    ego_pose.orientation = transform.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "Failed to get ego pose: %s", ex.what());
    // Return default pose at origin
    ego_pose.position.x = 0.0;
    ego_pose.position.y = 0.0;
    ego_pose.position.z = 0.0;
    ego_pose.orientation.w = 1.0;
  }
  return ego_pose;
}

bool PerceptionFilterNode::isDataReady()
{
  if (!enable_object_filtering_ && !enable_pointcloud_filtering_) {
    // If both filtering are disabled, no data dependencies
    return true;
  }

  if (enable_object_filtering_ && !planning_trajectory_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for planning_trajectory for object filtering...");
    return false;
  }

  if (enable_pointcloud_filtering_ && !planning_trajectory_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for planning_trajectory for pointcloud filtering...");
    return false;
  }

  if (enable_pointcloud_filtering_ && !latest_pointcloud_) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for pointcloud data...");
    return false;
  }

  return true;
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
