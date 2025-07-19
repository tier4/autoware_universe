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
#include <autoware/universe_utils/ros/parameter.hpp>

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

  // Declare parameters using get_or_declare_parameter
  using autoware::universe_utils::getOrDeclareParameter;
  enable_object_filtering_ = getOrDeclareParameter<bool>(*this, "enable_object_filtering");
  enable_pointcloud_filtering_ = getOrDeclareParameter<bool>(*this, "enable_pointcloud_filtering");
  max_filter_distance_ = getOrDeclareParameter<double>(*this, "max_filter_distance");
  pointcloud_safety_distance_ = getOrDeclareParameter<double>(*this, "pointcloud_safety_distance");
  object_classification_radius_ = getOrDeclareParameter<double>(*this, "object_classification_radius");

  // Add parameter for RTC recreation on stop (only stop velocity threshold needed)
  stop_velocity_threshold_ = getOrDeclareParameter<double>(*this, "stop_velocity_threshold");

  // Add parameter for ignoring specific object classes
  ignore_object_classes_ = getOrDeclareParameter<std::vector<std::string>>(*this, "ignore_object_classes");

  // Initialize vehicle stopped state
  is_vehicle_stopped_ = false;

  // Initialize RTC transition and frozen filtering state management
  previous_rtc_activated_ = false;
  frozen_filter_object_ids_.clear();

  // Initialize RTC approval state persistence
  rtc_ever_approved_ = false;

  // Initialize polygon-based filtering management
  filtering_polygon_created_ = false;
  filtering_polygon_.is_active = false;
  filtering_polygon_.start_distance_along_path = 0.0;
  filtering_polygon_.end_distance_along_path = 0.0;

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

  // Log ignored object classes configuration
  if (!ignore_object_classes_.empty()) {
    std::string ignored_classes_str = "";
    for (size_t i = 0; i < ignore_object_classes_.size(); ++i) {
      if (i > 0) ignored_classes_str += ", ";
      ignored_classes_str += ignore_object_classes_[i];
    }
    RCLCPP_INFO(get_logger(), "Ignoring object classes: %s", ignored_classes_str.c_str());
  } else {
    RCLCPP_INFO(
      get_logger(),
      "No object classes are ignored (all classes will be filtered if conditions are met)");
  }

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

    // Mark that RTC has ever been approved (persistent state)
    rtc_ever_approved_ = true;

    // Clear previous frozen object IDs
    frozen_filter_object_ids_.clear();

    // Find objects that should be filtered at this moment
    for (const auto & object : input_objects.objects) {
      const double distance_from_ego = getDistanceFromEgo(object);

      // Only consider objects within classification radius
      if (distance_from_ego <= object_classification_radius_) {
        // ignore_object_classesも含めて全ての物体を対象にする
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

          const std::string object_label = labelToString(getMostProbableLabel(object));
          RCLCPP_DEBUG(
            get_logger(),
            "Frozen object for filtering - UUID: %s, Class: %s, Distance to path: %.2f m",
            uuid_str.c_str(), object_label.c_str(), distance_to_path);
        }
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "Frozen %zu objects for filtering", frozen_filter_object_ids_.size());

    // Create filtering polygon for the approved filtering range
    createFilteringPolygon();
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
    const bool previous_rtc_was_activated =
      rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);

    // Clear current RTC status before recreating
    if (rtc_interface_) {
      rtc_interface_->clearCooperateStatus();
    }

    // Recreate RTC interface with new UUID
    initializeRTCInterface();

    // Restore the previous RTC activation state to maintain consistency
    // This ensures frozen list behavior is preserved across RTC recreation
    previous_rtc_activated_ = previous_rtc_was_activated;

    // Preserve the persistent RTC approval state
    // rtc_ever_approved_ is not reset, maintaining the persistent filtering state

    RCLCPP_INFO(get_logger(), "New RTC interface created on vehicle stop");

    if (frozen_list_size > 0) {
      RCLCPP_DEBUG(
        get_logger(), "Frozen filter list preserved across RTC recreation (%zu objects maintained)",
        frozen_list_size);
    }

    if (rtc_ever_approved_) {
      RCLCPP_INFO(get_logger(), "RTC approval state preserved - filtering will continue");
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

  // Publish planning factors (include objects that pass through now but would be filtered if RTC
  // approved)
  auto planning_factors = createPlanningFactors();
  planning_factors_pub_->publish(planning_factors);

  // Publish debug markers
  const bool rtc_activated = rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);
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

  // Check if required data is ready
  if (!isDataReady()) {
    // If data is not ready, publish input pointcloud as-is
    filtered_pointcloud_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_pointcloud_pub_, msg->header.stamp);
    return;
  }

  if (!enable_pointcloud_filtering_) {
    filtered_pointcloud_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_pointcloud_pub_, msg->header.stamp);
    return;
  }

  // Update filtering polygon status
  if (filtering_polygon_created_) {
    updateFilteringPolygonStatus();
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
    "filtered=%zu, Ignored classes=%zu",
    object_classification_radius_, is_currently_stopped ? "STOPPED" : "MOVING",
    classification.pass_through_always.size(), classification.pass_through_would_filter.size(),
    classification.currently_filtered.size(), ignore_object_classes_.size());

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

  // When filtering is active, use classification results to determine filtered objects
  // Pass through: pass_through_always + pass_through_would_filter (empty when filtering active)
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


  // Convert ROS PointCloud2 to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(input_pointcloud, *input_cloud);

  // Clear previous filtered points information
  filtered_points_info_.clear();


  // Filter points based on distance from planning trajectory
  for (const auto & point : input_cloud->points) {
    geometry_msgs::msg::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = point.z;

    // Check if point should be filtered out
    bool should_filter_point = false;
    bool in_polygon = false;
    bool near_path = false;

    if (filtering_polygon_created_ && filtering_polygon_.is_active) {
      // Use polygon-based filtering if available and active
      in_polygon = isPointInFilteringPolygon(ros_point);
      if (in_polygon) {
        // For points inside the polygon, check distance to path and safety distance
        // Filter points that are within max_filter_distance but outside safety_distance
        const double distance_to_path = getMinDistanceToPath(ros_point, *planning_trajectory_);
        near_path = (distance_to_path <= max_filter_distance_);
        const bool outside_safety_distance = (distance_to_path > pointcloud_safety_distance_);
        should_filter_point = near_path && outside_safety_distance;

      }
      // For points outside the polygon, do not filter (should_filter_point remains false)
    }

    if (!should_filter_point) {
      // Keep point if it's not near the path or if it's within safety distance
      filtered_cloud->points.push_back(point);
    } else {
      // Store information about filtered points for planning factors
      FilteredPointInfo filtered_info;
      filtered_info.point = ros_point;
      filtered_info.distance_to_path = getMinDistanceToPath(ros_point, *planning_trajectory_);
      filtered_points_info_.push_back(filtered_info);
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
    get_logger(), "Pointcloud filtering: input points=%zu, output points=%zu, filtered points=%zu",
    input_cloud->points.size(), filtered_cloud->points.size(), filtered_points_info_.size());

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

double PerceptionFilterNode::getMinDistanceToPath(
  const geometry_msgs::msg::Point & point, const autoware_planning_msgs::msg::Trajectory & path)
{
  // Find the minimum distance from point to any point on the path
  double min_distance = std::numeric_limits<double>::max();

  // Check if coordinate frame transformation is needed
  bool transform_needed = false;
  geometry_msgs::msg::TransformStamped transform;

  // Transform trajectory from map to base_link if needed
  if (!path.header.frame_id.empty() && path.header.frame_id == "map") {
    try {
      // Get transform from map to base_link
      transform = tf_buffer_->lookupTransform(
        "base_link", "map",
        rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
      transform_needed = true;

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to get transform from map to base_link: %s. Using original trajectory coordinates.",
        ex.what());
      transform_needed = false;
    }
  }

  for (const auto & path_point : path.points) {
    geometry_msgs::msg::Point transformed_path_point = path_point.pose.position;

    // Transform trajectory point from map to base_link if needed
    if (transform_needed) {
      tf2::doTransform(path_point.pose.position, transformed_path_point, transform);
    }

    // Calculate 2D distance between point and transformed path point
    const double dx = point.x - transformed_path_point.x;
    const double dy = point.y - transformed_path_point.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    min_distance = std::min(min_distance, distance);
  }


  // Return the minimum distance from point to path
  return min_distance;
}

double PerceptionFilterNode::getDistanceAlongPath(const geometry_msgs::msg::Point & point) const
{
  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    return 0.0;
  }

  // Get current ego pose in map coordinates
  const auto ego_pose_map = getCurrentEgoPose();

  // Check if coordinate frame transformation is needed
  bool transform_needed = false;
  geometry_msgs::msg::TransformStamped transform;

  // Transform trajectory from map to base_link if needed
  if (!planning_trajectory_->header.frame_id.empty() && planning_trajectory_->header.frame_id == "map") {
    try {
      // Get transform from map to base_link
      transform = tf_buffer_->lookupTransform(
        "base_link", "map",
        rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
      transform_needed = true;

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to get transform from map to base_link: %s. Using original trajectory coordinates.",
        ex.what());
      transform_needed = false;
    }
  }

  // Transform ego pose to base_link coordinates if needed
  geometry_msgs::msg::Pose ego_pose = ego_pose_map;
  if (transform_needed) {
    tf2::doTransform(ego_pose_map, ego_pose, transform);
  }

  // Debug: Log ego pose and trajectory info for first few calls
  static int distance_debug_counter = 0;
  if (++distance_debug_counter <= 3) {
    RCLCPP_WARN(
      get_logger(),
      "getDistanceAlongPath debug %d: ego_pose_map=(%.2f,%.2f,%.2f), ego_pose_base_link=(%.2f,%.2f,%.2f), point=(%.2f,%.2f,%.2f), trajectory_points=%zu, frame_id=%s, transform_needed=%s",
      distance_debug_counter, ego_pose_map.position.x, ego_pose_map.position.y, ego_pose_map.position.z,
      ego_pose.position.x, ego_pose.position.y, ego_pose.position.z,
      point.x, point.y, point.z, planning_trajectory_->points.size(),
      planning_trajectory_->header.frame_id.c_str(), transform_needed ? "true" : "false");
  }

  // Find the closest point on the trajectory to the ego vehicle
  double min_ego_distance = std::numeric_limits<double>::max();
  size_t ego_closest_index = 0;

  for (size_t i = 0; i < planning_trajectory_->points.size(); ++i) {
    geometry_msgs::msg::Point transformed_path_point = planning_trajectory_->points[i].pose.position;

    // Transform trajectory point from map to base_link if needed
    if (transform_needed) {
      tf2::doTransform(planning_trajectory_->points[i].pose.position, transformed_path_point, transform);
    }

    const double dx = ego_pose.position.x - transformed_path_point.x;
    const double dy = ego_pose.position.y - transformed_path_point.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_ego_distance) {
      min_ego_distance = distance;
      ego_closest_index = i;
    }
  }

  // Find the closest point on the trajectory to the given point
  double min_point_distance = std::numeric_limits<double>::max();
  size_t point_closest_index = 0;

  for (size_t i = 0; i < planning_trajectory_->points.size(); ++i) {
    geometry_msgs::msg::Point transformed_path_point = planning_trajectory_->points[i].pose.position;

    // Transform trajectory point from map to base_link if needed
    if (transform_needed) {
      tf2::doTransform(planning_trajectory_->points[i].pose.position, transformed_path_point, transform);
    }

    const double dx = point.x - transformed_path_point.x;
    const double dy = point.y - transformed_path_point.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < min_point_distance) {
      min_point_distance = distance;
      point_closest_index = i;
    }
  }

  // Calculate cumulative distance along path from ego's closest point to the point's closest point
  double cumulative_distance = 0.0;

  if (ego_closest_index <= point_closest_index) {
    // Point is ahead of ego vehicle (positive distance)
    for (size_t i = ego_closest_index; i < point_closest_index; ++i) {
      geometry_msgs::msg::Point current_point = planning_trajectory_->points[i].pose.position;
      geometry_msgs::msg::Point next_point = planning_trajectory_->points[i + 1].pose.position;

      // Transform points if needed
      if (transform_needed) {
        tf2::doTransform(planning_trajectory_->points[i].pose.position, current_point, transform);
        tf2::doTransform(planning_trajectory_->points[i + 1].pose.position, next_point, transform);
      }

      const double dx = next_point.x - current_point.x;
      const double dy = next_point.y - current_point.y;
      cumulative_distance += std::sqrt(dx * dx + dy * dy);
    }
  } else {
    // Point is behind ego vehicle (negative distance)
    for (size_t i = point_closest_index; i < ego_closest_index; ++i) {
      geometry_msgs::msg::Point current_point = planning_trajectory_->points[i].pose.position;
      geometry_msgs::msg::Point next_point = planning_trajectory_->points[i + 1].pose.position;

      // Transform points if needed
      if (transform_needed) {
        tf2::doTransform(planning_trajectory_->points[i].pose.position, current_point, transform);
        tf2::doTransform(planning_trajectory_->points[i + 1].pose.position, next_point, transform);
      }

      const double dx = next_point.x - current_point.x;
      const double dy = next_point.y - current_point.y;
      cumulative_distance -= std::sqrt(dx * dx + dy * dy);
    }
  }

  // Debug: Log distance calculation for some points
  static int distance_calc_debug_counter = 0;
  if (++distance_calc_debug_counter % 10000 == 0) {  // Log every 10000th point to avoid spam
    RCLCPP_WARN(
      get_logger(),
      "Distance calculation: ego_closest_index=%zu, point_closest_index=%zu, cumulative_distance=%.2f, ego_pos=(%.2f,%.2f), point_pos=(%.2f,%.2f), transform_needed=%s",
      ego_closest_index, point_closest_index, cumulative_distance,
      ego_pose.position.x, ego_pose.position.y, point.x, point.y,
      transform_needed ? "true" : "false");
  }

  return cumulative_distance;
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
  // Check if coordinate frame transformation is needed
  bool transform_needed = false;
  geometry_msgs::msg::TransformStamped transform;

  // Transform trajectory from map to base_link if needed
  if (!path.header.frame_id.empty() && path.header.frame_id == "map") {
    try {
      // Get transform from map to base_link
      transform = tf_buffer_->lookupTransform(
        "base_link", "map",
        rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
      transform_needed = true;

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to get transform from map to base_link: %s. Using original trajectory coordinates.",
        ex.what());
      transform_needed = false;
    }
  }

  // Find the minimum distance from point to any point on the path
  double min_dist_to_path = std::numeric_limits<double>::max();

  for (const auto & path_point : path.points) {
    geometry_msgs::msg::Point transformed_path_point = path_point.pose.position;

    // Transform trajectory point from map to base_link if needed
    if (transform_needed) {
      tf2::doTransform(path_point.pose.position, transformed_path_point, transform);
    }

    // Calculate 2D distance between point and transformed path point
    const double dx = point.x - transformed_path_point.x;
    const double dy = point.y - transformed_path_point.y;
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

  // Create filtering polygon marker if available
  if (filtering_polygon_created_ && filtering_polygon_.is_active) {
    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = "map";
    polygon_marker.header.stamp = this->now();
    polygon_marker.ns = "filtering_polygon";
    polygon_marker.id = 0;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.scale.x = 0.3;  // Line width
    polygon_marker.color.a = 0.8;
    polygon_marker.color.r = 0.0;
    polygon_marker.color.g = 1.0;
    polygon_marker.color.b = 0.0;  // Green color for polygon

    // Add polygon points to create a closed polygon
    for (const auto & point : filtering_polygon_.polygon.outer()) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x();
      ros_point.y = point.y();
      ros_point.z = 0.1;  // Slightly above ground
      polygon_marker.points.push_back(ros_point);
    }

    // Close the polygon by adding the first point again
    if (!filtering_polygon_.polygon.outer().empty()) {
      const auto & first_point = filtering_polygon_.polygon.outer().front();
      geometry_msgs::msg::Point ros_point;
      ros_point.x = first_point.x();
      ros_point.y = first_point.y();
      ros_point.z = 0.1;
      polygon_marker.points.push_back(ros_point);
    }

    marker_array.markers.push_back(polygon_marker);

    // Add polygon info text marker
    visualization_msgs::msg::Marker polygon_info_marker;
    polygon_info_marker.header.frame_id = "map";
    polygon_info_marker.header.stamp = this->now();
    polygon_info_marker.ns = "polygon_info";
    polygon_info_marker.id = 0;
    polygon_info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    polygon_info_marker.action = visualization_msgs::msg::Marker::ADD;

    // Position the text at the center of the polygon
    if (!filtering_polygon_.polygon.outer().empty()) {
      double center_x = 0.0, center_y = 0.0;
      for (const auto & point : filtering_polygon_.polygon.outer()) {
        center_x += point.x();
        center_y += point.y();
      }
      center_x /= filtering_polygon_.polygon.outer().size();
      center_y /= filtering_polygon_.polygon.outer().size();

      polygon_info_marker.pose.position.x = center_x;
      polygon_info_marker.pose.position.y = center_y;
      polygon_info_marker.pose.position.z = 1.0;
    }

    polygon_info_marker.scale.z = 0.8;
    polygon_info_marker.color.a = 1.0;
    polygon_info_marker.color.r = 0.0;
    polygon_info_marker.color.g = 1.0;
    polygon_info_marker.color.b = 0.0;

    std::string polygon_text = "FILTERING POLYGON\n";
    polygon_text += "Range: " + std::to_string(filtering_polygon_.start_distance_along_path) +
                   " to " + std::to_string(filtering_polygon_.end_distance_along_path) + " m\n";
    polygon_text += "Width: " + std::to_string(max_filter_distance_) + " m";

    polygon_info_marker.text = polygon_text;
    marker_array.markers.push_back(polygon_info_marker);
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


  // Add polygon information if available
  if (filtering_polygon_created_) {
    status_text += "\nFiltering Polygon: ";
    status_text += (filtering_polygon_.is_active ? std::string("ACTIVE") : std::string("INACTIVE"));
    status_text += "\nPolygon Range: " + std::to_string(filtering_polygon_.start_distance_along_path) + " to " + std::to_string(filtering_polygon_.end_distance_along_path) + " m";
  }

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

  // Check if RTC interface exists and is activated
  const bool rtc_interface_exists = rtc_interface_ != nullptr;
  const bool rtc_activated = rtc_interface_exists && rtc_interface_->isActivated(rtc_uuid_);

  // Create PlanningFactor when there are objects that would be filtered when RTC is approved
  // OR when there are filtered points (RTC is activated OR was previously approved)
  const bool filtering_active = rtc_activated || rtc_ever_approved_;
  if (!objects_to_be_filtered.empty() || (filtering_active && !filtered_points_info_.empty())) {
    autoware_internal_planning_msgs::msg::PlanningFactor factor;
    factor.module = "supervised_perception_filter";
    factor.behavior = autoware_internal_planning_msgs::msg::PlanningFactor::STOP;

    if (!objects_to_be_filtered.empty() && filtering_active && !filtered_points_info_.empty()) {
      factor.detail = "Objects and pointcloud that would be filtered when RTC is approved";
    } else if (!objects_to_be_filtered.empty()) {
      factor.detail = "Objects that would be filtered when RTC is approved";
    } else if (filtering_active && !filtered_points_info_.empty()) {
      factor.detail = "Pointcloud that would be filtered when RTC is approved";
    }

    // Add control point (nearest point on path)
    if (!planning_trajectory_->points.empty()) {
      autoware_internal_planning_msgs::msg::ControlPoint control_point;
      control_point.pose = planning_trajectory_->points.front().pose;
      control_point.distance = 0.0;
      factor.control_points.push_back(control_point);
    }

    // Add safety factors
    factor.safety_factors.is_safe = false;

    // Add object safety factors (include UUIDs of objects that would be filtered when RTC approved)
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

    // Add pointcloud safety factors (include filtered points when filtering is active)
    if (filtering_active && !filtered_points_info_.empty()) {
      autoware_internal_planning_msgs::msg::SafetyFactor pointcloud_safety_factor;
      pointcloud_safety_factor.type = autoware_internal_planning_msgs::msg::SafetyFactor::POINTCLOUD;
      pointcloud_safety_factor.is_safe = false;

      // Add filtered points positions
      for (const auto & filtered_info : filtered_points_info_) {
        pointcloud_safety_factor.points.push_back(filtered_info.point);
      }

      factor.safety_factors.factors.push_back(pointcloud_safety_factor);
    }

    planning_factors.factors.push_back(factor);

    RCLCPP_DEBUG(
      get_logger(),
      "Planning factors published with %zu objects and %zu filtered points",
      objects_to_be_filtered.size(), filtered_points_info_.size());
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

  // Determine if filtering should be active (current RTC activated OR previously approved)
  const bool filtering_active = rtc_activated || rtc_ever_approved_;

  for (const auto & object : input_objects.objects) {
    const double distance_from_ego = getDistanceFromEgo(object);
    // Classify only objects within the specified radius
    if (distance_from_ego <= object_classification_radius_) {
      // Check if this object should be ignored based on its class
      if (shouldIgnoreObject(object)) {
        // Objects in ignore list are candidates for filtering when RTC is approved
        // They are NOT always passed through
        const double distance_to_path = getMinDistanceToPath(object, *planning_trajectory_);
        const bool would_be_filtered = distance_to_path <= max_filter_distance_;

        // Check if this object ID is in the frozen filter list
        std::array<uint8_t, 16> uuid_array;
        std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());
        const bool is_frozen_for_filtering =
          frozen_filter_object_ids_.find(uuid_array) != frozen_filter_object_ids_.end();

        if (filtering_active) {
          // RTC is currently approved OR was previously approved: filtering function is active
          // Only filter objects that are in the ignore list AND were frozen at RTC approval time
          if (is_frozen_for_filtering) {
            classification.currently_filtered.push_back(object);
          } else {
            // Not in frozen list - classify based on whether RTC is currently activated
            if (rtc_activated) {
              // RTC is currently activated: new objects pass through (not frozen)
              classification.pass_through_always.push_back(object);

            } else {
              // RTC was previously approved but not currently activated: show "would filter"
              if (would_be_filtered) {
                classification.pass_through_would_filter.push_back(object);
              } else {
                classification.pass_through_always.push_back(object);
              }
            }
          }
        } else if (rtc_interface_exists && !rtc_activated && is_currently_stopped) {
          // RTC exists, not approved, and vehicle is stopped: filtering function is inactive
          // Show "would filter" for objects that are in the ignore list
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
        continue;  // Skip the rest of the processing for ignored objects
      }

      // Objects NOT in ignore list are always passed through
      classification.pass_through_always.push_back(object);
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
      get_logger(), *get_clock(), 5000,
      "waiting for planning_trajectory for pointcloud filtering...");
    return false;
  }

  if (enable_pointcloud_filtering_ && !latest_pointcloud_) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for pointcloud data...");
    return false;
  }

  return true;
}

bool PerceptionFilterNode::shouldIgnoreObject(
  const autoware_perception_msgs::msg::PredictedObject & object) const
{
  if (ignore_object_classes_.empty()) {
    return false;  // No classes to ignore
  }

  const uint8_t object_label = getMostProbableLabel(object);
  const std::string label_string = labelToString(object_label);

  // Check if the object's label is in the ignore list
  return std::find(ignore_object_classes_.begin(), ignore_object_classes_.end(), label_string) !=
         ignore_object_classes_.end();
}

uint8_t PerceptionFilterNode::getMostProbableLabel(
  const autoware_perception_msgs::msg::PredictedObject & object) const
{
  if (object.classification.empty()) {
    return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }

  // Find the classification with the highest probability
  double highest_probability = 0.0;
  uint8_t most_probable_label = autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;

  for (const auto & classification : object.classification) {
    if (classification.probability > highest_probability) {
      highest_probability = classification.probability;
      most_probable_label = classification.label;
    }
  }

  return most_probable_label;
}

std::string PerceptionFilterNode::labelToString(uint8_t label) const
{
  using autoware_perception_msgs::msg::ObjectClassification;
  switch (label) {
    case ObjectClassification::UNKNOWN:
      return "UNKNOWN";
    case ObjectClassification::CAR:
      return "CAR";
    case ObjectClassification::TRUCK:
      return "TRUCK";
    case ObjectClassification::BUS:
      return "BUS";
    case ObjectClassification::TRAILER:
      return "TRAILER";
    case ObjectClassification::MOTORCYCLE:
      return "MOTORCYCLE";
    case ObjectClassification::BICYCLE:
      return "BICYCLE";
    case ObjectClassification::PEDESTRIAN:
      return "PEDESTRIAN";
    default:
      return "UNKNOWN";
  }
}

uint8_t PerceptionFilterNode::stringToLabel(const std::string & label_string) const
{
  using autoware_perception_msgs::msg::ObjectClassification;
  if (label_string == "UNKNOWN") {
    return ObjectClassification::UNKNOWN;
  }
  if (label_string == "CAR") {
    return ObjectClassification::CAR;
  }
  if (label_string == "TRUCK") {
    return ObjectClassification::TRUCK;
  }
  if (label_string == "BUS") {
    return ObjectClassification::BUS;
  }
  if (label_string == "TRAILER") {
    return ObjectClassification::TRAILER;
  }
  if (label_string == "MOTORCYCLE") {
    return ObjectClassification::MOTORCYCLE;
  }
  if (label_string == "BICYCLE") {
    return ObjectClassification::BICYCLE;
  }
  if (label_string == "PEDESTRIAN") {
    return ObjectClassification::PEDESTRIAN;
  }
  return ObjectClassification::UNKNOWN;
}


void PerceptionFilterNode::createFilteringPolygon()
{
  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    RCLCPP_WARN(get_logger(), "Cannot create filtering polygon: no planning trajectory available");
    return;
  }

  // Use a fixed filtering distance (e.g., 50 meters) instead of rtc_approval_filtering_distance_
  const double filtering_distance = 50.0;  // Fixed filtering distance in meters

  // Create polygon from trajectory with max_filter_distance width
  filtering_polygon_.polygon = createPathPolygon(
    *planning_trajectory_, 0.0, filtering_distance, max_filter_distance_);

  filtering_polygon_.start_distance_along_path = 0.0;
  filtering_polygon_.end_distance_along_path = filtering_distance;
  filtering_polygon_.is_active = true;
  filtering_polygon_created_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Filtering polygon created: start=%.2f m, end=%.2f m, width=%.2f m",
    filtering_polygon_.start_distance_along_path,
    filtering_polygon_.end_distance_along_path,
    max_filter_distance_);
}

autoware::universe_utils::Polygon2d PerceptionFilterNode::createPathPolygon(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  double start_distance, double end_distance, double width) const
{
  autoware::universe_utils::Polygon2d polygon;

  if (trajectory.points.empty()) {
    return polygon;
  }

  // Calculate cumulative distances to find start and end indices
  double cumulative_distance = 0.0;
  size_t start_index = 0;
  size_t end_index = trajectory.points.size() - 1;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    const auto & current_point = trajectory.points[i].pose.position;
    const auto & next_point = trajectory.points[i + 1].pose.position;

    const double dx = next_point.x - current_point.x;
    const double dy = next_point.y - current_point.y;
    const double segment_distance = std::sqrt(dx * dx + dy * dy);

    if (cumulative_distance <= start_distance && cumulative_distance + segment_distance > start_distance) {
      start_index = i;
    }

    if (cumulative_distance <= end_distance && cumulative_distance + segment_distance > end_distance) {
      end_index = i + 1;
      break;
    }

    cumulative_distance += segment_distance;
  }

  // Create left and right offset points for the polygon
  autoware::universe_utils::LineString2d left_points;
  autoware::universe_utils::LineString2d right_points;

  for (size_t i = start_index; i <= end_index && i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i].pose.position;

    // Calculate perpendicular direction (normal to trajectory)
    autoware::universe_utils::Point2d normal_dir(0.0, 0.0);

    if (i > 0 && i < trajectory.points.size() - 1) {
      // Use direction from previous to next point
      const auto & prev_point = trajectory.points[i - 1].pose.position;
      const auto & next_point = trajectory.points[i + 1].pose.position;

      const double dx = next_point.x - prev_point.x;
      const double dy = next_point.y - prev_point.y;
      const double length = std::sqrt(dx * dx + dy * dy);

      if (length > 1e-6) {
        // Normal direction (perpendicular to trajectory)
        normal_dir.x() = -dy / length;
        normal_dir.y() = dx / length;
      }
    } else if (i > 0) {
      // Use direction from previous point
      const auto & prev_point = trajectory.points[i - 1].pose.position;
      const double dx = point.x - prev_point.x;
      const double dy = point.y - prev_point.y;
      const double length = std::sqrt(dx * dx + dy * dy);

      if (length > 1e-6) {
        normal_dir.x() = -dy / length;
        normal_dir.y() = dx / length;
      }
    } else if (i < trajectory.points.size() - 1) {
      // Use direction to next point
      const auto & next_point = trajectory.points[i + 1].pose.position;
      const double dx = next_point.x - point.x;
      const double dy = next_point.y - point.y;
      const double length = std::sqrt(dx * dx + dy * dy);

      if (length > 1e-6) {
        normal_dir.x() = -dy / length;
        normal_dir.y() = dx / length;
      }
    }

    // Create left and right offset points
    autoware::universe_utils::Point2d left_point(
      point.x + normal_dir.x() * width,
      point.y + normal_dir.y() * width);
    autoware::universe_utils::Point2d right_point(
      point.x - normal_dir.x() * width,
      point.y - normal_dir.y() * width);

    left_points.push_back(left_point);
    right_points.push_back(right_point);
  }

  // Build polygon from left and right points
  for (const auto & point : left_points) {
    polygon.outer().push_back(point);
  }

  // Add right points in reverse order to close the polygon
  for (auto it = right_points.rbegin(); it != right_points.rend(); ++it) {
    polygon.outer().push_back(*it);
  }

  // Close polygon if not already closed
  if (!polygon.outer().empty() &&
      (polygon.outer().front().x() != polygon.outer().back().x() ||
       polygon.outer().front().y() != polygon.outer().back().y())) {
    polygon.outer().push_back(polygon.outer().front());
  }

  return polygon;
}

bool PerceptionFilterNode::isPointInFilteringPolygon(const geometry_msgs::msg::Point & point) const
{
  if (!filtering_polygon_created_ || !filtering_polygon_.is_active) {
    return false;
  }

  // Transform point from base_link to map coordinates
  geometry_msgs::msg::Point transformed_point = point;

  try {
    // Get transform from base_link to map
    const auto transform = tf_buffer_->lookupTransform(
      "map", "base_link",
      rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

    // Transform the point
    tf2::doTransform(point, transformed_point, transform);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to get transform from base_link to map: %s. Using original point coordinates.",
      ex.what());
    // If transform fails, use original point (this may cause incorrect filtering)
  }

  autoware::universe_utils::Point2d test_point(transformed_point.x, transformed_point.y);
  return boost::geometry::within(test_point, filtering_polygon_.polygon);
}

void PerceptionFilterNode::updateFilteringPolygonStatus()
{
  if (!filtering_polygon_created_ || !filtering_polygon_.is_active) {
    return;
  }

  // Check if ego vehicle has passed through the filtering polygon
  const auto ego_pose = getCurrentEgoPose();
  const double current_distance_along_path = getDistanceAlongPath(ego_pose.position);

  // If ego has moved beyond the end of the filtering polygon, deactivate it
  if (current_distance_along_path > filtering_polygon_.end_distance_along_path) {
    filtering_polygon_.is_active = false;
    RCLCPP_INFO(
      get_logger(),
      "Filtering polygon deactivated: ego distance=%.2f m, polygon end=%.2f m",
      current_distance_along_path,
      filtering_polygon_.end_distance_along_path);
  }
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
