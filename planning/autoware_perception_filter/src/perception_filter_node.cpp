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

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <glog/logging.h>

// Add PCL headers for pointcloud processing
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Add autoware_universe_utils and boost geometry for object shape and distance calculation
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/envelope/interface.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::perception_filter
{

PerceptionFilterNode::PerceptionFilterNode(const rclcpp::NodeOptions & node_options)
: Node("perception_filter_node", node_options),
  vehicle_stop_checker_(this),
  planning_trajectory_(nullptr),
  latest_pointcloud_(nullptr),
  latest_classification_(ObjectClassification{})
{
  // Initialize glog
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("perception_filter_node");
    google::InstallFailureSignalHandler();
  }

  // Initialize TF buffer and listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare parameters using get_or_declare_parameter
  using autoware::universe_utils::getOrDeclareParameter;
  enable_object_filtering_ = getOrDeclareParameter<bool>(*this, "enable_object_filtering");
  enable_pointcloud_filtering_ = getOrDeclareParameter<bool>(*this, "enable_pointcloud_filtering");
  max_filter_distance_ = getOrDeclareParameter<double>(*this, "max_filter_distance");
  pointcloud_safety_distance_ = getOrDeclareParameter<double>(*this, "pointcloud_safety_distance");
  object_classification_radius_ =
    getOrDeclareParameter<double>(*this, "object_classification_radius");
  ignore_object_classes_ =
    getOrDeclareParameter<std::vector<std::string>>(*this, "ignore_object_classes");

  // Debug parameters
  debug_timer_period_ = getOrDeclareParameter<double>(*this, "debug_timer_period");  // 10Hz default

  // Initialize polygon-based filtering management
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

  // Initialize processing time publishers
  objects_processing_time_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/objects_processing_time_ms", rclcpp::QoS{1});

  pointcloud_processing_time_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pointcloud_processing_time_ms", rclcpp::QoS{1});

  // Initialize published time publisher
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // Initialize timer for debug markers (after all publishers are initialized)
  debug_timer_ = create_wall_timer(
    std::chrono::duration<double>(debug_timer_period_),
    std::bind(&PerceptionFilterNode::onTimer, this));

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
}

void PerceptionFilterNode::updateRTCStatus(
  const bool is_currently_stopped, const bool is_just_stopped)
{
  if (!is_currently_stopped) {
    // Clear RTC status and publish empty status when vehicle is not stopped
    rtc_interface_->clearCooperateStatus();
    rtc_interface_->publishCooperateStatus(this->now());
    return;
  }

  if (is_just_stopped) {
    rtc_uuid_ = autoware::universe_utils::generateUUID();
    RCLCPP_DEBUG(get_logger(), "Vehicle just stopped - generating new RTC UUID");
  }

  const bool safe = true;  // Always safe for perception filtering
  const auto state = tier4_rtc_msgs::msg::State::RUNNING;
  const double start_distance = 0.0;
  const double finish_distance = std::numeric_limits<double>::max();

  rtc_interface_->updateCooperateStatus(
    rtc_uuid_, safe, state, start_distance, finish_distance, this->now());
  rtc_interface_->publishCooperateStatus(this->now());
}

void PerceptionFilterNode::onObjects(
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  // Start processing time measurement
  const auto start_time = std::chrono::high_resolution_clock::now();
  latest_objects_ = msg;

  // Check if required data is ready
  if (!isDataReadyForObjects() || !enable_object_filtering_) {
    // If data is not ready, publish input objects as-is
    filtered_objects_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_objects_pub_, msg->header.stamp);

    // Publish processing time even when data is not ready
    const auto processing_time = std::chrono::duration<double, std::milli>(
                                   std::chrono::high_resolution_clock::now() - start_time)
                                   .count();

    autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
    processing_time_msg.stamp = this->now();
    processing_time_msg.data = processing_time;
    objects_processing_time_pub_->publish(processing_time_msg);
    return;
  }

  const bool is_currently_stopped = vehicle_stop_checker_.isVehicleStopped(1.0);
  const bool is_just_stopped = is_currently_stopped && !ego_previously_stopped_;
  ego_previously_stopped_ = is_currently_stopped;

  // Update RTC status
  updateRTCStatus(is_currently_stopped, is_just_stopped);

  const bool rtc_is_registered = rtc_interface_ && rtc_interface_->isRegistered(rtc_uuid_);
  const bool rtc_is_activated =
    rtc_interface_ && rtc_is_registered && rtc_interface_->isActivated(rtc_uuid_);
  const bool rtc_just_activated = rtc_is_activated && !previous_rtc_activated_objects_;
  previous_rtc_activated_objects_ = rtc_is_activated;

  // Add objects from latest classification to frozen list if RTC just activated
  if (rtc_just_activated) {
    for (const auto & object : latest_classification_.pass_through_would_filter) {
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());
      frozen_filter_object_ids_.insert(uuid_array);
    }
  }

  const auto ego_pose = getCurrentEgoPose();
  if (!ego_pose) {
    RCLCPP_DEBUG(get_logger(), "Ego pose not available for object classification");
    // If ego pose is not available, publish input objects as-is
    filtered_objects_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_objects_pub_, msg->header.stamp);
    return;
  }

  auto classification = classifyObjectsWithinRadius(
    *latest_objects_, planning_trajectory_, *ego_pose, rtc_is_registered, frozen_filter_object_ids_,
    max_filter_distance_, object_classification_radius_, ignore_object_classes_);
  latest_classification_ = classification;

  // Create filtered objects message
  autoware_perception_msgs::msg::PredictedObjects filtered_objects;
  filtered_objects.header = msg->header;

  // Add pass through objects
  filtered_objects.objects.reserve(
    classification.pass_through_always.size() + classification.pass_through_would_filter.size());

  filtered_objects.objects.insert(
    filtered_objects.objects.end(), classification.pass_through_always.begin(),
    classification.pass_through_always.end());

  filtered_objects.objects.insert(
    filtered_objects.objects.end(), classification.pass_through_would_filter.begin(),
    classification.pass_through_would_filter.end());

  filtered_objects_pub_->publish(filtered_objects);
  published_time_publisher_->publish_if_subscribed(
    filtered_objects_pub_, filtered_objects.header.stamp);

  // Publish planning factors
  planning_factors_pub_->publish(
    createPlanningFactors(classification, would_be_filtered_points_, planning_trajectory_));

  // Publish processing time
  const auto processing_time = std::chrono::duration<double, std::milli>(
                                 std::chrono::high_resolution_clock::now() - start_time)
                                 .count();

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = this->now();
  processing_time_msg.data = processing_time;
  objects_processing_time_pub_->publish(processing_time_msg);
}

void PerceptionFilterNode::onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Start processing time measurement
  const auto start_time = std::chrono::high_resolution_clock::now();
  latest_pointcloud_ = msg;

  if (!isDataReadyForPointCloud() || !enable_pointcloud_filtering_) {
    RCLCPP_DEBUG(
      get_logger(),
      "PointCloud filtering skipped: isDataReadyForPointCloud=%s, enable_pointcloud_filtering_=%s",
      isDataReadyForPointCloud() ? "true" : "false",
      enable_pointcloud_filtering_ ? "true" : "false");
    filtered_pointcloud_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_pointcloud_pub_, msg->header.stamp);

    const auto processing_time = std::chrono::duration<double, std::milli>(
                                   std::chrono::high_resolution_clock::now() - start_time)
                                   .count();

    autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
    processing_time_msg.stamp = this->now();
    processing_time_msg.data = processing_time;
    pointcloud_processing_time_pub_->publish(processing_time_msg);
    return;
  }

  const bool rtc_is_registered = rtc_interface_ && rtc_interface_->isRegistered(rtc_uuid_);
  const bool rtc_is_activated =
    rtc_interface_ && rtc_is_registered && rtc_interface_->isActivated(rtc_uuid_);
  const bool rtc_just_activated = rtc_is_activated && !previous_rtc_activated_pointcloud_;
  const bool is_currently_stopped = vehicle_stop_checker_.isVehicleStopped(1.0);
  const bool is_just_stopped = is_currently_stopped && !ego_previously_stopped_;

  previous_rtc_activated_pointcloud_ = rtc_is_activated;
  ego_previously_stopped_ = is_currently_stopped;

  // Update RTC status
  updateRTCStatus(is_currently_stopped, is_just_stopped);

  if (rtc_just_activated) {
    RCLCPP_DEBUG(get_logger(), "RTC just activated - creating filtering polygon");
    createFilteringPolygon();
  }

  // Update filtering polygon status
  if (filtering_polygon_created_) {
    updateFilteringPolygonStatus();
  }

  // If rtc_is_registered is true, classify the pointcloud for planning factors
  if (rtc_is_registered) {
    would_be_filtered_points_ = classifyPointCloudForPlanningFactors(*msg, rtc_is_registered);
  } else {
    would_be_filtered_points_.clear();
  }

  // Execute filtering logic
  auto filtered_pointcloud = filterPointCloud(
    *msg, planning_trajectory_, filtering_polygon_, filtering_polygon_created_,
    max_filter_distance_, pointcloud_safety_distance_);

  filtered_pointcloud_pub_->publish(filtered_pointcloud);
  published_time_publisher_->publish_if_subscribed(
    filtered_pointcloud_pub_, filtered_pointcloud.header.stamp);

  // Publish processing time
  const auto processing_time = std::chrono::duration<double, std::milli>(
                                 std::chrono::high_resolution_clock::now() - start_time)
                                 .count();

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = this->now();
  processing_time_msg.data = processing_time;
  pointcloud_processing_time_pub_->publish(processing_time_msg);
}

sensor_msgs::msg::PointCloud2 PerceptionFilterNode::filterPointCloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud,
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr & planning_trajectory,
  const FilteringPolygon & filtering_polygon, bool filtering_polygon_created,
  double max_filter_distance, double pointcloud_safety_distance)
{
  if (
    input_pointcloud.data.empty() || !filtering_polygon_created || !filtering_polygon.is_active ||
    !planning_trajectory) {
    return input_pointcloud;
  }

  // Use common processing function
  auto processing_result = processPointCloudCommon(
    input_pointcloud, filtering_polygon.polygon, planning_trajectory, *tf_buffer_);
  if (!processing_result.success) {
    return input_pointcloud;
  }

  // Filter points based on distance from planning trajectory
  pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices());
  for (size_t idx = 0; idx < processing_result.polygon_inside_indices->indices.size(); ++idx) {
    const auto i = processing_result.polygon_inside_indices->indices[idx];
    const double distance_to_path = processing_result.distances_to_path[idx];

    // For points inside the polygon, check distance to path and safety distance
    const bool is_near_path = (distance_to_path <= max_filter_distance);
    const bool is_outside_safety_distance = (distance_to_path > pointcloud_safety_distance);

    // Remove points that are:
    // 1. Near the path (within max_filter_distance), AND
    // 2. Outside safety distance (beyond pointcloud_safety_distance)
    if (is_near_path && is_outside_safety_distance) {
      indices_to_remove->indices.push_back(i);
    }
  }

  // Extract the filtered cloud by removing the indices to be filtered
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
  extract_indices.setInputCloud(processing_result.transformed_cloud);
  extract_indices.setIndices(indices_to_remove);
  extract_indices.setNegative(true);  // Keep points NOT in the indices_to_remove
  extract_indices.filter(*filtered_cloud);

  // Transform filtered points back to original coordinate frame (base_link)
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (!filtered_cloud->points.empty()) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(), "Failed to get transform for inverse transformation: %s", ex.what());
      return input_pointcloud;
    }

    const auto eigen_transform = tf2::transformToEigen(transform.transform).cast<float>();
    const auto inverse_transform = eigen_transform.inverse();
    pcl::transformPointCloud(*filtered_cloud, *final_filtered_cloud, inverse_transform);
  } else {
    final_filtered_cloud->clear();
  }

  RCLCPP_DEBUG(
    get_logger(),
    "Pointcloud filtering stages: input=%zu -> transformed=%zu -> polygon_inside=%zu -> "
    "removed=%zu -> final=%zu",
    processing_result.transformed_cloud->size(), processing_result.transformed_cloud->size(),
    processing_result.polygon_inside_indices->indices.size(), indices_to_remove->indices.size(),
    final_filtered_cloud->size());

  // Update cloud properties for filtered data
  final_filtered_cloud->width = final_filtered_cloud->points.size();
  final_filtered_cloud->height = 1;
  final_filtered_cloud->is_dense = true;  // Assume dense after filtering

  // Convert back to ROS PointCloud2
  sensor_msgs::msg::PointCloud2 filtered_pointcloud;
  pcl::toROSMsg(*final_filtered_cloud, filtered_pointcloud);
  filtered_pointcloud.header = input_pointcloud.header;

  RCLCPP_DEBUG(
    get_logger(), "Pointcloud filtering: input points=%zu, output points=%zu",
    processing_result.transformed_cloud->points.size(), final_filtered_cloud->points.size());

  return filtered_pointcloud;
}

std::vector<FilteredPointInfo> PerceptionFilterNode::classifyPointCloudForPlanningFactors(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud, bool rtc_is_registered)
{
  std::vector<FilteredPointInfo> would_be_filtered_points;

  if (!rtc_is_registered || input_pointcloud.data.empty() || !planning_trajectory_) {
    return would_be_filtered_points;
  }

  const double filtering_distance = 50.0;  // Fixed filtering distance in meters
  const autoware::universe_utils::Polygon2d filtering_polygon =
    createPathPolygon(*planning_trajectory_, 0.0, filtering_distance, max_filter_distance_);

  // Use common processing function
  auto processing_result =
    processPointCloudCommon(input_pointcloud, filtering_polygon, planning_trajectory_, *tf_buffer_);
  if (!processing_result.success) {
    return would_be_filtered_points;
  }

  // For points inside the polygon, check distance to path and safety distance
  int points_checked = 0;
  int points_inside_polygon = 0;
  int points_near_path = 0;
  int points_outside_safety = 0;
  int points_would_be_filtered = 0;

  for (size_t idx = 0; idx < processing_result.polygon_inside_indices->indices.size(); ++idx) {
    points_checked++;
    const auto i = processing_result.polygon_inside_indices->indices[idx];
    const auto & point = processing_result.transformed_cloud->points[i];
    geometry_msgs::msg::Point ros_point;
    ros_point.x = point.x;
    ros_point.y = point.y;
    ros_point.z = point.z;

    const double distance_to_path = processing_result.distances_to_path[idx];

    // Check if the point is within the filtering polygon
    autoware::universe_utils::Point2d point_2d(ros_point.x, ros_point.y);
    const bool is_inside_polygon = boost::geometry::within(point_2d, filtering_polygon);

    // Check if the point is near the path (within max_filter_distance)
    const bool is_near_path = (distance_to_path <= max_filter_distance_);

    // Check if the point is outside the safety distance
    const bool is_outside_safety_distance = (distance_to_path > pointcloud_safety_distance_);

    if (is_inside_polygon) points_inside_polygon++;
    if (is_near_path) points_near_path++;
    if (is_outside_safety_distance) points_outside_safety++;

    // If the point is inside the polygon AND near the path AND outside the safety distance,
    // it would be filtered by the perception filter.
    if (is_inside_polygon && is_near_path && is_outside_safety_distance) {
      FilteredPointInfo filtered_info;
      filtered_info.point = ros_point;
      filtered_info.distance_to_path = distance_to_path;
      would_be_filtered_points.push_back(filtered_info);
      points_would_be_filtered++;
    }
  }

  RCLCPP_DEBUG(
    get_logger(),
    "PointCloud classification: checked=%d, inside_polygon=%d, near_path=%d, outside_safety=%d, "
    "would_be_filtered=%d",
    points_checked, points_inside_polygon, points_near_path, points_outside_safety,
    points_would_be_filtered);

  return would_be_filtered_points;
}

void PerceptionFilterNode::onPlanningTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  planning_trajectory_ = msg;
}

void PerceptionFilterNode::onTimer()
{
  // Check if required data is available for debug markers
  if (!latest_objects_ || !planning_trajectory_) {
    return;
  }

  // Check if publishers are available
  if (!debug_markers_pub_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Debug markers publisher not available");
    return;
  }

  // Check if TF buffer is available
  if (!tf_buffer_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF buffer not available");
    return;
  }

  // Get ego pose with proper error handling
  auto ego_pose = getCurrentEgoPose();
  if (!ego_pose) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ego pose not available for debug markers");
    return;
  }

  // Check RTC interface state safely
  bool rtc_activated = false;
  if (rtc_interface_) {
    rtc_activated =
      rtc_interface_->isRegistered(rtc_uuid_) && rtc_interface_->isActivated(rtc_uuid_);
  }

  // Create debug markers using the latest data
  auto debug_markers = createDebugMarkers(
    *latest_objects_, latest_classification_, rtc_activated, *ego_pose, filtering_polygon_.polygon,
    filtering_polygon_created_);

  // Publish debug markers
  debug_markers_pub_->publish(debug_markers);
}

void PerceptionFilterNode::createFilteringPolygon()
{
  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot create filtering polygon: planning_trajectory_valid=%s, trajectory_points=%zu",
      planning_trajectory_ ? "true" : "false",
      planning_trajectory_ ? planning_trajectory_->points.size() : 0);
    return;
  }

  const double filtering_distance = 50.0;  // Fixed filtering distance in meters

  // Create polygon from trajectory with max_filter_distance width
  filtering_polygon_.polygon =
    createPathPolygon(*planning_trajectory_, 0.0, filtering_distance, max_filter_distance_);
  filtering_polygon_.start_distance_along_path = 0.0;
  filtering_polygon_.end_distance_along_path = filtering_distance;
  filtering_polygon_.is_active = true;
  filtering_polygon_created_ = true;

  RCLCPP_DEBUG(
    get_logger(),
    "Filtering polygon created successfully: start=%.2f m, end=%.2f m, width=%.2f m, "
    "polygon_points=%zu",
    filtering_polygon_.start_distance_along_path, filtering_polygon_.end_distance_along_path,
    max_filter_distance_, filtering_polygon_.polygon.outer().size());
}

void PerceptionFilterNode::updateFilteringPolygonStatus()
{
  if (!filtering_polygon_created_ || !filtering_polygon_.is_active) {
    return;
  }

  // Check if ego vehicle has passed through the filtering polygon
  auto ego_pose = getCurrentEgoPose();
  if (!ego_pose) {
    RCLCPP_WARN(get_logger(), "Ego pose not available for polygon status update");
    return;
  }

  const double current_distance_along_path =
    getDistanceAlongPath(ego_pose->position, planning_trajectory_, *ego_pose);

  RCLCPP_DEBUG(
    get_logger(), "updateFilteringPolygonStatus: current_distance=%.2f m, polygon_end=%.2f m",
    current_distance_along_path, filtering_polygon_.end_distance_along_path);

  // If ego has moved beyond the end of the filtering polygon, deactivate it
  if (current_distance_along_path > filtering_polygon_.end_distance_along_path) {
    filtering_polygon_.is_active = false;
    RCLCPP_DEBUG(
      get_logger(), "Filtering polygon deactivated: ego distance=%.2f m, polygon end=%.2f m",
      current_distance_along_path, filtering_polygon_.end_distance_along_path);
  }
}

std::optional<geometry_msgs::msg::Pose> PerceptionFilterNode::getCurrentEgoPose() const
{
  if (!tf_buffer_) {
    return std::nullopt;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));

    geometry_msgs::msg::Pose ego_pose;
    ego_pose.position.x = transform.transform.translation.x;
    ego_pose.position.y = transform.transform.translation.y;
    ego_pose.position.z = transform.transform.translation.z;
    ego_pose.orientation = transform.transform.rotation;

    return ego_pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "Failed to get ego pose: %s", ex.what());
    return std::nullopt;
  }
}

bool PerceptionFilterNode::isDataReadyForObjects()
{
  if (!enable_object_filtering_) {
    return true;
  }

  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for planning_trajectory for object filtering...");
    return false;
  }

  return true;
}

bool PerceptionFilterNode::isDataReadyForPointCloud()
{
  if (!enable_pointcloud_filtering_) {
    return true;
  }

  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "waiting for planning_trajectory for pointcloud filtering...");
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
