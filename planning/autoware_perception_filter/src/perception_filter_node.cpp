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
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
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
  stop_velocity_threshold_ = getOrDeclareParameter<double>(*this, "stop_velocity_threshold");
  ignore_object_classes_ =
    getOrDeclareParameter<std::vector<std::string>>(*this, "ignore_object_classes");

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

  const bool rtc_is_activated = rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);
  const bool rtc_just_activated = rtc_is_activated && !previous_rtc_activated_objects_;
  const bool rtc_is_registered = rtc_interface_ && rtc_interface_->isRegistered(rtc_uuid_);
  previous_rtc_activated_objects_ = rtc_is_activated;

  // Add objects from latest classification to frozen list if RTC just activated
  if (rtc_just_activated) {
    for (const auto & object : latest_classification_.pass_through_would_filter) {
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());
      frozen_filter_object_ids_.insert(uuid_array);
    }
  }

  auto classification = classifyObjectsWithinRadius(*latest_objects_, rtc_is_registered);
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
  planning_factors_pub_->publish(createPlanningFactors());

  // Publish debug markers
  publishDebugMarkers(*msg, rtc_is_activated);

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

  const bool rtc_is_activated = rtc_interface_ && rtc_interface_->isActivated(rtc_uuid_);
  const bool rtc_just_activated = rtc_is_activated && !previous_rtc_activated_pointcloud_;
  const bool is_currently_stopped = vehicle_stop_checker_.isVehicleStopped(1.0);
  const bool is_just_stopped = is_currently_stopped && !ego_previously_stopped_;

  previous_rtc_activated_pointcloud_ = rtc_is_activated;
  ego_previously_stopped_ = is_currently_stopped;

  // Update RTC status
  updateRTCStatus(is_currently_stopped, is_just_stopped);

  if (rtc_just_activated) {
    createFilteringPolygon();
  }

  // Update filtering polygon status
  if (filtering_polygon_created_) {
    updateFilteringPolygonStatus();
  }

  auto filtered_pointcloud = filterPointCloud(*msg);
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

void PerceptionFilterNode::onPlanningTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  planning_trajectory_ = msg;
}

sensor_msgs::msg::PointCloud2 PerceptionFilterNode::filterPointCloud(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud)
{
  sensor_msgs::msg::PointCloud2 filtered_pointcloud;
  filtered_pointcloud.header = input_pointcloud.header;
  if (input_pointcloud.data.empty()) {
    return filtered_pointcloud;
  }

  // Convert ROS PointCloud2 to PCL format
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(input_pointcloud, *input_cloud);
  filtered_points_info_.clear();

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      get_logger(),
      "Failed to get transform from base_link to map: %s. Using original point coordinates.",
      ex.what());
  }
  const auto eigen_transform = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::transformPointCloud(*input_cloud, *transformed_cloud, eigen_transform);

  if (filtering_polygon_created_ && filtering_polygon_.is_active) {
    const auto bbox = boost::geometry::return_envelope<autoware_utils_geometry::Box2d>(
      filtering_polygon_.polygon.outer());
    // Rough filter on the X/Y axis
    pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
    passthrough_filter.setInputCloud(transformed_cloud);
    passthrough_filter.setFilterFieldName("x");
    passthrough_filter.setFilterLimits(
      static_cast<float>(bbox.min_corner().x()), static_cast<float>(bbox.max_corner().y()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr x_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthrough_filter.filter(*x_filtered_cloud);
    passthrough_filter.setInputCloud(x_filtered_cloud);
    passthrough_filter.setFilterFieldName("y");
    passthrough_filter.setFilterLimits(
      static_cast<float>(bbox.min_corner().y()), static_cast<float>(bbox.max_corner().y()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr xy_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    passthrough_filter.filter(*xy_filtered_cloud);

    // Keep points inside the polygon
    for (const auto & p : filtering_polygon_.polygon.outer()) {
      polygon_points->push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.0});
    }
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
    prism.setInputCloud(xy_filtered_cloud);
    prism.setInputPlanarHull(polygon_points);
    prism.segment(*inliers);
    // Filter points based on distance from planning trajectory
    for (const auto & i : inliers->indices) {
      const auto point = transformed_cloud->points[i];
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x;
      ros_point.y = point.y;
      ros_point.z = point.z;
      // For points inside the polygon, check distance to path and safety distance
      const double distance_to_path = getMinDistanceToPath(ros_point, *planning_trajectory_);
      const bool is_near_path = (distance_to_path <= max_filter_distance_);
      const bool is_outside_safety_distance = (distance_to_path > pointcloud_safety_distance_);
      if (!is_near_path || !is_outside_safety_distance) {
        filtered_cloud->points.push_back(point);
      } else {
        // Store information about filtered points for planning factors
        FilteredPointInfo filtered_info;
        filtered_info.point = ros_point;
        filtered_info.distance_to_path = getMinDistanceToPath(ros_point, *planning_trajectory_);
        filtered_points_info_.push_back(filtered_info);
      }
    }
    std::printf(
      "%lu -> %lu -> %lu -> %lu -> %lu\n", input_cloud->size(), transformed_cloud->size(),
      x_filtered_cloud->size(), xy_filtered_cloud->size(), filtered_cloud->size());
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
  if (path.points.empty()) {
    return std::numeric_limits<double>::max();
  }

  const auto object_polygon = autoware::universe_utils::toPolygon2d(object);
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto & current_pose = path.points[i].pose;
    const auto & next_pose = path.points[i + 1].pose;

    autoware::universe_utils::LineString2d line_segment;
    line_segment.push_back(
      autoware::universe_utils::Point2d(current_pose.position.x, current_pose.position.y));
    line_segment.push_back(
      autoware::universe_utils::Point2d(next_pose.position.x, next_pose.position.y));

    double distance;
    if (boost::geometry::distance(line_segment[0], line_segment[1]) < 1e-6) {
      // Calculate distance to point
      distance = boost::geometry::distance(
        object_polygon,
        autoware::universe_utils::Point2d(current_pose.position.x, current_pose.position.y));
    } else {
      // Calculate distance to line segment
      distance = boost::geometry::distance(object_polygon, line_segment);
    }
    min_distance = std::min(min_distance, distance);
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
  double min_distance = std::numeric_limits<double>::max();

  // Check if coordinate frame transformation is needed
  for (const auto & path_point : path.points) {
    geometry_msgs::msg::Point transformed_path_point = path_point.pose.position;
    const double dx = point.x - transformed_path_point.x;
    const double dy = point.y - transformed_path_point.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    min_distance = std::min(min_distance, distance);
  }

  return min_distance;
}

double PerceptionFilterNode::getDistanceAlongPath(const geometry_msgs::msg::Point & point) const
{
  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    return 0.0;
  }

  const auto ego_pose_map = getCurrentEgoPose();

  // Check if coordinate frame transformation is needed
  geometry_msgs::msg::TransformStamped transform;
  bool transform_needed = false;

  if (
    !planning_trajectory_->header.frame_id.empty() &&
    planning_trajectory_->header.frame_id == "map") {
    try {
      transform = tf_buffer_->lookupTransform(
        "base_link", "map", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
      transform_needed = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to get transform from map to base_link: %s. Using original trajectory coordinates.",
        ex.what());
    }
  }

  // Transform ego pose to base_link coordinates if needed
  geometry_msgs::msg::Pose ego_pose = ego_pose_map;
  if (transform_needed) {
    tf2::doTransform(ego_pose_map, ego_pose, transform);
  }

  // Find the closest point on the trajectory to the ego vehicle
  double min_ego_distance = std::numeric_limits<double>::max();
  size_t ego_closest_index = 0;

  for (size_t i = 0; i < planning_trajectory_->points.size(); ++i) {
    geometry_msgs::msg::Point transformed_path_point =
      planning_trajectory_->points[i].pose.position;

    if (transform_needed) {
      tf2::doTransform(
        planning_trajectory_->points[i].pose.position, transformed_path_point, transform);
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
    geometry_msgs::msg::Point transformed_path_point =
      planning_trajectory_->points[i].pose.position;

    if (transform_needed) {
      tf2::doTransform(
        planning_trajectory_->points[i].pose.position, transformed_path_point, transform);
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
  const size_t start_index = std::min(ego_closest_index, point_closest_index);
  const size_t end_index = std::max(ego_closest_index, point_closest_index);
  const bool is_point_ahead = (point_closest_index >= ego_closest_index);

  for (size_t i = start_index; i < end_index; ++i) {
    geometry_msgs::msg::Point current_point = planning_trajectory_->points[i].pose.position;
    geometry_msgs::msg::Point next_point = planning_trajectory_->points[i + 1].pose.position;

    if (transform_needed) {
      tf2::doTransform(planning_trajectory_->points[i].pose.position, current_point, transform);
      tf2::doTransform(planning_trajectory_->points[i + 1].pose.position, next_point, transform);
    }

    const double dx = next_point.x - current_point.x;
    const double dy = next_point.y - current_point.y;
    const double segment_distance = std::sqrt(dx * dx + dy * dy);

    cumulative_distance += segment_distance;
  }

  return is_point_ahead ? cumulative_distance : -cumulative_distance;
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

  const auto ego_pose = getCurrentEgoPose();
  const auto & classification = latest_classification_;
  int marker_id = 0;

  // Helper lambda to create object markers with text
  auto create_object_markers =
    [&](
      const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
      const std::string & ns_prefix, const std::string & text,
      const std::array<double, 4> & color) {
      if (objects.empty()) return;

      autoware_perception_msgs::msg::PredictedObjects objects_msg;
      objects_msg.header = input_objects.header;
      objects_msg.objects = objects;

      auto object_marker = createObjectMarker(objects_msg, "map", marker_id++, color);
      object_marker.ns = ns_prefix + "_objects";
      marker_array.markers.push_back(object_marker);

      // Add text markers
      for (size_t i = 0; i < objects.size(); ++i) {
        const auto & object = objects[i];
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = ns_prefix + "_text";
        text_marker.id = static_cast<int>(i);
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position = object.kinematics.initial_pose_with_covariance.pose.position;
        text_marker.pose.position.z += 2.0;  // Display above object
        text_marker.scale.z = 0.5;
        text_marker.color.r = color[0];
        text_marker.color.g = color[1];
        text_marker.color.b = color[2];
        text_marker.color.a = 1.0;
        text_marker.text = text;
        marker_array.markers.push_back(text_marker);
      }
    };

  // Create markers for different object categories
  create_object_markers(
    classification.pass_through_always, "always_pass", "ALWAYS PASS", {0.0, 0.0, 1.0, 0.8});
  create_object_markers(
    classification.pass_through_would_filter, "would_filter", "WOULD FILTER", {1.0, 1.0, 0.0, 0.8});
  create_object_markers(
    classification.currently_filtered, "filtered", "FILTERED", {1.0, 0.0, 0.0, 0.8});

  // Create filtering polygon marker if available
  if (filtering_polygon_created_ && filtering_polygon_.is_active) {
    visualization_msgs::msg::Marker polygon_marker;
    polygon_marker.header.frame_id = "map";
    polygon_marker.header.stamp = this->now();
    polygon_marker.ns = "filtering_polygon";
    polygon_marker.id = 0;
    polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polygon_marker.action = visualization_msgs::msg::Marker::ADD;
    polygon_marker.scale.x = 0.3;
    polygon_marker.color.r = 0.0;
    polygon_marker.color.g = 1.0;
    polygon_marker.color.b = 0.0;
    polygon_marker.color.a = 0.8;

    // Add polygon points to create a closed polygon
    for (const auto & point : filtering_polygon_.polygon.outer()) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x();
      ros_point.y = point.y();
      ros_point.z = 0.1;
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
      const size_t polygon_size = filtering_polygon_.polygon.outer().size();
      center_x /= polygon_size;
      center_y /= polygon_size;

      polygon_info_marker.pose.position.x = center_x;
      polygon_info_marker.pose.position.y = center_y;
      polygon_info_marker.pose.position.z = 1.0;
    }

    polygon_info_marker.scale.z = 0.8;
    polygon_info_marker.color.r = 0.0;
    polygon_info_marker.color.g = 1.0;
    polygon_info_marker.color.b = 0.0;
    polygon_info_marker.color.a = 1.0;

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
  status_marker.color.r = rtc_activated ? 0.0 : 1.0;
  status_marker.color.g = rtc_activated ? 1.0 : 0.0;
  status_marker.color.b = 0.0;
  status_marker.color.a = 1.0;

  std::string status_text = rtc_activated ? "RTC: ACTIVATED" : "RTC: NOT ACTIVATED";
  status_text += "\nAlways Pass: " + std::to_string(classification.pass_through_always.size());
  status_text +=
    "\nWould Filter: " + std::to_string(classification.pass_through_would_filter.size());
  status_text += "\nFiltered: " + std::to_string(classification.currently_filtered.size());

  // Add polygon information if available
  if (filtering_polygon_created_) {
    status_text += "\nFiltering Polygon: ";
    status_text += (filtering_polygon_.is_active ? "ACTIVE" : "INACTIVE");
    status_text +=
      "\nPolygon Range: " + std::to_string(filtering_polygon_.start_distance_along_path) + " to " +
      std::to_string(filtering_polygon_.end_distance_along_path) + " m";
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
  planning_factors.header.stamp = this->now();
  planning_factors.header.frame_id = "map";

  const auto & classification = latest_classification_;

  // Get UUIDs of objects that pass through now but would be filtered if RTC approved
  std::vector<unique_identifier_msgs::msg::UUID> objects_to_be_filtered;
  for (const auto & object : classification.pass_through_would_filter) {
    objects_to_be_filtered.push_back(object.object_id);
  }

  const bool rtc_interface_exists = rtc_interface_ != nullptr;
  const bool rtc_activated = rtc_interface_exists && rtc_interface_->isActivated(rtc_uuid_);
  const bool filtering_active = rtc_activated || rtc_ever_approved_;

  // Create PlanningFactor when there are objects that would be filtered when RTC is approved
  // OR when there are filtered points (RTC is activated OR was previously approved)
  if (!objects_to_be_filtered.empty() || (filtering_active && !filtered_points_info_.empty())) {
    autoware_internal_planning_msgs::msg::PlanningFactor factor;
    factor.module = "supervised_perception_filter";
    factor.behavior = autoware_internal_planning_msgs::msg::PlanningFactor::STOP;

    // Set detail message based on what is being filtered
    const bool has_objects = !objects_to_be_filtered.empty();
    const bool has_points = filtering_active && !filtered_points_info_.empty();

    if (has_objects && has_points) {
      factor.detail = "Objects and pointcloud that would be filtered when RTC is approved";
    } else if (has_objects) {
      factor.detail = "Objects that would be filtered when RTC is approved";
    } else if (has_points) {
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

    // Add object safety factors
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
      pointcloud_safety_factor.type =
        autoware_internal_planning_msgs::msg::SafetyFactor::POINTCLOUD;
      pointcloud_safety_factor.is_safe = false;

      // Add filtered points positions
      for (const auto & filtered_info : filtered_points_info_) {
        pointcloud_safety_factor.points.push_back(filtered_info.point);
      }

      factor.safety_factors.factors.push_back(pointcloud_safety_factor);
    }

    planning_factors.factors.push_back(factor);

    RCLCPP_DEBUG(
      get_logger(), "Planning factors published with %zu objects and %zu filtered points",
      objects_to_be_filtered.size(), filtered_points_info_.size());
  }

  return planning_factors;
}

ObjectClassification PerceptionFilterNode::classifyObjectsWithinRadius(
  const autoware_perception_msgs::msg::PredictedObjects & input_objects, bool rtc_is_registered)
{
  ObjectClassification classification;

  // If frozen list is empty and RTC not registered, pass through all objects
  if (frozen_filter_object_ids_.empty() && !rtc_is_registered) {
    classification.pass_through_always = input_objects.objects;
    return classification;
  }

  // If frozen list is not empty and RTC not registered, keep filtered objects
  if (!frozen_filter_object_ids_.empty() && !rtc_is_registered) {
    for (const auto & object : input_objects.objects) {
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());

      if (frozen_filter_object_ids_.find(uuid_array) != frozen_filter_object_ids_.end()) {
        classification.currently_filtered.push_back(object);
      } else {
        classification.pass_through_always.push_back(object);
      }
    }
    return classification;
  }

  if (rtc_is_registered) {
    for (const auto & object : input_objects.objects) {
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());

      // Check if object is in frozen filter list
      if (frozen_filter_object_ids_.find(uuid_array) != frozen_filter_object_ids_.end()) {
        classification.currently_filtered.push_back(object);
        continue;
      }

      // Check if object should be ignored
      if (!shouldIgnoreObject(object)) {
        classification.pass_through_always.push_back(object);
        continue;
      }

      // Check if object is within classification radius
      const double distance_from_ego = getDistanceFromEgo(object);
      if (distance_from_ego <= object_classification_radius_) {
        const double distance_to_path = getMinDistanceToPath(object, *planning_trajectory_);
        const bool would_be_filtered = distance_to_path <= max_filter_distance_;

        if (would_be_filtered) {
          classification.pass_through_would_filter.push_back(object);
        } else {
          classification.pass_through_always.push_back(object);
        }
      } else {
        classification.pass_through_always.push_back(object);
      }
    }
  }

  return classification;
}

double PerceptionFilterNode::getDistanceFromEgo(
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  const auto ego_pose = getCurrentEgoPose();
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
  if (label_string == "UNKNOWN") return ObjectClassification::UNKNOWN;
  if (label_string == "CAR") return ObjectClassification::CAR;
  if (label_string == "TRUCK") return ObjectClassification::TRUCK;
  if (label_string == "BUS") return ObjectClassification::BUS;
  if (label_string == "TRAILER") return ObjectClassification::TRAILER;
  if (label_string == "MOTORCYCLE") return ObjectClassification::MOTORCYCLE;
  if (label_string == "BICYCLE") return ObjectClassification::BICYCLE;
  if (label_string == "PEDESTRIAN") return ObjectClassification::PEDESTRIAN;
  return ObjectClassification::UNKNOWN;
}

void PerceptionFilterNode::createFilteringPolygon()
{
  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    RCLCPP_WARN(get_logger(), "Cannot create filtering polygon: no planning trajectory available");
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

  RCLCPP_INFO(
    get_logger(), "Filtering polygon created: start=%.2f m, end=%.2f m, width=%.2f m",
    filtering_polygon_.start_distance_along_path, filtering_polygon_.end_distance_along_path,
    max_filter_distance_);
}

autoware::universe_utils::Polygon2d PerceptionFilterNode::createPathPolygon(
  const autoware_planning_msgs::msg::Trajectory & trajectory, double start_distance,
  double end_distance, double width) const
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

    if (
      cumulative_distance <= start_distance &&
      cumulative_distance + segment_distance > start_distance) {
      start_index = i;
    }

    if (
      cumulative_distance <= end_distance &&
      cumulative_distance + segment_distance > end_distance) {
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
      point.x + normal_dir.x() * width, point.y + normal_dir.y() * width);
    autoware::universe_utils::Point2d right_point(
      point.x - normal_dir.x() * width, point.y - normal_dir.y() * width);

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
  if (
    !polygon.outer().empty() && (polygon.outer().front().x() != polygon.outer().back().x() ||
                                 polygon.outer().front().y() != polygon.outer().back().y())) {
    polygon.outer().push_back(polygon.outer().front());
  }

  return polygon;
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
      get_logger(), "Filtering polygon deactivated: ego distance=%.2f m, polygon end=%.2f m",
      current_distance_along_path, filtering_polygon_.end_distance_along_path);
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
