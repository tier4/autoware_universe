// Copyright 2025 TIER IV, Inc.
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

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/processing_time_tree.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <glog/logging.h>

// Add PCL headers for pointcloud processing
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Add autoware_universe_utils and boost geometry for object shape and distance calculation
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
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
  filtering_distance_ = getOrDeclareParameter<double>(*this, "filtering_distance");
  object_classification_radius_ =
    getOrDeclareParameter<double>(*this, "object_classification_radius");
  ignore_object_classes_ =
    getOrDeclareParameter<std::vector<std::string>>(*this, "ignore_object_classes");
  stop_velocity_threshold_ = getOrDeclareParameter<double>(*this, "stop_velocity_threshold");

  // Debug parameters
  processing_rate_ = getOrDeclareParameter<double>(*this, "processing_rate");

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
    "input/pointcloud", rclcpp::QoS{1}.reliability(rclcpp::ReliabilityPolicy::BestEffort),
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

  debug_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "debug/filtering_markers", rclcpp::QoS{1});

  // Initialize processing time publishers
  objects_processing_time_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/objects_processing_time_ms", rclcpp::QoS{1});

  pointcloud_processing_time_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pointcloud_processing_time_ms", rclcpp::QoS{1});

  // Initialize processing time detail publisher for TimeKeeper
  processing_time_detail_pub_ = create_publisher<tier4_debug_msgs::msg::ProcessingTimeTree>(
    "debug/processing_time_detail", rclcpp::QoS{1});

  time_keeper_ =
    std::make_shared<autoware::universe_utils::TimeKeeper>(processing_time_detail_pub_);

  // Initialize published time publisher
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // Initialize timer for processing loop
  debug_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / processing_rate_),
    std::bind(&PerceptionFilterNode::onTimer, this));

  // Set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PerceptionFilterNode::onParameter, this, std::placeholders::_1));

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

rcl_interfaces::msg::SetParametersResult PerceptionFilterNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;

  // Update filtering parameters
  update_param<bool>(parameters, "enable_object_filtering", enable_object_filtering_);
  update_param<bool>(parameters, "enable_pointcloud_filtering", enable_pointcloud_filtering_);
  update_param<double>(parameters, "max_filter_distance", max_filter_distance_);
  update_param<double>(parameters, "pointcloud_safety_distance", pointcloud_safety_distance_);
  update_param<double>(parameters, "filtering_distance", filtering_distance_);
  update_param<double>(parameters, "object_classification_radius", object_classification_radius_);
  update_param<std::vector<std::string>>(
    parameters, "ignore_object_classes", ignore_object_classes_);

  // Update debug parameters
  update_param<double>(parameters, "processing_rate", processing_rate_);

  // Update stop velocity threshold
  update_param<double>(parameters, "stop_velocity_threshold", stop_velocity_threshold_);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void PerceptionFilterNode::initializeRTCInterface()
{
  rtc_interface_ =
    std::make_unique<autoware::rtc_interface::RTCInterface>(this, "supervised_perception_filter");
  rtc_uuid_ = autoware::universe_utils::generateUUID();
}

/**
 * @brief Check RTC interface state and detect activation changes
 * @param last_state Reference to the last RTC state variable
 * @param context_name Name for logging context (e.g., "object filtering", "pointcloud filtering")
 * @return true if RTC just became active, false otherwise
 */
bool PerceptionFilterNode::checkRTCStateChange(bool & last_state, const std::string & context_name)
{
  const bool rtc_is_registered = rtc_interface_ && rtc_interface_->isRegistered(rtc_uuid_);
  const bool rtc_is_activated =
    rtc_interface_ && rtc_is_registered && rtc_interface_->isActivated(rtc_uuid_);

  // Detect if RTC was just activated (transition from inactive to active)
  const bool rtc_became_active = rtc_is_activated && !last_state;

  // Store current state for next comparison
  last_state = rtc_is_activated;

  // Log RTC state changes for debugging
  if (rtc_became_active) {
    RCLCPP_INFO(get_logger(), "RTC interface just activated for %s", context_name.c_str());
  } else if (last_state && !rtc_is_activated) {
    RCLCPP_DEBUG(get_logger(), "RTC interface deactivated for %s", context_name.c_str());
  }

  return rtc_became_active;
}

// TODO(Sugahara): Skip publishing cooperate status when no filterable objects exist in the
// classification
void PerceptionFilterNode::updateRTCStatus()
{
  const bool is_currently_stopped =
    vehicle_stop_checker_.isVehicleStopped(stop_velocity_threshold_);
  const bool is_just_stopped = is_currently_stopped && !ego_previously_stopped_;
  ego_previously_stopped_ = is_currently_stopped;

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
  // Start processing time measurement using TimeKeeper
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
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

  // Check RTC interface state and detect activation changes
  {
    autoware::universe_utils::ScopedTimeTrack st_rtc("check_rtc_state", *time_keeper_);
    const bool rtc_became_active_in_objects =
      checkRTCStateChange(last_objects_rtc_state_, "object filtering");

    // Add objects from latest classification to frozen list if RTC just activated
    if (rtc_became_active_in_objects) {
      for (const auto & object : latest_classification_.pass_through_would_filter) {
        std::array<uint8_t, 16> uuid_array;
        std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());
        frozen_filter_object_ids_.insert(uuid_array);
      }
    }
  }

  // Get ego pose
  const auto ego_pose = [this]() {
    autoware::universe_utils::ScopedTimeTrack st_ego("get_ego_pose", *time_keeper_);
    return getCurrentEgoPose();
  }();

  if (!ego_pose) {
    RCLCPP_DEBUG(get_logger(), "Ego pose not available for object classification");
    // If ego pose is not available, publish input objects as-is
    filtered_objects_pub_->publish(*msg);
    published_time_publisher_->publish_if_subscribed(filtered_objects_pub_, msg->header.stamp);
    return;
  }

  // Classify objects
  auto classification = [this, &ego_pose]() {
    autoware::universe_utils::ScopedTimeTrack st_classify("classify_objects", *time_keeper_);
    return classifyObjectsWithinRadius(
      *latest_objects_, planning_trajectory_, *ego_pose, rtc_interface_->isRegistered(rtc_uuid_),
      frozen_filter_object_ids_, max_filter_distance_, object_classification_radius_,
      ignore_object_classes_);
  }();
  latest_classification_ = classification;

  // Create filtered objects message
  {
    autoware::universe_utils::ScopedTimeTrack st_create("create_filtered_objects", *time_keeper_);
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
  }

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
  // Start processing time measurement using TimeKeeper
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);
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

  // Check RTC interface state and detect activation changes
  {
    autoware::universe_utils::ScopedTimeTrack st_rtc("check_rtc_state_pointcloud", *time_keeper_);
    const bool rtc_became_active_in_pointcloud =
      checkRTCStateChange(last_pointcloud_rtc_state_, "pointcloud filtering");

    if (rtc_became_active_in_pointcloud) {
      RCLCPP_DEBUG(get_logger(), "RTC just activated - creating filtering polygon");
      createFilteringPolygon();
    }
  }

  // Update filtering polygon status
  if (filtering_polygon_created_) {
    autoware::universe_utils::ScopedTimeTrack st_polygon("update_filtering_polygon", *time_keeper_);
    updateFilteringPolygonStatus();
  }

  // If rtc_is_registered is true, classify the pointcloud for planning factors
  if (rtc_interface_->isRegistered(rtc_uuid_)) {
    autoware::universe_utils::ScopedTimeTrack st_classify(
      "classify_pointcloud_planning_factors", *time_keeper_);
    would_be_filtered_points_ =
      classifyPointCloudForPlanningFactors(*msg, rtc_interface_->isRegistered(rtc_uuid_));
  } else {
    would_be_filtered_points_.clear();
  }

  // Execute filtering logic
  auto filtered_pointcloud = [this, &msg]() {
    autoware::universe_utils::ScopedTimeTrack st_filter("filter_pointcloud", *time_keeper_);
    return filterPointCloud(
      *msg, planning_trajectory_, filtering_polygon_, filtering_polygon_created_,
      max_filter_distance_, pointcloud_safety_distance_);
  }();

  // Publish filtered pointcloud
  {
    autoware::universe_utils::ScopedTimeTrack st_publish(
      "publish_filtered_pointcloud", *time_keeper_);
    filtered_pointcloud_pub_->publish(filtered_pointcloud);
    published_time_publisher_->publish_if_subscribed(
      filtered_pointcloud_pub_, filtered_pointcloud.header.stamp);
  }

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
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (
    input_pointcloud.data.empty() || !filtering_polygon_created || !filtering_polygon.is_active ||
    !planning_trajectory) {
    return input_pointcloud;
  }

  // Use common processing function
  auto processing_result = [this, &input_pointcloud, &filtering_polygon, &planning_trajectory]() {
    autoware::universe_utils::ScopedTimeTrack st_process(
      "process_pointcloud_common", *time_keeper_);
    return processPointCloudCommon(
      input_pointcloud, filtering_polygon.polygon, planning_trajectory, *tf_buffer_, *time_keeper_);
  }();
  if (!processing_result.success) {
    return input_pointcloud;
  }

  // Filter points based on distance from planning trajectory
  pcl::PointIndices::Ptr indices_to_remove = [this, &processing_result, max_filter_distance,
                                              pointcloud_safety_distance]() {
    autoware::universe_utils::ScopedTimeTrack st_filter("filter_points_by_distance", *time_keeper_);
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
    return indices_to_remove;
  }();

  // Extract the filtered cloud by removing the indices to be filtered
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = [this, &processing_result,
                                                        &indices_to_remove]() {
    autoware::universe_utils::ScopedTimeTrack st_extract("extract_filtered_cloud", *time_keeper_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(processing_result.transformed_cloud);
    extract_indices.setIndices(indices_to_remove);
    extract_indices.setNegative(true);  // Keep points NOT in the indices_to_remove
    extract_indices.filter(*filtered_cloud);
    return filtered_cloud;
  }();

  // Transform filtered points back to original coordinate frame (base_link)
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered_cloud = [this, &filtered_cloud,
                                                              &input_pointcloud]() {
    autoware::universe_utils::ScopedTimeTrack st_transform(
      "transform_back_to_base_link", *time_keeper_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!filtered_cloud->points.empty()) {
      // TF lookup for inverse transform
      geometry_msgs::msg::TransformStamped transform;
      {
        autoware::universe_utils::ScopedTimeTrack st_tf_lookup("tf_lookup_inverse", *time_keeper_);
        try {
          transform = tf_buffer_->lookupTransform(
            "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN(
            get_logger(), "Failed to get transform for inverse transformation: %s", ex.what());
          return final_filtered_cloud;
        }
      }

      // Apply inverse transformation
      {
        autoware::universe_utils::ScopedTimeTrack st_apply_inverse(
          "apply_inverse_transform", *time_keeper_);
        const auto eigen_transform = tf2::transformToEigen(transform.transform).cast<float>();
        const auto inverse_transform = eigen_transform.inverse();
        pcl::transformPointCloud(*filtered_cloud, *final_filtered_cloud, inverse_transform);
      }
    } else {
      final_filtered_cloud->clear();
    }
    return final_filtered_cloud;
  }();

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
  sensor_msgs::msg::PointCloud2 filtered_pointcloud = [this, &final_filtered_cloud,
                                                       &input_pointcloud]() {
    autoware::universe_utils::ScopedTimeTrack st_convert(
      "convert_to_ros_pointcloud2", *time_keeper_);
    sensor_msgs::msg::PointCloud2 filtered_pointcloud;
    pcl::toROSMsg(*final_filtered_cloud, filtered_pointcloud);
    filtered_pointcloud.header = input_pointcloud.header;
    return filtered_pointcloud;
  }();

  RCLCPP_DEBUG(
    get_logger(), "Pointcloud filtering: input points=%zu, output points=%zu",
    processing_result.transformed_cloud->points.size(), final_filtered_cloud->points.size());

  return filtered_pointcloud;
}

std::vector<FilteredPointInfo> PerceptionFilterNode::classifyPointCloudForPlanningFactors(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud, bool rtc_is_registered)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<FilteredPointInfo> would_be_filtered_points;

  if (!rtc_is_registered || input_pointcloud.data.empty() || !planning_trajectory_) {
    return would_be_filtered_points;
  }

  // Create filtering polygon
  const autoware::universe_utils::Polygon2d filtering_polygon = [this]() {
    autoware::universe_utils::ScopedTimeTrack st_polygon("create_filtering_polygon", *time_keeper_);
    return createPathPolygon(*planning_trajectory_, 0.0, filtering_distance_, max_filter_distance_);
  }();

  // Use common processing function
  auto processing_result = [this, &input_pointcloud, &filtering_polygon]() {
    autoware::universe_utils::ScopedTimeTrack st_process(
      "process_pointcloud_common_classify", *time_keeper_);
    return processPointCloudCommon(
      input_pointcloud, filtering_polygon, planning_trajectory_, *tf_buffer_, *time_keeper_);
  }();
  if (!processing_result.success) {
    return would_be_filtered_points;
  }

  // Classify points
  {
    autoware::universe_utils::ScopedTimeTrack st_classify("classify_points", *time_keeper_);
    // For points inside the polygon, check distance to path and safety distance
    // Remove debug counters to improve performance
    for (size_t idx = 0; idx < processing_result.polygon_inside_indices->indices.size(); ++idx) {
      const auto i = processing_result.polygon_inside_indices->indices[idx];
      const auto & point = processing_result.transformed_cloud->points[i];
      const double distance_to_path = processing_result.distances_to_path[idx];

      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.x;
      ros_point.y = point.y;
      ros_point.z = point.z;

      // Distance and condition checks
      {
        // Points are already filtered to be inside polygon by processPointCloudCommon
        // No need to check boost::geometry::within again
        // autoware::universe_utils::Point2d point_2d(ros_point.x, ros_point.y);
        // const bool is_inside_polygon = boost::geometry::within(point_2d, filtering_polygon);
        const bool is_inside_polygon = true;  // All points in this loop are inside polygon

        // Check if the point is near the path (within max_filter_distance)
        const bool is_near_path = (distance_to_path <= max_filter_distance_);

        // Check if the point is outside the safety distance
        const bool is_outside_safety_distance = (distance_to_path > pointcloud_safety_distance_);

        // If the point is inside the polygon AND near the path AND outside the safety distance,
        // it would be filtered by the perception filter.
        if (is_inside_polygon && is_near_path && is_outside_safety_distance) {
          FilteredPointInfo filtered_info;
          filtered_info.point = ros_point;
          filtered_info.distance_to_path = distance_to_path;
          would_be_filtered_points.push_back(filtered_info);
        }
      }
    }
  }

  return would_be_filtered_points;
}

void PerceptionFilterNode::onPlanningTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  planning_trajectory_ = msg;
}

void PerceptionFilterNode::onTimer()
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Update RTC status
  {
    autoware::universe_utils::ScopedTimeTrack st_rtc("update_rtc_status", *time_keeper_);
    updateRTCStatus();
  }

  // Publish planning factors (always publish, even if empty)
  autoware_internal_planning_msgs::msg::PlanningFactorArray planning_factors;
  if (latest_objects_ && planning_trajectory_) {
    // If we have the required data, create planning factors
    {
      autoware::universe_utils::ScopedTimeTrack st_planning(
        "create_planning_factors", *time_keeper_);
      planning_factors = createPlanningFactors(
        latest_classification_, would_be_filtered_points_, planning_trajectory_);
    }
  } else {
    // If data is not ready, publish empty planning factors with timestamp
    planning_factors.header.stamp = this->now();
    planning_factors.header.frame_id = "map";
  }
  {
    autoware::universe_utils::ScopedTimeTrack st_publish("publish_planning_factors", *time_keeper_);
    planning_factors_pub_->publish(planning_factors);
  }

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
  auto ego_pose = [this]() {
    autoware::universe_utils::ScopedTimeTrack st_ego("get_ego_pose_timer", *time_keeper_);
    return getCurrentEgoPose();
  }();
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
  auto debug_markers = [this, &ego_pose, &rtc_activated]() {
    autoware::universe_utils::ScopedTimeTrack st_markers("create_debug_markers", *time_keeper_);
    return createDebugMarkers(
      *latest_objects_, latest_classification_, rtc_activated, *ego_pose,
      filtering_polygon_.polygon, filtering_polygon_created_);
  }();

  // Publish debug markers
  {
    autoware::universe_utils::ScopedTimeTrack st_publish_markers(
      "publish_debug_markers", *time_keeper_);
    debug_markers_pub_->publish(debug_markers);
  }
}

void PerceptionFilterNode::createFilteringPolygon()
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot create filtering polygon: planning_trajectory_valid=%s, trajectory_points=%zu",
      planning_trajectory_ ? "true" : "false",
      planning_trajectory_ ? planning_trajectory_->points.size() : 0);
    return;
  }

  // Get current ego pose for reference point
  auto ego_pose = getCurrentEgoPose();
  if (!ego_pose) {
    RCLCPP_ERROR(get_logger(), "Cannot get ego pose for filtering polygon creation");
    return;
  }

  // Create polygon from trajectory with max_filter_distance width
  {
    autoware::universe_utils::ScopedTimeTrack st_polygon("create_path_polygon", *time_keeper_);
    filtering_polygon_.polygon =
      createPathPolygon(*planning_trajectory_, 0.0, filtering_distance_, max_filter_distance_);
  }

  // Store ego pose at creation time and calculate distances
  filtering_polygon_.ego_pose_at_creation = *ego_pose;
  filtering_polygon_.start_distance_along_path = 0.0;
  filtering_polygon_.end_distance_along_path = filtering_distance_;
  filtering_polygon_.is_active = true;
  filtering_polygon_created_ = true;

  RCLCPP_DEBUG(
    get_logger(),
    "Filtering polygon created successfully: start=%.2f m, end=%.2f m, width=%.2f m, "
    "polygon_points=%zu, ego_pose_at_creation=(%.2f, %.2f)",
    filtering_polygon_.start_distance_along_path, filtering_polygon_.end_distance_along_path,
    max_filter_distance_, filtering_polygon_.polygon.outer().size(),
    filtering_polygon_.ego_pose_at_creation.position.x,
    filtering_polygon_.ego_pose_at_creation.position.y);
}

void PerceptionFilterNode::updateFilteringPolygonStatus()
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!filtering_polygon_created_ || !filtering_polygon_.is_active) {
    return;
  }

  // Check if ego vehicle has passed through the filtering polygon
  auto ego_pose = [this]() {
    autoware::universe_utils::ScopedTimeTrack st_ego("get_ego_pose_polygon_status", *time_keeper_);
    return getCurrentEgoPose();
  }();
  if (!ego_pose) {
    RCLCPP_WARN(get_logger(), "Ego pose not available for polygon status update");
    return;
  }

  const double current_distance_along_path = [this, &ego_pose]() {
    autoware::universe_utils::ScopedTimeTrack st_distance("get_distance_along_path", *time_keeper_);
    if (!planning_trajectory_ || planning_trajectory_->points.empty()) {
      return 0.0;
    }

    // Calculate distance from polygon creation point to current ego position
    return autoware::motion_utils::calcSignedArcLength(
      planning_trajectory_->points, filtering_polygon_.ego_pose_at_creation.position,
      ego_pose->position);
  }();

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
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!tf_buffer_) {
    return std::nullopt;
  }

  try {
    const auto transform = [this]() {
      autoware::universe_utils::ScopedTimeTrack st_lookup("lookup_transform", *time_keeper_);
      return tf_buffer_->lookupTransform(
        "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    }();

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
