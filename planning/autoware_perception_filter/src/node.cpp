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
#include "autoware/perception_filter/perception_filter_utils.hpp"

#include <Eigen/Dense>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/processing_time_tree.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

// Add PCL headers for pointcloud processing
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>

// Add autoware_universe_utils and boost geometry for object shape and distance calculation
#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/ros/parameter.hpp>

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
#include <unordered_set>
#include <vector>

namespace autoware::perception_filter
{

PerceptionFilterNode::PerceptionFilterNode(const rclcpp::NodeOptions & node_options)
: Node("perception_filter_node", node_options), vehicle_stop_checker_(this)
{
  // Initialize glog
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("perception_filter_node");
    google::InstallFailureSignalHandler();
  }

  // Initialize transform listener
  transform_listener_ = std::make_shared<autoware::universe_utils::TransformListener>(this);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Declare parameters using get_or_declare_parameter
  using autoware::universe_utils::getOrDeclareParameter;
  enable_object_filtering_ = getOrDeclareParameter<bool>(*this, "enable_object_filtering");
  enable_pointcloud_filtering_ = getOrDeclareParameter<bool>(*this, "enable_pointcloud_filtering");
  max_filter_distance_ = getOrDeclareParameter<double>(*this, "max_filter_distance");
  pointcloud_safety_distance_ = getOrDeclareParameter<double>(*this, "pointcloud_safety_distance");
  filtering_start_distance_ = getOrDeclareParameter<double>(*this, "filtering_start_distance");
  filtering_end_distance_ = getOrDeclareParameter<double>(*this, "filtering_end_distance");
  object_classification_radius_ =
    getOrDeclareParameter<double>(*this, "object_classification_radius");
  ignore_object_classes_ =
    getOrDeclareParameter<std::vector<std::string>>(*this, "ignore_object_classes");
  stop_velocity_threshold_ = getOrDeclareParameter<double>(*this, "stop_velocity_threshold");
  processing_rate_ = getOrDeclareParameter<double>(*this, "processing_rate");

  // Initialize RTC interface for supervised perception filtering
  rtc_interface_ =
    std::make_unique<autoware::rtc_interface::RTCInterface>(this, "supervised_perception_filter");
  rtc_uuid_ = autoware::universe_utils::generateUUID();

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

  polygon_debug_markers_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("debug/polygon_markers", rclcpp::QoS{1});

  objects_processing_time_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/objects_processing_time_ms", rclcpp::QoS{1});

  pointcloud_processing_time_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pointcloud_processing_time_ms", rclcpp::QoS{1});

  processing_time_detail_pub_ = create_publisher<tier4_debug_msgs::msg::ProcessingTimeTree>(
    "debug/processing_time_detail", rclcpp::QoS{1});

  time_keeper_ =
    std::make_shared<autoware::universe_utils::TimeKeeper>(processing_time_detail_pub_, &std::cerr);

  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  debug_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / processing_rate_),
    std::bind(&PerceptionFilterNode::onTimer, this));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PerceptionFilterNode::onParameter, this, std::placeholders::_1));
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
  update_param<double>(parameters, "filtering_start_distance", filtering_start_distance_);
  update_param<double>(parameters, "filtering_end_distance", filtering_end_distance_);
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

template <typename MessageType, typename PublisherType>
void PerceptionFilterNode::publishPassthroughMessage(
  const MessageType & msg, const std::shared_ptr<PublisherType> & publisher,
  const std::shared_ptr<rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>> &
    time_publisher,
  const std::chrono::high_resolution_clock::time_point & start_time) const
{
  // Publish input message as-is
  publisher->publish(msg);
  published_time_publisher_->publish_if_subscribed(publisher, msg.header.stamp);

  const auto processing_time = std::chrono::duration<double, std::milli>(
                                 std::chrono::high_resolution_clock::now() - start_time)
                                 .count();

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = this->now();
  processing_time_msg.data = processing_time;
  time_publisher->publish(processing_time_msg);
}

bool PerceptionFilterNode::checkRTCStateChange(bool & last_state)
{
  const bool rtc_is_activated =
    rtc_interface_->isRegistered(rtc_uuid_) && rtc_interface_->isActivated(rtc_uuid_);

  const bool rtc_became_active = rtc_is_activated && !last_state;
  // Store current state for next comparison
  last_state = rtc_is_activated;

  return rtc_became_active;
}

// TODO(Sugahara): don't publish the cooperate status when the object which can be overwritten does
// not exist
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

  // Early validation and publish passthrough if conditions not met
  const auto ego_pose = autoware::perception_filter::getEgoPose(*tf_buffer_);
  if (!enable_object_filtering_ || !isDataReadyForObjects() || !ego_pose) {
    publishPassthroughMessage(
      *msg, filtered_objects_pub_, objects_processing_time_pub_, start_time);
    return;
  }

  // Check RTC interface state and detect activation changes
  const bool rtc_became_active_in_objects = checkRTCStateChange(last_objects_rtc_state_);

  // Add objects from latest classification to frozen list if RTC just activated
  if (rtc_became_active_in_objects) {
    for (const auto & object : latest_classification_.would_be_removed) {
      std::array<uint8_t, 16> uuid_array;
      std::copy(object.object_id.uuid.begin(), object.object_id.uuid.end(), uuid_array.begin());
      frozen_filter_object_ids_.insert(uuid_array);
    }
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
      classification.kept_objects.size() + classification.would_be_removed.size());

    filtered_objects.objects.insert(
      filtered_objects.objects.end(), classification.kept_objects.begin(),
      classification.kept_objects.end());

    filtered_objects.objects.insert(
      filtered_objects.objects.end(), classification.would_be_removed.begin(),
      classification.would_be_removed.end());

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

  if (
    !isDataReadyForPointCloud() || !enable_pointcloud_filtering_ ||
    latest_pointcloud_->data.empty()) {
    publishPassthroughMessage(
      *msg, filtered_pointcloud_pub_, pointcloud_processing_time_pub_, start_time);
    return;
  }

  // Check RTC interface state and detect activation changes
  const bool rtc_became_active_in_pointcloud = checkRTCStateChange(last_pointcloud_rtc_state_);
  const bool is_stopped = vehicle_stop_checker_.isVehicleStopped(stop_velocity_threshold_);
  const bool filtering_polygon_is_active = filtering_polygon_.is_active;
  const bool rtc_is_activated =
    rtc_interface_->isRegistered(rtc_uuid_) && rtc_interface_->isActivated(rtc_uuid_);

  auto ego_pose = autoware::perception_filter::getEgoPose(*tf_buffer_);
  if (!ego_pose) {
    RCLCPP_WARN(get_logger(), "Ego pose not available for polygon status update");
    return;
  }

  // Update filtering polygon status
  if (filtering_polygon_is_active) {
    updateFilteringPolygonStatus(*ego_pose);
  }

  if (rtc_became_active_in_pointcloud) {
    createFilteringPolygon(*ego_pose);
  }

  // When RTC is ready for approval (but not yet approved), publish pointcloud that will be filtered
  // upon approval as planning factors
  if (is_stopped && !rtc_is_activated) {
    autoware::universe_utils::ScopedTimeTrack st_classify(
      "prepare_pointcloud_for_planning_factors", *time_keeper_);

    // 1. Resample trajectory with fixed interval
    constexpr double resample_interval = 0.5;
    const auto resampled_trajectory = autoware::motion_utils::resampleTrajectory(
      *planning_trajectory_, resample_interval, false, true, true);

    // 2. Cut trajectory by filtering distances from ego pose
    auto cut_trajectory = autoware::perception_filter::cutTrajectoryByFilteringDistance(
      resampled_trajectory, *ego_pose, filtering_start_distance_, filtering_end_distance_);

    // 3. Generate trajectory polygons for max and min filter distances
    const auto traj_max_polygons = autoware::perception_filter::generateTrajectoryPolygons(
      cut_trajectory, max_filter_distance_, transform_listener_);
    const auto traj_min_polygons = autoware::perception_filter::generateTrajectoryPolygons(
      cut_trajectory, pointcloud_safety_distance_, transform_listener_);

    // 4. Transform trajectory points to base link coordinate system
    const auto base_link_trajectory_points =
      autoware::perception_filter::transformTrajectoryToBaseLink(
        cut_trajectory.points, transform_listener_);

    // 5. Combine minimum polygons into a single polygon
    const auto combined_traj_min_polygon =
      autoware::perception_filter::combineTrajectoryPolygons(traj_min_polygons);

    // 6. Generate difference polygons (max_polygons - min_polygons)
    const auto difference_polygons =
      autoware::perception_filter::createDifferencePolygons(traj_max_polygons, traj_min_polygons);

    // 7. Point cloud filtering process
    {
      autoware::universe_utils::ScopedTimeTrack st_publish(
        "calculate_would_be_filtered_point_cloud", *time_keeper_);

      // Convert ROS pointcloud to PCL pointcloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr =
        std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::fromROSMsg(*msg, *input_pointcloud_ptr);

      // Filter pointcloud using multiple trajectory polygons
      const auto [filtered_pcl_cloud, indices_in_polygons] = filterByMultiTrajectoryPolygon(
        input_pointcloud_ptr, difference_polygons, time_keeper_.get());

      // Convert PCL pointcloud back to ROS pointcloud message
      auto filtered_ros_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*filtered_pcl_cloud, *filtered_ros_cloud);
      filtered_ros_cloud->header = msg->header;
      would_be_filtered_point_cloud_ = filtered_ros_cloud;
    }
  } else {
    would_be_filtered_point_cloud_ = nullptr;
  }

  if (filtering_polygon_.is_active) {
    autoware::universe_utils::ScopedTimeTrack st_publish("filter_pointcloud", *time_keeper_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_pointcloud_ptr =
      std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*msg, *input_pointcloud_ptr);
    const auto cut_trajectory = autoware::perception_filter::cutTrajectoryByPoses(
      *planning_trajectory_, filtering_polygon_.start_pose, filtering_polygon_.end_pose);

    const auto traj_max_polygons = autoware::perception_filter::generateTrajectoryPolygons(
      cut_trajectory, max_filter_distance_, transform_listener_);
    const auto traj_min_polygons = autoware::perception_filter::generateTrajectoryPolygons(
      cut_trajectory, pointcloud_safety_distance_, transform_listener_);

    const auto difference_polygons =
      autoware::perception_filter::createDifferencePolygons(traj_max_polygons, traj_min_polygons);

    // Filter pointcloud to keep points that are NOT in difference_polygons
    // First, get points that ARE in difference_polygons using the updated function
    const auto [points_in_difference_polygons, indices_in_polygons] =
      filterByMultiTrajectoryPolygon(input_pointcloud_ptr, difference_polygons, time_keeper_.get());

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(input_pointcloud_ptr);
    extract_indices.setIndices(indices_in_polygons);
    extract_indices.setNegative(true);  // ポリゴン内の点を除去
    extract_indices.filter(*filtered_pcl_pointcloud);

    // Convert PCL pointcloud to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 filtered_pointcloud;
    pcl::toROSMsg(*filtered_pcl_pointcloud, filtered_pointcloud);
    filtered_pointcloud.header = msg->header;

    // Publish filtered pointcloud
    {
      autoware::universe_utils::ScopedTimeTrack st_publish(
        "publish_filtered_pointcloud", *time_keeper_);

      filtered_pointcloud_pub_->publish(filtered_pointcloud);

      published_time_publisher_->publish_if_subscribed(
        filtered_pointcloud_pub_, filtered_pointcloud.header.stamp);
    }
  } else {
    publishPassthroughMessage(
      *msg, filtered_pointcloud_pub_, pointcloud_processing_time_pub_, start_time);
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
  const FilteringPolygon & filtering_polygon, double max_filter_distance,
  double pointcloud_safety_distance)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (input_pointcloud.data.empty() || !filtering_polygon.is_active || !planning_trajectory) {
    return input_pointcloud;
  }

  // Use common processing function
  const auto processing_result = [this, &input_pointcloud, &filtering_polygon,
                                  &planning_trajectory]() {
    autoware::universe_utils::ScopedTimeTrack st_process(
      "process_pointcloud_common", *time_keeper_);
    // Combine multiple polygons into a single polygon for processing
    const auto combined_polygon =
      autoware::perception_filter::combineTrajectoryPolygons(filtering_polygon.max_polygon);
    return processPointCloudCommon(
      input_pointcloud, combined_polygon, planning_trajectory, transform_listener_, *time_keeper_);
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered_cloud = [this, &filtered_cloud]() {
    autoware::universe_utils::ScopedTimeTrack st_transform(
      "transform_back_to_base_link", *time_keeper_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!filtered_cloud->points.empty()) {
      // TF lookup for inverse transform
      const auto transform_opt = [this]() {
        autoware::universe_utils::ScopedTimeTrack st_tf_lookup("tf_lookup_inverse", *time_keeper_);
        return transform_listener_->getLatestTransform("map", "base_link");
      }();

      if (!transform_opt) {
        RCLCPP_WARN(get_logger(), "Failed to get transform for inverse transformation");
        return final_filtered_cloud;
      }

      const auto transform = *transform_opt;

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
  const sensor_msgs::msg::PointCloud2 & input_pointcloud, bool rtc_is_registered,
  const std::vector<autoware::universe_utils::Polygon2d> & traj_max_polygons,
  const autoware::universe_utils::Polygon2d & combined_traj_min_polygon,
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & base_link_trajectory_points,
  const std::vector<autoware::universe_utils::Polygon2d> & crop_box_polygons)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::vector<FilteredPointInfo> would_be_filtered_points;

  if (!rtc_is_registered || input_pointcloud.data.empty() || !planning_trajectory_) {
    return would_be_filtered_points;
  }

  // Convert sensor_msgs::PointCloud2 to PCL pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(input_pointcloud, *input_pcl_cloud);

  // Use crop box polygons passed as parameter (generated by generateCropBoxPolygons)

  // Apply crop box filtering using trajectory polygons
  const auto filtered_pcl_cloud = [this, &input_pcl_cloud, &traj_max_polygons,
                                   &base_link_trajectory_points, &crop_box_polygons]() {
    autoware::universe_utils::ScopedTimeTrack st_crop("crop_box_filtering", *time_keeper_);

    RCLCPP_DEBUG(
      get_logger(), "Pointcloud filtering input: %zu points", input_pcl_cloud->points.size());

    const auto result = autoware::perception_filter::filterByTrajectoryPolygonsCropBox(
      input_pcl_cloud, traj_max_polygons, base_link_trajectory_points, crop_box_polygons, 10.0);

    RCLCPP_DEBUG(
      get_logger(), "Pointcloud filtering output: %zu points (filtered: %zu points)",
      result->points.size(), input_pcl_cloud->points.size() - result->points.size());

    return result;
  }();

  // Classify points based on polygon membership with 2-stage filtering
  {
    autoware::universe_utils::ScopedTimeTrack st_classify("classify_points", *time_keeper_);

    size_t points_would_be_filtered = 0;
    size_t points_in_traj_min_polygons = 0;

    // Step 1: Create crop box polygon for combined_traj_min_polygon
    std::vector<autoware::universe_utils::Polygon2d> min_polygon_crop_box;
    if (!combined_traj_min_polygon.outer().empty()) {
      // Calculate AABB of combined_traj_min_polygon and create crop box polygon
      using Point2D = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
      boost::geometry::model::box<Point2D> min_polygon_bbox;
      boost::geometry::envelope(combined_traj_min_polygon, min_polygon_bbox);

      // Create crop box polygon from AABB
      autoware::universe_utils::Polygon2d crop_box_polygon;
      crop_box_polygon.outer().push_back(autoware::universe_utils::Point2d(
        min_polygon_bbox.min_corner().get<0>(), min_polygon_bbox.min_corner().get<1>()));
      crop_box_polygon.outer().push_back(autoware::universe_utils::Point2d(
        min_polygon_bbox.max_corner().get<0>(), min_polygon_bbox.min_corner().get<1>()));
      crop_box_polygon.outer().push_back(autoware::universe_utils::Point2d(
        min_polygon_bbox.max_corner().get<0>(), min_polygon_bbox.max_corner().get<1>()));
      crop_box_polygon.outer().push_back(autoware::universe_utils::Point2d(
        min_polygon_bbox.min_corner().get<0>(), min_polygon_bbox.max_corner().get<1>()));
      crop_box_polygon.outer().push_back(autoware::universe_utils::Point2d(
        min_polygon_bbox.min_corner().get<0>(), min_polygon_bbox.min_corner().get<1>()));
      min_polygon_crop_box.push_back(crop_box_polygon);
    }

    // Step 2: Get points outside crop box (keep_inside=false)
    const auto crop_box_outside_cloud =
      autoware::perception_filter::filterByTrajectoryPolygonsCropBox(
        filtered_pcl_cloud, traj_max_polygons, base_link_trajectory_points, min_polygon_crop_box,
        10.0, false);

    // Step 3: Get points inside crop box (keep_inside=true)
    const auto crop_box_inside_cloud =
      autoware::perception_filter::filterByTrajectoryPolygonsCropBox(
        filtered_pcl_cloud, traj_max_polygons, base_link_trajectory_points, min_polygon_crop_box,
        10.0, true);

    RCLCPP_ERROR(
      get_logger(),
      "Crop box filtering results: filtered_pcl_cloud=%zu, crop_box_outside_cloud=%zu, "
      "crop_box_inside_cloud=%zu",
      filtered_pcl_cloud->points.size(), crop_box_outside_cloud->points.size(),
      crop_box_inside_cloud->points.size());

    // Step 4: Add points outside crop box to would_be_filtered_points
    for (const auto & point : crop_box_outside_cloud->points) {
      FilteredPointInfo filtered_info;
      filtered_info.point.x = point.x;
      filtered_info.point.y = point.y;
      filtered_info.point.z = point.z;
      would_be_filtered_points.push_back(filtered_info);
    }

    // Step 5: Perform strict polygon inside/outside check for points inside crop box
    size_t points_inside_polygon = 0;
    size_t points_outside_polygon = 0;
    for (const auto & point : crop_box_inside_cloud->points) {
      using Point2D = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
      Point2D boost_point(point.x, point.y);

      if (boost::geometry::within(boost_point, combined_traj_min_polygon)) {
        points_inside_polygon++;
      } else {
        points_outside_polygon++;
        FilteredPointInfo filtered_info;
        filtered_info.point.x = point.x;
        filtered_info.point.y = point.y;
        filtered_info.point.z = point.z;
        would_be_filtered_points.push_back(filtered_info);
      }
    }

    points_in_traj_min_polygons = points_inside_polygon;

    RCLCPP_ERROR(
      get_logger(),
      "Strict polygon filtering results: crop_box_inside_cloud=%zu, points_inside_polygon=%zu, "
      "points_outside_polygon=%zu",
      crop_box_inside_cloud->points.size(), points_inside_polygon, points_outside_polygon);

    points_would_be_filtered = would_be_filtered_points.size();

    // Log final classification results
    RCLCPP_ERROR(
      get_logger(),
      "Final 2-stage filtering results: total=%zu, in_polygon=%zu, would_be_filtered=%zu",
      filtered_pcl_cloud->points.size(), points_in_traj_min_polygons, points_would_be_filtered);
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
        latest_classification_, would_be_filtered_point_cloud_, planning_trajectory_);
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

  auto ego_pose = autoware::perception_filter::getEgoPose(*tf_buffer_);
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
      filtering_polygon_.max_polygon);
  }();

  // Publish debug markers
  {
    autoware::universe_utils::ScopedTimeTrack st_publish_markers(
      "publish_debug_markers", *time_keeper_);
    debug_markers_pub_->publish(debug_markers);
  }
}

void PerceptionFilterNode::createFilteringPolygon(const geometry_msgs::msg::Pose & ego_pose)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Create polygon from trajectory with max_filter_distance width
  constexpr double resample_interval = 0.5;
  const auto resampled_trajectory = autoware::motion_utils::resampleTrajectory(
    *planning_trajectory_, resample_interval, false, true, true);
  filtering_polygon_.max_polygon = autoware::perception_filter::generateTrajectoryPolygons(
    resampled_trajectory, max_filter_distance_, transform_listener_);
  filtering_polygon_.min_polygon = autoware::perception_filter::generateTrajectoryPolygons(
    resampled_trajectory, pointcloud_safety_distance_, transform_listener_);
  filtering_polygon_.trajectory = resampled_trajectory;
  filtering_polygon_.ego_pose = ego_pose;

  // Cut trajectory by filtering distances from ego pose
  const auto cut_trajectory = autoware::perception_filter::cutTrajectoryByFilteringDistance(
    resampled_trajectory, ego_pose, filtering_start_distance_, filtering_end_distance_);

  if (!cut_trajectory.points.empty()) {
    // Set start and end poses from cut trajectory
    filtering_polygon_.start_pose = cut_trajectory.points.front().pose;
    filtering_polygon_.end_pose = cut_trajectory.points.back().pose;
  } else {
    RCLCPP_WARN(get_logger(), "Cut trajectory is empty, using original trajectory bounds");
    // Fallback to original trajectory bounds if cut trajectory is empty
    if (!resampled_trajectory.points.empty()) {
      filtering_polygon_.start_pose = resampled_trajectory.points.front().pose;
      filtering_polygon_.end_pose = resampled_trajectory.points.back().pose;
    }
  }
  filtering_polygon_.is_active = true;
}

void PerceptionFilterNode::updateFilteringPolygonStatus(const geometry_msgs::msg::Pose & ego_pose)
{
  autoware::universe_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // Calculate current ego_pose distance along the trajectory
  const double current_ego_distance = [this, &ego_pose]() {
    return autoware::motion_utils::calcSignedArcLength(
      planning_trajectory_->points, 0, ego_pose.position);
  }();

  // Calculate end_pose distance along the trajectory
  const double end_pose_distance = [this]() {
    return autoware::motion_utils::calcSignedArcLength(
      planning_trajectory_->points, 0, filtering_polygon_.end_pose.position);
  }();

  // Check if end_pose has been passed
  if (current_ego_distance >= end_pose_distance) {
    RCLCPP_INFO(get_logger(), "Ego pose has passed end_pose, deactivating filtering polygon");
    filtering_polygon_.is_active = false;
    return;
  }

  // Calculate original ego_pose distance along the trajectory
  const double original_ego_distance = [this]() {
    return autoware::motion_utils::calcSignedArcLength(
      planning_trajectory_->points, 0, filtering_polygon_.ego_pose.position);
  }();

  const double start_pose_distance = [this]() {
    return autoware::motion_utils::calcSignedArcLength(
      planning_trajectory_->points, 0, filtering_polygon_.start_pose.position);
  }();

  // Advance start_pose by the amount ego_pose has advanced
  const double ego_advancement = current_ego_distance - original_ego_distance;
  if (ego_advancement > 0) {
    const double new_start_distance = start_pose_distance + ego_advancement;
    if (new_start_distance < end_pose_distance) {
      filtering_polygon_.start_pose = autoware::motion_utils::calcInterpolatedPose(
        planning_trajectory_->points, new_start_distance);
      RCLCPP_DEBUG(get_logger(), "Updated start_pose by advancement: %.2f m", ego_advancement);
    }
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

visualization_msgs::msg::MarkerArray PerceptionFilterNode::createTrajectoryPolygonMarkers(
  const std::vector<autoware::universe_utils::Polygon2d> & traj_polygons,
  const std::string & frame_id, const std::string & marker_namespace, double r, double g, double b,
  double a)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const auto & polygon = traj_polygons[i];

    // Create line strip marker for polygon outline
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = marker_namespace;
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker properties
    marker.scale.x = 0.1;  // Line width
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    // Add polygon points to marker
    for (const auto & point : polygon.outer()) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = 0.0;  // Ground level
      marker.points.push_back(p);
    }

    // Close the polygon by adding the first point again
    if (!polygon.outer().empty()) {
      geometry_msgs::msg::Point first_point;
      first_point.x = polygon.outer()[0].x();
      first_point.y = polygon.outer()[0].y();
      first_point.z = 0.0;
      marker.points.push_back(first_point);
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray PerceptionFilterNode::createCropBoxPolygonMarkers(
  const std::vector<autoware::universe_utils::Polygon2d> & bounding_polygons,
  const std::string & frame_id)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < bounding_polygons.size(); ++i) {
    const auto & polygon = bounding_polygons[i];

    // Create line strip marker for crop box outline
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->now();
    marker.ns = "crop_box_polygons";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set marker properties
    marker.scale.x = 0.15;  // Line width
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    // Add polygon points to marker
    for (const auto & point : polygon.outer()) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = 0.0;  // Ground level
      marker.points.push_back(p);
    }

    // Close the polygon by adding the first point again
    if (!polygon.outer().empty()) {
      geometry_msgs::msg::Point first_point;
      first_point.x = polygon.outer()[0].x();
      first_point.y = polygon.outer()[0].y();
      first_point.z = 0.0;
      marker.points.push_back(first_point);
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
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
