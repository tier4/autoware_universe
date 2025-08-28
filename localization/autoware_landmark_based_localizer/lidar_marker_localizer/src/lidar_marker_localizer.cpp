// Copyright 2023 Autoware Foundation
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

#include "lidar_marker_localizer.hpp"

#include <autoware_point_types/types.hpp>
#include <rclcpp/qos.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/transform/transforms.hpp>

#include <geometry_msgs/msg/vector3.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

LidarMarkerLocalizer::LidarMarkerLocalizer() : Node("lidar_marker_localizer"), is_activated_(false)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  param_.enable_read_all_target_ids = this->declare_parameter<bool>("enable_read_all_target_ids");
  param_.target_ids = this->declare_parameter<std::vector<std::string>>("target_ids");

  param_.marker_name = this->declare_parameter<std::string>("marker_name");
  param_.resolution = this->declare_parameter<double>("resolution");
  param_.intensity_pattern = this->declare_parameter<std::vector<int64_t>>("intensity_pattern");
  param_.match_intensity_difference_threshold =
    this->declare_parameter<int64_t>("match_intensity_difference_threshold");
  param_.positive_match_num_threshold =
    this->declare_parameter<int64_t>("positive_match_num_threshold");
  param_.negative_match_num_threshold =
    this->declare_parameter<int64_t>("negative_match_num_threshold");
  param_.vote_threshold_for_detect_marker =
    this->declare_parameter<int64_t>("vote_threshold_for_detect_marker");
  param_.marker_to_vehicle_offset_y = this->declare_parameter<double>("marker_to_vehicle_offset_y");
  param_.marker_height_from_ground = this->declare_parameter<double>("marker_height_from_ground");
  param_.self_pose_timeout_sec = this->declare_parameter<double>("self_pose_timeout_sec");
  param_.self_pose_distance_tolerance_m =
    this->declare_parameter<double>("self_pose_distance_tolerance_m");
  param_.limit_distance_from_self_pose_to_nearest_marker =
    this->declare_parameter<double>("limit_distance_from_self_pose_to_nearest_marker");
  param_.limit_distance_from_self_pose_to_marker =
    this->declare_parameter<double>("limit_distance_from_self_pose_to_marker");
  std::vector<double> base_covariance =
    this->declare_parameter<std::vector<double>>("base_covariance");
  for (std::size_t i = 0; i < base_covariance.size(); ++i) {
    param_.base_covariance[i] = base_covariance[i];
  }

  param_.enable_save_log = this->declare_parameter<bool>("enable_save_log");
  param_.savefile_directory_path = this->declare_parameter<std::string>("savefile_directory_path");
  param_.savefile_name = this->declare_parameter<std::string>("savefile_name");
  param_.save_frame_id = this->declare_parameter<std::string>("save_frame_id");


  ekf_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.self_pose_timeout_sec, param_.self_pose_distance_tolerance_m);

  rclcpp::CallbackGroup::SharedPtr points_callback_group;
  points_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto points_sub_opt = rclcpp::SubscriptionOptions();
  points_sub_opt.callback_group = points_callback_group;
  sub_points_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud_ex", rclcpp::QoS(1).best_effort(),
    std::bind(&LidarMarkerLocalizer::points_callback, this, _1), points_sub_opt);

  rclcpp::CallbackGroup::SharedPtr self_pose_callback_group;
  self_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto self_pose_sub_opt = rclcpp::SubscriptionOptions();
  self_pose_sub_opt.callback_group = self_pose_callback_group;
  sub_self_pose_ = this->create_subscription<PoseWithCovarianceStamped>(
    "~/input/ekf_pose", rclcpp::QoS(1),
    std::bind(&LidarMarkerLocalizer::self_pose_callback, this, _1), self_pose_sub_opt);
  sub_map_bin_ = this->create_subscription<HADMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&LidarMarkerLocalizer::map_bin_callback, this, _1));

  pub_base_link_pose_with_covariance_on_map_ =
    this->create_publisher<PoseWithCovarianceStamped>("~/output/pose_with_covariance", 10);
  rclcpp::QoS qos_marker = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_marker.transient_local();
  qos_marker.reliable();
  pub_marker_mapped_ = this->create_publisher<MarkerArray>("~/debug/marker_mapped", qos_marker);
  pub_marker_detected_ = this->create_publisher<PoseArray>("~/debug/marker_detected", 10);
  pub_debug_pose_with_covariance_ =
    this->create_publisher<PoseWithCovarianceStamped>("~/debug/pose_with_covariance", 10);
  pub_marker_pointcloud = this->create_publisher<PointCloud2>("~/debug/marker_pointcloud", 10);

  pub_center_intensity_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/center_intensity_grid", 10);
  pub_positive_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/positive_grid", 10);
  pub_negative_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/negative_grid", 10);
  pub_matched_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/matched_grid", 10);
  pub_vote_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/debug/vote_grid", 10);


  service_trigger_node_ = this->create_service<SetBool>(
    "~/service/trigger_node_srv",
    std::bind(&LidarMarkerLocalizer::service_trigger_node, this, _1, _2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), points_callback_group);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  diagnostics_module_.reset(new DiagnosticsModule(this, "localization", ""));
}

void LidarMarkerLocalizer::initialize_diagnostics()
{
  diagnostics_module_->clear();
  diagnostics_module_->addKeyValue("is_received_map", false);
  diagnostics_module_->addKeyValue("is_received_self_pose", false);
  diagnostics_module_->addKeyValue("detect_marker_num", 0);
  diagnostics_module_->addKeyValue("distance_self_pose_to_nearest_marker", 0.0);
  diagnostics_module_->addKeyValue("distance_self_pose_to_nearest_marker_y", 0.0);
  diagnostics_module_->addKeyValue(
    "limit_distance_from_self_pose_to_nearest_marker",
    param_.limit_distance_from_self_pose_to_nearest_marker);
  diagnostics_module_->addKeyValue("distance_lanelet2_marker_to_detected_marker", 0.0);
  diagnostics_module_->addKeyValue(
    "limit_distance_from_lanelet2_marker_to_detected_marker",
    param_.limit_distance_from_self_pose_to_marker);
}

void LidarMarkerLocalizer::map_bin_callback(const HADMapBin::ConstSharedPtr & msg)
{
  if (param_.enable_read_all_target_ids) {
    landmark_manager_.parse_landmarks(msg, param_.marker_name);
  }
  else {
    landmark_manager_.parse_landmarks(msg, param_.marker_name, param_.target_ids);
  }
  const MarkerArray marker_msg = landmark_manager_.get_landmarks_as_marker_array_msg();
  pub_marker_mapped_->publish(marker_msg);
}

void LidarMarkerLocalizer::self_pose_callback(
  const PoseWithCovarianceStamped::ConstSharedPtr & self_pose_msg_ptr)
{
  // TODO(YamatoAndo)
  // if (!is_activated_) return;

  if (self_pose_msg_ptr->header.frame_id == "map") {
    ekf_pose_buffer_->push_back(self_pose_msg_ptr);
  } else {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), 1000,
      "Received initial pose message with frame_id "
        << self_pose_msg_ptr->header.frame_id << ", but expected map. "
        << "Please check the frame_id in the input topic and ensure it is correct.");
  }
}

void LidarMarkerLocalizer::points_callback(const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  initialize_diagnostics();

  main_process(std::move(points_msg_ptr));

  diagnostics_module_->publish();
}

void LidarMarkerLocalizer::main_process(const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  const builtin_interfaces::msg::Time sensor_ros_time = points_msg_ptr->header.stamp;

  // (1) check if the map have be received
  const std::vector<landmark_manager::Landmark> map_landmarks = landmark_manager_.get_landmarks();
  const bool is_received_map = !map_landmarks.empty();
  diagnostics_module_->addKeyValue("is_received_map", is_received_map);
  if (!is_received_map) {
    std::stringstream message;
    message << "Not receive the landmark information. Please check if the vector map is being "
            << "published and if the landmark information is correctly specified.";
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // (2) get Self Pose
  const std::optional<SmartPoseBuffer::InterpolateResult> interpolate_result =
    ekf_pose_buffer_->interpolate(sensor_ros_time);

  const bool is_received_self_pose = interpolate_result != std::nullopt;
  diagnostics_module_->addKeyValue("is_received_self_pose", is_received_self_pose);
  if (!is_received_self_pose) {
    std::stringstream message;
    message << "Could not get self_pose. Please check if the self pose is being published and if "
            << "the timestamp of the self pose is correctly specified";
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  ekf_pose_buffer_->pop_old(sensor_ros_time);
  const Pose self_pose = interpolate_result.value().interpolated_pose.pose.pose;

  // (3) detect marker
  const std::vector<landmark_manager::Landmark> detected_landmarks =
    detect_landmarks(points_msg_ptr);

  const bool is_detected_marker = !detected_landmarks.empty();
  diagnostics_module_->addKeyValue("detect_marker_num", detected_landmarks.size());

  // (4) check distance to the nearest marker
  const landmark_manager::Landmark nearest_marker = get_nearest_landmark(self_pose, map_landmarks);
  const Pose nearest_marker_pose_on_base_link =
    autoware::universe_utils::inverseTransformPose(nearest_marker.pose, self_pose);

  const double distance_from_self_pose_to_nearest_marker =
    std::abs(nearest_marker_pose_on_base_link.position.x);
  diagnostics_module_->addKeyValue(
    "distance_self_pose_to_nearest_marker", distance_from_self_pose_to_nearest_marker);

  const double distance_from_self_pose_to_nearest_marker_y =
    std::abs(nearest_marker_pose_on_base_link.position.y - param_.marker_to_vehicle_offset_y);
  diagnostics_module_->addKeyValue(
    "distance_from_self_pose_to_nearest_marker_y", distance_from_self_pose_to_nearest_marker_y);

  const bool is_exist_marker_within_self_pose =
    (distance_from_self_pose_to_nearest_marker < param_.limit_distance_from_self_pose_to_nearest_marker)
    && (distance_from_self_pose_to_nearest_marker_y < 1.0);

  if (!is_detected_marker) {
    if (!is_exist_marker_within_self_pose) {
      std::stringstream message;
      message << "Could not detect marker, because the distance from self_pose to nearest_marker "
              << "is too far (" << distance_from_self_pose_to_nearest_marker << " [m]).";
      diagnostics_module_->updateLevelAndMessage(
        diagnostic_msgs::msg::DiagnosticStatus::OK, message.str());
    } else {
      std::stringstream message;
      message << "Could not detect marker, although the distance from self_pose to nearest_marker "
              << "is near (" << distance_from_self_pose_to_nearest_marker << " [m]).";
      RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      diagnostics_module_->updateLevelAndMessage(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }
    return;
  }

  // for debug
  if (pub_marker_detected_->get_subscription_count() > 0) {
    PoseArray pose_array_msg;
    pose_array_msg.header.stamp = sensor_ros_time;
    pose_array_msg.header.frame_id = "map";
    for (const landmark_manager::Landmark & landmark : detected_landmarks) {
      const Pose detected_marker_on_map =
        autoware::universe_utils::transformPose(landmark.pose, self_pose);
      pose_array_msg.poses.push_back(detected_marker_on_map);
    }
    pub_marker_detected_->publish(pose_array_msg);
  }

  // (4) calculate diff pose
  const Pose new_self_pose =
    landmark_manager_.calculate_new_self_pose(detected_landmarks, self_pose, false);

  const double diff_x = new_self_pose.position.x - self_pose.position.x;
  const double diff_y = new_self_pose.position.y - self_pose.position.y;

  const double diff_norm = std::hypot(diff_x, diff_y);
  const bool is_exist_marker_within_lanelet2_map =
    diff_norm < param_.limit_distance_from_self_pose_to_marker;

  diagnostics_module_->addKeyValue("distance_lanelet2_marker_to_detected_marker", diff_norm);
  if (!is_exist_marker_within_lanelet2_map) {
    std::stringstream message;
    message << "The distance from lanelet2 to the detect marker is too far(" << diff_norm
            << " [m]). The limit is " << param_.limit_distance_from_self_pose_to_marker << ".";
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_module_->updateLevelAndMessage(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  const landmark_manager::Landmark nearest_detected_landmark =
    get_nearest_landmark(self_pose, detected_landmarks);
  save_intensity(points_msg_ptr, nearest_detected_landmark.pose);

  // (5) Apply diff pose to self pose
  // only x and y is changed
  PoseWithCovarianceStamped result;
  result.header.stamp = sensor_ros_time;
  result.header.frame_id = "map";
  result.pose.pose.position.x = new_self_pose.position.x;
  result.pose.pose.position.y = new_self_pose.position.y;
  result.pose.pose.position.z = new_self_pose.position.z;
  result.pose.pose.orientation = self_pose.orientation;

  // set covariance
  const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
    result.pose.pose.orientation.w, result.pose.pose.orientation.x, result.pose.pose.orientation.y,
    result.pose.pose.orientation.z);
  const Eigen::Matrix3d map_to_base_link_rotation =
    map_to_base_link_quat.normalized().toRotationMatrix();
  result.pose.covariance = rotate_covariance(param_.base_covariance, map_to_base_link_rotation);

  pub_base_link_pose_with_covariance_on_map_->publish(result);
  pub_debug_pose_with_covariance_->publish(result);
}

void LidarMarkerLocalizer::service_trigger_node(
  const SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res)
{
  is_activated_ = req->data;
  if (is_activated_) {
    ekf_pose_buffer_->clear();
  } else {
  }
  res->success = true;
}

std::vector<landmark_manager::Landmark> LidarMarkerLocalizer::detect_landmarks(
  const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
  // TODO(YamatoAndo)
  // Transform sensor_frame to base_link

  pcl::PointCloud<autoware_point_types::PointXYZIRADRT>::Ptr points_ptr(
    new pcl::PointCloud<autoware_point_types::PointXYZIRADRT>);
  pcl::fromROSMsg(*points_msg_ptr, *points_ptr);

  if (points_ptr->empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No points!");
    return std::vector<landmark_manager::Landmark>{};
  }

  uint16_t lower_ring_id = 128;
  uint16_t upper_ring_id = 0;
  for (const autoware_point_types::PointXYZIRADRT & point : points_ptr->points) {
    lower_ring_id = std::min(point.ring, lower_ring_id);
    upper_ring_id = std::max(point.ring, upper_ring_id);
  }
  uint16_t ring_num = upper_ring_id - lower_ring_id + 1;

  std::vector<pcl::PointCloud<autoware_point_types::PointXYZIRADRT>> ring_points(ring_num);

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  for (const autoware_point_types::PointXYZIRADRT & point : points_ptr->points) {
    ring_points[point.ring - lower_ring_id].push_back(point);
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
  }

  // Check that the leaf size is not too small, given the size of the data
  const int bin_num = static_cast<int>((max_x - min_x) / param_.resolution + 1);

  if (bin_num < static_cast<int>(param_.intensity_pattern.size())) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "bin_num is too small!");
    return std::vector<landmark_manager::Landmark>{};
  }

  // initialize variables
  std::vector<int> vote(bin_num, 0);
  std::vector<float> reference_ring_y(bin_num, std::numeric_limits<float>::max());

  nav_msgs::msg::OccupancyGrid center_intensity_grid_msg;
  center_intensity_grid_msg.header = points_msg_ptr->header;
  center_intensity_grid_msg.header.frame_id = "base_link";
  center_intensity_grid_msg.info.map_load_time = center_intensity_grid_msg.header.stamp;
  center_intensity_grid_msg.info.resolution = param_.resolution;
  center_intensity_grid_msg.info.width = bin_num; 
  center_intensity_grid_msg.info.height = ring_num;
  center_intensity_grid_msg.info.origin.position.x = min_x;
  // center_intensity_grid_msg.info.origin.position.y = param_.marker_to_vehicle_offset_y - center_intensity_grid_msg.info.height * center_intensity_grid_msg.info.resolution / 2.0;
  center_intensity_grid_msg.info.origin.position.y = param_.marker_to_vehicle_offset_y;
  center_intensity_grid_msg.info.origin.position.z = param_.marker_height_from_ground + center_intensity_grid_msg.info.height * center_intensity_grid_msg.info.resolution / 2.0;
  center_intensity_grid_msg.info.origin.orientation = autoware::universe_utils::createQuaternionFromRPY(-M_PI/2.0, 0.0, 0.0);
  center_intensity_grid_msg.data = std::vector<int8_t>(center_intensity_grid_msg.info.width * center_intensity_grid_msg.info.height, -1);

  nav_msgs::msg::OccupancyGrid positive_grid_msg;
  positive_grid_msg = center_intensity_grid_msg;

  nav_msgs::msg::OccupancyGrid negative_grid_msg;
  negative_grid_msg = center_intensity_grid_msg;

  nav_msgs::msg::OccupancyGrid matched_grid_msg;
  matched_grid_msg = center_intensity_grid_msg;
  
  // for each ring
  for (const pcl::PointCloud<autoware_point_types::PointXYZIRADRT> & one_ring : ring_points) {
    if (one_ring.empty()) {
      continue;
    }

    std::vector<double> intensity_sum(bin_num, 0.0);
    std::vector<int> intensity_num(bin_num, 0);
    std::vector<double> average_intensity(bin_num, 0.0);
    const size_t ring_id = one_ring.front().ring - ring_points.front().points.front().ring;

    for (const autoware_point_types::PointXYZIRADRT & point : one_ring.points) {
      const int bin_index = static_cast<int>((point.x - min_x) / param_.resolution);
      intensity_sum[bin_index] += point.intensity;
      intensity_num[bin_index]++;
      if (point.ring == 5) {
        reference_ring_y[bin_index] = std::min(reference_ring_y[bin_index], point.y);
      }

    }

    // calc average
    for (int bin_index = 0; bin_index < bin_num; bin_index++) {
      if (intensity_num[bin_index] == 0) {
        continue;
      }

      average_intensity[bin_index] = intensity_sum[bin_index] / intensity_num[bin_index];
    }

    // pattern matching
    for (size_t i = 0; i <= bin_num - param_.intensity_pattern.size(); i++) {
      int64_t pos = 0;
      int64_t neg = 0;
      double min_intensity = std::numeric_limits<double>::max();
      double max_intensity = std::numeric_limits<double>::lowest();

      // find max_min
      for (size_t j = 0; j < param_.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }

        min_intensity = std::min(min_intensity, average_intensity[i + j]);
        max_intensity = std::max(max_intensity, average_intensity[i + j]);
      }

      if (max_intensity <= min_intensity) {
        continue;
      }

      const double center_intensity = (max_intensity - min_intensity) / 2.0 + min_intensity;
      for (size_t j = 0; j < param_.intensity_pattern.size(); j++) {
        if (intensity_num[i + j] == 0) {
          continue;
        }

        if (param_.intensity_pattern[j] == 1) {
          // check positive
          if (
            average_intensity[i + j] >
            center_intensity + param_.match_intensity_difference_threshold) {
            pos++;
          }
        } else if (param_.intensity_pattern[j] == -1) {
          // check negative
          if (
            average_intensity[i + j] <
            center_intensity - param_.match_intensity_difference_threshold) {
            neg++;
          }
        } else {
          // ignore param_.intensity_pattern[j] == 0
        }
      }
      const size_t bin_position = i + param_.intensity_pattern.size() / 2 + ring_id * bin_num;
      center_intensity_grid_msg.data[bin_position] = std::min(static_cast<int>(center_intensity), 100);
      positive_grid_msg.data[bin_position] = std::min(static_cast<int>(pos * (100/param_.positive_match_num_threshold)), 100);
      negative_grid_msg.data[bin_position] = std::min(static_cast<int>(neg * (100/param_.negative_match_num_threshold)), 100);

      if (
        pos >= param_.positive_match_num_threshold && neg >= param_.negative_match_num_threshold) {
        matched_grid_msg.data[bin_position] = 100;
        vote[i]++;
      }
    }
  }
  std::vector<landmark_manager::Landmark> detected_landmarks;

  for (size_t i = 0; i < bin_num - param_.intensity_pattern.size(); i++) {
    if (vote[i] > param_.vote_threshold_for_detect_marker) {
      const double bin_position =
        static_cast<double>(i) + static_cast<double>(param_.intensity_pattern.size()) / 2.0;
      Pose marker_pose_on_base_link;
      marker_pose_on_base_link.position.x = bin_position * param_.resolution + min_x;
      marker_pose_on_base_link.position.y = reference_ring_y[i];
      marker_pose_on_base_link.position.z = param_.marker_height_from_ground;
      marker_pose_on_base_link.orientation =
        autoware::universe_utils::createQuaternionFromRPY(M_PI_2, 0.0, 0.0);  // TODO(YamatoAndo)
      detected_landmarks.push_back(landmark_manager::Landmark{"0", marker_pose_on_base_link});
    }
  }

  nav_msgs::msg::OccupancyGrid vote_grid_msg;
  vote_grid_msg.header = points_msg_ptr->header;
  vote_grid_msg.header.frame_id = "base_link";
  vote_grid_msg.info.map_load_time = vote_grid_msg.header.stamp;
  vote_grid_msg.info.resolution = param_.resolution;
  vote_grid_msg.info.width = bin_num; 
  vote_grid_msg.info.height = 1;
  vote_grid_msg.info.origin.position.x = min_x;
  vote_grid_msg.info.origin.position.y = (param_.marker_to_vehicle_offset_y >= 0.0) ? 1.0 : -1.0;
  vote_grid_msg.info.origin.position.z = 0.0;
  vote_grid_msg.info.origin.orientation.x = 0.0;
  vote_grid_msg.info.origin.orientation.y = 0.0;
  vote_grid_msg.info.origin.orientation.z = 0.0;
  vote_grid_msg.info.origin.orientation.w = 1.0;
  vote_grid_msg.data = std::vector<int8_t>(vote_grid_msg.info.width * vote_grid_msg.info.height);

  int vote_max = 1;  // NOTE: set non-zero to avoid zero division error
  for (int i = 0; i < bin_num ; i++) {
    vote_max = std::max(vote[i], vote_max);
  }

  for (size_t i = 0; i < bin_num - param_.intensity_pattern.size(); i++) {
    const size_t bin_position = i + param_.intensity_pattern.size() / 2;
    vote_grid_msg.data[bin_position] = vote[i] * (100 / vote_max);
  }

  pub_center_intensity_grid->publish(center_intensity_grid_msg);
  pub_positive_grid->publish(positive_grid_msg);
  pub_negative_grid->publish(negative_grid_msg);
  pub_matched_grid->publish(matched_grid_msg);
  pub_vote_grid->publish(vote_grid_msg);


  return detected_landmarks;
}

landmark_manager::Landmark LidarMarkerLocalizer::get_nearest_landmark(
  const geometry_msgs::msg::Pose & self_pose,
  const std::vector<landmark_manager::Landmark> & landmarks) const
{
  landmark_manager::Landmark nearest_landmark;
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & landmark : landmarks) {
    const double curr_distance =
      autoware::universe_utils::calcDistance3d(landmark.pose.position, self_pose.position);

    if (curr_distance > min_distance) {
      continue;
    }

    min_distance = curr_distance;
    nearest_landmark = landmark;
  }
  return nearest_landmark;
}

std::array<double, 36> LidarMarkerLocalizer::rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation) const
{
  std::array<double, 36> ret_covariance = src_covariance;

  Eigen::Matrix3d src_cov;
  src_cov << src_covariance[0], src_covariance[1], src_covariance[2], src_covariance[6],
    src_covariance[7], src_covariance[8], src_covariance[12], src_covariance[13],
    src_covariance[14];

  Eigen::Matrix3d ret_cov;
  ret_cov = rotation * src_cov * rotation.transpose();

  for (Eigen::Index i = 0; i < 3; ++i) {
    ret_covariance[i] = ret_cov(0, i);
    ret_covariance[i + 6] = ret_cov(1, i);
    ret_covariance[i + 12] = ret_cov(2, i);
  }

  return ret_covariance;
}

void LidarMarkerLocalizer::save_intensity(
  const PointCloud2::ConstSharedPtr & points_msg_ptr, const Pose marker_pose)
{
  // convert from ROSMsg to PCL
  pcl::PointCloud<autoware_point_types::PointXYZIRADRT>::Ptr points_ptr(
    new pcl::PointCloud<autoware_point_types::PointXYZIRADRT>);
  pcl::fromROSMsg(*points_msg_ptr, *points_ptr);

  pcl::PointCloud<autoware_point_types::PointXYZIRADRT>::Ptr marker_points_ptr(
    new pcl::PointCloud<autoware_point_types::PointXYZIRADRT>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr marker_points_xyzi_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  std::vector<int> ring_array;

  // extract marker pointcloud
  for (const autoware_point_types::PointXYZIRADRT & point : points_ptr->points) {
    const double xy_distance = std::sqrt(
      std::pow(point.x - marker_pose.position.x, 2.0) +
      std::pow(point.y - marker_pose.position.y, 2.0));
    // const double z_distance = std::fabs(point.z - marker_pose.position.z);
    // if (xy_distance < 0.40 && z_distance < 0.55) {
    if (xy_distance < 0.40) {
      marker_points_ptr->push_back(point);

      pcl::PointXYZI xyzi;
      xyzi.x = point.x;
      xyzi.y = point.y;
      xyzi.z = point.z;
      xyzi.intensity = point.intensity;
      marker_points_xyzi_ptr->push_back(xyzi);

      ring_array.push_back(point.ring);
    }
  }

  // transform input_frame to save_frame_id
  pcl::PointCloud<pcl::PointXYZI>::Ptr marker_points_sensor_frame_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  transform_sensor_measurement(
    param_.save_frame_id, points_msg_ptr->header.frame_id, marker_points_xyzi_ptr,
    marker_points_sensor_frame_ptr);

  // visualize for debug
  marker_points_sensor_frame_ptr->width = marker_points_sensor_frame_ptr->size();
  marker_points_sensor_frame_ptr->height = 1;
  marker_points_sensor_frame_ptr->is_dense = false;

  PointCloud2 viz_pointcloud_msg;
  pcl::toROSMsg(*marker_points_sensor_frame_ptr, viz_pointcloud_msg);
  viz_pointcloud_msg.header = points_msg_ptr->header;
  viz_pointcloud_msg.header.frame_id = param_.save_frame_id;

  pub_marker_pointcloud->publish(viz_pointcloud_msg);

  if (!param_.enable_save_log) {
    return;
  }

  // to csv format
  std::stringstream log_message;
  size_t i = 0;
  log_message << "point.position.x,point.position.y,point.position.z,point.intensity,point.ring"
              << std::endl;
  for (const auto & point : marker_points_sensor_frame_ptr->points) {
    log_message << point.x;
    log_message << "," << point.y;
    log_message << "," << point.z;
    log_message << "," << point.intensity;
    log_message << "," << ring_array.at(i++);
    log_message << std::endl;
  }

  // create file name
  const double times_seconds = rclcpp::Time(points_msg_ptr->header.stamp).seconds();
  double time_integer_tmp;
  double time_decimal = std::modf(times_seconds, &time_integer_tmp);
  long int time_integer = static_cast<long int>(time_integer_tmp);
  struct tm * time_info;
  time_info = std::localtime(&time_integer);
  std::stringstream file_name;
  file_name << param_.savefile_name << std::put_time(time_info, "%Y%m%d-%H%M%S") << "-"
            << std::setw(3) << std::setfill('0') << static_cast<int>((time_decimal)*1000) << ".csv";

  // write log_message to file
  std::filesystem::path savefile_directory_path = param_.savefile_directory_path;
  std::filesystem::create_directories(savefile_directory_path);
  std::ofstream csv_file(savefile_directory_path.append(file_name.str()));
  csv_file << log_message.str();
  csv_file.close();
}

template <typename PointType>
void LidarMarkerLocalizer::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointType>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointType>> & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_WARN(
      this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());
    // Since there is no clear error handling policy, temporarily return as is.
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    autoware::universe_utils::transform2pose(transform);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  autoware::universe_utils::transformPointCloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}
