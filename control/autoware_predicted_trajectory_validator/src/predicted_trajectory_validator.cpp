// Copyright 2026 TIER IV, Inc.
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

#include "autoware/predicted_trajectory_validator/predicted_trajectory_validator.hpp"

#include "autoware/lanelet2_utils/conversion.hpp"
#include "autoware/predicted_trajectory_validator/utils.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <utility>

namespace autoware::predicted_trajectory_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{
void update_invalid_count(ValidationResult & result)
{
  result.invalid_count = result.is_valid ? 0 : result.invalid_count + 1;
}
}  // namespace

PredictedTrajectoryValidator::PredictedTrajectoryValidator(const rclcpp::NodeOptions & options)
: Node("predicted_trajectory_validator", options),
  param_listener_(get_node_parameters_interface()),
  params_(param_listener_.get_params()),
  vehicle_info_(),
  uncrossable_bound_departure_validator_(get_logger(), params_),
  collision_validator_(get_logger(), params_, hazard_time_keeper_),
  hazard_warning_(
    get_logger(), params_,
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/hazard/markers", 1),
    hazard_time_keeper_)
{
  using std::placeholders::_1;

  sub_predicted_traj_ = create_subscription<Trajectory>(
    "~/input/predicted_trajectory", 1,
    std::bind(&PredictedTrajectoryValidator::on_predicted_trajectory, this, _1));
  sub_operational_state_ =
    autoware_utils::InterProcessPollingSubscriber<OperationModeState>::create_subscription(
      this, "~/input/operational_mode_state", 1);
  sub_kinematics_ =
    autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>::create_subscription(
      this, "~/input/kinematics", 1);
  sub_measured_acc_ =
    autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>::create_subscription(
      this, "~/input/measured_acceleration", 1);
  sub_lanelet_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&PredictedTrajectoryValidator::on_lanelet_map, this, _1));
  sub_objects_ =
    autoware_utils::InterProcessPollingSubscriber<PredictedObjects>::create_subscription(
      this, "~/input/objects", 1);
  sub_pointcloud_ = autoware_utils::InterProcessPollingSubscriber<PointCloud2>::create_subscription(
    this, "~/input/pointcloud", autoware_utils::single_depth_sensor_qos());

  pub_status_ =
    create_publisher<PredictedTrajectoryValidatorStatus>("~/output/validation_status", 1);

  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);

  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);

  pub_hazard_alert_ = create_publisher<HazardAlert>("~/output/hazard_alert", 1);
  pub_hazard_processing_time_ = create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/hazard/processing_time_ms", 1);
  pub_hazard_processing_time_detail_ = create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
    "~/debug/hazard/processing_time_detail_ms", 1);
  hazard_time_keeper_->add_reporter(pub_hazard_processing_time_detail_);

  debug_pose_publisher_ = std::make_shared<PredictedTrajectoryValidatorDebugMarkerPublisher>(this);

  setup_parameters();

  for (auto & result : results_) {
    result.is_valid = true;
  }

  setup_diag();
}

void PredictedTrajectoryValidator::setup_parameters()
{
  diag_error_count_threshold_ = params_.diag_error_count_threshold;

  try {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  } catch (...) {
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
    RCLCPP_ERROR(
      get_logger(),
      "failed to get vehicle info. use default value. vehicle_info_.front_overhang_m: %.2f, "
      "vehicle_info_.wheel_base_m: %.2f",
      vehicle_info_.front_overhang_m, vehicle_info_.wheel_base_m);
  }
}

void PredictedTrajectoryValidator::set_status(
  DiagnosticStatusWrapper & stat, const ValidationResult & result, const std::string & msg) const
{
  if (result.is_valid) {
    stat.summary(DiagnosticStatus::OK, "validated.");
  } else if (result.invalid_count < diag_error_count_threshold_) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(result.invalid_count) + " < " +
                          std::to_string(diag_error_count_threshold_) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
  } else {
    stat.summary(DiagnosticStatus::ERROR, msg);
  }
}

void PredictedTrajectoryValidator::setup_diag()
{
  auto & d = diag_updater_;
  d.setHardwareID("predicted_trajectory_validator");

  d.add("uncrossable_bound_departure", [&](auto & stat) {
    set_status(
      stat, results_[UNCROSSABLE_BOUND_DEPARTURE],
      "The control predicted trajectory departs an uncrossable boundary.");
  });
}

bool PredictedTrajectoryValidator::infer_autonomous_control_state(
  const OperationModeState::ConstSharedPtr msg)
{
  return (msg->mode == OperationModeState::AUTONOMOUS) && (msg->is_autoware_control_enabled);
}

void PredictedTrajectoryValidator::on_lanelet_map(const LaneletMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
}

void PredictedTrajectoryValidator::on_predicted_trajectory(const Trajectory::ConstSharedPtr msg)
{
  stop_watch_.tic();

  // refresh parameters if they were updated at runtime
  if (param_listener_.is_old(params_)) {
    params_ = param_listener_.get_params();
    diag_error_count_threshold_ = params_.diag_error_count_threshold;
    uncrossable_bound_departure_validator_.update_parameters(params_);
    collision_validator_.update_parameters(params_);
    hazard_warning_.update_parameters(params_);
  }

  // prepare ros topics
  const auto waiting = [this](const auto topic_name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", topic_name);
    return;
  };

  OperationModeState::ConstSharedPtr operation_mode_msg = sub_operational_state_->take_data();
  if (operation_mode_msg) {
    flag_autonomous_control_enabled_ = infer_autonomous_control_state(operation_mode_msg);
  }
  Odometry::ConstSharedPtr kinematics_msg = sub_kinematics_->take_data();
  if (!kinematics_msg) {
    return waiting(sub_kinematics_->subscriber()->get_topic_name());
  }
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_msg = sub_measured_acc_->take_data();
  if (!acceleration_msg) {
    return waiting(sub_measured_acc_->subscriber()->get_topic_name());
  }
  const auto objects_msg = sub_objects_->take_data();
  const auto pointcloud_msg = sub_pointcloud_->take_data();

  HazardWarningSources hazard_sources;

  // pre process
  debug_pose_publisher_->clear_markers();

  // validation process
  visualization_msgs::msg::MarkerArray boundary_departure_markers;
  {
    autoware_utils_debug::ScopedTimeTrack st("hazard_warning", *hazard_time_keeper_);
    hazard_stop_watch_.tic();
    {
      autoware_utils_debug::ScopedTimeTrack st_departure(
        "uncrossable_bound_departure_validate", *hazard_time_keeper_);
      uncrossable_bound_departure_validator_.validate(
        results_[UNCROSSABLE_BOUND_DEPARTURE], *msg, *kinematics_msg, *acceleration_msg,
        lanelet_map_ptr_, vehicle_info_, boundary_departure_markers, hazard_sources);
    }
    const auto now = get_clock()->now();
    collision_validator_.validate(
      *msg, *kinematics_msg, objects_msg, vehicle_info_, pointcloud_msg, hazard_sources);
    if (
      msg->points.size() < 2 && (params_.collision_validator.enable_warning ||
                                 params_.uncrossable_bound_departure_validator.enable_warning)) {
      hazard_sources = HazardWarningSources::empty();
    }
    const auto alert = hazard_warning_.make_alert(hazard_sources, vehicle_info_, now);
    const double hazard_processing_time_ms = hazard_stop_watch_.toc();
    if (alert) {
      pub_hazard_alert_->publish(*alert);
      autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
      processing_time_msg.stamp = now;
      processing_time_msg.data = hazard_processing_time_ms;
      pub_hazard_processing_time_->publish(processing_time_msg);
    }
  }

  publish_boundary_departure_markers(std::move(boundary_departure_markers));

  if (!flag_autonomous_control_enabled_) {
    // if warnings or errors are being suppressed, printing simple logs
    if (!is_all_valid()) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Suppressing predicted trajectory validation during manual driving");
    }
    for (auto & result : results_) {
      result.is_valid = true;
    }
  }

  // post process
  for (auto & result : results_) {
    update_invalid_count(result);
  }

  diag_updater_.force_update();

  publish_debug_info(kinematics_msg->pose.pose);
  display_status();
}

void PredictedTrajectoryValidator::publish_debug_info(const geometry_msgs::msg::Pose & ego_pose)
{
  pub_status_->publish(to_msg());

  if (!results_[UNCROSSABLE_BOUND_DEPARTURE].is_valid) {
    geometry_msgs::msg::Pose front_pose = ego_pose;
    shift_pose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    debug_pose_publisher_->push_virtual_wall(front_pose, "BOUNDARY DEPARTURE");
  }
  debug_pose_publisher_->publish();

  // Publish ProcessingTime
  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch_.toc();
  pub_processing_time_->publish(processing_time_msg);
}

void PredictedTrajectoryValidator::publish_boundary_departure_markers(
  visualization_msgs::msg::MarkerArray markers)
{
  visualization_msgs::msg::MarkerArray output_markers;
  output_markers.markers.reserve(markers.markers.size() + 1);

  // Clear the markers published in the previous cycle before adding the current ones.
  visualization_msgs::msg::Marker delete_all_marker;
  delete_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  output_markers.markers.push_back(delete_all_marker);

  std::move(
    markers.markers.begin(), markers.markers.end(), std::back_inserter(output_markers.markers));

  pub_markers_->publish(output_markers);
}

bool PredictedTrajectoryValidator::is_all_valid() const
{
  return std::all_of(
    results_.begin(), results_.end(), [](const auto & result) { return result.is_valid; });
}

PredictedTrajectoryValidatorStatus PredictedTrajectoryValidator::to_msg() const
{
  PredictedTrajectoryValidatorStatus msg;
  msg.stamp = now();
  msg.uncrossable_bound_departure_result = results_[UNCROSSABLE_BOUND_DEPARTURE];
  return msg;
}

void PredictedTrajectoryValidator::display_status()
{
  static rclcpp::Clock clock{RCL_ROS_TIME};

  if (!results_[UNCROSSABLE_BOUND_DEPARTURE].is_valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), clock, 2000, "predicted trajectory cross uncrossable bound");
  }
}

}  // namespace autoware::predicted_trajectory_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::predicted_trajectory_validator::PredictedTrajectoryValidator)
