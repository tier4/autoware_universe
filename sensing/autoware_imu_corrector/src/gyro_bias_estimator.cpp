// Copyright 2023 TIER IV, Inc.
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

#include "gyro_bias_estimator.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::imu_corrector
{
GyroBiasEstimator::GyroBiasEstimator(const rclcpp::NodeOptions & options)
: rclcpp::Node(
    "gyro_bias_validator_scale",
    options),  // Todo(SergioReyesSan): Change node name to "gyro_bias_validator"
  gyro_bias_threshold_(declare_parameter<double>("gyro_bias_threshold")),
  angular_velocity_offset_x_(declare_parameter<double>("angular_velocity_offset_x")),
  angular_velocity_offset_y_(declare_parameter<double>("angular_velocity_offset_y")),
  angular_velocity_offset_z_(declare_parameter<double>("angular_velocity_offset_z")),
  timer_callback_interval_sec_(declare_parameter<double>("timer_callback_interval_sec")),
  diagnostics_updater_interval_sec_(declare_parameter<double>("diagnostics_updater_interval_sec")),
  straight_motion_ang_vel_upper_limit_(
    declare_parameter<double>("straight_motion_ang_vel_upper_limit")),
  estimate_scale_init_(declare_parameter<double>("estimate_scale_init")),
  ekf_variance_p_(declare_parameter<double>("ekf_variance_p")),
  ekf_process_noise_q_(declare_parameter<double>("ekf_process_noise_q")),
  ekf_measurement_noise_r_(declare_parameter<double>("ekf_measurement_noise_r")),
  time_window_secs_(declare_parameter<double>("time_window_secs")),
  threshold_scale_change_(declare_parameter<double>("threshold_scale_change")),
  threshold_error_rate_(declare_parameter<double>("threshold_error_rate")),
  num_consecutive_scale_change_(declare_parameter<double>("num_consecutive_scale_change")),
  min_allowed_scale_(declare_parameter<double>("min_allowed_scale")),
  max_allowed_scale_(declare_parameter<double>("max_allowed_scale")),
  scale_on_purpose_(declare_parameter<double>("scale_on_purpose")),
  bias_on_purpose_(declare_parameter<double>("bias_on_purpose")),
  drift_scale_(declare_parameter<double>("drift_scale")),
  drift_bias(declare_parameter<double>("drift_bias")),
  updater_(this),
  gyro_bias_(std::nullopt)
{
  updater_.setHardwareID(get_name());
  updater_.add("gyro_bias_validator", this, &GyroBiasEstimator::update_diagnostics);
  updater_.setPeriod(diagnostics_updater_interval_sec_);

  gyro_bias_estimation_module_ = std::make_unique<GyroBiasEstimationModule>();

  imu_sub_ = create_subscription<Imu>(
    "~/input/imu_raw", rclcpp::SensorDataQoS(),
    [this](const Imu::ConstSharedPtr msg) { callback_imu(msg); });
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odom", rclcpp::SensorDataQoS(),
    [this](const Odometry::ConstSharedPtr msg) { callback_odom(msg); });
  gyro_bias_pub_ = create_publisher<Vector3Stamped>("~/output/gyro_bias", rclcpp::SensorDataQoS());
  pose_sub_ = create_subscription<PoseWithCovarianceStamped>(
    "~/input/pose_ndt", rclcpp::SensorDataQoS(),
    [this](const PoseWithCovarianceStamped::ConstSharedPtr msg) { callback_pose(msg); });
  gyro_scale_pub_ =
    create_publisher<Vector3Stamped>("~/output/gyro_scale", rclcpp::SensorDataQoS());
  imu_scaled_pub_ = create_publisher<Imu>("~/output/imu_scaled", rclcpp::QoS{1});

  auto bound_timer_callback = std::bind(&GyroBiasEstimator::timer_callback, this);
  auto period_control = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timer_callback_interval_sec_));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(bound_timer_callback)>>(
    this->get_clock(), period_control, std::move(bound_timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);

  transform_listener_ = std::make_shared<autoware_utils::TransformListener>(this);

  // initialize diagnostics_info_
  {
    diagnostics_info_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics_info_.summary_message = "Not initialized";
    diagnostics_info_.gyro_bias_x_for_imu_corrector = std::nan("");
    diagnostics_info_.gyro_bias_y_for_imu_corrector = std::nan("");
    diagnostics_info_.gyro_bias_z_for_imu_corrector = std::nan("");
    diagnostics_info_.estimated_gyro_bias_x = std::nan("");
    diagnostics_info_.estimated_gyro_bias_y = std::nan("");
    diagnostics_info_.estimated_gyro_bias_z = std::nan("");
    diagnostics_info_.estimated_gyro_scale_x = std::nan("");
    diagnostics_info_.estimated_gyro_scale_y = std::nan("");
    diagnostics_info_.estimated_gyro_scale_z = std::nan("");
  }
  // initialize gyro_info
  {
    gyro_info_.bias_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    gyro_info_.bias_status_summary = "OK";
    gyro_info_.bias_summary_message = "Not initialized";
    gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    gyro_info_.scale_status_summary = "OK";
    gyro_info_.scale_summary_message = "Not initialized";
  }
  // EKF variables initialization
  estimated_scale = estimate_scale_init_;
  P = ekf_variance_p_;
  Q = ekf_process_noise_q_;
  R = ekf_measurement_noise_r_ * ekf_measurement_noise_r_;
  start_time_check_scale = this->get_clock()->now();
  previous_scale = estimate_scale_init_;
  final_scale_on_purpose_ = scale_on_purpose_;
  final_bias_on_purpose_ = bias_on_purpose_;
}

void GyroBiasEstimator::callback_imu(const Imu::ConstSharedPtr imu_msg_ptr)
{
  // Replace again the original code after testing (SergioReyesSan):
  // imu_frame_ = imu_msg_ptr->header.frame_id;
  // geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
  //   transform_listener_->get_latest_transform(imu_frame_, output_frame_);
  // if (!tf_imu2base_ptr) {
  //   RCLCPP_ERROR(
  //     this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
  //     (imu_frame_).c_str());
  //   return;
  // }

  // geometry_msgs::msg::Vector3Stamped gyro;
  // gyro.header.stamp = imu_msg_ptr->header.stamp;
  // gyro.vector = transform_vector3(imu_msg_ptr->angular_velocity, *tf_imu2base_ptr);

  // gyro_all_.push_back(gyro);

  static auto bias_final = bias_on_purpose_;
  static auto scale_final = scale_on_purpose_;
  Imu imu_msg = *imu_msg_ptr;
  imu_frame_ = imu_msg.header.frame_id;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_imu2base_ptr =
    transform_listener_->get_latest_transform(imu_frame_, output_frame_);
  if (!tf_imu2base_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_frame_).c_str());
    return;
  }

  // Add bias and scale on purpose

  scale_final += drift_scale_;
  bias_final += drift_bias;
  final_scale_on_purpose_ = scale_final;
  final_bias_on_purpose_ = bias_final;
  Imu imu_msg_mod = imu_msg;

  imu_msg_mod.header.stamp = this->now();
  imu_msg_mod.header.frame_id = "virtual_imu";  // imu_frame_;
  imu_msg_mod.angular_velocity.x = -scale_final * imu_msg.angular_velocity.x + bias_final;
  imu_msg_mod.angular_velocity.y = -scale_final * imu_msg.angular_velocity.y + bias_final;
  imu_msg_mod.angular_velocity.z = -scale_final * imu_msg.angular_velocity.z + bias_final;

  imu_scaled_pub_->publish(imu_msg_mod);

  geometry_msgs::msg::Vector3Stamped gyro;
  gyro.header.stamp = imu_msg_ptr->header.stamp;
  gyro.vector = transform_vector3(imu_msg_mod.angular_velocity, *tf_imu2base_ptr);
  // RCLCPP_INFO(this->get_logger(), "velz_rot %f", gyro.vector.z);
  gyro_all_.push_back(gyro);

  // EKF always update P
  gyro_yaw_rate = gyro.vector.z;
  P = P + Q;

  // Publish results for debugging
  if (gyro_bias_ != std::nullopt) {
    Vector3Stamped gyro_bias_msg;

    gyro_bias_msg.header.stamp = this->now();
    gyro_bias_msg.vector = gyro_bias_.value();

    gyro_bias_pub_->publish(gyro_bias_msg);
  }
}

void GyroBiasEstimator::callback_odom(const Odometry::ConstSharedPtr odom_msg_ptr)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = odom_msg_ptr->header;
  pose.pose = odom_msg_ptr->pose.pose;
  pose_buf_.push_back(pose);
}

void GyroBiasEstimator::callback_pose(const PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr)
{
  static rclcpp::Time last_time_rx_pose_ = this->get_clock()->now();
  double dt = (this->get_clock()->now() - last_time_rx_pose_).seconds();
  last_time_rx_pose_ = this->get_clock()->now();
  static int window_scale_change = 0;  // Initialized once
  static double previous_yaw_angle = 0.0;
  auto pose_frame_ = pose_msg_ptr->header.frame_id;

  double dt = (this->get_clock()->now() - last_time_rx_pose_).seconds();
  if (dt == 0.0) {
    throw std::runtime_error("dt_pose is zero");
  }

  last_time_rx_pose_ = this->get_clock()->now();

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_base2pose_ptr =
    transform_listener_->get_latest_transform(pose_frame_, output_frame_);
  if (!tf_base2pose_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", pose_frame_.c_str(), output_frame_.c_str());

    diagnostics_info_.summary_message =
      "Skipped update (tf between base and pose is not available)";
    return;
  }

  auto quat_rotated = tf_base2pose_ptr->transform.rotation;

  // Convert to tf2 quaternion
  tf2::Quaternion quat(quat_rotated.x, quat_rotated.y, quat_rotated.z, quat_rotated.w);

  // Convert to Euler angles
  double roll_ndt, pitch_ndt, yaw_ndt;
  tf2::Matrix3x3(quat).getRPY(roll_ndt, pitch_ndt, yaw_ndt);

  double unwrapped_angle = yaw_ndt;
  double delta_angle = yaw_ndt - previous_yaw_angle;

  // Wrapping angle in case of big jumps
  if (delta_angle > 180.0) {
    unwrapped_angle -= 360.0;
  } else if (delta_angle < -180.0) {
    unwrapped_angle += 360.0;
  }

  ndt_yaw_rate = (unwrapped_angle - previous_yaw_angle) / dt;

  previous_yaw_angle = unwrapped_angle;
  try {
    if (gyro_bias_.has_value()) {
      auto H = ndt_yaw_rate;
      // auto y = gyro_yaw_rate - (estimated_scale * ndt_yaw_rate + gyro_bias_.value().z);
      auto y = gyro_yaw_rate - (estimated_scale * ndt_yaw_rate + gyro_bias_.value().z);
      auto S = H * P * H + R;
      auto K = P * H / S;
      estimated_scale = estimated_scale + K * y;
      P = (1 - K * H) * P;

      geometry_msgs::msg::Vector3Stamped vector_scale;

      if (estimated_scale < 0.0) {
        gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        gyro_info_.scale_status_summary = "ERR";
        gyro_info_.scale_summary_message =
          "Scale is negative, please check the TF or unstable bias.";
      }

      vector_scale.header.stamp = this->now();
      if (estimated_scale < min_allowed_scale_) {
        gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        gyro_info_.scale_status_summary = "ERR";
        gyro_info_.scale_summary_message =
          "Scale is under the minimum, check the IMU, NDT device or TF.";
        vector_scale.vector.z = min_allowed_scale_;
      } else if (estimated_scale > max_allowed_scale_) {
        gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        gyro_info_.scale_status_summary = "ERR";
        gyro_info_.scale_summary_message =
          "Scale is over the maximum, check the IMU, NDT device or TF.";
        vector_scale.vector.z = max_allowed_scale_;
      } else {
        vector_scale.vector.z = estimated_scale;
      }
      vector_scale.vector.z = previous_scale;
      // Scale on x , y axis is not estimated, but set to 1.0 for consistency
      vector_scale.vector.x = 1.0;
      vector_scale.vector.y = 1.0;
      gyro_scale_pub_->publish(vector_scale);
      diagnostics_info_.estimated_gyro_scale_z = estimated_scale;
      scale_list_all_.push_back(estimated_scale);

      if ((this->get_clock()->now() - start_time_check_scale).seconds() > time_window_secs_) {
        const std::vector<double> scale_all = scale_list_all_;
        scale_list_all_.clear();
        double mean_scale_window =
          std::accumulate(scale_all.begin(), scale_all.end(), 0.0) / scale_all.size();
        start_time_check_scale = this->get_clock()->now();

        if (
          std::abs(mean_scale_window - previous_scale) >
          std::abs(previous_scale * threshold_scale_change_)) {
          window_scale_change++;
          gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
          gyro_info_.scale_status_summary = "WARN";
          gyro_info_.scale_summary_message = "Gyro scale unstable.";
          if (window_scale_change >= static_cast<int>(num_consecutive_scale_change_)) {
            gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            gyro_info_.scale_status_summary = "WARN";
            gyro_info_.scale_summary_message = "Scale changed too much in a short time.";
            previous_scale = estimated_scale;
            window_scale_change = 0;
          }
        } else {
          window_scale_change--;
          gyro_info_.scale_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
          gyro_info_.scale_status_summary = "OK";
          gyro_info_.scale_summary_message = "Scale without changes.";
          if (window_scale_change < 0) {
            window_scale_change = 0;
          }
        }
      }
    }
  } catch (...) {
    RCLCPP_INFO(this->get_logger(), "Error getting gyro bias value");
  }
}

void GyroBiasEstimator::timer_callback()
{
  if (pose_buf_.empty()) {
    gyro_info_.bias_summary_message = "Skipped update (pose_buf is empty).";
    diagnostics_info_.summary_message = "Skipped update (pose_buf is empty)";
    return;
  }

  // Copy data
  const std::vector<geometry_msgs::msg::PoseStamped> pose_buf = pose_buf_;
  const std::vector<geometry_msgs::msg::Vector3Stamped> gyro_all = gyro_all_;
  pose_buf_.clear();
  gyro_all_.clear();

  // Check time
  const rclcpp::Time t0_rclcpp_time = rclcpp::Time(pose_buf.front().header.stamp);
  const rclcpp::Time t1_rclcpp_time = rclcpp::Time(pose_buf.back().header.stamp);
  if (t1_rclcpp_time <= t0_rclcpp_time) {
    gyro_info_.bias_summary_message = "Skipped update (pose_buf is not in chronological order).";
    diagnostics_info_.summary_message = "Skipped update (pose_buf is not in chronological order)";
    return;
  }

  // Filter gyro data
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_filtered;
  for (const auto & gyro : gyro_all) {
    const rclcpp::Time t = rclcpp::Time(gyro.header.stamp);
    if (t0_rclcpp_time <= t && t < t1_rclcpp_time) {
      gyro_filtered.push_back(gyro);
    }
  }

  // Check gyro data size
  // Data size must be greater than or equal to 2 since the time difference will be taken later
  if (gyro_filtered.size() <= 1) {
    gyro_info_.bias_summary_message = "Skipped update (gyro_filtered size is less than 2).";
    diagnostics_info_.summary_message = "Skipped update (gyro_filtered size is less than 2)";
    return;
  }

  // Check if the vehicle is moving straight
  const geometry_msgs::msg::Vector3 rpy_0 =
    autoware_utils::get_rpy(pose_buf.front().pose.orientation);
  const geometry_msgs::msg::Vector3 rpy_1 =
    autoware_utils::get_rpy(pose_buf.back().pose.orientation);
  const double yaw_diff = std::abs(autoware_utils::normalize_radian(rpy_1.z - rpy_0.z));
  const double time_diff = (t1_rclcpp_time - t0_rclcpp_time).seconds();
  const double yaw_vel = yaw_diff / time_diff;
  const bool is_straight = (yaw_vel < straight_motion_ang_vel_upper_limit_);
  if (!is_straight) {
    gyro_info_.bias_summary_message =
      "Skipped update (yaw angular velocity is greater than straight_motion_ang_vel_upper_limit).";
    diagnostics_info_.summary_message =
      "Skipped update (yaw angular velocity is greater than straight_motion_ang_vel_upper_limit)";
    return;
  }

  // Calculate gyro bias
  gyro_bias_estimation_module_->update_bias(pose_buf, gyro_filtered);

  geometry_msgs::msg::TransformStamped::ConstSharedPtr tf_base2imu_ptr =
    transform_listener_->get_latest_transform(output_frame_, imu_frame_);
  if (!tf_base2imu_ptr) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", imu_frame_.c_str(), output_frame_.c_str());
    gyro_info_.bias_summary_message = "Skipped update (tf between base and imu is not available).";
    diagnostics_info_.summary_message = "Skipped update (tf between base and imu is not available)";
    return;
  }

  gyro_bias_ =
    transform_vector3(gyro_bias_estimation_module_->get_bias_base_link(), *tf_base2imu_ptr);

  validate_gyro_bias();
}

void GyroBiasEstimator::validate_gyro_bias()
{
  // Calculate diagnostics key-values
  diagnostics_info_.gyro_bias_x_for_imu_corrector = gyro_bias_.value().x;
  diagnostics_info_.gyro_bias_y_for_imu_corrector = gyro_bias_.value().y;
  diagnostics_info_.gyro_bias_z_for_imu_corrector = gyro_bias_.value().z;
  diagnostics_info_.estimated_gyro_bias_x = gyro_bias_.value().x - angular_velocity_offset_x_;
  diagnostics_info_.estimated_gyro_bias_y = gyro_bias_.value().y - angular_velocity_offset_y_;
  diagnostics_info_.estimated_gyro_bias_z = gyro_bias_.value().z - angular_velocity_offset_z_;

  // Validation
  const bool is_bias_small_enough =
    std::abs(diagnostics_info_.estimated_gyro_bias_x) < gyro_bias_threshold_ &&
    std::abs(diagnostics_info_.estimated_gyro_bias_y) < gyro_bias_threshold_ &&
    std::abs(diagnostics_info_.estimated_gyro_bias_z) < gyro_bias_threshold_;

  // Update diagnostics
  if (is_bias_small_enough) {
    diagnostics_info_.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnostics_info_.summary_message = "Successfully updated";
    gyro_info_.bias_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    gyro_info_.bias_status_summary = "OK";
    gyro_info_.bias_summary_message = "Successfully updated";
  } else {
    diagnostics_info_.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnostics_info_.summary_message =
      "Gyro bias may be incorrect. Please calibrate IMU and reflect the result in imu_corrector. "
      "You may also use the output of gyro_bias_estimator.";
    gyro_info_.bias_status = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    gyro_info_.bias_status_summary = "WARN";
    gyro_info_.bias_summary_message =
      "Gyro bias may be incorrect. Please calibrate IMU and reflect the result in imu_corrector. "
      "You may also use the output of gyro_bias_estimator.";
  }
}

geometry_msgs::msg::Vector3 GyroBiasEstimator::transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Vector3Stamped vec_stamped;
  vec_stamped.vector = vec;

  geometry_msgs::msg::Vector3Stamped vec_stamped_transformed;
  tf2::doTransform(vec_stamped, vec_stamped_transformed, transform);
  return vec_stamped_transformed.vector;
}

void GyroBiasEstimator::update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  auto f = [](const double & value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(8) << value;
    return ss.str();
  };

  // Check for the highest status priority
  if (gyro_info_.scale_status >= diagnostics_info_.level) {
    diagnostics_info_.level = gyro_info_.scale_status;
    diagnostics_info_.summary_message = gyro_info_.scale_summary_message;
  }

  stat.summary(diagnostics_info_.level, diagnostics_info_.summary_message);
  stat.add("gyro_bias_x_for_imu_corrector", f(diagnostics_info_.gyro_bias_x_for_imu_corrector));
  stat.add("gyro_bias_y_for_imu_corrector", f(diagnostics_info_.gyro_bias_y_for_imu_corrector));
  stat.add("gyro_bias_z_for_imu_corrector", f(diagnostics_info_.gyro_bias_z_for_imu_corrector));

  stat.add("estimated_gyro_bias_x", f(diagnostics_info_.estimated_gyro_bias_x));
  stat.add("estimated_gyro_bias_y", f(diagnostics_info_.estimated_gyro_bias_y));
  stat.add("estimated_gyro_bias_z", f(diagnostics_info_.estimated_gyro_bias_z));

  stat.add("estimated_gyro_scale_z", f(diagnostics_info_.estimated_gyro_scale_z));

  stat.add("gyro_bias_status", gyro_info_.bias_status_summary);
  stat.add("gyro_bias_summary_message", gyro_info_.bias_summary_message);

  stat.add("gyro_scale_status", gyro_info_.scale_status_summary);
  stat.add("gyro_scale_summary_message", gyro_info_.scale_summary_message);

  stat.add("gyro_yaw_rate", f(gyro_yaw_rate));
  stat.add("ndt_yaw_rate", f(ndt_yaw_rate));

  stat.add("bias_on_purpose", f(final_bias_on_purpose_));
  stat.add("scale_on_purpose", f(final_scale_on_purpose_));

  stat.add("published_scale", f(previous_scale));

  stat.add("gyro_bias_threshold", f(gyro_bias_threshold_));
}

}  // namespace autoware::imu_corrector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::imu_corrector::GyroBiasEstimator)
