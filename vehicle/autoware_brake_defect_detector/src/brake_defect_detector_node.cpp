// Copyright 2026 The Autoware Contributors
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

#include "autoware_brake_defect_detector/brake_defect_detector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>

namespace autoware::brake_defect_detector
{

class BrakeDefectDetectorNode : public rclcpp::Node
{
public:
  explicit BrakeDefectDetectorNode(const rclcpp::NodeOptions & options)
  : Node("brake_defect_detector", options), detector_(load_params())
  {
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.5);
    if (!std::isfinite(input_timeout_sec_) || input_timeout_sec_ <= 0.0) {
      throw std::invalid_argument("input_timeout_sec must be finite and positive");
    }

    using std::placeholders::_1;
    control_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
      "~/input/control_cmd", rclcpp::QoS(10),
      std::bind(&BrakeDefectDetectorNode::on_control, this, _1));
    actuation_sub_ = create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
      "~/input/actuation_cmd", rclcpp::QoS(10),
      std::bind(&BrakeDefectDetectorNode::on_actuation, this, _1));
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "~/input/kinematics", rclcpp::QoS(10),
      std::bind(&BrakeDefectDetectorNode::on_odometry, this, _1));
    acceleration_sub_ = create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "~/input/measured_acceleration", rclcpp::QoS(10),
      std::bind(&BrakeDefectDetectorNode::on_acceleration, this, _1));

    defect_pub_ =
      create_publisher<std_msgs::msg::Bool>("~/output/brake_defect_detected", rclcpp::QoS(10));
    residual_pub_ =
      create_publisher<std_msgs::msg::Float64>("~/output/filtered_residual", rclcpp::QoS(10));
    cusum_pub_ =
      create_publisher<std_msgs::msg::Float64>("~/output/cusum_statistic", rclcpp::QoS(10));

    watchdog_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&BrakeDefectDetectorNode::on_watchdog, this));
  }

private:
  DetectorParams load_params()
  {
    DetectorParams params;
    params.actuation_delay_sec =
      declare_parameter<double>("actuation_delay_sec", params.actuation_delay_sec);
    params.cusum_drift_k = declare_parameter<double>("cusum_drift_k", params.cusum_drift_k);
    params.cusum_threshold_h =
      declare_parameter<double>("cusum_threshold_h", params.cusum_threshold_h);
    params.min_speed_mps = declare_parameter<double>("min_speed_mps", params.min_speed_mps);
    params.max_decel_cmd = declare_parameter<double>("max_decel_cmd", params.max_decel_cmd);
    params.brake_cmd_min = declare_parameter<double>("brake_cmd_min", params.brake_cmd_min);
    params.brake_cmd_max = declare_parameter<double>("brake_cmd_max", params.brake_cmd_max);
    params.jerk_limit_mps3 = declare_parameter<double>("jerk_limit_mps3", params.jerk_limit_mps3);
    params.settling_time_sec =
      declare_parameter<double>("settling_time_sec", params.settling_time_sec);
    params.residual_filter_tau_sec =
      declare_parameter<double>("residual_filter_tau_sec", params.residual_filter_tau_sec);
    params.max_update_gap_sec =
      declare_parameter<double>("max_update_gap_sec", params.max_update_gap_sec);

    const auto nonnegative = [](const double value) {
      return std::isfinite(value) && value >= 0.0;
    };
    if (
      !nonnegative(params.actuation_delay_sec) || !nonnegative(params.cusum_drift_k) ||
      !std::isfinite(params.cusum_threshold_h) || params.cusum_threshold_h <= 0.0 ||
      !nonnegative(params.min_speed_mps) || !std::isfinite(params.max_decel_cmd) ||
      params.max_decel_cmd >= 0.0 || !nonnegative(params.brake_cmd_min) ||
      !std::isfinite(params.brake_cmd_max) || params.brake_cmd_max < params.brake_cmd_min ||
      !std::isfinite(params.jerk_limit_mps3) || params.jerk_limit_mps3 <= 0.0 ||
      !nonnegative(params.settling_time_sec) || !nonnegative(params.residual_filter_tau_sec) ||
      !std::isfinite(params.max_update_gap_sec) || params.max_update_gap_sec <= 0.0) {
      throw std::invalid_argument("invalid brake defect detector parameters");
    }
    return params;
  }

  bool is_fresh(const std::optional<rclcpp::Time> & received, const rclcpp::Time & now) const
  {
    if (!received) {
      return false;
    }
    const double age = (now - *received).seconds();
    return age >= 0.0 && age <= input_timeout_sec_;
  }

  static std::optional<double> pitch_from_odometry(const nav_msgs::msg::Odometry & odometry)
  {
    const auto & q = odometry.pose.pose.orientation;
    const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12) {
      return std::nullopt;
    }
    const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x) / norm_squared;
    return std::asin(std::clamp(sin_pitch, -1.0, 1.0));
  }

  void publish(const DiagnosticStatus & status)
  {
    std_msgs::msg::Bool defect;
    defect.data = status.brake_defect_detected;
    defect_pub_->publish(defect);

    std_msgs::msg::Float64 residual;
    residual.data = status.filtered_residual;
    residual_pub_->publish(residual);

    std_msgs::msg::Float64 cusum;
    cusum.data = status.cusum_statistic;
    cusum_pub_->publish(cusum);
  }

  void invalidate()
  {
    detector_.reset();
    publish({});
  }

  void on_control(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
  {
    const auto received = now();
    control_ = msg;
    control_received_ = received;
    detector_.observe_command(msg->longitudinal.acceleration, received.seconds());
  }

  void on_actuation(const tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr msg)
  {
    actuation_ = msg;
    actuation_received_ = now();
  }

  void on_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    odometry_ = msg;
    odometry_received_ = now();
  }

  void on_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg)
  {
    const auto received = now();
    acceleration_received_ = received;
    if (
      !control_ || !actuation_ || !odometry_ || !is_fresh(control_received_, received) ||
      !is_fresh(actuation_received_, received) || !is_fresh(odometry_received_, received)) {
      invalidate();
      return;
    }

    const auto pitch = pitch_from_odometry(*odometry_);
    if (!pitch) {
      invalidate();
      return;
    }

    publish(detector_.update(
      control_->longitudinal.acceleration, actuation_->actuation.brake_cmd,
      msg->accel.accel.linear.x, odometry_->twist.twist.linear.x, *pitch, received.seconds()));
  }

  void on_watchdog()
  {
    const auto current = now();
    if (
      !is_fresh(control_received_, current) || !is_fresh(actuation_received_, current) ||
      !is_fresh(odometry_received_, current) || !is_fresh(acceleration_received_, current)) {
      invalidate();
    }
  }

  BrakeDefectDetector detector_;
  double input_timeout_sec_{0.5};

  autoware_control_msgs::msg::Control::ConstSharedPtr control_;
  tier4_vehicle_msgs::msg::ActuationCommandStamped::ConstSharedPtr actuation_;
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_;
  std::optional<rclcpp::Time> control_received_;
  std::optional<rclcpp::Time> actuation_received_;
  std::optional<rclcpp::Time> odometry_received_;
  std::optional<rclcpp::Time> acceleration_received_;

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr actuation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr acceleration_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr defect_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr residual_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cusum_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace autoware::brake_defect_detector

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::brake_defect_detector::BrakeDefectDetectorNode)
