// Copyright 2025 Autoware Foundation
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

#ifndef AUTOWARE__LOCALIZATION_MONITOR_HOST__POSE_INSTABILITY_PLUGIN_HPP_
#define AUTOWARE__LOCALIZATION_MONITOR_HOST__POSE_INSTABILITY_PLUGIN_HPP_

#include "localization_monitor_plugin_base.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace autoware::localization_monitor_host::plugin
{

class PoseInstabilityPlugin : public LocalizationMonitorPluginBase
{
  using Quaternion = geometry_msgs::msg::Quaternion;
  using Twist = geometry_msgs::msg::Twist;
  using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Odometry = nav_msgs::msg::Odometry;
  using KeyValue = diagnostic_msgs::msg::KeyValue;
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

public:
  struct ThresholdValues
  {
    double position_x;
    double position_y;
    double position_z;
    double angle_x;
    double angle_y;
    double angle_z;
  };

  void initialize(const std::string & name, rclcpp::Node * node_ptr) override;
  void set_up_params() override;
  void on_parameter(const std::vector<rclcpp::Parameter> & parameters) override;
  void evaluate(
    const MonitorData & data, diagnostic_updater::Updater & updater) override;

  ThresholdValues calculate_threshold(double interval_sec) const;
  static void dead_reckon(
    PoseStamped::SharedPtr & initial_pose, const rclcpp::Time & end_time,
    const std::deque<TwistWithCovarianceStamped> & twist_deque, Pose::SharedPtr & estimated_pose);

private:
  static std::deque<TwistWithCovarianceStamped> clip_out_necessary_twist(
    const std::deque<TwistWithCovarianceStamped> & twist_buffer, const rclcpp::Time & start_time,
    const rclcpp::Time & end_time);

  void update_params();

  rclcpp::Publisher<PoseStamped>::SharedPtr diff_pose_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostics_pub_;

  double timer_period_{0.5};
  double heading_velocity_maximum_{16.667};
  double heading_velocity_scale_factor_tolerance_{3.0};
  double angular_velocity_maximum_{0.523};
  double angular_velocity_scale_factor_tolerance_{0.2};
  double angular_velocity_bias_tolerance_{0.00698};
  double pose_estimator_longitudinal_tolerance_{0.11};
  double pose_estimator_lateral_tolerance_{0.11};
  double pose_estimator_vertical_tolerance_{0.5};
  double pose_estimator_angular_tolerance_{0.0175};

  std::vector<bool> enable_validation_flags_{true, true, true, false, false, true};

  std::optional<Odometry> prev_odom_{std::nullopt};
  bool first_run_{true};
};

}  // namespace autoware::localization_monitor_host::plugin

#endif  // AUTOWARE__LOCALIZATION_MONITOR_HOST__POSE_INSTABILITY_PLUGIN_HPP_
