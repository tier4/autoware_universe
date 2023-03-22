// Copyright 2023 TierIV
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

#ifndef IMU_ANOMALY_MONITOR__IMU_ANOMALY_MONITOR_HPP_
#define IMU_ANOMALY_MONITOR__IMU_ANOMALY_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tier4_autoware_utils/ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <kalman_filter/kalman_filter.hpp>
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <tier4_debug_msgs/msg/float32_stamped.hpp>

#include <array>
#include <string>
#include <vector>


class KalmanFilter1d
{
public:
  KalmanFilter1d()
  {
    initialized_ = false;
    x_ = 0;
    dev_ = 1e9;
    proc_dev_x_c_ = 0.0;
    return;
  };
  void init(const double init_obs, const double obs_dev, const rclcpp::Time time)
  {
    x_ = init_obs;
    dev_ = obs_dev;
    latest_time_ = time;
    initialized_ = true;
    return;
  };
  void update(const double obs, const double obs_dev, const rclcpp::Time time)
  {
    if (!initialized_) {
      init(obs, obs_dev, time);
      return;
    }

    // Prediction step (current stddev_)
    double dt = (time - latest_time_).seconds();
    double proc_dev_x_d = proc_dev_x_c_ * dt * dt;
    dev_ = dev_ + proc_dev_x_d;

    // Update step
    double kalman_gain = dev_ / (dev_ + obs_dev);
    x_ = x_ + kalman_gain * (obs - x_);
    dev_ = (1 - kalman_gain) * dev_;

    latest_time_ = time;
    return;
  };
  void set_proc_dev(const double proc_dev) { proc_dev_x_c_ = proc_dev; }
  double get_x() { return x_; }

private:
  bool initialized_;
  double x_;
  double dev_;
  double proc_dev_x_c_;
  rclcpp::Time latest_time_;
};

class ImuMonitor : public rclcpp::Node
{
public:
  ImuMonitor();
  ~ImuMonitor() = default;

private:
  void on_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr
    imu_sub_;

  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr imu_yaw_rate_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr vehicle_yaw_rate_pub_;

  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  diagnostic_updater::Updater updater_;

  std::string frame_id_;
  double stddev_vx_;
  double stddev_wz_;
  std::array<double, 36> twist_covariance_;

  KalmanFilter1d imu_filter_;
  KalmanFilter1d twist_filter_;
  bool first_observation_{true}; 
};

#endif  // IMU_ANOMALY_MONITOR__IMU_ANOMALY_MONITOR_HPP_
