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

#ifndef IMU_MONITOR__IMU_MONITOR_HPP_
#define IMU_MONITOR__IMU_MONITOR_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>

namespace imu_monitor
{

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
  explicit ImuMonitor(const rclcpp::NodeOptions & node_options);
  ~ImuMonitor() = default;

private:
  void on_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void check_yaw_rate(diagnostic_updater::DiagnosticStatusWrapper & stat);

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  std::string frame_id_;
  double yaw_rate_diff_threshold_;

  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_ptr_;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_ptr_;

  double imu_yaw_rate_;
  double twist_yaw_rate_;

  KalmanFilter1d imu_filter_;
  KalmanFilter1d twist_filter_;

  diagnostic_updater::Updater updater_;
};
}  // namespace imu_monitor

#endif  // IMU_MONITOR__IMU_MONITOR_HPP_
