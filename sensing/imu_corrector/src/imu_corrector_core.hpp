// Copyright 2020 Tier IV, Inc.
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
#ifndef IMU_CORRECTOR_CORE_HPP_
#define IMU_CORRECTOR_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tier4_calibration_msgs/msg/bool_stamped.hpp>

#include <optional>


namespace imu_corrector
{
class ImuCorrector : public rclcpp::Node
{
public:
  explicit ImuCorrector(const rclcpp::NodeOptions & node_options);

private:

  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void callbackGyroBias(const geometry_msgs::msg::Vector3Stamped::ConstSharedPtr gyro_bias_msg_ptr);
  void onTimer();
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<tier4_calibration_msgs::msg::BoolStamped>::SharedPtr is_calibrated_pub_;

  std::string sensor_model_;

  bool is_calibrated_;
  
  double angular_velocity_stddev_xx_;
  double angular_velocity_stddev_yy_;
  double angular_velocity_stddev_zz_;

private:
  std::optional<double> angular_velocity_offset_x_;
  std::optional<double> angular_velocity_offset_y_;
  std::optional<double> angular_velocity_offset_z_;
  rclcpp::TimerBase::SharedPtr timer_;

};
}  // namespace imu_corrector

#endif  // IMU_CORRECTOR_CORE_HPP_
