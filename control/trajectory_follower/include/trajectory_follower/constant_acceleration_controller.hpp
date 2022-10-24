// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef TRAJECTORY_FOLLOWER__CONSTANT_ACCELERATION_CONTROLLER_HPP_
#define TRAJECTORY_FOLLOWER__CONSTANT_ACCELERATION_CONTROLLER_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "motion_common/motion_common.hpp"
#include "motion_common/trajectory_common.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "trajectory_follower/debug_values.hpp"
#include "trajectory_follower/longitudinal_controller_base.hpp"
#include "trajectory_follower/longitudinal_controller_utils.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/pid.hpp"
#include "trajectory_follower/smooth_stop.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_system_msgs/msg/float32_multi_array_diagnostic.hpp"
#include "autoware_auto_vehicle_msgs/msg/vehicle_odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tier4_debug_msgs/msg/float32_stamped.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using tier4_debug_msgs::msg::Float32Stamped;
namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;
namespace motion_common = ::autoware::motion::motion_common;

/// \class ConstantAccelController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC ConstantAccelController : public LongitudinalControllerBase
{
public:
  explicit ConstantAccelController(rclcpp::Node & node);

private:
  rclcpp::Node * node_;
  // ros variables
  rclcpp::Publisher<autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic>::SharedPtr
    m_pub_slope;
  rclcpp::Publisher<autoware_auto_system_msgs::msg::Float32MultiArrayDiagnostic>::SharedPtr
    m_pub_debug;

  rclcpp::Subscription<Float32Stamped>::SharedPtr m_sub_test_acceleration;

  // control state
  enum class ControlState { DRIVE = 0, STOPPING, STOPPED, EMERGENCY };
  ControlState m_control_state{ControlState::STOPPED};

  // control period
  float64_t m_longitudinal_ctrl_period;
  Float32Stamped::SharedPtr test_acc;

  /**
   * @brief compute control command, and publish periodically
   */
  boost::optional<LongitudinalOutput> run() override;

  /**
   * @brief set input data like current odometry and trajectory.
   */
  void setInputData([[maybe_unused]] InputData const & input_data) {}
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__CONSTANT_ACCELERATION_CONTROLLER_HPP_
