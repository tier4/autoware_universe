// Copyright 2024 TIER IV, Inc.
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

#ifndef MRM_STOP_OPERATOR_HPP_
#define MRM_STOP_OPERATOR_HPP_

// include
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <tier4_system_msgs/msg/mrm_behavior.hpp>
#include <tier4_system_msgs/msg/mrm_state.hpp>

namespace mrm_stop_operator
{

struct Parameters
{
  double min_acceleration;
  double max_jerk;
  double min_jerk;
};

class MrmStopOperator : public rclcpp::Node
{
public:
  explicit MrmStopOperator(const rclcpp::NodeOptions & node_options);
  ~MrmStopOperator() = default;

private:
  // Parameter
  Parameters params_;

  // Subscriber
  rclcpp::Subscription<tier4_system_msgs::msg::MrmBehavior>::SharedPtr sub_mrm_request_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_velocity_;

  void onMrmRequest(const tier4_system_msgs::msg::MrmBehavior::ConstSharedPtr msg);

  // Service

  // Publisher
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimit>::SharedPtr pub_velocity_limit_;
  rclcpp::Publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>::SharedPtr
    pub_velocity_limit_clear_command_;
  rclcpp::Publisher<tier4_system_msgs::msg::MrmState>::SharedPtr pub_mrm_state_;
  // Service

  // Client

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::CallbackGroup::SharedPtr sub_velocity_group_;

  // State
  tier4_system_msgs::msg::MrmBehavior last_mrm_request_;
  tier4_system_msgs::msg::MrmState current_mrm_state_;

  void initState();
  void onTimer();
  bool isStopped();

  // Diagnostics
};
}  // namespace mrm_stop_operator

#endif  // MRM_STOP_OPERATOR_HPP_
