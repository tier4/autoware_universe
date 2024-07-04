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

#include "mrm_stop_operator.hpp"

namespace mrm_stop_operator
{

MrmStopOperator::MrmStopOperator(const rclcpp::NodeOptions & node_options)
: Node("mrm_stop_operator", node_options)
{
  // Parameter
  params_.min_acceleration = declare_parameter<double>("min_acceleration", -4.0);
  params_.max_jerk = declare_parameter<double>("max_jerk", 5.0);
  params_.min_jerk = declare_parameter<double>("min_jerk", -5.0);

  // Subscriber
  sub_mrm_request_ = create_subscription<tier4_system_msgs::msg::MrmBehavior>(
    "~/input/mrm_request", 10,
    std::bind(&MrmStopOperator::onMrmRequest, this, std::placeholders::_1));

  sub_velocity_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions velocity_options = rclcpp::SubscriptionOptions();
  velocity_options.callback_group = sub_velocity_group_;
  auto not_executed_callback = []([[maybe_unused]] const typename autoware_auto_vehicle_msgs::msg::
                                    VelocityReport::ConstSharedPtr msg) {};
  sub_velocity_ = create_subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>(
    "~/input/velocity", 10, not_executed_callback, velocity_options);

  // Publisher
  pub_velocity_limit_ = create_publisher<tier4_planning_msgs::msg::VelocityLimit>(
    "~/output/velocity_limit", rclcpp::QoS{10}.transient_local());
  pub_velocity_limit_clear_command_ =
    create_publisher<tier4_planning_msgs::msg::VelocityLimitClearCommand>(
      "~/output/velocity_limit_clear_command", rclcpp::QoS{10}.transient_local());
  pub_mrm_state_ =
    create_publisher<tier4_system_msgs::msg::MrmState>("~/output/mrm_state", rclcpp::QoS{1});

  // Timer
  const auto update_period_ns = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MrmStopOperator::onTimer, this));

  // Service

  // Client

  // Timer

  // State
  initState();

  // Diagnostics
}

void MrmStopOperator::onMrmRequest(const tier4_system_msgs::msg::MrmBehavior::ConstSharedPtr msg)
{
  if (
    msg->type == tier4_system_msgs::msg::MrmBehavior::COMFORTABLE_STOP &&
    last_mrm_request_.type != tier4_system_msgs::msg::MrmBehavior::COMFORTABLE_STOP) {
    tier4_planning_msgs::msg::VelocityLimit velocity_limit;
    velocity_limit.stamp = this->now();
    velocity_limit.max_velocity = 0.0;
    velocity_limit.use_constraints = true;
    velocity_limit.constraints.min_acceleration = params_.min_acceleration;
    velocity_limit.constraints.max_jerk = params_.max_jerk;
    velocity_limit.constraints.min_jerk = params_.min_jerk;
    velocity_limit.sender = "mrm_stop_operator";
    pub_velocity_limit_->publish(velocity_limit);

    last_mrm_request_ = *msg;
    current_mrm_state_.behavior = *msg;
    current_mrm_state_.state = tier4_system_msgs::msg::MrmState::MRM_OPERATING;
  }
}

void MrmStopOperator::initState()
{
  last_mrm_request_.type = tier4_system_msgs::msg::MrmBehavior::NONE;
  current_mrm_state_.state = tier4_system_msgs::msg::MrmState::NORMAL;
  current_mrm_state_.behavior.type = tier4_system_msgs::msg::MrmBehavior::NONE;
}

void MrmStopOperator::onTimer()
{
  if (current_mrm_state_.state == tier4_system_msgs::msg::MrmState::MRM_OPERATING) {
    if (current_mrm_state_.behavior.type == tier4_system_msgs::msg::MrmBehavior::COMFORTABLE_STOP) {
      if (isStopped()) {
        current_mrm_state_.state = tier4_system_msgs::msg::MrmState::MRM_SUCCEEDED;
      } else {
        // nothing to do
      }
    } else {
      // TODO
    }
  }
  current_mrm_state_.stamp = this->now();
  pub_mrm_state_->publish(current_mrm_state_);
}

bool MrmStopOperator::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  auto current_velocity = std::make_shared<autoware_auto_vehicle_msgs::msg::VelocityReport>();
  rclcpp::MessageInfo message_info;

  const bool success = sub_velocity_->take(*current_velocity, message_info);
  if (success) {
    return current_velocity->longitudinal_velocity < th_stopped_velocity;
  } else {
    return false;
  }
}

}  // namespace mrm_stop_operator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrm_stop_operator::MrmStopOperator)
