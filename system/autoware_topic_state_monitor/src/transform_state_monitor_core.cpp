// Copyright 2025 TIER IV, Inc.
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

#include "transform_state_monitor_core.hpp"

#include <memory>
#include <string>
#include <vector>

namespace
{
template <typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
  }
}
}  // namespace

namespace autoware::topic_state_monitor
{
TransformStateMonitorNode::TransformStateMonitorNode(const rclcpp::NodeOptions & node_options)
: agnocast::Node("topic_state_monitor", node_options)
{
  using std::placeholders::_1;

  // Parameter
  const double update_rate = declare_parameter("update_rate", 10.0);
  topic_ = declare_parameter<std::string>("topic");
  diag_name_ = declare_parameter<std::string>("diag_name");
  frame_id_ = declare_parameter<std::string>("frame_id");
  child_frame_id_ = declare_parameter<std::string>("child_frame_id");

  param_.warn_rate = declare_parameter("warn_rate", 0.5);
  param_.error_rate = declare_parameter("error_rate", 0.1);
  param_.timeout = declare_parameter("timeout", 1.0);
  param_.window_size = declare_parameter("window_size", 10);

  // Parameter Reconfigure
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&TransformStateMonitorNode::onParameter, this, _1));

  // Core
  topic_state_monitor_ = std::make_unique<TopicStateMonitor>(get_clock(), param_);

  // Subscriber
  rclcpp::QoS qos = rclcpp::QoS{1};
  sub_transform_ = create_subscription<tf2_msgs::msg::TFMessage>(
    topic_, qos,
    [this](const agnocast::ipc_shared_ptr<const tf2_msgs::msg::TFMessage> & msg) {
      for (const auto & transform : msg->transforms) {
        if (
          transform.header.frame_id == frame_id_ &&
          transform.child_frame_id == child_frame_id_) {
          topic_state_monitor_->update();
        }
      }
    });

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ = create_timer(period_ns, std::bind(&TransformStateMonitorNode::onTimer, this));
}

rcl_interfaces::msg::SetParametersResult TransformStateMonitorNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    update_param(parameters, "warn_rate", param_.warn_rate);
    update_param(parameters, "error_rate", param_.error_rate);
    update_param(parameters, "timeout", param_.timeout);
    update_param(parameters, "window_size", param_.window_size);
    topic_state_monitor_->setParam(param_);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void TransformStateMonitorNode::onTimer()
{
  using diagnostic_msgs::msg::DiagnosticStatus;

  const auto topic_status = topic_state_monitor_->getTopicStatus();

  if (topic_status == TopicStatus::NotReceived) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "%s has not received. Set ERROR in diagnostics.",
      topic_.c_str());
  } else if (topic_status == TopicStatus::WarnRate) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s topic rate has dropped to the warning level. Set WARN in diagnostics.", topic_.c_str());
  } else if (topic_status == TopicStatus::ErrorRate) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "%s topic rate has dropped to the error level. Set ERROR in diagnostics.", topic_.c_str());
  } else if (topic_status == TopicStatus::Timeout) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "%s topic is timeout. Set ERROR in diagnostics.",
      topic_.c_str());
  }
}

}  // namespace autoware::topic_state_monitor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::topic_state_monitor::TransformStateMonitorNode)
