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

#include "typed_topic_state_monitor_core.hpp"

// ============================================================================
// WORKAROUND: Compile-time type registry for agnocast (no GenericSubscription).
//
// Every message type that a topic_state_monitor instance might subscribe to
// must be included here and registered in createTypedSubscription() via REGISTER_TYPE.
//
// To add a new type: (1) add the #include, (2) add a REGISTER_TYPE entry.
//
// TODO(agnocast): Remove this workaround when agnocast adds GenericSubscription.
// ============================================================================
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nebula_msgs/msg/nebula_packets.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

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

// ============================================================================
// Compile-time type registry implementation.
//
// Each REGISTER_TYPE macro entry maps a ROS 2 type name string
// (e.g. "sensor_msgs/msg/PointCloud2") to its C++ type, creating a typed
// agnocast::Subscription that calls topic_state_monitor_->update() on receipt.
//
// The callback intentionally ignores the message content — topic_state_monitor
// only needs to know WHEN a message arrived, not WHAT it contains.
// ============================================================================
std::shared_ptr<void> TypedTopicStateMonitorNode::createTypedSubscription(
  const std::string & topic, const std::string & topic_type, const rclcpp::QoS & qos)
{
  // Helper macro: creates a typed agnocast subscription and returns it as shared_ptr<void>.
  // The lambda captures `this` to call topic_state_monitor_->update().
#define REGISTER_TYPE(TYPE_STRING, CPP_TYPE)                                             \
  if (topic_type == TYPE_STRING) {                                                       \
    auto sub = create_subscription<CPP_TYPE>(                                            \
      topic, qos, [this](const agnocast::ipc_shared_ptr<const CPP_TYPE> &) {             \
        topic_state_monitor_->update();                                                  \
      });                                                                                \
    return sub;                                                                          \
  }

  // --- Core Autoware types ---
  REGISTER_TYPE("autoware_control_msgs/msg/Control", autoware_control_msgs::msg::Control)
  REGISTER_TYPE("autoware_map_msgs/msg/LaneletMapBin", autoware_map_msgs::msg::LaneletMapBin)
  REGISTER_TYPE(
    "autoware_perception_msgs/msg/PredictedObjects",
    autoware_perception_msgs::msg::PredictedObjects)
  REGISTER_TYPE(
    "autoware_perception_msgs/msg/TrafficLightGroupArray",
    autoware_perception_msgs::msg::TrafficLightGroupArray)
  REGISTER_TYPE(
    "autoware_planning_msgs/msg/LaneletRoute", autoware_planning_msgs::msg::LaneletRoute)
  REGISTER_TYPE("autoware_planning_msgs/msg/Trajectory", autoware_planning_msgs::msg::Trajectory)
  REGISTER_TYPE(
    "autoware_vehicle_msgs/msg/SteeringReport", autoware_vehicle_msgs::msg::SteeringReport)
  REGISTER_TYPE(
    "autoware_vehicle_msgs/msg/VelocityReport", autoware_vehicle_msgs::msg::VelocityReport)

  // --- Standard ROS 2 types ---
  REGISTER_TYPE("geometry_msgs/msg/PoseStamped", geometry_msgs::msg::PoseStamped)
  REGISTER_TYPE(
    "geometry_msgs/msg/PoseWithCovarianceStamped", geometry_msgs::msg::PoseWithCovarianceStamped)
  REGISTER_TYPE("sensor_msgs/msg/PointCloud2", sensor_msgs::msg::PointCloud2)
  REGISTER_TYPE("sensor_msgs/msg/Imu", sensor_msgs::msg::Imu)

  // --- Vendor/Tier4 types ---
  REGISTER_TYPE("nebula_msgs/msg/NebulaPackets", nebula_msgs::msg::NebulaPackets)
  REGISTER_TYPE(
    "tier4_perception_msgs/msg/DetectedObjectsWithFeature",
    tier4_perception_msgs::msg::DetectedObjectsWithFeature)
  REGISTER_TYPE(
    "tier4_perception_msgs/msg/TrafficLightArray", tier4_perception_msgs::msg::TrafficLightArray)

#undef REGISTER_TYPE

  // Unknown type — fatal error at startup, not a silent failure.
  RCLCPP_ERROR(
    get_logger(), "Unsupported topic_type '%s' for typed agnocast subscription. "
                  "Add it to REGISTER_TYPE in typed_topic_state_monitor_core.cpp.",
    topic_type.c_str());
  return nullptr;
}

TypedTopicStateMonitorNode::TypedTopicStateMonitorNode(const rclcpp::NodeOptions & node_options)
: agnocast::Node("topic_state_monitor", node_options)
{
  using std::placeholders::_1;

  // Parameter
  const double update_rate = declare_parameter("update_rate", 10.0);
  topic_ = declare_parameter<std::string>("topic");
  topic_type_ = declare_parameter<std::string>("topic_type");
  diag_name_ = declare_parameter<std::string>("diag_name");

  param_.warn_rate = declare_parameter("warn_rate", 0.5);
  param_.error_rate = declare_parameter("error_rate", 0.1);
  param_.timeout = declare_parameter("timeout", 1.0);
  param_.window_size = declare_parameter("window_size", 10);

  // Parameter Reconfigure
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&TypedTopicStateMonitorNode::onParameter, this, _1));

  // Core
  topic_state_monitor_ = std::make_unique<TopicStateMonitor>(get_clock(), param_);

  // Subscriber — create typed subscription via compile-time registry
  rclcpp::QoS qos = rclcpp::QoS{1};
  const bool transient_local = declare_parameter("transient_local", false);
  const bool best_effort = declare_parameter("best_effort", false);
  if (transient_local) {
    qos.transient_local();
  }
  if (best_effort) {
    qos.best_effort();
  }

  sub_topic_ = createTypedSubscription(topic_, topic_type_, qos);
  if (!sub_topic_) {
    throw std::runtime_error(
      "Failed to create subscription for topic_type: " + topic_type_ +
      ". Register it in typed_topic_state_monitor_core.cpp.");
  }

  // Timer
  const auto period_ns = rclcpp::Rate(update_rate).period();
  timer_ = create_timer(period_ns, std::bind(&TypedTopicStateMonitorNode::onTimer, this));
}

rcl_interfaces::msg::SetParametersResult TypedTopicStateMonitorNode::onParameter(
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

void TypedTopicStateMonitorNode::onTimer()
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
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::topic_state_monitor::TypedTopicStateMonitorNode)
