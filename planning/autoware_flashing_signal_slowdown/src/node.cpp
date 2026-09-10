// Copyright 2026 TIER IV, Inc.
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

#include "node.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace autoware::flashing_signal_slowdown
{
namespace
{
using autoware_perception_msgs::msg::TrafficLightElement;
using autoware_perception_msgs::msg::TrafficLightGroup;

constexpr char sender_name[] = "flashing_signal_slowdown";

LampState to_lamp_state(const TrafficLightGroup & group)
{
  const auto is_amber_on = [](const TrafficLightElement & element) {
    return element.color == TrafficLightElement::AMBER &&
           element.status == TrafficLightElement::SOLID_ON;
  };
  const auto is_off = [](const TrafficLightElement & element) {
    return element.status == TrafficLightElement::SOLID_OFF;
  };

  if (std::any_of(group.elements.begin(), group.elements.end(), is_amber_on)) {
    return LampState::On;
  }
  if (std::any_of(group.elements.begin(), group.elements.end(), is_off)) {
    return LampState::Off;
  }
  return LampState::Unknown;
}
}  // namespace

FlashingSignalSlowdownNode::FlashingSignalSlowdownNode(const rclcpp::NodeOptions & node_options)
: Node("flashing_signal_slowdown", node_options)
{
  using std::placeholders::_1;

  // Parameters
  const auto update_rate_hz = declare_parameter<double>("update_rate_hz");
  slowdown_velocity_mps_ = declare_parameter<double>("slowdown_velocity_mps");

  FlashingDetectorParams detector_params;
  detector_params.history_window_sec = declare_parameter<double>("history_window_sec");
  detector_params.detect_min_transitions =
    static_cast<int>(declare_parameter<std::int64_t>("detect_min_transitions"));
  detector_params.detect_min_span_sec = declare_parameter<double>("detect_min_span_sec");
  detector_params.release_no_transition_sec =
    declare_parameter<double>("release_no_transition_sec");

  const auto ids = declare_parameter<std::vector<std::int64_t>>(
    "target_traffic_light_group_ids", std::vector<std::int64_t>{});
  for (const auto id : ids) {
    detectors_.emplace(id, FlashingDetector(detector_params));
  }
  RCLCPP_INFO(get_logger(), "monitoring %zu traffic light(s)", detectors_.size());

  // Subscriber
  sub_traffic_signals_ = create_subscription<TrafficLightGroupArray>(
    "~/input/traffic_signals", rclcpp::QoS{1},
    std::bind(&FlashingSignalSlowdownNode::on_traffic_signals, this, _1));

  // Publishers
  pub_velocity_limit_ =
    create_publisher<VelocityLimit>("~/output/max_velocity", rclcpp::QoS{1}.transient_local());
  pub_clear_velocity_limit_ = create_publisher<VelocityLimitClearCommand>(
    "~/output/velocity_limit_clear_command", rclcpp::QoS{1}.transient_local());

  // Timer
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / update_rate_hz),
    std::bind(&FlashingSignalSlowdownNode::on_timer, this));
}

void FlashingSignalSlowdownNode::on_traffic_signals(
  const TrafficLightGroupArray::ConstSharedPtr msg)
{
  const double stamp_sec = now().seconds();

  for (const auto & group : msg->traffic_light_groups) {
    const auto detector = detectors_.find(group.traffic_light_group_id);
    if (detector != detectors_.end()) {
      detector->second.update(stamp_sec, to_lamp_state(group));
    }
  }
}

void FlashingSignalSlowdownNode::on_timer()
{
  const double stamp_sec = now().seconds();

  // The timer also runs without a message, so the release still fires once the light is gone.
  for (auto & [id, detector] : detectors_) {
    detector.evaluate(stamp_sec);
  }

  // Hold the slowdown while any monitored light is still flashing.
  const bool flashing = std::any_of(detectors_.begin(), detectors_.end(), [](const auto & entry) {
    return entry.second.is_flashing();
  });

  if (flashing == slowdown_active_) {
    return;
  }
  slowdown_active_ = flashing;

  if (flashing) {
    publish_velocity_limit();
  } else {
    publish_clear_command();
  }
}

void FlashingSignalSlowdownNode::publish_velocity_limit()
{
  VelocityLimit velocity_limit;
  velocity_limit.stamp = now();
  velocity_limit.max_velocity = static_cast<float>(slowdown_velocity_mps_);
  velocity_limit.use_constraints = false;
  velocity_limit.sender = sender_name;

  pub_velocity_limit_->publish(velocity_limit);
  RCLCPP_INFO(get_logger(), "set velocity limit %.2f m/s", slowdown_velocity_mps_);
}

void FlashingSignalSlowdownNode::publish_clear_command()
{
  VelocityLimitClearCommand clear_command;
  clear_command.stamp = now();
  clear_command.command = true;
  clear_command.sender = sender_name;

  pub_clear_velocity_limit_->publish(clear_command);
  RCLCPP_INFO(get_logger(), "clear velocity limit");
}

}  // namespace autoware::flashing_signal_slowdown

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::flashing_signal_slowdown::FlashingSignalSlowdownNode)
