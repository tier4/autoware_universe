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

#include "scene_module/bus_stop/debug.hpp"

namespace behavior_velocity_planner
{
namespace bus_stop
{
DebugData::DebugData(rclcpp::Node & node) : node_(node)
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("debug"), "constructor");
  pub_predicted_velocity_ =
    node_.create_publisher<Float32Stamped>("~/debug/bus_stop/predicted_vel_kmph", 1);
  pub_predicted_velocity_lpf_ =
    node_.create_publisher<Float32Stamped>("~/debug/bus_stop/predicted_vel_lpf_kmph", 1);
  pub_safe_vel_count_ = node_.create_publisher<Int32Stamped>("~/debug/bus_stop/safe_vel_count", 1);
}

void DebugData::pushPredictedVelKmph(const double predicted_vel_kmph)
{
  Float32Stamped predicted_vel_kmph_msg;
  predicted_vel_kmph_msg.data = predicted_vel_kmph;
  predicted_vel_kmph_msg.stamp = node_.now();

  predicted_vel_kmph_ = predicted_vel_kmph_msg;
}

void DebugData::pushPredictedVelLpfKmph(const double predicted_vel_lpf_kmph)
{
  Float32Stamped predicted_vel_kmph_lpf_msg;
  predicted_vel_kmph_lpf_msg.data = predicted_vel_lpf_kmph;
  predicted_vel_kmph_lpf_msg.stamp = node_.now();

  predicted_vel_lpf_kmph_ = predicted_vel_kmph_lpf_msg;
}

void DebugData::pushSafeVelCount(const size_t safe_vel_count)
{
  safe_vel_count_.data = safe_vel_count;
  safe_vel_count_.stamp = node_.now();
}

void DebugData::publishDebugValue()
{
  pub_predicted_velocity_->publish(predicted_vel_kmph_);
  pub_predicted_velocity_lpf_->publish(predicted_vel_lpf_kmph_);
  pub_safe_vel_count_->publish(safe_vel_count_);
}

void DebugData::clearDebugData() { stop_poses.clear(); }
}  // namespace bus_stop
}  // namespace behavior_velocity_planner
