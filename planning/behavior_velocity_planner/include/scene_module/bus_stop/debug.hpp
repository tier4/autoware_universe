// Copyright 2023 Tier IV, Inc.
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

#ifndef SCENE_MODULE__BUS_STOP__DEBUG_HPP_
#define SCENE_MODULE__BUS_STOP__DEBUG_HPP_

#include <motion_utils/motion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utilization/debug.hpp>
#include <utilization/util.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>

#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
namespace bus_stop
{
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_debug_msgs::msg::Float32Stamped;
using tier4_debug_msgs::msg::Int32Stamped;

class DebugData
{
public:
  explicit DebugData(rclcpp::Node & node);
  ~DebugData() {}
  void publishDebugValue();
  void pushPredictedVelKmph(const double predicted_vel_kmph);
  void pushPredictedVelLpfKmph(const double predicted_vel_lpf_kmph);
  void pushSafeVelCount(const size_t safe_vel_count);
  void clearDebugData();

  std::vector<geometry_msgs::msg::Pose> stop_poses;
  geometry_msgs::msg::Point nearest_point;
  std::shared_ptr<double> base_link2front;

private:
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_predicted_velocity_;
  rclcpp::Publisher<Float32Stamped>::SharedPtr pub_predicted_velocity_lpf_;
  rclcpp::Publisher<Int32Stamped>::SharedPtr pub_safe_vel_count_;

  rclcpp::Node & node_;

  Float32Stamped predicted_vel_kmph_;
  Float32Stamped predicted_vel_lpf_kmph_;
  Int32Stamped save_vel_count_;
};

}  // namespace bus_stop
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__BUS_STOP__DEBUG_HPP_
