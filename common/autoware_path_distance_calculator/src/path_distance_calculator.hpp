// Copyright 2021 Tier IV, Inc.
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

#ifndef PATH_DISTANCE_CALCULATOR_HPP_
#define PATH_DISTANCE_CALCULATOR_HPP_

#include <agnocast/agnocast.hpp>
#include <autoware_utils_tf/self_pose_listener.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
#include <autoware_planning_msgs/msg/path.hpp>

namespace autoware::path_distance_calculator
{

class PathDistanceCalculator : public agnocast::Node
{
public:
  explicit PathDistanceCalculator(const rclcpp::NodeOptions & options);

private:
  agnocast::PollingSubscriber<autoware_planning_msgs::msg::Path>::SharedPtr sub_path_;
  agnocast::Publisher<autoware_internal_debug_msgs::msg::Float64Stamped>::SharedPtr pub_dist_;
  agnocast::TimerBase::SharedPtr timer_;
  autoware_utils_tf::SelfPoseListener self_pose_listener_;
};

}  // namespace autoware::path_distance_calculator

#endif  // PATH_DISTANCE_CALCULATOR_HPP_
