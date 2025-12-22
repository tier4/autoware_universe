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

#ifndef TIME_TO_SPACE_CONVERTER_NODE_HPP_
#define TIME_TO_SPACE_CONVERTER_NODE_HPP_

#include "data_types.hpp"
#include "parameters.hpp"
#include "stop_state_machine.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{

class TimeToSpaceConverterNode : public rclcpp::Node
{
public:
  explicit TimeToSpaceConverterNode(const rclcpp::NodeOptions & options);

private:
  // Publishers & Subscribers
  autoware_utils_rclcpp::InterProcessPollingSubscriber<autoware_planning_msgs::msg::Trajectory>
    polling_sub_traj_{this, "~/time_to_space_converter/input/trajectory"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> polling_sub_odom_{
    this, "~/time_to_space_converter/input/odometry"};
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr pub_traj_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr sub_traj_ptr_;
  nav_msgs::msg::Odometry::ConstSharedPtr sub_odom_ptr_;

  // State
  double current_s_ = 0.0;
  double current_v_ = 0.0;
  StopStateMachine stop_state_machine_;
  NodeParam params_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  std::deque<nav_msgs::msg::Odometry> odom_history_;

  void take_data();
  std::optional<std::string> has_invalid_data() const;
  void on_timer();
  void update_history(const nav_msgs::msg::Odometry & odom);

  std::vector<PlannerPoint> get_current_trajectory_points() const;
  std::vector<PlannerPoint> get_history_as_planner_points() const;
  std::vector<PlannerPoint> assemble_input_points();

  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  template <typename... Args>
  void warn_throttle(const char * fmt, Args... args)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, fmt, args...);
  }

  template <typename... Args>
  void debug_throttle(const char * fmt, Args... args)
  {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, fmt, args...);
  }
};

}  // namespace autoware::time_to_space_trajectory_converter

#endif  // TIME_TO_SPACE_CONVERTER_NODE_HPP_
