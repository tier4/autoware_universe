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

#ifndef INITIAL_POSE_SETTER_HPP_
#define INITIAL_POSE_SETTER_HPP_

#include <autoware/component_interface_specs/localization.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <optional>
#include <string>

namespace autoware::initial_pose_setter
{

class InitialPoseSetter : public rclcpp::Node
{
public:
  explicit InitialPoseSetter(const rclcpp::NodeOptions & options);

private:
  using Initialize = autoware::component_interface_specs::localization::Initialize;
  using State = autoware::component_interface_specs::localization::InitializationState;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Trigger = std_srvs::srv::Trigger;

  void on_state(const State::Message::ConstSharedPtr msg);
  void on_pose(const PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_timer();
  void on_trigger(const Trigger::Request::SharedPtr req, const Trigger::Response::SharedPtr res);

  // Returns the latest pose if it is available and fresh. Otherwise returns the reason in `error`.
  std::optional<PoseWithCovarianceStamped> get_valid_pose(std::string & error) const;
  // Sends one request to /localization/initialize. Returns false if it could not be sent.
  bool send_initialize_request(const PoseWithCovarianceStamped & pose, std::string & error);

  // parameters
  double pose_timeout_;  // [s]

  // state
  uint16_t state_;
  std::optional<PoseWithCovarianceStamped> latest_pose_;
  rclcpp::Time latest_pose_received_time_;
  std::atomic<int> request_in_flight_;

  // Everything except the client response runs in the default (mutually exclusive) group, so the
  // pose and the state do not need a mutex.
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<Initialize::Service>::SharedPtr cli_initialize_;
  rclcpp::Service<Trigger>::SharedPtr srv_initialize_;
  rclcpp::Subscription<State::Message>::SharedPtr sub_state_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_pose_;
};

}  // namespace autoware::initial_pose_setter

#endif  // INITIAL_POSE_SETTER_HPP_
