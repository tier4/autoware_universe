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

#include "initial_pose_setter.hpp"

#include <memory>
#include <string>

namespace autoware::initial_pose_setter
{

InitialPoseSetter::InitialPoseSetter(const rclcpp::NodeOptions & options)
: Node("initial_pose_setter", options),
  state_(State::Message::UNKNOWN),
  latest_pose_received_time_(0, 0, RCL_ROS_TIME),
  request_in_flight_(0)
{
  pose_timeout_ = declare_parameter<double>("pose_timeout");
  const double retry_period = declare_parameter<double>("retry_period");

  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Client of the Autoware pose initializer.
  cli_initialize_ = create_client<Initialize::Service>(
    Initialize::name, rmw_qos_profile_services_default, group_cli_);

  // Autoware localization initialization state.
  sub_state_ = create_subscription<State::Message>(
    State::name, autoware::component_interface_specs::get_qos<State>(),
    std::bind(&InitialPoseSetter::on_state, this, std::placeholders::_1));

  // Pose estimated by the ar_tag_based_localizer.
  sub_pose_ = create_subscription<PoseWithCovarianceStamped>(
    "~/input/pose_with_covariance", rclcpp::QoS(1),
    std::bind(&InitialPoseSetter::on_pose, this, std::placeholders::_1));

  // One-shot initialization request.
  srv_initialize_ = create_service<Trigger>(
    "~/initialize",
    std::bind(&InitialPoseSetter::on_trigger, this, std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default);

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(retry_period), [this]() { on_timer(); });
}

void InitialPoseSetter::on_state(const State::Message::ConstSharedPtr msg)
{
  state_ = msg->state;
}

void InitialPoseSetter::on_pose(const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  latest_pose_ = *msg;
  latest_pose_received_time_ = now();
}

std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> InitialPoseSetter::get_valid_pose(
  std::string & error) const
{
  if (!latest_pose_) {
    error = "No pose has been received from the ar_tag_based_localizer yet.";
    return std::nullopt;
  }
  const double age = (now() - latest_pose_received_time_).seconds();
  if (pose_timeout_ > 0.0 && age > pose_timeout_) {
    error = "The latest pose is too old (" + std::to_string(age) + " [s]).";
    return std::nullopt;
  }
  return latest_pose_;
}

bool InitialPoseSetter::send_initialize_request(
  const PoseWithCovarianceStamped & pose, std::string & error)
{
  if (!cli_initialize_->service_is_ready()) {
    error = std::string(Initialize::name) + " is not available.";
    return false;
  }

  auto req = std::make_shared<Initialize::Service::Request>();
  req->pose_with_covariance.push_back(pose);
  req->method = Initialize::Service::Request::DIRECT;

  ++request_in_flight_;
  cli_initialize_->async_send_request(
    req, [this](rclcpp::Client<Initialize::Service>::SharedFuture future) {
      --request_in_flight_;
      const auto res = future.get();
      if (res->status.success) {
        RCLCPP_INFO(get_logger(), "Initial pose has been set.");
      } else {
        RCLCPP_WARN(
          get_logger(), "Failed to set the initial pose: %s (code: %u)",
          res->status.message.c_str(), res->status.code);
      }
    });
  return true;
}

void InitialPoseSetter::on_timer()
{
  // Nothing to do once the localization has been initialized.
  if (state_ == State::Message::INITIALIZED) {
    return;
  }
  // The pose initializer is working on a request. Do not stack requests.
  if (state_ == State::Message::INITIALIZING || request_in_flight_ > 0) {
    return;
  }

  std::string error;
  const auto pose = get_valid_pose(error);
  if (!pose) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
    return;
  }
  if (!send_initialize_request(*pose, error)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
  }
}

void InitialPoseSetter::on_trigger(
  const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr res)
{
  // Executed regardless of the initialization state.
  std::string error;
  const auto pose = get_valid_pose(error);
  if (!pose) {
    res->success = false;
    res->message = error;
    return;
  }
  if (!send_initialize_request(*pose, error)) {
    res->success = false;
    res->message = error;
    return;
  }
  res->success = true;
  res->message = "The initialize request has been sent.";
}

}  // namespace autoware::initial_pose_setter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::initial_pose_setter::InitialPoseSetter)
