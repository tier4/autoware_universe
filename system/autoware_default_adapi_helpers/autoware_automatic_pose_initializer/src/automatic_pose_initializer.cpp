// Copyright 2022 TIER IV, Inc.
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

#include "automatic_pose_initializer.hpp"

#include <memory>

namespace autoware::automatic_pose_initializer
{

AutomaticPoseInitializer::AutomaticPoseInitializer(const rclcpp::NodeOptions & options)
: Node("autoware_automatic_pose_initializer", options)
{
  RCLCPP_INFO(get_logger(), "AutomaticPoseInitializer constructor started");

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(get_logger(), "Initializing client and subscriber...");
  adaptor.init_cli(cli_initialize_, group_cli_);

  RCLCPP_INFO(get_logger(), "Creating subscription for initialization_state...");
  adaptor.init_sub(sub_state_, [this](const State::Message::ConstSharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "Received initialization_state: state=%d", msg->state);
    state_ = *msg;
  });
  RCLCPP_INFO(get_logger(), "Subscription created successfully");

  const auto period = rclcpp::Rate(1.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });

  state_.stamp = now();
  state_.state = State::Message::UNKNOWN;

  RCLCPP_INFO(get_logger(), "AutomaticPoseInitializer initialized with UNKNOWN state");
}

void AutomaticPoseInitializer::on_timer()
{
  timer_->cancel();
  if (state_.state == State::Message::UNINITIALIZED) {
    try {
      const auto req = std::make_shared<Initialize::Service::Request>();
      cli_initialize_->call(req);
    } catch (const autoware::component_interface_utils::ServiceException & error) {
      RCLCPP_ERROR(get_logger(), "service exception: %s", error.what());
    }
  }
  timer_->reset();
}

}  // namespace autoware::automatic_pose_initializer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::automatic_pose_initializer::AutomaticPoseInitializer)
