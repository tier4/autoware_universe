// Copyright 2022 The Autoware Contributors
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
// limitations under the License.S
#include "pose_initializer/localization_trigger_module.hpp"

#include <memory>

LocalizationTriggerModule::LocalizationTriggerModule(rclcpp::Node * node)
: logger_(node->get_logger())
{
  client_ekf_trigger_ = node->create_client<SetBool>("ekf_trigger_node");
  client_ndt_trigger_ = node->create_client<SetBool>("ndt_trigger_node");
}

void LocalizationTriggerModule::deactivate() const
{
  const auto req = std::make_shared<SetBool::Request>();
  req->data = false;

  if (!client_ekf_trigger_->service_is_ready()) {
    RCLCPP_INFO(logger_, "EKF triggering service is not ready");
  }
  if (!client_ndt_trigger_->service_is_ready()) {
    RCLCPP_INFO(logger_, "NDT triggering service is not ready");
  }

  auto future_ekf = client_ekf_trigger_->async_send_request(req);
  auto future_ndt = client_ndt_trigger_->async_send_request(req);

  if (future_ekf.get()->success & future_ndt.get()->success) {
    RCLCPP_INFO(logger_, "Deactivation succeeded");
  } else {
    RCLCPP_INFO(logger_, "Deactivation failed");
  }
}

void LocalizationTriggerModule::activate() const
{
  const auto req = std::make_shared<SetBool::Request>();
  req->data = true;

  if (!client_ekf_trigger_->service_is_ready()) {
    RCLCPP_INFO(logger_, "EKF triggering service is not ready");
  }
  if (!client_ndt_trigger_->service_is_ready()) {
    RCLCPP_INFO(logger_, "NDT triggering service is not ready");
  }

  auto future_ekf = client_ekf_trigger_->async_send_request(req);
  auto future_ndt = client_ndt_trigger_->async_send_request(req);

  if (future_ekf.get()->success & future_ndt.get()->success) {
    RCLCPP_INFO(logger_, "Activation succeeded");
  } else {
    RCLCPP_INFO(logger_, "Activation failed");
  }
}
