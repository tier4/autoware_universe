// Copyright 2023 TIER IV, Inc.
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

#include "distributor.hpp"

#include <memory>

namespace ad_api_distributor
{

Distributor::Distributor() : Node("distributor")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  // Service
  srv_initialize_ = create_service<autoware_adapi_v1_msgs::srv::InitializeLocalization>(
    "/api/localization/initialize", std::bind(&Distributor::on_initialize, this, _1, _2));
  srv_set_route_ = create_service<autoware_adapi_v1_msgs::srv::SetRoute>(
    "/api/routing/set_route", std::bind(&Distributor::on_set_route, this, _1, _2));
  srv_clear_route_ = create_service<autoware_adapi_v1_msgs::srv::ClearRoute>(
    "/api/routing/clear_route", std::bind(&Distributor::on_clear_route, this, _1, _2));
  srv_stop_mode_ = create_service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_stop", std::bind(&Distributor::on_change_to_stop, this, _1, _2));
  srv_autonomous_mode_ = create_service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous",
    std::bind(&Distributor::on_change_to_autonomous, this, _1, _2));

  // Client
  cli_main_initialize_ = create_client<autoware_adapi_v1_msgs::srv::InitializeLocalization>(
    "/main/api/localization/initialize");
  cli_sub_initialize_ = create_client<autoware_adapi_v1_msgs::srv::InitializeLocalization>(
    "/sub/api/localization/initialize");
  cli_main_set_route_ =
    create_client<autoware_adapi_v1_msgs::srv::SetRoute>("/main/api/localization/initialize");
  cli_sub_set_route_ =
    create_client<autoware_adapi_v1_msgs::srv::SetRoute>("/sub/api/localization/initialize");
  cli_main_clear_route_ =
    create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/main/api/localization/initialize");
  cli_sub_clear_route_ =
    create_client<autoware_adapi_v1_msgs::srv::ClearRoute>("/sub/api/localization/initialize");
  cli_main_stop_mode_ = create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/main/api/localization/initialize");
  cli_sub_stop_mode_ = create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/sub/api/localization/initialize");
  cli_main_autonomous_mode_ = create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/main/api/localization/initialize");
  cli_sub_autonomous_mode_ = create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/sub/api/localization/initialize");
}

void Distributor::on_initialize(
  const autoware_adapi_v1_msgs::srv::InitializeLocalization::Request::SharedPtr req,
  const autoware_adapi_v1_msgs::srv::InitializeLocalization::Response::SharedPtr res)
{
  if (!cli_main_initialize_->service_is_ready() || !cli_sub_initialize_->service_is_ready()) {
    res->status.success = false;
    return;
  }
  cli_main_initialize_->async_send_request(req);
  cli_sub_initialize_->async_send_request(req);
  res->status.success = true;
}

void Distributor::on_set_route(
  const autoware_adapi_v1_msgs::srv::SetRoute::Request::SharedPtr req,
  const autoware_adapi_v1_msgs::srv::SetRoute::Response::SharedPtr res)
{
  if (!cli_main_set_route_->service_is_ready() || !cli_sub_set_route_->service_is_ready()) {
    res->status.success = false;
    return;
  }
  cli_main_set_route_->async_send_request(req);
  cli_sub_set_route_->async_send_request(req);
  res->status.success = true;
}

void Distributor::on_clear_route(
  const autoware_adapi_v1_msgs::srv::ClearRoute::Request::SharedPtr req,
  const autoware_adapi_v1_msgs::srv::ClearRoute::Response::SharedPtr res)
{
  if (!cli_main_clear_route_->service_is_ready() || !cli_sub_clear_route_->service_is_ready()) {
    res->status.success = false;
    return;
  }
  cli_main_clear_route_->async_send_request(req);
  cli_sub_clear_route_->async_send_request(req);
  res->status.success = true;
}

void Distributor::on_change_to_stop(
  const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr req,
  const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr res)
{
  if (!cli_main_stop_mode_->service_is_ready() || !cli_sub_stop_mode_->service_is_ready()) {
    res->status.success = false;
    return;
  }
  cli_main_stop_mode_->async_send_request(req);
  cli_sub_stop_mode_->async_send_request(req);
  res->status.success = true;
}

void Distributor::on_change_to_autonomous(
  const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr req,
  const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr res)
{
  if (
    !cli_main_autonomous_mode_->service_is_ready() ||
    !cli_sub_autonomous_mode_->service_is_ready()) {
    res->status.success = false;
    return;
  }
  cli_main_autonomous_mode_->async_send_request(req);
  cli_sub_autonomous_mode_->async_send_request(req);
  res->status.success = true;
}

}  // namespace ad_api_distributor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ad_api_distributor::Distributor>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
