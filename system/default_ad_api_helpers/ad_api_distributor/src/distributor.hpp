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

#ifndef DISTRIBUTOR_HPP_
#define DISTRIBUTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>

namespace ad_api_distributor
{

class Distributor : public rclcpp::Node
{
public:
  Distributor();

private:
  // Service
  rclcpp::Service<autoware_adapi_v1_msgs::srv::InitializeLocalization>::SharedPtr srv_initialize_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::SetRoute>::SharedPtr srv_set_route_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr srv_clear_route_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr srv_stop_mode_;
  rclcpp::Service<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr srv_autonomous_mode_;

  // Client
  rclcpp::Client<autoware_adapi_v1_msgs::srv::InitializeLocalization>::SharedPtr
    cli_main_initialize_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::InitializeLocalization>::SharedPtr
    cli_sub_initialize_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoute>::SharedPtr cli_main_set_route_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::SetRoute>::SharedPtr cli_sub_set_route_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr cli_main_clear_route_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr cli_sub_clear_route_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr cli_main_stop_mode_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr cli_sub_stop_mode_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    cli_main_autonomous_mode_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr
    cli_sub_autonomous_mode_;

  // Function
  void on_initialize(
    const autoware_adapi_v1_msgs::srv::InitializeLocalization::Request::SharedPtr req,
    const autoware_adapi_v1_msgs::srv::InitializeLocalization::Response::SharedPtr res);

  void on_set_route(
    const autoware_adapi_v1_msgs::srv::SetRoute::Request::SharedPtr req,
    const autoware_adapi_v1_msgs::srv::SetRoute::Response::SharedPtr res);

  void on_clear_route(
    const autoware_adapi_v1_msgs::srv::ClearRoute::Request::SharedPtr req,
    const autoware_adapi_v1_msgs::srv::ClearRoute::Response::SharedPtr res);

  void on_change_to_stop(
    const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr req,
    const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr res);

  void on_change_to_autonomous(
    const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request::SharedPtr req,
    const autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response::SharedPtr res);
};

}  // namespace ad_api_distributor

#endif  // DISTRIBUTOR_HPP_
