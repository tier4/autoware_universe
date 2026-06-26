// Copyright 2023 Autoware Foundation
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

#include "switch_rule/service_call_based_rule.hpp"

#include <magic_enum.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace autoware::pose_estimator_arbiter::switch_rule
{
ServiceCallBasedRule::ServiceCallBasedRule(
  rclcpp::Node & node, std::unordered_set<PoseEstimatorType> running_estimator_list,
  std::shared_ptr<const SharedData> shared_data)
: BaseSwitchRule(node),
  running_estimator_list_(std::move(running_estimator_list)),
  shared_data_(std::move(shared_data))
{
  pose_estimator_area_ = std::make_unique<rule_helper::PoseEstimatorArea>(node.get_logger());

  switch_service_ =
    node.create_service<autoware_internal_localization_msgs::srv::PoseEstimatorSwitch>(
      "~/switcher",
      std::bind(
        &ServiceCallBasedRule::switcher_service, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile());

  // Register callback
  // shared_data_->vector_map.register_callback(
  //   [this](autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) -> void {
  //     pose_estimator_area_->init(msg);
  //   });

  RCLCPP_INFO_STREAM(get_logger(), "ServiceCallBasedRule is initialized successfully");
}

ServiceCallBasedRule::MarkerArray ServiceCallBasedRule::debug_marker_array()
{
  MarkerArray array_msg;

  // if (pose_estimator_area_) {
  //   const auto & additional = pose_estimator_area_->debug_marker_array().markers;
  //   array_msg.markers.insert(array_msg.markers.end(), additional.begin(), additional.end());
  // }

  return array_msg;
}

void ServiceCallBasedRule::switcher_service(
  const autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Response::SharedPtr res)
{
  enable_list_.clear();

  const bool enable_ndt =  req->method & autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::NDT;
  enable_list_.emplace(PoseEstimatorType::ndt, enable_ndt);

  const bool enable_yabloc =  req->method & autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::YABLOC;
  enable_list_.emplace(PoseEstimatorType::yabloc, enable_yabloc);

  const bool enable_eagleye =  req->method & autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::EAGLEYE;
  enable_list_.emplace(PoseEstimatorType::eagleye, enable_eagleye);

  const bool enable_artag =  req->method & autoware_internal_localization_msgs::srv::PoseEstimatorSwitch::Request::ARTAG;
  enable_list_.emplace(PoseEstimatorType::artag, enable_artag);

  RCLCPP_INFO(get_logger(), "ndt: %d, yabloc:%d, eagleye:%d, artag:%d", enable_ndt, enable_yabloc, enable_eagleye, enable_artag);


  res->success = true;

  return;
}

std::unordered_map<PoseEstimatorType, bool> ServiceCallBasedRule::update()
{
  debug_string_ = "";

  if(enable_list_.empty()) {
    debug_string_ = "Enable all: enable_list is empty";
    return {
      {PoseEstimatorType::ndt, true},
      {PoseEstimatorType::yabloc, true},
      {PoseEstimatorType::eagleye, true},
      {PoseEstimatorType::artag, true},
    };
  }

  bool at_least_one_is_enabled = false;
  for (const auto & estimator_enable : enable_list_ ) {
    if (estimator_enable.second) {
      const std::string estimator_name{magic_enum::enum_name(estimator_enable.first)};
      debug_string_ += " " + estimator_name;
    }
    at_least_one_is_enabled |= estimator_enable.second;
  }
  if (at_least_one_is_enabled) {
    // debug_string_ =
    //   "Enable at least one pose_estimators: self vehicle is within the area of at least one "
    //   "pose_estimator_area";
  } else {
    // debug_string_ = "Enable ndt: All methods are false.";
    // enable_list_[PoseEstimatorType::ndt] = true;
  }
  RCLCPP_DEBUG(get_logger(), "%s", debug_string_.c_str());

  return enable_list_;

}

}  // namespace autoware::pose_estimator_arbiter::switch_rule
