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

#include "in_lane_mrm_planner_node.hpp"

#include <memory>

namespace autoware::in_lane_mrm_planner
{

InLaneMrmPlannerNode::InLaneMrmPlannerNode(const rclcpp::NodeOptions & options)
: Node("in_lane_mrm_planner", options)
{
  param_listener_ = std::make_shared<::in_lane_mrm_planner::ParamListener>(
    get_node_parameters_interface());
  const auto params = param_listener_->get_params();

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params.planning_frequency_hz).period(),
    std::bind(&InLaneMrmPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "InLaneMrmPlannerNode started (skeleton).");
}

void InLaneMrmPlannerNode::on_timer()
{
  // Phase1 pipeline will be wired in Task 9.
}

}  // namespace autoware::in_lane_mrm_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::in_lane_mrm_planner::InLaneMrmPlannerNode)
