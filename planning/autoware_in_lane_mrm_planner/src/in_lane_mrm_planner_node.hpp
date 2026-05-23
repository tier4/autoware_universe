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

#ifndef IN_LANE_MRM_PLANNER_NODE_HPP_
#define IN_LANE_MRM_PLANNER_NODE_HPP_

#include <in_lane_mrm_planner_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::in_lane_mrm_planner
{

class InLaneMrmPlannerNode : public rclcpp::Node
{
public:
  explicit InLaneMrmPlannerNode(const rclcpp::NodeOptions & options);

private:
  void on_timer();

  std::shared_ptr<::in_lane_mrm_planner::ParamListener> param_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // IN_LANE_MRM_PLANNER_NODE_HPP_
