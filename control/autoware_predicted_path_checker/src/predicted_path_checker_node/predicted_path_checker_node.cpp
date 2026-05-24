// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#include "autoware/predicted_path_checker/predicted_path_checker_node.hpp"

#include <chrono>
#include <functional>

namespace autoware::predicted_path_checker
{

PredictedPathCheckerNode::PredictedPathCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("predicted_path_checker_node", node_options), updater_(this)
{
  core_ = std::make_unique<PredictedPathCheckerCore>(this);

  updater_.setHardwareID("predicted_path_checker");
  updater_.add("predicted_path_checker", this, &PredictedPathCheckerNode::check_vehicle_state);

  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / core_->get_update_rate()));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&PredictedPathCheckerNode::on_timer, this));
}

void PredictedPathCheckerNode::on_timer()
{
  core_->on_timer_cycle();
  updater_.force_update();
}

void PredictedPathCheckerNode::check_vehicle_state(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  core_->check_vehicle_state(stat);
}

}  // namespace autoware::predicted_path_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::predicted_path_checker::PredictedPathCheckerNode)
