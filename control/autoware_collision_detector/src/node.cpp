// Copyright 2024-2025 TIER IV, Inc.
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

#include "autoware/collision_detector/node.hpp"

namespace autoware::collision_detector
{

CollisionDetectorNode::CollisionDetectorNode(const rclcpp::NodeOptions & node_options)
: Node("collision_detector_node", node_options), updater_(this)
{
  core_ = std::make_unique<CollisionDetectorCore>(this);
  core_->setup_diagnostics();

  updater_.setHardwareID("collision_detector");
  updater_.add("collision_detect", this, &CollisionDetectorNode::check_collision);
  updater_.setPeriod(0.1);
}

void CollisionDetectorNode::check_collision(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  core_->update_diagnostics(stat);
}

}  // namespace autoware::collision_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::collision_detector::CollisionDetectorNode)
