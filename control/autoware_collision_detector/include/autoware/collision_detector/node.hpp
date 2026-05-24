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

#ifndef AUTOWARE__COLLISION_DETECTOR__NODE_HPP_
#define AUTOWARE__COLLISION_DETECTOR__NODE_HPP_

#include "autoware/collision_detector/collision_detector_core.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::collision_detector
{

class CollisionDetectorNode : public rclcpp::Node
{
public:
  explicit CollisionDetectorNode(const rclcpp::NodeOptions & node_options);

private:
  void check_collision(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::unique_ptr<CollisionDetectorCore> core_;
  diagnostic_updater::Updater updater_;
};

}  // namespace autoware::collision_detector

#endif  // AUTOWARE__COLLISION_DETECTOR__NODE_HPP_
