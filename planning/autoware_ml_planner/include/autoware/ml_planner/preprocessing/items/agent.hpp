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

#ifndef AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__AGENT_HPP_
#define AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__AGENT_HPP_

#include <Eigen/Dense>

#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

namespace autoware::ml_planner::preprocess
{
using autoware_perception_msgs::msg::TrackedObject;

using AgentId = unique_identifier_msgs::msg::UUID::_uuid_type;

struct SelectedAgent
{
  AgentId object_id;
  TrackedObject current_object;
  Eigen::Matrix4d current_pose_ego{Eigen::Matrix4d::Identity()};
};

}  // namespace autoware::ml_planner::preprocess
#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__AGENT_HPP_
