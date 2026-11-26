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

#include "Eigen/Dense"
#include "autoware/ml_planner/preprocessing/message_view.hpp"

#include <rclcpp/time.hpp>
#include <xtensor/xarray.hpp>

#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <cstddef>
#include <optional>
#include <vector>

namespace autoware::ml_planner::preprocess
{
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

constexpr size_t AGENT_POSE_DIM = 4;
constexpr size_t AGENT_SHAPE_DIM = 2;
constexpr size_t AGENT_LABEL_DIM = 3;

enum AgentLabel { VEHICLE = 0, PEDESTRIAN = 1, BICYCLE = 2, IGNORE = 3 };

using AgentId = unique_identifier_msgs::msg::UUID::_uuid_type;

struct AgentIdHash
{
  size_t operator()(const AgentId & id) const noexcept;
};

struct SelectedAgent
{
  AgentId object_id;
  TrackedObject current_object;
  Eigen::Matrix4d current_pose_ego{Eigen::Matrix4d::Identity()};
};

enum class AgentSequenceDirection { Past, Future };

std::vector<SelectedAgent> select_current_agents(
  const MessageView<TrackedObjects> & objects_msgs, const rclcpp::Time & current_time,
  const Eigen::Matrix4d & map_to_ego_transform, size_t max_num_agent);

xt::xarray<float> create_neighbor_agent_sequence(
  const MessageView<TrackedObjects> & objects_msgs, const std::vector<SelectedAgent> & agents,
  const rclcpp::Time & current_time, const Eigen::Matrix4d & map_to_ego_transform,
  size_t max_num_agent, size_t time_length, double time_step_s, AgentSequenceDirection direction,
  std::optional<double> observation_timeout_s = std::nullopt);

xt::xarray<float> create_agent_shape(
  const std::vector<SelectedAgent> & agents, size_t max_num_agent);

xt::xarray<float> create_agent_label(
  const std::vector<SelectedAgent> & agents, size_t max_num_agent);

}  // namespace autoware::ml_planner::preprocess
#endif  // AUTOWARE__ML_PLANNER__PREPROCESSING__ITEMS__AGENT_HPP_
