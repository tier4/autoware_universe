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

#include "autoware/ml_planner/preprocessing/items/agent.hpp"

#include "autoware/ml_planner/utils/utils.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <xtensor/xbuilder.hpp>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::ml_planner::preprocess
{

namespace
{

AgentLabel get_model_label(const TrackedObject & object)
{
  const uint8_t autoware_label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);

  switch (autoware_label) {
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
    case autoware_perception_msgs::msg::ObjectClassification::TRAILER:
      return AgentLabel::VEHICLE;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      return AgentLabel::BICYCLE;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      return AgentLabel::PEDESTRIAN;
    default:
      return AgentLabel::IGNORE;
  }
}

}  // namespace

size_t AgentIdHash::operator()(const AgentId & id) const noexcept
{
  size_t value = 0;
  for (const uint8_t byte : id) {
    value = value * 131U + byte;
  }
  return value;
}

std::vector<SelectedAgent> select_current_agents(
  const MessageView<TrackedObjects> & objects_msgs, const rclcpp::Time & current_time,
  const Eigen::Matrix4d & map_to_ego_transform, const size_t max_num_agent)
{
  const TrackedObjects * current_msg = nullptr;
  for (auto it = objects_msgs.rbegin(); it != objects_msgs.rend(); ++it) {
    if (rclcpp::Time(it->header.stamp) <= current_time) {
      current_msg = &*it;
      break;
    }
  }
  if (current_msg == nullptr) {
    return {};
  }

  struct Candidate
  {
    SelectedAgent agent;
    double distance;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(current_msg->objects.size());
  std::unordered_set<AgentId, AgentIdHash> seen;
  for (const auto & object : current_msg->objects) {
    if (
      get_model_label(object) == AgentLabel::IGNORE ||
      object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
      continue;
    }
    const AgentId & id = object.object_id.uuid;
    if (!seen.insert(id).second) {
      continue;
    }
    const Eigen::Matrix4d pose =
      map_to_ego_transform * utils::pose_to_matrix4d(object.kinematics.pose_with_covariance.pose);
    candidates.push_back({SelectedAgent{id, object, pose}, std::hypot(pose(0, 3), pose(1, 3))});
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.distance < rhs.distance;
  });
  if (candidates.size() > max_num_agent) {
    candidates.resize(max_num_agent);
  }
  std::vector<SelectedAgent> result;
  result.reserve(candidates.size());
  for (auto & candidate : candidates) {
    result.push_back(std::move(candidate.agent));
  }
  return result;
}

xt::xarray<float> create_neighbor_agent_sequence(
  const MessageView<TrackedObjects> & objects_msgs, const std::vector<SelectedAgent> & agents,
  const rclcpp::Time & current_time, const Eigen::Matrix4d & map_to_ego_transform,
  const size_t max_num_agent, const size_t time_length, const double time_step_s,
  const AgentSequenceDirection direction, const std::optional<double> observation_timeout_s)
{
  xt::xarray<float> data = xt::zeros<float>({max_num_agent, time_length, AGENT_POSE_DIM});
  const double current_sec = current_time.seconds();
  struct Observation
  {
    double stamp_sec;
    const TrackedObject * object;
  };
  std::unordered_map<AgentId, size_t, AgentIdHash> selected_indices;
  const size_t num_agents = std::min(agents.size(), max_num_agent);
  for (size_t i = 0; i < num_agents; ++i) {
    selected_indices.emplace(agents[i].object_id, i);
  }
  std::vector<std::vector<Observation>> observations(num_agents);
  for (const auto & msg : objects_msgs) {
    const double stamp_sec = rclcpp::Time(msg.header.stamp).seconds();
    if (direction == AgentSequenceDirection::Past) {
      if (stamp_sec > current_sec + 1e-6) {
        break;
      }
    } else if (stamp_sec <= current_sec) {
      continue;
    }
    for (const auto & object : msg.objects) {
      const auto selected = selected_indices.find(object.object_id.uuid);
      if (selected != selected_indices.end()) {
        observations[selected->second].push_back({stamp_sec, &object});
      }
    }
  }
  for (size_t agent_idx = 0; agent_idx < num_agents; ++agent_idx) {
    const auto & agent_observations = observations[agent_idx];
    if (agent_observations.empty()) {
      continue;
    }
    size_t observation_idx = 0;
    for (size_t t = 0; t < time_length; ++t) {
      const double grid_sec =
        direction == AgentSequenceDirection::Past
          ? current_sec - static_cast<double>(time_length - 1 - t) * time_step_s
          : current_sec + static_cast<double>(t + 1) * time_step_s;
      while (observation_idx + 1 < agent_observations.size() &&
             agent_observations[observation_idx + 1].stamp_sec <= grid_sec + 1e-6) {
        ++observation_idx;
      }
      if (agent_observations[observation_idx].stamp_sec > grid_sec + 1e-6) {
        continue;
      }
      if (
        observation_timeout_s.has_value() &&
        grid_sec - agent_observations[observation_idx].stamp_sec > *observation_timeout_s) {
        continue;
      }
      const TrackedObject * observation = agent_observations[observation_idx].object;
      const Eigen::Matrix4d pose =
        map_to_ego_transform *
        utils::pose_to_matrix4d(observation->kinematics.pose_with_covariance.pose);
      const auto [cos_yaw, sin_yaw] = utils::rotation_matrix_to_cos_sin(pose.block<3, 3>(0, 0));
      data(agent_idx, t, 0) = static_cast<float>(pose(0, 3));
      data(agent_idx, t, 1) = static_cast<float>(pose(1, 3));
      data(agent_idx, t, 2) = cos_yaw;
      data(agent_idx, t, 3) = sin_yaw;
    }
  }
  return data;
}

xt::xarray<float> create_agent_shape(
  const std::vector<SelectedAgent> & agents, const size_t max_num_agent)
{
  xt::xarray<float> data = xt::zeros<float>({max_num_agent, AGENT_SHAPE_DIM});
  const size_t num_agents = std::min(agents.size(), max_num_agent);
  for (size_t i = 0; i < num_agents; ++i) {
    data(i, 0) = static_cast<float>(agents[i].current_object.shape.dimensions.y);
    data(i, 1) = static_cast<float>(agents[i].current_object.shape.dimensions.x);
  }
  return data;
}

xt::xarray<float> create_agent_label(
  const std::vector<SelectedAgent> & agents, const size_t max_num_agent)
{
  xt::xarray<float> data = xt::zeros<float>({max_num_agent, AGENT_LABEL_DIM});
  const size_t num_agents = std::min(agents.size(), max_num_agent);
  for (size_t i = 0; i < num_agents; ++i) {
    const AgentLabel label = get_model_label(agents[i].current_object);
    if (label != AgentLabel::IGNORE) {
      data(i, static_cast<size_t>(label)) = 1.0F;
    }
  }
  return data;
}

}  // namespace autoware::ml_planner::preprocess
