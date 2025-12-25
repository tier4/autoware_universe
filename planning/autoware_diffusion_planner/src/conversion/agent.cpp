// Copyright 2025 TIER IV, Inc.
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

#include "autoware/diffusion_planner/conversion/agent.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::diffusion_planner
{
AgentLabel get_model_label(const autoware_perception_msgs::msg::TrackedObject & object)
{
  auto autoware_label =
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
      return AgentLabel::VEHICLE;
  }
}

AgentState::AgentState(const TrackedObject & object)
{
  position_ = object.kinematics.pose_with_covariance.pose.position;
  float yaw =
    autoware_utils_geometry::get_rpy(object.kinematics.pose_with_covariance.pose.orientation).z;
  yaw_ = yaw;
  cos_yaw_ = std::cos(yaw);
  sin_yaw_ = std::sin(yaw);
  velocity_ = object.kinematics.twist_with_covariance.twist.linear;
  label_ = get_model_label(object);
  object_id_ = autoware_utils_uuid::to_hex_string(object.object_id);
  autoware_label_ = autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  tracked_object_info_ = object;
}

void AgentState::apply_transform(const Eigen::Matrix4d & transform)
{
  Eigen::Vector4d pos_vec(position_.x, position_.y, position_.z, 1.0);
  Eigen::Vector4d transformed_pos = transform * pos_vec;
  position_.x = transformed_pos.x();
  position_.y = transformed_pos.y();
  position_.z = transformed_pos.z();

  Eigen::Vector4d dir_vec(cos_yaw_, sin_yaw_, 0.0, 0.0);
  Eigen::Vector4d transformed_dir = transform * dir_vec;
  cos_yaw_ = transformed_dir.x();
  sin_yaw_ = transformed_dir.y();
  yaw_ = std::atan2(sin_yaw_, cos_yaw_);

  const double velocity_norm = std::hypot(velocity_.x, velocity_.y);
  velocity_.x = velocity_norm * cos_yaw_;
  velocity_.y = velocity_norm * sin_yaw_;
}

// Return the state attribute as an array.
[[nodiscard]] std::array<float, AGENT_STATE_DIM> AgentState::as_array() const noexcept
{
  return {
    static_cast<float>(position_.x),
    static_cast<float>(position_.y),
    cos_yaw_,
    sin_yaw_,
    static_cast<float>(velocity_.x),
    static_cast<float>(velocity_.y),
    static_cast<float>(tracked_object_info_.shape.dimensions.y),  // width
    static_cast<float>(tracked_object_info_.shape.dimensions.x),  // length
    static_cast<float>(label_ == AgentLabel::VEHICLE),
    static_cast<float>(label_ == AgentLabel::PEDESTRIAN),
    static_cast<float>(label_ == AgentLabel::BICYCLE),
  };
}

AgentHistory::AgentHistory(
  const AgentState & state, const size_t label_id, const double current_time,
  const size_t max_time_length, bool is_pad_history)
: queue_(max_time_length),
  object_id_(state.object_id_),
  label_id_(label_id),
  autoware_label_(state.autoware_label_),
  latest_time_(current_time),
  max_size_(max_time_length)
{
  queue_.push_back(state);
  if (is_pad_history) {
    pad_history();
  }
}

void AgentHistory::update(double current_time, const TrackedObject & object)
{
  AgentState state(object);
  if (state.object_id_ != object_id_) {
    throw std::runtime_error("Object ID mismatch");
  }
  queue_.push_back(state);
  latest_time_ = current_time;
}

void AgentHistory::update(double current_time, const AgentState & state)
{
  if (state.object_id_ != object_id_) {
    throw std::runtime_error("Object ID mismatch");
  }
  queue_.push_back(state);
  latest_time_ = current_time;
}

[[nodiscard]] std::vector<float> AgentHistory::as_array() const noexcept
{
  std::vector<float> output;
  for (const auto & state : queue_) {
    for (const auto & v : state.as_array()) {
      output.push_back(v);
    }
  }
  return output;
}

void AgentHistory::pad_history(bool pad_front)
{
  auto state = (pad_front) ? queue_.front() : queue_.back();
  while (!is_full()) {
    if (pad_front) {
      queue_.push_front(state);
    } else {
      queue_.push_back(state);
    }
  }
}

AgentData::AgentData(
  const autoware_perception_msgs::msg::TrackedObjects & objects, const size_t max_num_agent,
  const size_t num_timestamps, const bool ignore_unknown_agents)
: max_num_agent_(max_num_agent), time_length_(num_timestamps)
{
  std::vector<AgentHistory> histories;
  for (auto object : objects.objects) {
    if (ignore_unknown_agents && is_unknown_object(object)) {
      continue;
    }
    auto agent_state = AgentState(object);
    auto current_time = static_cast<double>(objects.header.stamp.sec) +
                        static_cast<double>(objects.header.stamp.nanosec) * 1e-9;
    histories.emplace_back(
      agent_state, get_model_label(object), current_time, num_timestamps, true);
  }
  set_histories(histories);
}

bool AgentData::is_unknown_object(const autoware_perception_msgs::msg::TrackedObject & object)
{
  const auto autoware_label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);
  return autoware_label == autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
}

void AgentData::update_histories(
  const autoware_perception_msgs::msg::TrackedObjects & objects, const bool ignore_unknown_agents)
{
  auto current_time = static_cast<double>(objects.header.stamp.sec) +
                      static_cast<double>(objects.header.stamp.nanosec) * 1e-9;
  std::vector<std::string> found_ids;
  for (auto object : objects.objects) {
    if (ignore_unknown_agents && is_unknown_object(object)) {
      continue;
    }
    auto object_id = autoware_utils_uuid::to_hex_string(object.object_id);
    auto it = histories_idx_map_.find(object_id);
    if (it != histories_idx_map_.end()) {
      histories_[it->second].update(current_time, object);
    } else {
      auto agent_state = AgentState(object);
      histories_.emplace_back(
        agent_state, get_model_label(object), current_time, time_length_, true);
    }
    found_ids.push_back(object_id);
  }
  // Remove histories that are not found in the current objects
  histories_.erase(
    std::remove_if(
      histories_.begin(), histories_.end(),
      [&found_ids](const AgentHistory & history) {
        return std::find(found_ids.begin(), found_ids.end(), history.object_id()) ==
               found_ids.end();
      }),
    histories_.end());

  set_histories(histories_);
}

void AgentData::trim_to_k_closest_agents()
{
  geometry_msgs::msg::Point position;
  position.x = 0.0;
  position.y = 0.0;
  position.z = 0.0;
  trim_to_k_closest_agents(position);
}

void AgentData::trim_to_k_closest_agents(const geometry_msgs::msg::Point & position)
{
  std::sort(
    histories_.begin(), histories_.end(),
    [&position](const AgentHistory & a, const AgentHistory & b) {
      return autoware_utils_geometry::calc_distance2d(position, a.get_latest_state_position()) <
             autoware_utils_geometry::calc_distance2d(position, b.get_latest_state_position());
    });
  auto k = std::min(num_agent_, max_num_agent_);
  std::vector<AgentHistory> closest_agents(histories_.begin(), histories_.begin() + k);
  set_histories(closest_agents);
}

void AgentData::set_histories(const std::vector<AgentHistory> & histories)
{
  histories_ = histories;
  num_agent_ = histories_.size();
  histories_idx_map_.clear();

  size_t agent_id = 0;
  for (const auto & history : histories_) {
    histories_idx_map_.emplace(history.object_id(), agent_id++);
  }
}

std::vector<float> AgentData::build_data_vector(bool pad_with_zeroes) const noexcept
{
  std::vector<float> data;
  data.reserve(histories_.size() * time_length_ * state_dim());

  for (const auto & history : histories_) {
    const auto history_array = history.as_array();
    data.insert(data.end(), history_array.begin(), history_array.end());
  }

  if (pad_with_zeroes) {
    data.resize(max_num_agent_ * time_length_ * state_dim(), 0.0f);
  }

  return data;
}

std::vector<float> AgentData::as_vector(bool pad_with_zeroes) const noexcept
{
  return build_data_vector(pad_with_zeroes);
}

}  // namespace autoware::diffusion_planner
