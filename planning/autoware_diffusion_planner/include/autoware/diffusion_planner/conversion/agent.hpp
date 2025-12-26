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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HPP_

#include "Eigen/Dense"
#include "autoware/diffusion_planner/utils/fixed_queue.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/detail/tracked_objects__struct.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
namespace autoware::diffusion_planner
{
using autoware_perception_msgs::msg::TrackedObject;

using autoware_perception_msgs::msg::ObjectClassification;
constexpr size_t AGENT_STATE_DIM = 11;

enum AgentLabel { VEHICLE = 0, PEDESTRIAN = 1, BICYCLE = 2 };

AgentLabel get_model_label(const autoware_perception_msgs::msg::TrackedObject & object);

/**
 * @brief A class to represent a single state of an agent.
 */
struct AgentState
{
  // Construct a new instance filling all elements by `0.0f`.
  AgentState() = default;

  explicit AgentState(const TrackedObject & object);

  /**
   * @brief Construct a new instance with specified values.
   *
   * @param position 3D position [m].
   * @param dimension Box dimension [m].
   * @param yaw Heading yaw angle [rad].
   * @param velocity Velocity [m/s].
   * @param label Agent label
   */

  // Construct a new instance filling all elements by `0.0f`.
  static AgentState empty() noexcept { return {}; }

  // Return the agent state dimensions `D`.
  static size_t dim() { return AGENT_STATE_DIM; }

  // Return TrackedObject info
  [[nodiscard]] TrackedObject tracked_object() const { return tracked_object_info_; }

  void apply_transform(const Eigen::Matrix4d & transform);

  // Return the state attribute as an array.
  [[nodiscard]] std::array<float, AGENT_STATE_DIM> as_array() const noexcept;

  geometry_msgs::msg::Point position_;
  double yaw_{0.0};
  double cos_yaw_{0.0};
  double sin_yaw_{0.0};
  geometry_msgs::msg::Vector3 velocity_;
  AgentLabel label_{AgentLabel::VEHICLE};
  uint8_t autoware_label_;
  std::string object_id_;
  TrackedObject tracked_object_info_;
};

/**
 * @brief A class to represent the state history of an agent.
 */
struct AgentHistory
{
  /**
   * @brief Construct a new Agent History filling the latest state by input state.
   *
   * @param state Object current state.
   * @param current_time Current timestamp.
   * @param max_time_length History length.
   */
  AgentHistory(
    const AgentState & state, const double current_time,
    const size_t max_time_length, bool is_pad_history = true);

  // Return the history time length `T`.
  [[nodiscard]] size_t length() const { return max_size_; }

  // Return the data size of history `T * D`.
  [[nodiscard]] size_t size() const { return max_size_ * AGENT_STATE_DIM; }

  /**
   * @brief Update history with input state and latest time.
   *
   * @param current_time The current timestamp.
   * @param object The object info.
   */
  void update(double current_time, const TrackedObject & object);

  // Return a history states as an array.
  [[nodiscard]] std::vector<float> as_array() const noexcept;

  // Get the latest agent state at `T`.
  [[nodiscard]] const AgentState & get_latest_state() const { return queue_.back(); }

  [[nodiscard]] const geometry_msgs::msg::Point & get_latest_state_position() const
  {
    return get_latest_state().position_;
  }

  void apply_transform(const Eigen::Matrix4d & transform)
  {
    for (auto & state : queue_) {
      state.apply_transform(transform);
    }
  }

  // private:
  FixedQueue<AgentState> queue_;
  double latest_time_;
  size_t max_size_;
};

/**
 * @brief A class containing whole state histories of all agent.
 */
struct AgentData
{
  void update_histories(
    const autoware_perception_msgs::msg::TrackedObjects & objects,
    const bool ignore_unknown_agents = false);

  static bool is_unknown_object(const autoware_perception_msgs::msg::TrackedObject & object);

  // Transform histories, trim to max_num_agent, and return the processed vector.
  std::vector<AgentHistory> transformed_and_trimmed_histories(
    const Eigen::Matrix4d & transform, size_t max_num_agent) const;

private:
  std::unordered_map<std::string, AgentHistory> histories_map_;
};

// Convert histories to a flattened vector
inline std::vector<float> flatten_histories_to_vector(
  const std::vector<AgentHistory> & histories, size_t max_num_agent, size_t time_length)
{
  std::vector<float> data;
  data.reserve(histories.size() * time_length * AGENT_STATE_DIM);

  for (const auto & history : histories) {
    const auto history_array = history.as_array();
    data.insert(data.end(), history_array.begin(), history_array.end());
  }

  data.resize(max_num_agent * time_length * AGENT_STATE_DIM, 0.0f);

  return data;
}

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__AGENT_HPP_
