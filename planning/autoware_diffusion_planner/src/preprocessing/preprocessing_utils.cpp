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

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <Eigen/Geometry>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
namespace
{
constexpr float kMaxYawRate = 0.95f;
constexpr float kMaxSteerAngle = static_cast<float>((2.0 / 3.0) * M_PI);

rclcpp::Time target_time_for_timestep(
  const rclcpp::Time & frame_time, size_t timestep_idx, size_t num_timesteps)
{
  const double offset_sec =
    static_cast<double>(num_timesteps - 1 - timestep_idx) / PLANNING_FREQUENCY;
  return frame_time - rclcpp::Duration::from_seconds(offset_sec);
}

double calc_time_ratio(
  const rclcpp::Time & start_time, const rclcpp::Time & end_time, const rclcpp::Time & target_time)
{
  const auto duration_ns = static_cast<double>((end_time - start_time).nanoseconds());
  if (duration_ns == 0.0) {
    return 0.0;
  }
  const auto offset_ns = static_cast<double>((target_time - start_time).nanoseconds());
  return std::clamp(offset_ns / duration_ns, 0.0, 1.0);
}

std::array<float, AGENT_STATE_DIM> interpolate_agent_state_array(
  const AgentState & start_state, const AgentState & end_state, const double ratio)
{
  const double clamped_ratio = std::clamp(ratio, 0.0, 1.0);

  const auto start_pose = utils::matrix4d_to_pose(start_state.pose);
  const auto end_pose = utils::matrix4d_to_pose(end_state.pose);
  const auto interp_pose =
    autoware::universe_utils::calcInterpolatedPose(start_pose, end_pose, clamped_ratio, false);
  const Eigen::Quaterniond interp_q(
    interp_pose.orientation.w, interp_pose.orientation.x, interp_pose.orientation.y,
    interp_pose.orientation.z);
  const auto [cos_yaw, sin_yaw] = utils::rotation_matrix_to_cos_sin(interp_q.toRotationMatrix());

  const auto & meta_state = (clamped_ratio <= 0.5) ? start_state : end_state;
  return {
    static_cast<float>(interp_pose.position.x),
    static_cast<float>(interp_pose.position.y),
    cos_yaw,
    sin_yaw,
    0.0f,
    0.0f,
    static_cast<float>(meta_state.original_info.shape.dimensions.y),
    static_cast<float>(meta_state.original_info.shape.dimensions.x),
    static_cast<float>(meta_state.label == AgentLabel::VEHICLE),
    static_cast<float>(meta_state.label == AgentLabel::PEDESTRIAN),
    static_cast<float>(meta_state.label == AgentLabel::BICYCLE),
  };
}

}  // namespace

void normalize_input_data(InputDataMap & input_data_map, const NormalizationMap & normalization_map)
{
  auto normalize_vector = [](
                            std::vector<float> & data, const std::vector<float> & mean,
                            const std::vector<float> & std_dev) -> void {
    assert(!data.empty() && "Data vector must not be empty");
    assert((mean.size() == std_dev.size()) && "Mean and std must be same size");
    assert((data.size() % std_dev.size() == 0) && "Data size must be divisible by std_dev size");
    auto cols = std_dev.size();
    auto rows = data.size() / cols;

    for (size_t row = 0; row < rows; ++row) {
      const auto offset = row * cols;
      const auto row_begin =
        data.begin() + static_cast<std::vector<float>::difference_type>(offset);

      bool is_zero_row = std::all_of(
        row_begin, row_begin + static_cast<std::vector<float>::difference_type>(cols),
        [](float x) { return std::abs(x) < std::numeric_limits<float>::epsilon(); });

      if (is_zero_row) continue;

      for (size_t col = 0; col < cols; ++col) {
        float m = (mean.size() == 1) ? mean[0] : mean[col];
        float s = (std_dev.size() == 1) ? std_dev[0] : std_dev[col];
        // Prevent division by zero
        if (std::abs(s) < std::numeric_limits<float>::epsilon()) {
          throw std::runtime_error("Standard deviation is zero, cannot normalize data");
        }
        data[offset + col] = (data[offset + col] - m) / s;
      }
    }
  };

  for (auto & [key, value] : input_data_map) {
    // Skip normalization for ego_shape and sampled_trajectories
    if (key == "ego_shape" || key == "sampled_trajectories" || key == "turn_indicators") {
      continue;
    }

    if (normalization_map.find(key) == normalization_map.end()) {
      std::string err{"Missing key " + key + " from normalization map"};
      throw std::runtime_error(err.c_str());
    }

    const auto & [mean, std_dev] = normalization_map.at(key);
    normalize_vector(value, mean, std_dev);
  }
}

std::vector<float> create_ego_current_state(
  const nav_msgs::msg::Odometry & kinematic_state_msg,
  const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration_msg, float wheel_base)
{
  const auto & lin = kinematic_state_msg.twist.twist.linear;
  const auto & ang = kinematic_state_msg.twist.twist.angular;

  float yaw_rate = 0.0f;
  float steering_angle = 0.0f;
  const float linear_vel = std::hypot(lin.x, lin.y);
  if (linear_vel < constants::MOVING_VELOCITY_THRESHOLD_MPS) {
    yaw_rate = 0.0f;
    steering_angle = 0.0f;
  } else {
    yaw_rate = std::clamp(static_cast<float>(ang.z), -kMaxYawRate, kMaxYawRate);
    const float raw_steer = std::atan(yaw_rate * wheel_base / std::abs(linear_vel));
    steering_angle = std::clamp(raw_steer, -kMaxSteerAngle, kMaxSteerAngle);
  }

  const float vx = static_cast<float>(lin.x);
  const float vy = static_cast<float>(lin.y);
  const float ax = static_cast<float>(acceleration_msg.accel.accel.linear.x);
  const float ay = static_cast<float>(acceleration_msg.accel.accel.linear.y);

  return {0.0f, 0.0f, 1.0f, 0.0f, vx, vy, ax, ay, steering_angle, yaw_rate};
}

std::vector<float> create_ego_agent_past(
  const std::deque<nav_msgs::msg::Odometry> & odom_msgs, size_t num_timesteps,
  const Eigen::Matrix4d & map_to_ego_transform, const rclcpp::Time & frame_time)
{
  const size_t features_per_timestep = 4;  // x, y, cos, sin
  const size_t total_size = num_timesteps * features_per_timestep;

  std::vector<float> ego_agent_past(total_size, 0.0f);

  size_t odom_index = 0;
  for (size_t timestep_idx = 0; timestep_idx < num_timesteps; ++timestep_idx) {
    if (odom_msgs.empty()) {
      break;
    }

    const auto target_time = target_time_for_timestep(frame_time, timestep_idx, num_timesteps);

    // Advance odom_index to the correct position
    while (odom_index + 1 < odom_msgs.size()) {
      const rclcpp::Time curr_time(odom_msgs[odom_index].header.stamp);
      const rclcpp::Time next_time(odom_msgs[odom_index + 1].header.stamp);
      if (next_time <= target_time && curr_time <= next_time) {
        ++odom_index;
      } else {
        break;
      }
    }

    const size_t start_index = odom_index;
    const size_t end_index = (start_index + 1 < odom_msgs.size()) ? start_index + 1 : start_index;
    const rclcpp::Time start_time(odom_msgs[start_index].header.stamp);
    const rclcpp::Time end_time(odom_msgs[end_index].header.stamp);
    const double ratio = calc_time_ratio(start_time, end_time, target_time);
    const auto interpolated_pose = autoware::universe_utils::calcInterpolatedPose(
      odom_msgs[start_index].pose.pose, odom_msgs[end_index].pose.pose, ratio, false);

    // Convert pose to 4x4 matrix
    const Eigen::Matrix4d pose_map_4x4 = utils::pose_to_matrix4f(interpolated_pose);

    // Transform to ego frame
    const Eigen::Matrix4d pose_ego_4x4 = map_to_ego_transform * pose_map_4x4;

    // Extract position
    const float x = pose_ego_4x4(0, 3);
    const float y = pose_ego_4x4(1, 3);

    // Extract heading as cos/sin
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(pose_ego_4x4.block<3, 3>(0, 0));

    // Store in flat array: [timestep, features]
    const size_t base_idx = timestep_idx * features_per_timestep;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_X] = x;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_Y] = y;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_COS] = cos_yaw;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_SIN] = sin_yaw;
  }

  return ego_agent_past;
}

std::vector<float> create_neighbor_agents_past(
  const std::vector<AgentHistory> & histories, size_t max_num_agent, size_t num_timesteps,
  const rclcpp::Time & frame_time)
{
  const size_t total_size = max_num_agent * num_timesteps * AGENT_STATE_DIM;
  std::vector<float> data(total_size, 0.0f);

  const size_t agent_count = std::min(histories.size(), max_num_agent);
  for (size_t agent_idx = 0; agent_idx < agent_count; ++agent_idx) {
    const auto & history = histories[agent_idx];
    if (history.size() == 0) {
      continue;
    }
    const size_t base_idx = agent_idx * num_timesteps * AGENT_STATE_DIM;

    size_t history_index = 0;
    for (size_t timestep_idx = 0; timestep_idx < num_timesteps; ++timestep_idx) {
      const auto target_time = target_time_for_timestep(frame_time, timestep_idx, num_timesteps);

      // Advance history_index to the correct position
      while (history_index + 1 < history.size()) {
        const auto & curr_time = history.at(history_index).timestamp;
        const auto & next_time = history.at(history_index + 1).timestamp;
        if (next_time <= target_time && curr_time <= next_time) {
          ++history_index;
        } else {
          break;
        }
      }

      const size_t start_index = history_index;
      const size_t end_index = (start_index + 1 < history.size()) ? start_index + 1 : start_index;
      const auto & start_state = history.at(start_index);
      const auto & end_state = history.at(end_index);
      const double ratio = calc_time_ratio(start_state.timestamp, end_state.timestamp, target_time);
      const auto state_array = interpolate_agent_state_array(start_state, end_state, ratio);
      const size_t timestep_base_idx = base_idx + timestep_idx * AGENT_STATE_DIM;
      for (size_t dim_idx = 0; dim_idx < state_array.size(); ++dim_idx) {
        data[timestep_base_idx + dim_idx] = state_array[dim_idx];
      }
    }
  }

  return data;
}

std::vector<float> create_turn_indicators_past(
  const std::deque<TurnIndicatorsReport> & history, size_t num_timesteps,
  const rclcpp::Time & frame_time)
{
  std::vector<float> turn_indicators(num_timesteps, 0.0f);
  if (history.empty()) {
    return turn_indicators;
  }

  size_t history_index = 0;
  for (size_t timestep_idx = 0; timestep_idx < num_timesteps; ++timestep_idx) {
    const auto target_time = target_time_for_timestep(frame_time, timestep_idx, num_timesteps);

    while (history_index + 1 < history.size()) {
      const rclcpp::Time next_time(history[history_index + 1].stamp);
      if (next_time <= target_time) {
        ++history_index;
      } else {
        break;
      }
    }

    size_t selected_index = history_index;
    if (history_index + 1 < history.size()) {
      const rclcpp::Time curr_time(history[history_index].stamp);
      const rclcpp::Time next_time(history[history_index + 1].stamp);
      const auto curr_diff_ns = std::llabs((target_time - curr_time).nanoseconds());
      const auto next_diff_ns = std::llabs((target_time - next_time).nanoseconds());
      if (next_diff_ns < curr_diff_ns) {
        selected_index = history_index + 1;
      }
    }
    turn_indicators[timestep_idx] = static_cast<float>(history[selected_index].report);
  }

  return turn_indicators;
}

std::vector<float> create_sampled_trajectories(const double temperature)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist(0.0f, 1.0f);
  std::vector<float> sampled_trajectories((MAX_NUM_NEIGHBORS + 1) * (OUTPUT_T + 1) * POSE_DIM);
  for (float & val : sampled_trajectories) {
    val = dist(gen) * static_cast<float>(temperature);
  }
  return sampled_trajectories;
}

}  // namespace autoware::diffusion_planner::preprocess
