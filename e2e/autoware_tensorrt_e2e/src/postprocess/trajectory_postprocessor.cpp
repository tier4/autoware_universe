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

#include "autoware/tensorrt_e2e/postprocess/trajectory_postprocessor.hpp"

#include <autoware/diffusion_planner/dimensions.hpp>
#include <autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace dp = autoware::diffusion_planner;

namespace
{

/// Append one candidate trajectory with its generator info.
void append_candidate(
  autoware_internal_planning_msgs::msg::CandidateTrajectories & candidates,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const std::string & generator_name,
  const unique_identifier_msgs::msg::UUID & generator_uuid)
{
  const auto candidate_trajectory =
    autoware_internal_planning_msgs::build<
      autoware_internal_planning_msgs::msg::CandidateTrajectory>()
      .header(trajectory.header)
      .generator_id(generator_uuid)
      .points(trajectory.points);

  std_msgs::msg::String generator_name_msg;
  generator_name_msg.data = generator_name;

  const auto generator_info =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::GeneratorInfo>()
      .generator_id(generator_uuid)
      .generator_name(generator_name_msg);

  candidates.candidate_trajectories.push_back(candidate_trajectory);
  candidates.generator_info.push_back(generator_info);
}

}  // namespace

TrajectoryPostprocessor::TrajectoryPostprocessor(const PostprocessParams & params)
: params_(params)
{
  // The reused diffusion planner postprocessing hard-codes a 0.1 s step (velocity computation
  // and time_from_start). Reject configurations that silently disagree.
  if (std::abs(params_.time_step - 0.1) > std::numeric_limits<double>::epsilon()) {
    throw std::runtime_error(
      "postprocess.time_step must be 0.1 s (fixed by the reused diffusion planner "
      "postprocessing); got " +
      std::to_string(params_.time_step));
  }
  if (params_.horizon_seconds <= 0.0) {
    throw std::runtime_error("postprocess.horizon_seconds must be positive");
  }
}

void TrajectoryPostprocessor::validate_output_specs(const std::vector<TensorSpec> & output_specs)
{
  const TensorSpec * spec = find_spec(output_specs, params_.prediction_tensor);
  if (!spec) {
    throw std::runtime_error(
      "The model has no output tensor named '" + params_.prediction_tensor +
      "' (set postprocess.prediction_tensor to match the model)");
  }

  const auto & shape = spec->shape;
  if (shape.size() == 4) {
    batch_size_ = shape[0];
    num_agents_ = shape[1];
    num_timesteps_ = shape[2];
  } else if (shape.size() == 3) {
    batch_size_ = shape[0];
    num_agents_ = 1;
    num_timesteps_ = shape[1];
  } else {
    throw std::runtime_error(
      "Model output '" + spec->name + "' has shape " + shape_to_string(shape) +
      "; expected [B, A, T, 4] or [B, T, 4]");
  }
  if (shape.back() != dp::POSE_DIM) {
    throw std::runtime_error(
      "Model output '" + spec->name + "' has shape " + shape_to_string(shape) +
      "; the last dimension must be " + std::to_string(dp::POSE_DIM) +
      " (x, y, cos(yaw), sin(yaw))");
  }

  const auto expected_timesteps =
    static_cast<int64_t>(std::llround(params_.horizon_seconds / params_.time_step));
  if (num_timesteps_ != expected_timesteps) {
    throw std::runtime_error(
      "Model output '" + spec->name + "' has " + std::to_string(num_timesteps_) +
      " timesteps, but postprocess.horizon_seconds=" + std::to_string(params_.horizon_seconds) +
      " with a 0.1 s step expects " + std::to_string(expected_timesteps));
  }
  if (num_timesteps_ <= params_.velocity_smoothing_window) {
    throw std::runtime_error(
      "postprocess.velocity_smoothing_window (" +
      std::to_string(params_.velocity_smoothing_window) +
      ") must be smaller than the number of trajectory points (" +
      std::to_string(num_timesteps_) + ")");
  }

  for (const auto & extra_name : params_.extra_trajectory_tensors) {
    const TensorSpec * extra_spec = find_spec(output_specs, extra_name);
    if (!extra_spec) {
      throw std::runtime_error(
        "The model has no output tensor named '" + extra_name +
        "' (listed in postprocess.extra_trajectory_tensors)");
    }
    const std::vector<int64_t> expected_shape = {batch_size_, num_timesteps_, dp::POSE_DIM};
    if (extra_spec->shape != expected_shape) {
      throw std::runtime_error(
        "Model output '" + extra_name + "' has shape " + shape_to_string(extra_spec->shape) +
        "; extra trajectory tensors must be ego-only with shape " +
        shape_to_string(expected_shape));
    }
  }
}

std::vector<std::vector<std::vector<Eigen::Matrix4d>>> TrajectoryPostprocessor::parse_predictions(
  const std::vector<float> & prediction, const Eigen::Matrix4d & ego_to_map,
  const int64_t num_agents) const
{
  const size_t required_size = batch_size_ * num_agents * num_timesteps_ * dp::POSE_DIM;
  if (prediction.size() < required_size) {
    throw std::runtime_error(
      "Prediction vector size (" + std::to_string(prediction.size()) +
      ") is smaller than required (" + std::to_string(required_size) + ")");
  }

  std::vector<std::vector<std::vector<Eigen::Matrix4d>>> parsed(
    batch_size_, std::vector<std::vector<Eigen::Matrix4d>>(
                   num_agents, std::vector<Eigen::Matrix4d>(
                                 num_timesteps_, Eigen::Matrix4d::Identity())));

  for (int64_t batch_idx = 0; batch_idx < batch_size_; ++batch_idx) {
    for (int64_t agent_idx = 0; agent_idx < num_agents; ++agent_idx) {
      for (int64_t time_idx = 0; time_idx < num_timesteps_; ++time_idx) {
        const int64_t base_idx =
          ((batch_idx * num_agents + agent_idx) * num_timesteps_ + time_idx) * dp::POSE_DIM;

        const auto x = static_cast<double>(prediction[base_idx + 0]);
        const auto y = static_cast<double>(prediction[base_idx + 1]);
        const auto cos_yaw = static_cast<double>(prediction[base_idx + 2]);
        const auto sin_yaw = static_cast<double>(prediction[base_idx + 3]);

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 0) = cos_yaw;
        pose(0, 1) = -sin_yaw;
        pose(1, 0) = sin_yaw;
        pose(1, 1) = cos_yaw;
        pose(0, 3) = x;
        pose(1, 3) = y;

        parsed[batch_idx][agent_idx][time_idx] = ego_to_map * pose;
      }
    }
  }
  return parsed;
}

autoware_planning_msgs::msg::Trajectory TrajectoryPostprocessor::create_trajectory(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const EgoFrame & ego, const rclcpp::Time & stamp, const int64_t batch_idx) const
{
  const bool enable_force_stop =
    ego.odometry.twist.twist.linear.x > std::numeric_limits<double>::epsilon();

  auto trajectory = dp::postprocess::create_ego_trajectory(
    agent_poses, stamp, ego.odometry.pose.pose.position, batch_idx,
    params_.velocity_smoothing_window, enable_force_stop, params_.stopping_threshold);

  if (params_.base_link_offset != 0.0) {
    for (auto & point : trajectory.points) {
      point.pose = dp::utils::shift_x(point.pose, -params_.base_link_offset);
    }
  }
  return trajectory;
}

TrajectoryPostprocessor::Output TrajectoryPostprocessor::process(
  const TensorMap & outputs, const EgoFrame & ego,
  const std::vector<autoware::diffusion_planner::AgentHistory> * neighbor_histories,
  const rclcpp::Time & stamp, const unique_identifier_msgs::msg::UUID & generator_uuid) const
{
  const auto it = outputs.find(params_.prediction_tensor);
  if (it == outputs.end()) {
    throw std::runtime_error("Inference outputs lack '" + params_.prediction_tensor + "'");
  }

  const auto agent_poses = parse_predictions(it->second.host_data, ego.ego_to_map, num_agents_);

  Output output;
  for (int64_t batch_idx = 0; batch_idx < batch_size_; ++batch_idx) {
    const auto trajectory = create_trajectory(agent_poses, ego, stamp, batch_idx);
    if (batch_idx == 0) {
      output.trajectory = trajectory;
    }
    append_candidate(
      output.candidate_trajectories, trajectory,
      params_.generator_name + "_batch_" + std::to_string(batch_idx), generator_uuid);
  }

  // Extra ego-only trajectory outputs (e.g. a prior before refinement) become candidates.
  for (const auto & extra_name : params_.extra_trajectory_tensors) {
    const auto extra_it = outputs.find(extra_name);
    if (extra_it == outputs.end()) {
      throw std::runtime_error("Inference outputs lack '" + extra_name + "'");
    }
    const auto extra_poses =
      parse_predictions(extra_it->second.host_data, ego.ego_to_map, /*num_agents=*/1);
    for (int64_t batch_idx = 0; batch_idx < batch_size_; ++batch_idx) {
      append_candidate(
        output.candidate_trajectories, create_trajectory(extra_poses, ego, stamp, batch_idx),
        params_.generator_name + "_" + extra_name + "_batch_" + std::to_string(batch_idx),
        generator_uuid);
    }
  }

  if (num_agents_ > 1 && neighbor_histories && !neighbor_histories->empty()) {
    // The reused function indexes agent_poses[batch][i + 1] for every history entry, so the
    // histories must not outnumber the model's predicted neighbors.
    const auto max_paths = static_cast<size_t>(num_agents_ - 1);
    const std::vector<autoware::diffusion_planner::AgentHistory> truncated_histories(
      neighbor_histories->begin(),
      neighbor_histories->begin() +
        std::min(neighbor_histories->size(), max_paths));
    constexpr int64_t batch_idx = 0;
    output.predicted_objects = dp::postprocess::create_predicted_objects(
      agent_poses, truncated_histories, stamp, batch_idx);
  }

  return output;
}

}  // namespace autoware::tensorrt_e2e
