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

#include "autoware/tensorrt_e2e/postprocess/latentdrive_postprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_e2e
{

namespace latentdrive
{

double wrap_pi(const double rad)
{
  return std::atan2(std::sin(rad), std::cos(rad));
}

Waypoint sample_at(const Plan & plan, double u)
{
  const int n = static_cast<int>(plan.size());
  const auto at = [&plan, n](const int i) -> Waypoint {
    if (i < 0) {
      return Waypoint{};
    }
    if (i < n) {
      return plan[i];
    }
    if (n < 2) {
      return plan[n - 1];
    }
    const Waypoint & a = plan[n - 2];
    const Waypoint & b = plan[n - 1];
    const double s = static_cast<double>(i - (n - 1));
    return Waypoint{b.x + s * (b.x - a.x), b.y + s * (b.y - a.y), b.yaw};
  };

  u = std::max(u, -1.0);
  const int i = static_cast<int>(std::floor(u));
  const double t = u - static_cast<double>(i);
  const Waypoint a = at(i);
  const Waypoint b = at(i + 1);
  return Waypoint{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.yaw + t * wrap_pi(b.yaw - a.yaw)};
}

Waypoint to_local(const Waypoint & point, const Waypoint & origin)
{
  const double dx = point.x - origin.x;
  const double dy = point.y - origin.y;
  const double c = std::cos(origin.yaw);
  const double s = std::sin(origin.yaw);
  return Waypoint{c * dx + s * dy, -s * dx + c * dy, wrap_pi(point.yaw - origin.yaw)};
}

Plan carry_forward(
  const Plan & previous, const Waypoint & ego_now, const double elapsed_s, const double time_step)
{
  // The previous plan's waypoint i sits at (i + 1) * time_step after its own stamp; the same
  // slot of the new grid is `shift` waypoints further along it.
  const double shift = time_step > 0.0 ? elapsed_s / time_step : 0.0;
  Plan out(previous.size());
  for (size_t i = 0; i < previous.size(); ++i) {
    out[i] = to_local(sample_at(previous, static_cast<double>(i) + shift), ego_now);
  }
  return out;
}

Plan PlanSmoother::update(
  const Plan & plan, const Waypoint & ego_now, const double elapsed_s, const double time_step)
{
  if (elapsed_s <= 0.0 || elapsed_s > params_.max_gap_seconds) {
    previous_.clear();
  }
  // Pass-through: filtering off, no state yet, or a plan whose shape the state cannot describe.
  if (params_.alpha >= 1.0 || plan.empty() || previous_.size() != plan.size()) {
    previous_ = plan;
    return plan;
  }

  const Plan carried = carry_forward(previous_, ego_now, elapsed_s, time_step);

  // A large disagreement is the plan changing its mind, not jitter; smoothing through it would
  // drag a stale trajectory along for several ticks.
  const Waypoint & a = carried.back();
  const Waypoint & b = plan.back();
  if (std::hypot(b.x - a.x, b.y - a.y) > params_.reset_jump_m) {
    previous_ = plan;
    return plan;
  }

  const double w = params_.alpha;
  Plan out(plan.size());
  for (size_t i = 0; i < plan.size(); ++i) {
    out[i].x = carried[i].x + w * (plan[i].x - carried[i].x);
    out[i].y = carried[i].y + w * (plan[i].y - carried[i].y);
    out[i].yaw = carried[i].yaw + w * wrap_pi(plan[i].yaw - carried[i].yaw);
  }
  previous_ = out;
  return out;
}

}  // namespace latentdrive

namespace
{
constexpr int64_t YAW_POSE_DIM = 3;
}  // namespace

LatentDrivePostprocessor::LatentDrivePostprocessor(
  rclcpp::Node & node, const PostprocessParams & params)
: TrajectoryPostprocessor(params), node_(node), smoother_(latentdrive::SmoothingParams{})
{
  smoothing_.enable = node_.declare_parameter<bool>("latentdrive.smoothing.enable", true);
  smoothing_.alpha = node_.declare_parameter<double>("latentdrive.smoothing.alpha", 0.35);
  smoothing_.reset_jump_m =
    node_.declare_parameter<double>("latentdrive.smoothing.reset_jump_m", 8.0);
  smoothing_.max_gap_seconds =
    node_.declare_parameter<double>("latentdrive.smoothing.max_gap_seconds", 1.0);
  smoothing_.publish_raw_candidate =
    node_.declare_parameter<bool>("latentdrive.smoothing.publish_raw_candidate", true);
  if (smoothing_.alpha <= 0.0 || smoothing_.alpha > 1.0) {
    throw std::runtime_error("latentdrive.smoothing.alpha must be in (0, 1]");
  }
  if (smoothing_.reset_jump_m <= 0.0 || smoothing_.max_gap_seconds <= 0.0) {
    throw std::runtime_error(
      "latentdrive.smoothing.reset_jump_m and max_gap_seconds must be positive");
  }
  smoother_ = latentdrive::PlanSmoother(smoothing_);
  pub_plan_ = node_.create_publisher<nav_msgs::msg::Path>("~/debug/latentdrive/plan", 1);
}

void LatentDrivePostprocessor::publish_plan(
  const latentdrive::Plan & plan, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = "base_link";
  path.poses.reserve(plan.size());
  for (const auto & wp : plan) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.orientation.z = std::sin(wp.yaw / 2.0);
    pose.pose.orientation.w = std::cos(wp.yaw / 2.0);
    path.poses.push_back(pose);
  }
  pub_plan_->publish(path);
}

void LatentDrivePostprocessor::validate_output_specs(const std::vector<TensorSpec> & output_specs)
{
  TrajectoryPostprocessor::validate_output_specs(output_specs);
  if (smoothing_.enable && (pose_dim() != YAW_POSE_DIM || num_agents() != 1)) {
    throw std::runtime_error(
      "latentdrive.smoothing works on an ego-only (x, y, yaw) plan; '" +
      params().prediction_tensor + "' has " + std::to_string(num_agents()) +
      " agents and a pose dimension of " + std::to_string(pose_dim()));
  }
}

latentdrive::Plan LatentDrivePostprocessor::read_plan(const Tensor & tensor) const
{
  const auto steps = static_cast<size_t>(num_timesteps());
  if (tensor.host_data.size() < steps * YAW_POSE_DIM) {
    throw std::runtime_error(
      "Prediction tensor holds " + std::to_string(tensor.host_data.size()) +
      " values, fewer than the " + std::to_string(steps * YAW_POSE_DIM) + " expected");
  }
  latentdrive::Plan plan(steps);
  for (size_t i = 0; i < steps; ++i) {
    plan[i] = {
      static_cast<double>(tensor.host_data[i * YAW_POSE_DIM + 0]),
      static_cast<double>(tensor.host_data[i * YAW_POSE_DIM + 1]),
      static_cast<double>(tensor.host_data[i * YAW_POSE_DIM + 2])};
  }
  return plan;
}

void LatentDrivePostprocessor::write_plan(const latentdrive::Plan & plan, Tensor & tensor) const
{
  for (size_t i = 0; i < plan.size(); ++i) {
    tensor.host_data[i * YAW_POSE_DIM + 0] = static_cast<float>(plan[i].x);
    tensor.host_data[i * YAW_POSE_DIM + 1] = static_cast<float>(plan[i].y);
    tensor.host_data[i * YAW_POSE_DIM + 2] = static_cast<float>(plan[i].yaw);
  }
}

TrajectoryPostprocessor::Output LatentDrivePostprocessor::process(
  const TensorMap & outputs, const EgoFrame & ego,
  const std::vector<autoware::diffusion_planner::AgentHistory> * neighbor_histories,
  const rclcpp::Time & stamp, const unique_identifier_msgs::msg::UUID & generator_uuid)
{
  const auto it = outputs.find(params().prediction_tensor);
  if (it == outputs.end()) {
    throw std::runtime_error("Inference outputs lack '" + params().prediction_tensor + "'");
  }
  // The debug path and the smoother both need an ego-only (x, y, yaw) plan; the base class
  // handles anything else on its own.
  const bool ego_yaw_plan = pose_dim() == YAW_POSE_DIM && num_agents() == 1;
  if (!smoothing_.enable || !ego_yaw_plan) {
    if (ego_yaw_plan) {
      publish_plan(read_plan(it->second), stamp);
    }
    return TrajectoryPostprocessor::process(
      outputs, ego, neighbor_histories, stamp, generator_uuid);
  }
  const latentdrive::Plan plan = read_plan(it->second);

  // The current ego pose in the previous plan's ego frame, from odometry rather than from the
  // plan, so the carried-forward plan is anchored where the vehicle actually went.
  latentdrive::Waypoint ego_now;
  double elapsed_s = 0.0;
  if (previous_map_to_ego_ && previous_stamp_) {
    const Eigen::Matrix4d motion = *previous_map_to_ego_ * ego.ego_to_map;
    ego_now = {motion(0, 3), motion(1, 3), std::atan2(motion(1, 0), motion(0, 0))};
    elapsed_s = (ego.stamp - *previous_stamp_).seconds();
  }
  previous_map_to_ego_ = ego.map_to_ego;
  previous_stamp_ = ego.stamp;

  const latentdrive::Plan smoothed = smoother_.update(plan, ego_now, elapsed_s, params().time_step);
  publish_plan(smoothed, stamp);

  TensorMap smoothed_outputs = outputs;
  write_plan(smoothed, smoothed_outputs[params().prediction_tensor]);
  Output output = TrajectoryPostprocessor::process(
    smoothed_outputs, ego, neighbor_histories, stamp, generator_uuid);

  if (!smoothing_.publish_raw_candidate) {
    return output;
  }

  // The unfiltered plan rides along as a candidate, so what the model said stays observable.
  Output raw =
    TrajectoryPostprocessor::process(outputs, ego, neighbor_histories, stamp, generator_uuid);
  for (auto & info : raw.candidate_trajectories.generator_info) {
    info.generator_name.data += "_raw";
  }
  output.candidate_trajectories.candidate_trajectories.insert(
    output.candidate_trajectories.candidate_trajectories.end(),
    raw.candidate_trajectories.candidate_trajectories.begin(),
    raw.candidate_trajectories.candidate_trajectories.end());
  output.candidate_trajectories.generator_info.insert(
    output.candidate_trajectories.generator_info.end(),
    raw.candidate_trajectories.generator_info.begin(),
    raw.candidate_trajectories.generator_info.end());
  return output;
}

}  // namespace autoware::tensorrt_e2e
