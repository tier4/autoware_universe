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

Plan resample_plan(const Plan & plan, const double plan_dt, const double dt, const size_t steps)
{
  if (plan.empty() || plan_dt <= 0.0 || dt <= 0.0) {
    throw std::runtime_error("resample_plan needs a non-empty plan and positive time steps");
  }
  const double end_t = static_cast<double>(plan.size()) * plan_dt;
  Plan out(steps);
  for (size_t i = 0; i < steps; ++i) {
    const double t = std::min(static_cast<double>(i + 1) * dt, end_t);
    // Segment j runs from t = j * plan_dt to (j + 1) * plan_dt; segment 0 starts at the origin.
    const auto j = static_cast<size_t>(std::floor(t / plan_dt));
    if (j >= plan.size()) {
      out[i] = plan.back();
      continue;
    }
    const Waypoint & from = j == 0 ? Waypoint{} : plan[j - 1];
    const Waypoint & to = plan[j];
    const double u = t / plan_dt - static_cast<double>(j);
    out[i] = {
      from.x + u * (to.x - from.x), from.y + u * (to.y - from.y),
      from.yaw + u * (to.yaw - from.yaw)};
  }
  return out;
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
  // What the trajectory needs, and what this model actually emits.
  const auto trajectory_steps =
    static_cast<int64_t>(std::llround(params().horizon_seconds / params().time_step));
  std::vector<TensorSpec> specs = output_specs;
  for (auto & spec : specs) {
    if (spec.name != params().prediction_tensor || spec.shape.size() != 3) {
      continue;
    }
    model_steps_ = spec.shape[1];
    model_time_step_ = params().horizon_seconds / static_cast<double>(model_steps_);
    if (model_steps_ == trajectory_steps) {
      break;
    }
    if (spec.shape.back() != YAW_POSE_DIM) {
      throw std::runtime_error(
        "Model output '" + spec.name + "' plans " + std::to_string(model_steps_) +
        " waypoints, which have to be interpolated onto the " + std::to_string(trajectory_steps) +
        " of the trajectory, and that is only defined for an (x, y, yaw) plan");
    }
    // The base class checks and later reads the resampled plan, so it is told that shape.
    spec.shape[1] = trajectory_steps;
    RCLCPP_INFO(
      node_.get_logger(),
      "Model plans %ld waypoints %.2f s apart; interpolating them onto the trajectory's %ld "
      "waypoints %.2f s apart",
      model_steps_, model_time_step_, trajectory_steps, params().time_step);
    break;
  }

  TrajectoryPostprocessor::validate_output_specs(specs);
  if (smoothing_.enable && (pose_dim() != YAW_POSE_DIM || num_agents() != 1)) {
    throw std::runtime_error(
      "latentdrive.smoothing works on an ego-only (x, y, yaw) plan; '" +
      params().prediction_tensor + "' has " + std::to_string(num_agents()) +
      " agents and a pose dimension of " + std::to_string(pose_dim()));
  }
}

latentdrive::Plan LatentDrivePostprocessor::read_plan(const Tensor & tensor) const
{
  const auto steps = static_cast<size_t>(model_steps_ > 0 ? model_steps_ : num_timesteps());
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
  const bool resampling = ego_yaw_plan && model_steps_ != num_timesteps();

  if (!smoothing_.enable && !resampling) {
    if (ego_yaw_plan) {
      publish_plan(read_plan(it->second), stamp);
    }
    return TrajectoryPostprocessor::process(
      outputs, ego, neighbor_histories, stamp, generator_uuid);
  }
  if (!ego_yaw_plan) {
    return TrajectoryPostprocessor::process(
      outputs, ego, neighbor_histories, stamp, generator_uuid);
  }

  latentdrive::Plan plan = read_plan(it->second);
  if (resampling) {
    plan = latentdrive::resample_plan(
      plan, model_time_step_, params().time_step, static_cast<size_t>(num_timesteps()));
  }
  // Everything below works on the trajectory's own grid, so the tensor handed on carries that
  // many waypoints whether the model planned them or the interpolation did.
  const auto plan_to_outputs = [this, &outputs](const latentdrive::Plan & p) {
    TensorMap out = outputs;
    Tensor & tensor = out[params().prediction_tensor];
    tensor.shape = {1, num_timesteps(), YAW_POSE_DIM};
    tensor.host_data.assign(static_cast<size_t>(num_timesteps() * YAW_POSE_DIM), 0.0F);
    tensor.device_data = nullptr;
    write_plan(p, tensor);
    return out;
  };

  if (!smoothing_.enable) {
    publish_plan(plan, stamp);
    return TrajectoryPostprocessor::process(
      plan_to_outputs(plan), ego, neighbor_histories, stamp, generator_uuid);
  }

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

  Output output = TrajectoryPostprocessor::process(
    plan_to_outputs(smoothed), ego, neighbor_histories, stamp, generator_uuid);

  if (!smoothing_.publish_raw_candidate) {
    return output;
  }

  // The unfiltered plan rides along as a candidate, so what the model said stays observable.
  Output raw = TrajectoryPostprocessor::process(
    plan_to_outputs(plan), ego, neighbor_histories, stamp, generator_uuid);
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
