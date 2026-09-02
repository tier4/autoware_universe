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

#include "autoware/ml_planner/postprocessing/postprocessing_utils.hpp"

#include "autoware/ml_planner/dimensions.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ml_planner::postprocess
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_planning_msgs::msg::TrajectoryPoint;

// internal functions
namespace
{
/**
 * @brief Converts a vector of poses to a Trajectory message.
 *
 * Only pose information is filled: the model predicts positions and headings only, so
 * velocity, acceleration, and steering are intentionally left at zero instead of being
 * derived by finite differences (the trajectory optimization computes them consistently).
 *
 * @param poses The vector of 4x4 transformation matrices representing poses.
 * @param stamp The ROS time stamp for the message.
 * @return A Trajectory message in map coordinates.
 */
Trajectory get_trajectory_from_poses(
  const std::vector<Eigen::Matrix4d> & poses, const rclcpp::Time & stamp);
};  // namespace

std::vector<float> denormalize_prediction(const std::vector<float> & prediction)
{
  const size_t trajectory_size = static_cast<size_t>(MAX_NUM_AGENTS) * OUTPUT_T * POSE_DIM;
  if (prediction.empty() || prediction.size() % trajectory_size != 0) {
    throw std::runtime_error("Unsupported prediction shape for trajectory denormalization.");
  }

  std::vector<float> denormalized = prediction;
  for (size_t index = 0; index < denormalized.size(); index += POSE_DIM) {
    denormalized[index] *= POSITION_SCALE;
    denormalized[index + 1] *= POSITION_SCALE;
    const float norm = std::hypot(denormalized[index + 2], denormalized[index + 3]);
    const float safe_norm = std::max(norm, 1.0e-6F);
    denormalized[index + 2] /= safe_norm;
    denormalized[index + 3] /= safe_norm;
  }

  return denormalized;
}

std::vector<std::vector<std::vector<Eigen::Matrix4d>>> parse_predictions(
  const std::vector<float> & prediction, const Eigen::Matrix4d & transform_ego_to_map)
{
  const int64_t batch_size = prediction.size() / (MAX_NUM_AGENTS * OUTPUT_T * POSE_DIM);

  // Ensure prediction has enough data
  const size_t required_size = batch_size * MAX_NUM_AGENTS * OUTPUT_T * POSE_DIM;
  if (prediction.size() < required_size) {
    throw std::runtime_error(
      "Prediction vector size (" + std::to_string(prediction.size()) +
      ") is smaller than required (" + std::to_string(required_size) + ")");
  }

  // Structure: batch -> agent -> timestep -> pose
  std::vector<std::vector<std::vector<Eigen::Matrix4d>>> parsed_predictions(
    batch_size,
    std::vector<std::vector<Eigen::Matrix4d>>(
      MAX_NUM_AGENTS, std::vector<Eigen::Matrix4d>(OUTPUT_T, Eigen::Matrix4d::Identity())));

  for (int64_t batch_idx = 0; batch_idx < batch_size; ++batch_idx) {
    for (int64_t agent_idx = 0; agent_idx < MAX_NUM_AGENTS; ++agent_idx) {
      for (int64_t time_idx = 0; time_idx < OUTPUT_T; ++time_idx) {
        const int64_t pred_base_idx =
          (batch_idx * MAX_NUM_AGENTS * OUTPUT_T + agent_idx * OUTPUT_T + time_idx) * POSE_DIM;

        const double x = static_cast<double>(prediction[pred_base_idx + 0]);
        const double y = static_cast<double>(prediction[pred_base_idx + 1]);
        const double cos_yaw = static_cast<double>(prediction[pred_base_idx + 2]);
        const double sin_yaw = static_cast<double>(prediction[pred_base_idx + 3]);

        // Create 4x4 transformation matrix from x, y, cos(yaw), sin(yaw)
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 0) = cos_yaw;
        pose(0, 1) = -sin_yaw;
        pose(1, 0) = sin_yaw;
        pose(1, 1) = cos_yaw;
        pose(0, 3) = x;
        pose(1, 3) = y;

        parsed_predictions[batch_idx][agent_idx][time_idx] = transform_ego_to_map * pose;
      }
    }
  }

  return parsed_predictions;
}

PredictedObjects create_predicted_objects(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const std::vector<SelectedAgent> & selected_agents, const rclcpp::Time & stamp,
  const int64_t batch_index)
{
  auto trajectory_path_to_pose_path = [](const Trajectory & trajectory, const double object_z)
    -> std::vector<geometry_msgs::msg::Pose> {
    std::vector<geometry_msgs::msg::Pose> pose_path;
    std::for_each(trajectory.points.begin(), trajectory.points.end(), [&](const auto & p) {
      auto object_pose = p.pose;
      object_pose.position.z = object_z;  // Set the z coordinate to the object's z
      pose_path.push_back(object_pose);
    });

    return pose_path;
  };

  PredictedObjects predicted_objects;
  predicted_objects.header.stamp = stamp;
  predicted_objects.header.frame_id = "map";

  constexpr double time_step{0.1};

  // selected_agents contains current neighbor information ordered by distance.
  for (int64_t neighbor_id = 0; neighbor_id < MAX_NUM_NEIGHBORS; ++neighbor_id) {
    if (static_cast<size_t>(neighbor_id) >= selected_agents.size()) {
      break;
    }

    // Extract poses for this neighbor (neighbor_id + 1 because 0 is ego)
    const auto & neighbor_poses = agent_poses[batch_index][neighbor_id + 1];

    const Trajectory trajectory_points_in_map_reference =
      get_trajectory_from_poses(neighbor_poses, stamp);

    PredictedObject object;
    const TrackedObject & object_info = selected_agents.at(neighbor_id).current_object;
    {  // Extract path from prediction
      PredictedPath predicted_path;
      const double object_pose_z = object_info.kinematics.pose_with_covariance.pose.position.z;

      predicted_path.path =
        trajectory_path_to_pose_path(trajectory_points_in_map_reference, object_pose_z);
      predicted_path.time_step = rclcpp::Duration::from_seconds(time_step);
      predicted_path.confidence = 1.0;
      object.kinematics.predicted_paths.push_back(predicted_path);
    }
    {  // Copy kinematics
      object.kinematics.initial_twist_with_covariance =
        object_info.kinematics.twist_with_covariance;
      object.kinematics.initial_acceleration_with_covariance =
        object_info.kinematics.acceleration_with_covariance;
      object.kinematics.initial_pose_with_covariance = object_info.kinematics.pose_with_covariance;
    }
    {  // Copy the remaining info
      object.object_id = object_info.object_id;
      object.classification = object_info.classification;
      object.shape = object_info.shape;
      object.existence_probability = object_info.existence_probability;
    }
    predicted_objects.objects.push_back(object);
  }
  return predicted_objects;
}

Trajectory create_ego_trajectory(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const rclcpp::Time & stamp, const int64_t batch_index)
{
  const int64_t ego_index = 0;

  // Validate batch index
  if (batch_index < 0 || batch_index >= static_cast<int64_t>(agent_poses.size())) {
    throw std::out_of_range(
      "Invalid batch_index: " + std::to_string(batch_index) +
      ", batch_size=" + std::to_string(agent_poses.size()));
  }

  // Extract ego poses (ego_index = 0)
  const auto & ego_poses = agent_poses[batch_index][ego_index];

  return get_trajectory_from_poses(ego_poses, stamp);
}

int64_t count_valid_elements(
  const xt::xarray<float> & data, int64_t len, int64_t dim2, int64_t dim3, int64_t batch_idx)
{
  const int64_t single_batch_size = len * dim2 * dim3;
  const int64_t batch_offset = batch_idx * single_batch_size;

  if (batch_offset + single_batch_size > static_cast<int64_t>(data.size()) || batch_idx < 0) {
    return 0;  // Invalid batch index or data size
  }

  int64_t valid_count = 0;
  const float epsilon = std::numeric_limits<float>::epsilon();

  // Iterate through each element in the len dimension for the specified batch
  for (int64_t i = 0; i < len; ++i) {
    bool is_valid_element = false;

    // Check all values in the (dim2, dim3) block for this element
    const int64_t element_offset = batch_offset + i * dim2 * dim3;
    for (int64_t j = 0; j < dim2 * dim3; ++j) {
      const int64_t idx = element_offset + j;
      if (std::abs(data.data()[idx]) > epsilon) {
        is_valid_element = true;
        break;  // Found non-zero value, element is valid
      }
    }

    if (is_valid_element) {
      valid_count++;
    }
  }

  return valid_count;
}

std::optional<size_t> fix_stop_points(Trajectory & trajectory, const StopPointFixingParams & params)
{
  auto & points = trajectory.points;
  auto stop_it = points.end();
  auto deceleration_start_it = points.end();
  for (auto it = points.begin(); it != points.end(); ++it) {
    if (it->acceleration_mps2 >= -0.01F) {
      deceleration_start_it = points.end();
      continue;
    }

    if (deceleration_start_it == points.end()) {
      deceleration_start_it = it;
    }
    const auto & start_time = deceleration_start_it->time_from_start;
    const auto & current_time = it->time_from_start;
    const double deceleration_duration = static_cast<double>(current_time.sec - start_time.sec) +
                                         1.0e-9 * static_cast<double>(
                                                    static_cast<int64_t>(current_time.nanosec) -
                                                    static_cast<int64_t>(start_time.nanosec));
    if (
      deceleration_duration >= params.min_deceleration_duration_sec &&
      it->longitudinal_velocity_mps <= params.velocity_threshold_mps) {
      stop_it = it;
      break;
    }
  }
  if (stop_it == points.end()) {
    return std::nullopt;
  }

  const auto stop_pose = stop_it->pose;
  const float stop_steering = stop_it->front_wheel_angle_rad;
  for (auto it = stop_it; it != points.end(); ++it) {
    it->pose = stop_pose;
    it->longitudinal_velocity_mps = 0.0F;
    it->lateral_velocity_mps = 0.0F;
    it->acceleration_mps2 = 0.0F;
    it->heading_rate_rps = 0.0F;
    it->front_wheel_angle_rad = stop_steering;
  }
  return static_cast<size_t>(std::distance(points.begin(), stop_it));
}

namespace
{
Trajectory get_trajectory_from_poses(
  const std::vector<Eigen::Matrix4d> & poses, const rclcpp::Time & stamp)
{
  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  constexpr double dt = 0.1;

  for (size_t i = 0; i < poses.size(); ++i) {
    const double curr_time = dt * static_cast<double>(i + 1);
    TrajectoryPoint p;
    p.time_from_start.sec = static_cast<int>(curr_time);
    p.time_from_start.nanosec = static_cast<uint32_t>((curr_time - p.time_from_start.sec) * 1e9);

    // Extract position from transformation matrix
    p.pose.position.x = poses[i](0, 3);
    p.pose.position.y = poses[i](1, 3);
    p.pose.position.z = poses[i](2, 3);

    // Extract 3x3 rotation matrix and convert to quaternion
    const Eigen::Matrix3d rotation_matrix = poses[i].block<3, 3>(0, 0);
    const Eigen::Quaterniond quaternion(rotation_matrix);
    p.pose.orientation.x = quaternion.x();
    p.pose.orientation.y = quaternion.y();
    p.pose.orientation.z = quaternion.z();
    p.pose.orientation.w = quaternion.w();

    trajectory.points.push_back(p);
  }

  return trajectory;
}

}  // namespace

}  // namespace autoware::ml_planner::postprocess
