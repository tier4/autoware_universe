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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/postprocessing/pseudo_controller.hpp"
#include "autoware/diffusion_planner/utils/arg_reader.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cassert>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::postprocess
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using std_msgs::msg::Float32MultiArray;
using unique_identifier_msgs::msg::UUID;

/**
 * @brief Parses raw prediction data into structured pose matrices in map coordinates.
 *
 * @param prediction The raw tensor prediction output (x, y, cos(yaw), sin(yaw) for each timestep).
 * @param transform_ego_to_map The transformation matrix from ego to map coordinates.
 * @return A 3D vector structure: [batch][agent][timestep] -> Eigen::Matrix4d (4x4 pose matrix).
 */
std::vector<std::vector<std::vector<Eigen::Matrix4d>>> parse_predictions(
  const std::vector<float> & prediction, const Eigen::Matrix4d & transform_ego_to_map);

std::vector<float> denormalize_prediction(
  const std::vector<float> & prediction, const utils::StateNormalization & state_normalization,
  bool keep_current_state = false);

Float32MultiArray create_denoising_steps_message(
  const std::vector<float> & denoising_predictions, const std::vector<float> & denoising_timesteps);

/**
 * @brief Creates PredictedObjects message from parsed agent poses.
 *
 * @param agent_poses The parsed agent poses [batch][agent][timestep] -> pose matrix.
 * @param ego_centric_histories The agent histories in ego-centric coordinates.
 * @param stamp The ROS time stamp for the message.
 * @param batch_index The batch index to use.
 * @return A PredictedObjects message containing predicted paths for each agent.
 */
PredictedObjects create_predicted_objects(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const std::vector<AgentHistory> & ego_centric_histories, const rclcpp::Time & stamp,
  const int64_t batch_index);

/**
 * @brief Creates a Trajectory message from parsed agent poses for a specific batch and ego agent.
 *
 * The raw network output path is not guaranteed to start at the ego base-link (data
 * augmentation during training shifts the origin of the predicted path), so `pseudo_controller`
 * simulates ego motion converging from the current state onto the predicted path/velocity plan,
 * converging smoothly instead of jumping onto it.
 *
 * @param agent_poses The parsed agent poses [batch][agent][timestep] -> pose matrix.
 * @param stamp The ROS time stamp for the message.
 * @param base_pose The current ego pose (position and heading) in map coordinates.
 * @param batch_index The batch index to extract.
 * @param enable_force_stop Whether to latch the predicted velocity to zero once it first drops to
 * (near-)zero, so a spurious/premature re-departure later in the same predicted plan does not
 * move the vehicle. Typically driven by `StopManager` rather than a raw ego-speed check.
 * @param stopping_threshold The threshold for keeping the stopping state [m/s].
 * @param wheel_base The ego vehicle wheel base [m], used for the convergence simulation.
 * @param current_steering_angle The current ego steering (tire) angle [rad], used as the starting
 * point of the convergence simulation so it continues smoothly from the vehicle's actual wheel
 * angle instead of assuming it starts straight.
 * @param pseudo_controller The convergence algorithm used to simulate ego motion from the current
 * state onto the predicted path/velocity plan.
 * @param ego_velocity_future The predicted longitudinal velocity profile of the ego vehicle. Must
 * contain at least `batch_index * OUTPUT_T + OUTPUT_T` elements.
 * @return A Trajectory message for the ego agent in the specified batch.
 * @throws std::runtime_error if `ego_velocity_future` is too small for the requested batch.
 */
Trajectory create_ego_trajectory(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & base_pose, const int64_t batch_index,
  const bool enable_force_stop, const double stopping_threshold, const double wheel_base,
  const double current_steering_angle, const PseudoController & pseudo_controller,
  const std::vector<float> & ego_velocity_future);

/**
 * @brief Creates a spatial Trajectory message from parsed ego path poses without temporal
 * resampling.
 *
 * @param agent_poses The parsed agent poses [batch][agent][timestep] -> pose matrix.
 * @param stamp The ROS time stamp for the message.
 * @param base_pose The current ego pose (position and heading) in map coordinates.
 * @param batch_index The batch index to extract.
 * @return A Trajectory message containing the spatial ego path in the specified batch.
 */
Trajectory create_spatial_ego_trajectory(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & base_pose,
  const int64_t batch_index);

/**
 * @brief Counts valid elements in a tensor with shape (B, len, dim2, dim3).
 * An element is considered valid if not all values in the (dim2, dim3) block are zero.
 *
 * @param data The input tensor data (flattened).
 * @param len The length dimension.
 * @param dim2 The second-to-last dimension.
 * @param dim3 The last dimension.
 * @param batch_idx The batch index to examine (0-based).
 * @return The number of valid elements in the specified batch.
 */
int64_t count_valid_elements(
  const std::vector<float> & data, int64_t len, int64_t dim2, int64_t dim3, int64_t batch_idx);

}  // namespace autoware::diffusion_planner::postprocess
#endif  // AUTOWARE__DIFFUSION_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
