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

#ifndef AUTOWARE__ML_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
#define AUTOWARE__ML_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_

#include "autoware/ml_planner/preprocessing/items/agent.hpp"

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <xtensor/xarray.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cassert>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace autoware::ml_planner::postprocess
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using preprocess::SelectedAgent;
using preprocess::TrackedObject;
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

std::vector<float> denormalize_prediction(const std::vector<float> & prediction);

/**
 * @brief Creates PredictedObjects message from parsed agent poses.
 *
 * @param agent_poses The parsed agent poses [batch][agent][timestep] -> pose matrix.
 * @param selected_agents The currently visible agents ordered by distance from ego.
 * @param stamp The ROS time stamp for the message.
 * @param batch_index The batch index to use.
 * @return A PredictedObjects message containing predicted paths for each agent.
 */
PredictedObjects create_predicted_objects(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const std::vector<SelectedAgent> & selected_agents, const rclcpp::Time & stamp,
  const int64_t batch_index);

/**
 * @brief Creates a Trajectory message from parsed agent poses for a specific batch and ego agent.
 *
 * The velocity of each point is its distance to the previous point divided by the time step,
 * with the current ego position as the predecessor of the first point, and the acceleration is
 * the forward difference of that speed profile (0 on the last point). The model predicts poses
 * only, so the heading rate and steering angle are left at zero. When the trajectory
 * optimization is enabled it recomputes all of these consistently with the vehicle model.
 *
 * @param agent_poses The parsed agent poses [batch][agent][timestep] -> pose matrix.
 * @param stamp The ROS time stamp for the message.
 * @param base_position The current ego position in map coordinates.
 * @param batch_index The batch index to extract.
 * @return A Trajectory message for the ego agent in the specified batch.
 */
Trajectory create_ego_trajectory(
  const std::vector<std::vector<std::vector<Eigen::Matrix4d>>> & agent_poses,
  const rclcpp::Time & stamp, const geometry_msgs::msg::Point & base_position, int64_t batch_index);

/**
 * @brief Counts valid elements in a tensor with shape (B, len, dim2, dim3).
 * An element is considered valid if not all values in the (dim2, dim3) block are zero.
 *
 * @param data The input tensor data.
 * @param len The length dimension.
 * @param dim2 The second-to-last dimension.
 * @param dim3 The last dimension.
 * @param batch_idx The batch index to examine (0-based).
 * @return The number of valid elements in the specified batch.
 */
int64_t count_valid_elements(
  const xt::xarray<float> & data, int64_t len, int64_t dim2, int64_t dim3, int64_t batch_idx);

struct StopPointFixingParams
{
  bool enable{false};
  double velocity_threshold_mps{0.3};
  double min_deceleration_duration_sec{1.0};
};

/**
 * @brief Freeze the trajectory tail at the first stopping point.
 *
 * Finds the first point whose velocity is at or below the threshold after acceleration has
 * stayed negative for the configured duration. A non-decelerating point resets the duration.
 * The stop point and all subsequent points are fixed to its pose with zero velocity,
 * acceleration, and heading rate (steering angle is kept).
 *
 * @param trajectory Trajectory to modify in place.
 * @param params Stop detection parameters.
 * @return Index of the stop point, or std::nullopt when no stopping point was found.
 */
std::optional<size_t> fix_stop_points(
  Trajectory & trajectory, const StopPointFixingParams & params);

struct VelocitySmoothingParams
{
  bool enable{false};
  double horizon_sec{1.5};
};

/**
 * @brief Replace the finite-difference velocity and acceleration of the leading points.
 *
 * The speed of the model output is differentiated from its poses, so a few centimetres of
 * spacing noise become several m/s^2 of acceleration. This fits a quadratic of the arc length
 * over time to the points within the horizon and takes the velocity and the (constant)
 * acceleration from the fit. Points after the horizon are left unchanged. Poses are not touched.
 *
 * @param trajectory Trajectory to modify in place.
 * @param params Smoothing parameters.
 */
void smooth_initial_velocity(Trajectory & trajectory, const VelocitySmoothingParams & params);

struct CurveSpeedLimitParams
{
  bool enable{false};
  double max_lateral_acceleration_mps2{1.0};
  double max_deceleration_mps2{1.0};
};

/**
 * @brief Cap the speed in curves by a lateral acceleration and brake for the cap in advance.
 *
 * Without the trajectory optimizer (whose lateral acceleration constraint slows the vehicle in
 * curves) the model speed is kept through tight curves, and the controller cuts them. The curvature
 * of each point comes from the circle through it and its neighbours at least 1 m away along the
 * path; the speed is capped at sqrt(max lateral acceleration / curvature), then a backward pass
 * limits the deceleration toward each cap. Accelerations are recomputed where the speed changed.
 * Poses and times are not changed.
 *
 * @param trajectory Trajectory to modify in place.
 * @param params Limit parameters.
 */
void limit_curve_speed(Trajectory & trajectory, const CurveSpeedLimitParams & params);

struct PathSmoothingParams
{
  bool enable{false};
  double horizon_sec{1.5};
  double blend_sec{0.5};
  double tail_half_window_sec{0.5};
};

/**
 * @brief Replace the leading points by a smooth path that starts at the vehicle.
 *
 * Without the trajectory optimizer the model output starts a few centimetres beside the
 * vehicle and its first points jitter sideways, so the leading headings jump by several
 * degrees from one cycle to the next. In the vehicle frame this fits x(t) = v t + a t^2 + b t^3
 * and y(t) = c t^2 + d t^3 to the points up to horizon + blend: the path starts at the vehicle
 * with its heading and speed. Points up to the horizon take the fitted pose (heading from the
 * fitted direction of travel), points in the blend window move linearly back to the model
 * output, later points are left unchanged. Velocities are not changed.
 *
 * With the virtual pose the path starts at the virtual pose (the pose the model planned from), not
 * at the measured vehicle: started at the vehicle, a virtual pose taken from this trajectory would
 * return onto the vehicle every cycle.
 *
 * @param trajectory Trajectory to modify in place.
 * @param ego_pose Pose the path starts at (the planning pose: virtual pose when enabled).
 * @param ego_speed_mps Speed at that pose.
 * @param params Smoothing parameters.
 */
void smooth_initial_path(
  Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose, double ego_speed_mps,
  const PathSmoothingParams & params);

/**
 * @brief Smooth the points after the path smoothing horizon and take their headings from the path.
 *
 * Without the trajectory optimizer the model output wobbles a few centimetres sideways and its
 * point headings do not follow its own geometry: the controller tracks a path whose heading jumps,
 * and a spline through the points (the virtual pose) disagrees with the point headings. Each point
 * after the horizon takes the value at its time of a quadratic fit of x(t) and y(t) to the points
 * within the half window around it (shrunk near the end so it stays symmetric), and the fitted
 * direction of travel as its heading. A point whose fitted speed is too low for a direction, and
 * the last points (window of fewer than five points), keep the heading of the point before it.
 * Points up to the horizon (see smooth_initial_path) and velocities are not changed; a zero half
 * window disables this.
 *
 * @param trajectory Trajectory to modify in place.
 * @param params Smoothing parameters.
 */
void smooth_path_tail(Trajectory & trajectory, const PathSmoothingParams & params);

}  // namespace autoware::ml_planner::postprocess
#endif  // AUTOWARE__ML_PLANNER__POSTPROCESSING__POSTPROCESSING_UTILS_HPP_
