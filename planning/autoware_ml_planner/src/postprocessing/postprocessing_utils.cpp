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
 * The speed of each point is its distance to the previous point divided by the time step, with
 * the base position as the predecessor of the first point, and the acceleration is the forward
 * difference of that speed profile. The heading rate and steering angle stay at zero: the model
 * predicts poses only. The trajectory optimization, when enabled, recomputes all of them.
 *
 * @param poses The vector of 4x4 transformation matrices representing poses.
 * @param base_x The base x position to calculate relative velocities.
 * @param base_y The base y position to calculate relative velocities.
 * @param base_z The base z position to calculate relative velocities.
 * @param stamp The ROS time stamp for the message.
 * @return A Trajectory message in map coordinates.
 */
Trajectory get_trajectory_from_poses(
  const std::vector<Eigen::Matrix4d> & poses, const double base_x, const double base_y,
  const double base_z, const rclcpp::Time & stamp);
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

    PredictedObject object;
    const TrackedObject & object_info = selected_agents.at(neighbor_id).current_object;

    const auto & base_position = object_info.kinematics.pose_with_covariance.pose.position;
    const Trajectory trajectory_points_in_map_reference = get_trajectory_from_poses(
      neighbor_poses, base_position.x, base_position.y, base_position.z, stamp);

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
  const rclcpp::Time & stamp, const geometry_msgs::msg::Point & base_position,
  const int64_t batch_index)
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

  const double base_x = base_position.x;
  const double base_y = base_position.y;
  const double base_z = base_position.z;

  return get_trajectory_from_poses(ego_poses, base_x, base_y, base_z, stamp);
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

void smooth_initial_velocity(Trajectory & trajectory, const VelocitySmoothingParams & params)
{
  auto & points = trajectory.points;
  std::vector<double> times;
  std::vector<double> arc_lengths;
  double arc_length = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto & stamp = points[i].time_from_start;
    const double time =
      static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
    if (time > params.horizon_sec + 1.0e-6) {
      break;
    }
    if (i > 0) {
      const auto & a = points[i - 1].pose.position;
      const auto & b = points[i].pose.position;
      arc_length += std::hypot(b.x - a.x, b.y - a.y, b.z - a.z);
    }
    times.push_back(time);
    arc_lengths.push_back(arc_length);
  }
  // A quadratic needs three points; fewer leaves the finite differences in place.
  if (times.size() < 3) {
    return;
  }

  // Least squares s(t) = c0 + c1 t + c2 t^2.
  Eigen::MatrixXd design(times.size(), 3);
  Eigen::VectorXd target(times.size());
  for (size_t i = 0; i < times.size(); ++i) {
    design(static_cast<Eigen::Index>(i), 0) = 1.0;
    design(static_cast<Eigen::Index>(i), 1) = times[i];
    design(static_cast<Eigen::Index>(i), 2) = times[i] * times[i];
    target(static_cast<Eigen::Index>(i)) = arc_lengths[i];
  }
  Eigen::Vector3d coefficients = design.colPivHouseholderQr().solve(target);

  // A plan that waits a moment before moving off fits a negative initial speed, and the clamp
  // below then leaves zeros at the leading points: the longitudinal controller sees a stop at the
  // vehicle and never departs, and the next plan, made at standstill, waits again. Such a plan
  // moves off from rest, so refit it with the initial speed fixed at zero, s(t) = c0 + c2 t^2.
  if (coefficients(1) < 0.0 && coefficients(2) > 0.0) {
    Eigen::MatrixXd from_rest(times.size(), 2);
    from_rest.col(0) = design.col(0);
    from_rest.col(1) = design.col(2);
    const Eigen::Vector2d refit = from_rest.colPivHouseholderQr().solve(target);
    if (refit(1) > 0.0) {
      coefficients = Eigen::Vector3d(refit(0), 0.0, refit(1));
    }
  }

  const double acceleration = 2.0 * coefficients(2);
  for (size_t i = 0; i < times.size(); ++i) {
    const double velocity = coefficients(1) + acceleration * times[i];
    points[i].longitudinal_velocity_mps = static_cast<float>(std::max(velocity, 0.0));
    points[i].acceleration_mps2 = static_cast<float>(acceleration);
  }
}

void limit_curve_speed(Trajectory & trajectory, const CurveSpeedLimitParams & params)
{
  auto & points = trajectory.points;
  if (points.size() < 3 || params.max_lateral_acceleration_mps2 <= 0.0) {
    return;
  }
  const auto n = points.size();
  std::vector<Eigen::Vector2d> positions;
  std::vector<double> arc_lengths(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    positions.emplace_back(points[i].pose.position.x, points[i].pose.position.y);
    if (i > 0) {
      arc_lengths[i] = arc_lengths[i - 1] + (positions[i] - positions[i - 1]).norm();
    }
  }

  // Neighbours this far along the path, so centimetre spacing noise does not become curvature.
  constexpr double CURVATURE_BASE_M = 1.0;
  constexpr double MIN_CURVATURE = 1.0e-4;
  std::vector<std::optional<double>> curvatures(n);
  size_t previous = 0;
  size_t next = 0;
  for (size_t i = 1; i + 1 < n; ++i) {
    while (previous + 1 < i && arc_lengths[i] - arc_lengths[previous + 1] >= CURVATURE_BASE_M) {
      ++previous;
    }
    next = std::max(next, i + 1);
    while (next + 1 < n && arc_lengths[next] - arc_lengths[i] < CURVATURE_BASE_M) {
      ++next;
    }
    if (
      arc_lengths[i] - arc_lengths[previous] < CURVATURE_BASE_M ||
      arc_lengths[next] - arc_lengths[i] < CURVATURE_BASE_M) {
      continue;
    }
    // Curvature of the circle through the three points.
    const Eigen::Vector2d a = positions[i] - positions[previous];
    const Eigen::Vector2d b = positions[next] - positions[i];
    const Eigen::Vector2d c = positions[next] - positions[previous];
    const double cross = a.x() * b.y() - a.y() * b.x();
    curvatures[i] = std::abs(2.0 * cross) / (a.norm() * b.norm() * c.norm());
  }
  // Points within the base length of either end take the curvature of the nearest point that has
  // one, so the end of a plan inside a curve is capped too.
  const auto first = std::find_if(
    curvatures.begin(), curvatures.end(), [](const auto & k) { return k.has_value(); });
  if (first == curvatures.end()) {
    return;
  }
  std::fill(curvatures.begin(), first, *first);
  for (size_t i = 1; i < n; ++i) {
    if (!curvatures[i]) {
      curvatures[i] = curvatures[i - 1];
    }
  }

  std::vector<double> velocities(n);
  for (size_t i = 0; i < n; ++i) {
    velocities[i] = points[i].longitudinal_velocity_mps;
    if (*curvatures[i] > MIN_CURVATURE) {
      velocities[i] =
        std::min(velocities[i], std::sqrt(params.max_lateral_acceleration_mps2 / *curvatures[i]));
    }
  }
  // Brake for each cap in advance: v_i^2 <= v_{i+1}^2 + 2 a_dec ds.
  if (params.max_deceleration_mps2 > 0.0) {
    for (size_t i = n - 1; i-- > 0;) {
      const double ds = arc_lengths[i + 1] - arc_lengths[i];
      velocities[i] = std::min(
        velocities[i],
        std::sqrt(velocities[i + 1] * velocities[i + 1] + 2.0 * params.max_deceleration_mps2 * ds));
    }
  }

  constexpr double MIN_SEGMENT_M = 1.0e-3;
  for (size_t i = 0; i < n; ++i) {
    const bool changed = velocities[i] < points[i].longitudinal_velocity_mps;
    const bool next_changed =
      i + 1 < n && velocities[i + 1] < points[i + 1].longitudinal_velocity_mps;
    if (changed) {
      points[i].longitudinal_velocity_mps = static_cast<float>(velocities[i]);
    }
    if ((changed || next_changed) && i + 1 < n) {
      const double ds = arc_lengths[i + 1] - arc_lengths[i];
      if (ds > MIN_SEGMENT_M) {
        points[i].acceleration_mps2 = static_cast<float>(
          (velocities[i + 1] * velocities[i + 1] - velocities[i] * velocities[i]) / (2.0 * ds));
      }
    }
  }
}

void smooth_initial_path(
  Trajectory & trajectory, const geometry_msgs::msg::Pose & ego_pose, const double ego_speed_mps,
  const PathSmoothingParams & params)
{
  auto & points = trajectory.points;
  const double fit_end_sec = params.horizon_sec + params.blend_sec;
  const double ego_yaw = autoware_utils::get_rpy(ego_pose.orientation).z;
  const double cos_yaw = std::cos(ego_yaw);
  const double sin_yaw = std::sin(ego_yaw);
  const auto point_time = [](const TrajectoryPoint & point) {
    return static_cast<double>(point.time_from_start.sec) +
           1.0e-9 * static_cast<double>(point.time_from_start.nanosec);
  };

  // Points within the fit window in the vehicle frame.
  std::vector<double> times;
  std::vector<Eigen::Vector2d> local_positions;
  for (const auto & point : points) {
    const double time = point_time(point);
    if (time > fit_end_sec + 1.0e-6) {
      break;
    }
    const double dx = point.pose.position.x - ego_pose.position.x;
    const double dy = point.pose.position.y - ego_pose.position.y;
    times.push_back(time);
    local_positions.emplace_back(cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy);
  }
  // Two coefficients per axis; fewer than three points leaves the model output in place.
  if (times.size() < 3) {
    return;
  }

  // Least squares with the start fixed at the vehicle: x - v t = a t^2 + b t^3, y = c t^2 + d t^3.
  Eigen::MatrixXd design(times.size(), 2);
  Eigen::MatrixXd target(times.size(), 2);
  for (size_t i = 0; i < times.size(); ++i) {
    const auto row = static_cast<Eigen::Index>(i);
    const double t = times[i];
    design(row, 0) = t * t;
    design(row, 1) = t * t * t;
    target(row, 0) = local_positions[i].x() - ego_speed_mps * t;
    target(row, 1) = local_positions[i].y();
  }
  const Eigen::MatrixXd coefficients = design.colPivHouseholderQr().solve(target);
  const double ax = coefficients(0, 0);
  const double bx = coefficients(1, 0);
  const double ay = coefficients(0, 1);
  const double by = coefficients(1, 1);

  // Below this fitted speed the direction of travel is undefined; the vehicle heading is used.
  constexpr double MIN_HEADING_SPEED_MPS = 0.1;
  for (size_t i = 0; i < times.size(); ++i) {
    const double t = times[i];
    const double fit_x = ego_speed_mps * t + ax * t * t + bx * t * t * t;
    const double fit_y = ay * t * t + by * t * t * t;
    const double fit_vx = ego_speed_mps + 2.0 * ax * t + 3.0 * bx * t * t;
    const double fit_vy = 2.0 * ay * t + 3.0 * by * t * t;
    const double fit_yaw =
      std::hypot(fit_vx, fit_vy) > MIN_HEADING_SPEED_MPS ? std::atan2(fit_vy, fit_vx) : 0.0;

    // Weight of the fit: 1 up to the horizon, falling linearly to 0 at the end of the blend.
    const double weight =
      t <= params.horizon_sec || params.blend_sec <= 0.0
        ? 1.0
        : std::clamp(1.0 - (t - params.horizon_sec) / params.blend_sec, 0.0, 1.0);
    auto & pose = points[i].pose;
    const double model_yaw = autoware_utils::get_rpy(pose.orientation).z - ego_yaw;
    const double local_x = weight * fit_x + (1.0 - weight) * local_positions[i].x();
    const double local_y = weight * fit_y + (1.0 - weight) * local_positions[i].y();
    const double local_yaw =
      model_yaw + weight * autoware_utils::normalize_radian(fit_yaw - model_yaw);

    pose.position.x = ego_pose.position.x + cos_yaw * local_x - sin_yaw * local_y;
    pose.position.y = ego_pose.position.y + sin_yaw * local_x + cos_yaw * local_y;
    pose.orientation = autoware_utils::create_quaternion_from_yaw(
      autoware_utils::normalize_radian(ego_yaw + local_yaw));
  }
}

void smooth_path_tail(Trajectory & trajectory, const PathSmoothingParams & params)
{
  auto & points = trajectory.points;
  if (params.tail_half_window_sec <= 0.0 || points.size() < 3) {
    return;
  }
  std::vector<double> times;
  std::vector<Eigen::Vector2d> positions;
  for (const auto & point : points) {
    times.push_back(
      static_cast<double>(point.time_from_start.sec) +
      1.0e-9 * static_cast<double>(point.time_from_start.nanosec));
    positions.emplace_back(point.pose.position.x, point.pose.position.y);
  }

  // Below this fitted speed the direction of travel is undefined.
  constexpr double MIN_HEADING_SPEED_MPS = 0.1;
  constexpr double TIME_TOLERANCE_S = 1.0e-6;
  std::optional<double> previous_yaw;
  for (size_t i = 0; i < points.size(); ++i) {
    if (times[i] <= params.horizon_sec + TIME_TOLERANCE_S) {
      previous_yaw = autoware_utils::get_rpy(points[i].pose.orientation).z;
      continue;
    }
    // Quadratic in the time from this point, so the value and the derivative at it are the
    // first two coefficients; fitted to the input positions, not the already smoothed ones. The
    // window is kept symmetric (shrunk near the end) so the fit does not extrapolate.
    const double half_window_sec =
      std::min(params.tail_half_window_sec, times.back() - times[i]) + TIME_TOLERANCE_S;
    std::vector<size_t> window;
    for (size_t j = 0; j < points.size(); ++j) {
      if (std::abs(times[j] - times[i]) <= half_window_sec) {
        window.push_back(j);
      }
    }
    // Three points would be interpolated, not smoothed.
    constexpr size_t MIN_WINDOW_POINTS = 5;
    if (window.size() < MIN_WINDOW_POINTS) {
      // The last points: their position is kept, their heading continues the one before.
      if (previous_yaw) {
        points[i].pose.orientation = autoware_utils::create_quaternion_from_yaw(*previous_yaw);
      }
      continue;
    }
    Eigen::MatrixXd design(window.size(), 3);
    Eigen::MatrixXd target(window.size(), 2);
    for (size_t k = 0; k < window.size(); ++k) {
      const auto row = static_cast<Eigen::Index>(k);
      const double dt = times[window[k]] - times[i];
      design(row, 0) = 1.0;
      design(row, 1) = dt;
      design(row, 2) = dt * dt;
      target.row(row) = positions[window[k]].transpose();
    }
    const Eigen::MatrixXd coefficients = design.colPivHouseholderQr().solve(target);

    auto & pose = points[i].pose;
    pose.position.x = coefficients(0, 0);
    pose.position.y = coefficients(0, 1);
    const double fit_vx = coefficients(1, 0);
    const double fit_vy = coefficients(1, 1);
    if (std::hypot(fit_vx, fit_vy) > MIN_HEADING_SPEED_MPS) {
      previous_yaw = std::atan2(fit_vy, fit_vx);
    }
    if (previous_yaw) {
      pose.orientation = autoware_utils::create_quaternion_from_yaw(*previous_yaw);
    }
  }
}

namespace
{
Trajectory get_trajectory_from_poses(
  const std::vector<Eigen::Matrix4d> & poses, const double base_x, const double base_y,
  const double base_z, const rclcpp::Time & stamp)
{
  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  constexpr double dt = 0.1;

  double previous_x = base_x;
  double previous_y = base_y;
  double previous_z = base_z;

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

    const double distance = std::hypot(
      p.pose.position.x - previous_x, p.pose.position.y - previous_y,
      p.pose.position.z - previous_z);
    p.longitudinal_velocity_mps = static_cast<float>(distance / dt);

    previous_x = p.pose.position.x;
    previous_y = p.pose.position.y;
    previous_z = p.pose.position.z;

    trajectory.points.push_back(p);
  }

  // Acceleration as the forward difference of the velocity profile; the last point has no
  // successor and keeps zero.
  for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    const double v0 = trajectory.points[i].longitudinal_velocity_mps;
    const double v1 = trajectory.points[i + 1].longitudinal_velocity_mps;
    trajectory.points[i].acceleration_mps2 = static_cast<float>((v1 - v0) / dt);
  }

  return trajectory;
}

}  // namespace

}  // namespace autoware::ml_planner::postprocess
