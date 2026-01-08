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

#include "autoware/steer_offset_estimator/steer_offset_estimator.hpp"

#include "autoware/steer_offset_estimator/utils.hpp"

#include <rclcpp/rclcpp/duration.hpp>
#include <rclcpp/rclcpp/time.hpp>
#include <tl_expected/expected.hpp>

#include <range/v3/algorithm/find_if.hpp>
#include <range/v3/view/reverse.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <numeric>
#include <optional>
#include <vector>
namespace autoware::steer_offset_estimator
{

SteerOffsetEstimator::SteerOffsetEstimator(const SteerOffsetEstimatorParameters & parameters)
: params_(parameters),
  estimated_offset_(parameters.initial_offset),
  covariance_(parameters.initial_covariance)
{
}

tl::expected<SteerOffsetEstimationUpdated, SteerOffsetEstimationNotUpdated>
SteerOffsetEstimator::update(
  const std::vector<PoseStamped> & poses, const std::vector<SteeringReport> & steers)
{
  if (poses.empty()) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"poses is empty"});
  }

  if (steers.empty()) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"steers is empty"});
  }

  geometry_msgs::msg::Twist twist;

  // Determine which poses to use for motion calculation
  if (!previous_pose_) {
    // First call - need at least 2 poses to calculate motion
    if (poses.size() < 2) {
      previous_pose_ = poses.back();
      return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"previous_pose has not been set"});
    }
    // Use first pose as previous, last pose as current
    twist = utils::calc_twist_from_pose(poses[0], poses.back());
    previous_pose_ = poses.back();
  } else {
    // Subsequent calls - use stored previous pose and current pose
    twist = utils::calc_twist_from_pose(previous_pose_.value(), poses.back());
    previous_pose_ = poses.back();
  }

  double velocity = twist.linear.x;
  double angular_velocity = twist.angular.z;

  update_steering_buffer(steers);
  const auto steering_angle = get_steering_at_timestamp(
    rclcpp::Time(previous_pose_->header.stamp));

  // Validate input data quality
  if (velocity < params_.min_velocity) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"velocity is too low"});
  }
  if (!steering_angle) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"steering angle is not available"});
  }
  if (std::abs(steering_angle.value()) > params_.max_steer) {
    return tl::make_unexpected(SteerOffsetEstimationNotUpdated{"steering angle is too large"});
  }

  // 1) Regressor and measurement for the regression model y = phi * theta + noise
  const double phi = velocity / params_.wheel_base;          // regressor
  const double y = angular_velocity - phi * steering_angle.value();  // measurement for regression

  // 2) Time update (process model): theta_k = theta_{k-1} + w,  w ~ N(0, Q)
  //    This inflates the PRIOR covariance to allow parameter drift.
  const double Q = params_.process_noise_covariance;  // NOLINT
  const double P_prior = covariance_ + Q;             // prior covariance // NOLINT

  // 3) Measurement update: compute denominator with PRIOR covariance
  //    denom = R + phi^2 * P_prior
  double denom = params_.measurement_noise_covariance + phi * phi * P_prior;
  if (denom < params_.denominator_floor) denom = params_.denominator_floor;  // numerical safety

  // 4) Kalman-like gain using PRIOR covariance
  const double K = (P_prior * phi) / denom;  // NOLINT

  // 5) Innovation (residual) and parameter update
  const double residual = y - phi * estimated_offset_;   // r = y - phi * theta_{k-1}
  estimated_offset_ = estimated_offset_ + K * residual;  // theta_k

  // 6) Covariance update (scalar Joseph form)
  //    P_k = P_prior - (P_prior * phi^2 * P_prior) / denom = (1 - K*phi) * P_prior
  double P_new = P_prior - (P_prior * phi * phi * P_prior) / denom;  // NOLINT
  if (P_new < params_.covariance_floor) P_new = params_.covariance_floor;
  covariance_ = P_new;

  SteerOffsetEstimationUpdated updated_result;
  updated_result.offset = estimated_offset_;
  updated_result.covariance = covariance_;
  updated_result.velocity = velocity;
  updated_result.angular_velocity = angular_velocity;
  updated_result.steering_angle = steering_angle.value();
  updated_result.kalman_gain = K;
  updated_result.residual = residual;
  return updated_result;
}

void SteerOffsetEstimator::update_steering_buffer(
  const std::vector<SteeringReport> & steers)
{
  for (const auto & steer : steers) {
    if (rclcpp::Time(steer.stamp) < rclcpp::Time(steering_buffer_.back()->stamp)) {
      continue;
    }
    steering_buffer_.emplace_back(std::make_shared<SteeringReport>(steer));
  }

  // Keep buffer size manageable
  while (!steering_buffer_.empty() &&
    (rclcpp::Time(steering_buffer_.back()->stamp) - rclcpp::Time(steering_buffer_.front()->stamp)).seconds()
    > max_steering_buffer_s) {
    steering_buffer_.pop_front();
  }
}

std::optional<double> SteerOffsetEstimator::get_steering_at_timestamp(
  const rclcpp::Time & timestamp) const
{
  if (steering_buffer_.empty()) return std::nullopt;

  const auto upper = std::find_if(steering_buffer_.begin(), steering_buffer_.end(),
    [&timestamp](const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr & steer_ptr) {
      return rclcpp::Time(steer_ptr->stamp) > timestamp;
    });

  const auto pivot = (upper == steering_buffer_.begin()) ? upper : std::prev(upper);

  const int window = 2;
  auto start = (std::distance(steering_buffer_.begin(), pivot) > window) ?
    pivot - window : steering_buffer_.begin();
  auto finish = (std::distance(pivot, steering_buffer_.end()) > window) ?
    pivot + window + 1 : steering_buffer_.end();

  const auto count = std::distance(start, finish);

  if (count == 0) return std::nullopt;

  double steering_sum = std::accumulate(start, finish, 0.0, [](double sum, const auto & steer_ptr) {
      return sum + steer_ptr->steering_tire_angle;
    });
  return steering_sum / count;
}

}  // namespace autoware::steer_offset_estimator
