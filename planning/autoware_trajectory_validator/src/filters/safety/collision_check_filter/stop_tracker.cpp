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

#include "stop_tracker.hpp"

#include <algorithm>
#include <stdexcept>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
void validate(const StopTrackingParams & params)
{
  if (!(params.stopped_velocity_threshold > 0.0)) {
    throw std::invalid_argument("stopped_velocity_threshold must be greater than zero");
  }
  if (!(params.history_timeout >= 0.0)) {
    throw std::invalid_argument("history_timeout must be non-negative");
  }
}

bool is_stopped(const geometry_msgs::msg::Twist & twist, const StopTrackingParams & params)
{
  const auto & vel = twist.linear;
  const double speed_sq = vel.x * vel.x + vel.y * vel.y;
  const double speed_threshold_sq =
    params.stopped_velocity_threshold * params.stopped_velocity_threshold;
  return speed_sq < speed_threshold_sq;
}

bool is_history_expired(
  const detail::StopTimeRecord & record, const rclcpp::Time & observation_time,
  const StopTrackingParams & params)
{
  return (observation_time - record.last_seen).seconds() > params.history_timeout;
}

std::optional<rclcpp::Duration> calculate_stopped_duration(
  const detail::StopTimeRecord * record, const std::optional<rclcpp::Time> & last_update_time)
{
  if (record == nullptr || !last_update_time.has_value()) {
    return std::nullopt;
  }
  return last_update_time.value() - record->stopped_since;
}
}  // namespace

ObjectStopTracker::ObjectStopTracker(const StopTrackingParams & params)
{
  stop_times_.reserve(kExpectedMaxObjectCount);
  set_parameters(params);
}

void ObjectStopTracker::set_parameters(const StopTrackingParams & params)
{
  validate(params);
  params_ = params;
}

void ObjectStopTracker::update(const autoware_perception_msgs::msg::PredictedObjects & objects)
{
  const rclcpp::Time observation_time(objects.header.stamp);

  if (last_update_time_.has_value() && observation_time < last_update_time_.value()) {
    stop_times_.clear();
  }
  last_update_time_ = observation_time;

  for (auto iter = stop_times_.begin(); iter != stop_times_.end();) {
    if (is_history_expired(iter->second, observation_time, params_)) {
      iter = stop_times_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (const auto & object : objects.objects) {
    const bool stopped = is_stopped(object.kinematics.initial_twist_with_covariance.twist, params_);
    const auto & object_id = object.object_id.uuid;

    if (!stopped) {
      stop_times_.erase(object_id);
      continue;
    }

    const auto [iter, inserted] = stop_times_.try_emplace(
      object_id, detail::StopTimeRecord{observation_time, observation_time});
    if (!inserted) {
      iter->second.last_seen = observation_time;
    }
  }
}

std::optional<rclcpp::Duration> ObjectStopTracker::get_stopped_duration(
  const unique_identifier_msgs::msg::UUID & object_id) const
{
  const auto iter = stop_times_.find(object_id.uuid);
  const auto * record = iter != stop_times_.end() ? &iter->second : nullptr;
  return calculate_stopped_duration(record, last_update_time_);
}

EgoStopTracker::EgoStopTracker(const StopTrackingParams & params)
{
  set_parameters(params);
}

void EgoStopTracker::set_parameters(const StopTrackingParams & params)
{
  validate(params);
  params_ = params;
}

void EgoStopTracker::update(const nav_msgs::msg::Odometry & odometry)
{
  const rclcpp::Time observation_time(odometry.header.stamp);

  if (last_update_time_.has_value() && observation_time < last_update_time_.value()) {
    stop_time_.reset();
  }
  last_update_time_ = observation_time;

  if (stop_time_.has_value() && is_history_expired(stop_time_.value(), observation_time, params_)) {
    stop_time_.reset();
  }

  if (!is_stopped(odometry.twist.twist, params_)) {
    stop_time_.reset();
    return;
  }

  if (!stop_time_.has_value()) {
    stop_time_.emplace(detail::StopTimeRecord{observation_time, observation_time});
  } else {
    stop_time_->last_seen = observation_time;
  }
}

std::optional<rclcpp::Duration> EgoStopTracker::get_stopped_duration() const
{
  const auto * record = stop_time_.has_value() ? &stop_time_.value() : nullptr;
  return calculate_stopped_duration(record, last_update_time_);
}

std::optional<rclcpp::Duration> StopTrackers::get_stopped_duration(
  const unique_identifier_msgs::msg::UUID & object_id) const
{
  const auto ego_duration = ego_.get_stopped_duration();
  const auto object_duration = object_.get_stopped_duration(object_id);

  if (!ego_duration.has_value() || !object_duration.has_value()) {
    return std::nullopt;
  }

  return std::min(ego_duration.value(), object_duration.value());
}

}  // namespace autoware::trajectory_validator::plugin::safety
