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


#include "autoware/tensorrt_e2e/start_assist.hpp"

#include <rclcpp/duration.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autoware::tensorrt_e2e
{

StartAssist::StartAssist(const StartAssistParams & params) : params_(params)
{
  if (params_.distance_m <= 0.0 || params_.duration_s <= 0.0 || params_.max_speed_mps <= 0.0) {
    throw std::invalid_argument(
      "start_assist.distance_m, duration_s and max_speed_mps must be positive");
  }
}

double StartAssist::acceleration() const
{
  return 2.0 * params_.distance_m / (params_.duration_s * params_.duration_s);
}

double StartAssist::speed_at(const double t) const
{
  return std::min(acceleration() * std::max(t, 0.0), params_.max_speed_mps);
}

double StartAssist::distance_at(const double t) const
{
  const double a = acceleration();
  const double t_cap = params_.max_speed_mps / a;  // when the ramp reaches max speed
  const double tc = std::max(t, 0.0);
  if (tc <= t_cap) {
    return 0.5 * a * tc * tc;
  }
  return 0.5 * a * t_cap * t_cap + params_.max_speed_mps * (tc - t_cap);
}

StartAssist::Event StartAssist::update(const bool engaged, const double speed, const double now)
{
  const bool engaged_now = engaged && !was_engaged_.value_or(true);
  was_engaged_ = engaged;
  if (!params_.enable) {
    active_ = false;
    return Event::kNone;
  }
  if (active_) {
    if (!engaged) {
      active_ = false;
      return Event::kDisengaged;
    }
    if (std::abs(speed) >= params_.handover_speed_mps) {
      active_ = false;
      return Event::kHandedOver;
    }
    if (now - engage_time_ >= params_.timeout_s) {
      active_ = false;
      return Event::kTimedOut;
    }
    return Event::kNone;
  }
  if (engaged_now && std::abs(speed) < params_.standstill_speed_mps) {
    active_ = true;
    engage_time_ = now;
    return Event::kStarted;
  }
  return Event::kNone;
}

std::vector<autoware_planning_msgs::msg::TrajectoryPoint> StartAssist::make_points(
  const geometry_msgs::msg::Pose & pose, const double now, const double time_step,
  const double horizon) const
{
  const auto & q = pose.orientation;
  const double yaw =
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double t0 = now - engage_time_;
  const double s0 = distance_at(t0);
  const auto count = static_cast<int>(std::lround(horizon / time_step));

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points;
  points.reserve(static_cast<size_t>(std::max(count, 0)));
  for (int k = 1; k <= count; ++k) {
    const double dt = k * time_step;
    const double s = distance_at(t0 + dt) - s0;
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(dt);
    point.pose = pose;
    point.pose.position.x += s * std::cos(yaw);
    point.pose.position.y += s * std::sin(yaw);
    point.longitudinal_velocity_mps = static_cast<float>(speed_at(t0 + dt));
    point.acceleration_mps2 =
      static_cast<float>(speed_at(t0 + dt) < params_.max_speed_mps ? acceleration() : 0.0);
    points.push_back(point);
  }
  return points;
}

}  // namespace autoware::tensorrt_e2e
