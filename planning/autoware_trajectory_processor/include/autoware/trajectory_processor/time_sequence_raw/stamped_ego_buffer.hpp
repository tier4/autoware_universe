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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__STAMPED_EGO_BUFFER_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__STAMPED_EGO_BUFFER_HPP_

#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/time.hpp>
#include <tf2/utils.h>

#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

namespace autoware::trajectory_processor::time_sequence_raw
{

/// Ego pose/twist/accel/steer sampled at localization rate for stamp-aligned OCP init.
struct StampedEgoState
{
  nav_msgs::msg::Odometry odometry;
  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  autoware_vehicle_msgs::msg::SteeringReport steering;
  bool has_acceleration{false};
  bool has_steering{false};
  bool interpolated{false};
  bool fallback_latest{false};
  /// query_stamp - used_odom_stamp; ~0 when aligned to planner frame_time.
  double lookup_dt_s{0.0};
  /// latest_odom_stamp - query_stamp; ~inference+transport lag when loc is 40 Hz.
  double live_lag_s{0.0};
};

class StampedEgoBuffer
{
public:
  void set_duration(const double duration_s)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    duration_s_ = std::max(0.1, duration_s);
    prune_locked(rclcpp::Time(0, 0, RCL_ROS_TIME));
  }

  void push_odometry(const nav_msgs::msg::Odometry & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    push_sorted(odometry_, msg, stamp_of(msg));
  }

  void push_acceleration(const geometry_msgs::msg::AccelWithCovarianceStamped & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    push_sorted(acceleration_, msg, stamp_of(msg));
  }

  void push_steering(const autoware_vehicle_msgs::msg::SteeringReport & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    push_sorted(steering_, msg, stamp_of(msg));
  }

  [[nodiscard]] std::optional<nav_msgs::msg::Odometry> latest_odometry() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (odometry_.empty()) {
      return std::nullopt;
    }
    return odometry_.back();
  }

  /// Interpolate ego at planner stamp. Falls back to the newest sample if the query is
  /// newer than the buffer, or older than `max_mismatch_s` past the oldest sample.
  [[nodiscard]] std::optional<StampedEgoState> lookup(
    const rclcpp::Time & query, const double max_mismatch_s) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (odometry_.empty()) {
      return std::nullopt;
    }

    StampedEgoState out;
    const auto latest_stamp = stamp_of(odometry_.back());
    out.live_lag_s = (latest_stamp - query).seconds();

    bool interpolated = false;
    bool fallback_latest = false;
    out.odometry = interpolate_odometry_locked(query, interpolated, fallback_latest);
    out.interpolated = interpolated;
    out.fallback_latest = fallback_latest;
    out.lookup_dt_s = (query - stamp_of(out.odometry)).seconds();

    if (std::abs(out.lookup_dt_s) > max_mismatch_s && fallback_latest) {
      // Keep fallback; caller logs.
    }

    if (auto accel = interpolate_accel_locked(query)) {
      out.acceleration = *accel;
      out.has_acceleration = true;
    }
    if (auto steer = interpolate_steering_locked(query)) {
      out.steering = *steer;
      out.has_steering = true;
    }
    return out;
  }

private:
  static rclcpp::Time stamp_of(const nav_msgs::msg::Odometry & msg)
  {
    return rclcpp::Time(msg.header.stamp, RCL_ROS_TIME);
  }
  static rclcpp::Time stamp_of(const geometry_msgs::msg::AccelWithCovarianceStamped & msg)
  {
    return rclcpp::Time(msg.header.stamp, RCL_ROS_TIME);
  }
  static rclcpp::Time stamp_of(const autoware_vehicle_msgs::msg::SteeringReport & msg)
  {
    return rclcpp::Time(msg.stamp, RCL_ROS_TIME);
  }

  static double normalize_angle(double yaw)
  {
    while (yaw > M_PI) {
      yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
      yaw += 2.0 * M_PI;
    }
    return yaw;
  }

  template <typename Msg>
  void push_sorted(std::deque<Msg> & buf, const Msg & msg, const rclcpp::Time & stamp)
  {
    if (stamp.nanoseconds() == 0) {
      return;
    }
    while (!buf.empty() && stamp_of(buf.back()) >= stamp) {
      buf.pop_back();
    }
    buf.push_back(msg);
    prune_locked(stamp);
  }

  void prune_locked(const rclcpp::Time & newest)
  {
    const auto drop_older_than = [&](auto & buf) {
      while (buf.size() > 1) {
        const auto age = (newest.nanoseconds() == 0)
                           ? duration_s_ + 1.0
                           : (newest - stamp_of(buf.front())).seconds();
        if (age <= duration_s_) {
          break;
        }
        buf.pop_front();
      }
    };
    if (newest.nanoseconds() != 0) {
      drop_older_than(odometry_);
      drop_older_than(acceleration_);
      drop_older_than(steering_);
    }
  }

  template <typename Msg>
  static std::pair<const Msg *, const Msg *> bracketing(
    const std::deque<Msg> & buf, const rclcpp::Time & query)
  {
    if (buf.empty()) {
      return {nullptr, nullptr};
    }
    if (query <= stamp_of(buf.front())) {
      return {&buf.front(), &buf.front()};
    }
    if (query >= stamp_of(buf.back())) {
      return {&buf.back(), &buf.back()};
    }
    for (size_t i = 0; i + 1 < buf.size(); ++i) {
      if (query >= stamp_of(buf[i]) && query <= stamp_of(buf[i + 1])) {
        return {&buf[i], &buf[i + 1]};
      }
    }
    return {&buf.back(), &buf.back()};
  }

  static double lerp_alpha(const rclcpp::Time & t0, const rclcpp::Time & t1, const rclcpp::Time & q)
  {
    const double dt = (t1 - t0).seconds();
    if (std::abs(dt) < 1e-9) {
      return 0.0;
    }
    return std::clamp((q - t0).seconds() / dt, 0.0, 1.0);
  }

  nav_msgs::msg::Odometry interpolate_odometry_locked(
    const rclcpp::Time & query, bool & interpolated, bool & fallback_latest) const
  {
    interpolated = false;
    fallback_latest = false;
    const auto [a, b] = bracketing(odometry_, query);
    if (a == nullptr) {
      fallback_latest = true;
      return odometry_.back();
    }
    if (a == b) {
      fallback_latest = query != stamp_of(*a);
      return *a;
    }
    interpolated = true;
    const double alpha = lerp_alpha(stamp_of(*a), stamp_of(*b), query);
    nav_msgs::msg::Odometry out = *b;
    out.header.stamp = query;
    out.pose.pose.position.x =
      a->pose.pose.position.x + alpha * (b->pose.pose.position.x - a->pose.pose.position.x);
    out.pose.pose.position.y =
      a->pose.pose.position.y + alpha * (b->pose.pose.position.y - a->pose.pose.position.y);
    out.pose.pose.position.z =
      a->pose.pose.position.z + alpha * (b->pose.pose.position.z - a->pose.pose.position.z);
    const double yaw_a = tf2::getYaw(a->pose.pose.orientation);
    const double yaw_b = tf2::getYaw(b->pose.pose.orientation);
    const double yaw = yaw_a + alpha * normalize_angle(yaw_b - yaw_a);
    out.pose.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
    out.twist.twist.linear.x =
      a->twist.twist.linear.x + alpha * (b->twist.twist.linear.x - a->twist.twist.linear.x);
    out.twist.twist.linear.y =
      a->twist.twist.linear.y + alpha * (b->twist.twist.linear.y - a->twist.twist.linear.y);
    out.twist.twist.angular.z =
      a->twist.twist.angular.z + alpha * (b->twist.twist.angular.z - a->twist.twist.angular.z);
    return out;
  }

  std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> interpolate_accel_locked(
    const rclcpp::Time & query) const
  {
    if (acceleration_.empty()) {
      return std::nullopt;
    }
    const auto [a, b] = bracketing(acceleration_, query);
    if (a == nullptr) {
      return acceleration_.back();
    }
    if (a == b) {
      return *a;
    }
    const double alpha = lerp_alpha(stamp_of(*a), stamp_of(*b), query);
    auto out = *b;
    out.header.stamp = query;
    out.accel.accel.linear.x =
      a->accel.accel.linear.x + alpha * (b->accel.accel.linear.x - a->accel.accel.linear.x);
    return out;
  }

  std::optional<autoware_vehicle_msgs::msg::SteeringReport> interpolate_steering_locked(
    const rclcpp::Time & query) const
  {
    if (steering_.empty()) {
      return std::nullopt;
    }
    const auto [a, b] = bracketing(steering_, query);
    if (a == nullptr) {
      return steering_.back();
    }
    if (a == b) {
      return *a;
    }
    const double alpha = lerp_alpha(stamp_of(*a), stamp_of(*b), query);
    auto out = *b;
    out.stamp = query;
    out.steering_tire_angle = static_cast<float>(
      a->steering_tire_angle +
      alpha * (b->steering_tire_angle - a->steering_tire_angle));
    return out;
  }

  mutable std::mutex mutex_;
  double duration_s_{1.0};
  std::deque<nav_msgs::msg::Odometry> odometry_;
  std::deque<geometry_msgs::msg::AccelWithCovarianceStamped> acceleration_;
  std::deque<autoware_vehicle_msgs::msg::SteeringReport> steering_;
};

}  // namespace autoware::trajectory_processor::time_sequence_raw

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TIME_SEQUENCE_RAW__STAMPED_EGO_BUFFER_HPP_
