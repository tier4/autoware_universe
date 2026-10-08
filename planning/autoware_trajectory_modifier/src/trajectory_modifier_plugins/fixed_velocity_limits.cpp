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

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/velocity_limits.hpp"

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::trajectory_modifier::plugin::detail
{
namespace
{
constexpr double numerical_tolerance = 1e-6;
constexpr std::size_t maximum_cached_candidates = 64;

double distance2d(const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

double heading_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double heading_difference(const double lhs, const double rhs)
{
  return std::abs(std::atan2(std::sin(lhs - rhs), std::cos(lhs - rhs)));
}

std::optional<double> valid_limit(const std::optional<double> & limit)
{
  if (!limit || !std::isfinite(*limit) || *limit < 0.0) {
    return std::nullopt;
  }
  return limit;
}

bool same_limit(
  const std::optional<double> & lhs, const std::optional<double> & rhs,
  const double difference_threshold)
{
  const auto a = valid_limit(lhs);
  const auto b = valid_limit(rhs);
  if (!a || !b) {
    return !a && !b;
  }
  return std::abs(*a - *b) <= difference_threshold;
}

bool same_key(
  const FixedVelocityLimitKey & lhs, const FixedVelocityLimitKey & rhs,
  const FixedVelocityLimitParameters & parameters)
{
  return lhs.spatial == rhs.spatial && lhs.context_revision == rhs.context_revision &&
         same_limit(lhs.uniform_limit, rhs.uniform_limit, parameters.limit_difference_threshold) &&
         std::abs(lhs.deceleration - rhs.deceleration) <=
           parameters.constraint_difference_threshold &&
         std::abs(lhs.jerk - rhs.jerk) <= parameters.constraint_difference_threshold;
}

bool same_parameters(
  const FixedVelocityLimitParameters & lhs, const FixedVelocityLimitParameters & rhs)
{
  return lhs.speed_deviation_threshold == rhs.speed_deviation_threshold &&
         lhs.acceleration_deviation_threshold == rhs.acceleration_deviation_threshold &&
         lhs.longitudinal_deviation_threshold == rhs.longitudinal_deviation_threshold &&
         lhs.lateral_deviation_threshold == rhs.lateral_deviation_threshold &&
         lhs.heading_deviation_threshold == rhs.heading_deviation_threshold &&
         lhs.stamp_difference_threshold == rhs.stamp_difference_threshold &&
         lhs.limit_difference_threshold == rhs.limit_difference_threshold &&
         lhs.constraint_difference_threshold == rhs.constraint_difference_threshold;
}

bool valid_times(const TrajectoryPoints & points)
{
  double previous = -1.0;
  for (const auto & point : points) {
    const double time = rclcpp::Duration(point.time_from_start).seconds();
    if (!std::isfinite(time) || time < 0.0 || time <= previous) {
      return false;
    }
    previous = time;
  }
  return true;
}

bool valid_stamps(
  const TrajectoryModifierData & data, const FixedVelocityLimitParameters & parameters)
{
  if (!data.current_odometry || !data.current_acceleration) {
    return false;
  }
  const auto odometry =
    rclcpp::Time(data.current_odometry->header.stamp, RCL_ROS_TIME).nanoseconds();
  const auto acceleration =
    rclcpp::Time(data.current_acceleration->header.stamp, RCL_ROS_TIME).nanoseconds();
  const auto candidate = rclcpp::Time(data.candidate_header.stamp, RCL_ROS_TIME).nanoseconds();
  if (odometry <= 0 || acceleration <= 0 || candidate <= 0) {
    return false;
  }
  const auto seconds = [](const std::int64_t nanoseconds) {
    return static_cast<double>(nanoseconds) * 1e-9;
  };
  return std::abs(seconds(odometry - acceleration)) <= parameters.stamp_difference_threshold &&
         std::abs(seconds(odometry - candidate)) <= parameters.stamp_difference_threshold &&
         data.current_odometry->header.frame_id == data.candidate_header.frame_id;
}

bool empty_uuid(const unique_identifier_msgs::msg::UUID & id)
{
  return std::all_of(id.uuid.begin(), id.uuid.end(), [](const auto byte) { return byte == 0; });
}

}  // namespace

FixedVelocityLimitParameters get_fixed_velocity_limit_parameters(
  const TrajectoryModifierParams & params)
{
  const auto & configured = params.fixed_velocity_limit_profile;
  FixedVelocityLimitParameters result;
  result.speed_deviation_threshold = configured.speed_deviation_threshold;
  result.acceleration_deviation_threshold = configured.acceleration_deviation_threshold;
  result.longitudinal_deviation_threshold = configured.longitudinal_deviation_threshold;
  result.lateral_deviation_threshold = configured.lateral_deviation_threshold;
  result.heading_deviation_threshold = configured.heading_deviation_threshold;
  result.stamp_difference_threshold = configured.stamp_difference_threshold;
  result.limit_difference_threshold = configured.limit_difference_threshold;
  result.constraint_difference_threshold = configured.constraint_difference_threshold;
  return result;
}

void FixedVelocityLimitCache::set_parameters(const FixedVelocityLimitParameters & parameters)
{
  if (!same_parameters(parameters_, parameters)) {
    clear();
    parameters_ = parameters;
  }
}

std::optional<FixedVelocityLimitCache::Projection> FixedVelocityLimitCache::project(
  const Entry & entry, const geometry_msgs::msg::Point & point)
{
  if (entry.reference.size() < 2 || entry.reference_stations.size() != entry.reference.size()) {
    return std::nullopt;
  }
  Projection best;
  best.separation = std::numeric_limits<double>::infinity();
  double other_distance = std::numeric_limits<double>::infinity();
  double other_station = 0.0;
  for (std::size_t i = 1; i < entry.reference.size(); ++i) {
    const auto & p0 = entry.reference[i - 1].pose.position;
    const auto & p1 = entry.reference[i].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double length_squared = dx * dx + dy * dy;
    if (length_squared < numerical_tolerance * numerical_tolerance) {
      continue;
    }
    const double unbounded_ratio = ((point.x - p0.x) * dx + (point.y - p0.y) * dy) / length_squared;
    double ratio = std::clamp(unbounded_ratio, 0.0, 1.0);
    if (i == 1 && i + 1 == entry.reference.size()) {
      ratio = unbounded_ratio;
    } else if (i == 1) {
      ratio = std::min(1.0, unbounded_ratio);
    } else if (i + 1 == entry.reference.size()) {
      ratio = std::max(0.0, unbounded_ratio);
    }
    const double separation =
      std::hypot(point.x - (p0.x + ratio * dx), point.y - (p0.y + ratio * dy));
    const double station = entry.reference_stations[i - 1] +
                           ratio * (entry.reference_stations[i] - entry.reference_stations[i - 1]);
    if (separation < best.separation) {
      other_distance = best.separation;
      other_station = best.station;
      best = {station, separation, std::atan2(dy, dx)};
    } else if (separation < other_distance && std::abs(station - best.station) > 1.0) {
      other_distance = separation;
      other_station = station;
    }
  }
  if (
    !std::isfinite(best.separation) ||
    (other_distance - best.separation < 0.1 && std::abs(other_station - best.station) > 1.0)) {
    return std::nullopt;
  }
  return best;
}

FixedVelocityLimitCache::ProfileState FixedVelocityLimitCache::sample(
  const Entry & entry, const double time)
{
  if (time >= entry.terminal_time) {
    return {
      entry.terminal_state.velocity, 0.0,
      entry.terminal_state.distance + entry.terminal_state.velocity * (time - entry.terminal_time)};
  }
  const auto segment = std::find_if(
    entry.segments.begin(), entry.segments.end(),
    [time](const Segment & item) { return time <= item.start_time + item.duration; });
  if (segment == entry.segments.end()) {
    return entry.terminal_state;
  }
  const double dt = std::clamp(time - segment->start_time, 0.0, segment->duration);
  return {
    segment->initial_velocity + segment->initial_acceleration * dt + 0.5 * segment->jerk * dt * dt,
    segment->initial_acceleration + segment->jerk * dt,
    segment->initial_distance + segment->initial_velocity * dt +
      0.5 * segment->initial_acceleration * dt * dt + segment->jerk * dt * dt * dt / 6.0};
}

bool FixedVelocityLimitCache::build_profile(
  Entry & entry, const TrajectoryPoints & limited, const double initial_velocity,
  const double initial_acceleration)
{
  const double jerk_bound = entry.key.jerk;
  const double deceleration = entry.key.deceleration;
  if (
    !std::isfinite(jerk_bound) || jerk_bound <= numerical_tolerance ||
    !std::isfinite(deceleration) || deceleration <= numerical_tolerance ||
    !std::isfinite(initial_velocity) || initial_velocity < 0.0 ||
    !std::isfinite(initial_acceleration)) {
    return false;
  }

  double time = 0.0;
  ProfileState state{initial_velocity, initial_acceleration, 0.0};
  const auto append = [&](const double duration, const double jerk) {
    if (!std::isfinite(duration) || !std::isfinite(jerk) || duration < 0.0) {
      return false;
    }
    if (duration <= numerical_tolerance) {
      return true;
    }
    const double new_velocity =
      state.velocity + state.acceleration * duration + 0.5 * jerk * duration * duration;
    const double new_acceleration = state.acceleration + jerk * duration;
    const double minimum_time =
      jerk > numerical_tolerance ? std::clamp(-state.acceleration / jerk, 0.0, duration) : duration;
    const double minimum_velocity =
      state.velocity + state.acceleration * minimum_time + 0.5 * jerk * minimum_time * minimum_time;
    if (
      new_velocity < -numerical_tolerance || minimum_velocity < -numerical_tolerance ||
      new_acceleration < -deceleration - numerical_tolerance ||
      std::abs(jerk) > jerk_bound + numerical_tolerance) {
      return false;
    }
    entry.segments.push_back(
      {time, duration, state.velocity, state.acceleration, state.distance, jerk});
    state.distance += state.velocity * duration + 0.5 * state.acceleration * duration * duration +
                      jerk * duration * duration * duration / 6.0;
    state.velocity = std::max(0.0, new_velocity);
    state.acceleration = new_acceleration;
    time += duration;
    return true;
  };

  // For a uniform stop limit, plan the complete stop at once so the acceleration can return to
  // zero before the first zero-speed sample. The discrete limiter's target knots do not carry
  // enough information to fit a smooth release after the vehicle has stopped.
  const bool uniform_stop = entry.key.uniform_limit && *entry.key.uniform_limit == 0.0;
  for (const auto & point : limited) {
    if (uniform_stop) {
      break;
    }
    const double target_time = rclcpp::Duration(point.time_from_start).seconds();
    const double dt = target_time - time;
    if (dt <= numerical_tolerance) {
      continue;
    }
    const double target_velocity = point.longitudinal_velocity_mps;
    if (!std::isfinite(target_velocity) || target_velocity < 0.0) {
      return false;
    }
    const double wanted_jerk =
      2.0 * (target_velocity - state.velocity - state.acceleration * dt) / (dt * dt);
    double bounded_jerk = std::clamp(wanted_jerk, -jerk_bound, jerk_bound);
    const double minimum_jerk = (-deceleration - state.acceleration) / dt;
    bounded_jerk = std::max(bounded_jerk, minimum_jerk);
    if (bounded_jerk > jerk_bound) {
      return false;
    }
    if (!append(dt, bounded_jerk)) {
      return false;
    }
  }

  // Extend the original braking maneuver to a settled speed. A new map restriction discovered
  // outside the initial horizon invalidates the cache before these samples are reused.
  const double target = entry.terminal_limit.value_or(state.velocity);
  if (state.acceleration < -deceleration) {
    if (!append((-deceleration - state.acceleration) / jerk_bound, jerk_bound)) {
      return false;
    }
  }
  if (state.velocity < target && std::abs(state.acceleration) > numerical_tolerance) {
    const double release_jerk = state.acceleration < 0.0 ? jerk_bound : -jerk_bound;
    if (!append(std::abs(state.acceleration) / jerk_bound, release_jerk)) {
      return false;
    }
  }
  const double loss = std::max(0.0, state.velocity - target);
  const double minimum_peak = std::max(0.0, -state.acceleration);
  const double peak = std::clamp(
    std::sqrt(jerk_bound * loss + state.acceleration * state.acceleration / 2.0), minimum_peak,
    deceleration);
  const double ramp_down = (state.acceleration + peak) / jerk_bound;
  const double triangular_loss =
    (2.0 * peak * peak - state.acceleration * state.acceleration) / (2.0 * jerk_bound);
  const double hold =
    peak > numerical_tolerance ? std::max(0.0, (loss - triangular_loss) / peak) : 0.0;
  if (
    !append(ramp_down, -jerk_bound) || !append(hold, 0.0) ||
    !append(peak / jerk_bound, jerk_bound)) {
    return false;
  }
  if (std::abs(state.acceleration) > numerical_tolerance) {
    return false;
  }
  entry.terminal_time = time;
  entry.terminal_state = state;
  return true;
}

bool FixedVelocityLimitCache::tracking_is_consistent(
  const Entry & entry, const TrajectoryModifierData & data, const double elapsed,
  const FixedVelocityLimitParameters & parameters)
{
  const auto expected = sample(entry, elapsed);
  const auto & odometry = *data.current_odometry;
  const auto actual = project(entry, odometry.pose.pose.position);
  if (!actual || actual->separation > parameters.lateral_deviation_threshold) {
    return false;
  }
  const double actual_heading = heading_of(odometry.pose.pose.orientation);
  const double measured_acceleration = data.current_acceleration->accel.accel.linear.x;
  return std::isfinite(measured_acceleration) &&
         std::abs(odometry.twist.twist.linear.x - expected.velocity) <=
           parameters.speed_deviation_threshold &&
         std::abs(measured_acceleration - expected.acceleration) <=
           parameters.acceleration_deviation_threshold &&
         std::abs(actual->station - (entry.anchor_station + expected.distance)) <=
           parameters.longitudinal_deviation_threshold &&
         heading_difference(actual_heading, actual->heading) <=
           parameters.heading_deviation_threshold;
}

bool FixedVelocityLimitCache::limits_are_consistent(
  const Entry & entry, const TrajectoryPoints & points, const LimitFunction & velocity_limit,
  const FixedVelocityLimitParameters & parameters)
{
  if (!entry.key.spatial) {
    return true;
  }
  for (const auto & point : points) {
    const auto projection = project(entry, point.pose.position);
    if (!projection || projection->separation > parameters.lateral_deviation_threshold) {
      return false;
    }
    std::optional<double> old_limit;
    if (projection->station > entry.reference_stations.back() + numerical_tolerance) {
      old_limit = entry.terminal_limit;
    } else {
      const auto upper = std::upper_bound(
        entry.reference_stations.begin(), entry.reference_stations.end(), projection->station);
      const std::size_t end = std::clamp<std::size_t>(
        static_cast<std::size_t>(std::distance(entry.reference_stations.begin(), upper)), 1,
        entry.reference.size() - 1);
      const double length = entry.reference_stations[end] - entry.reference_stations[end - 1];
      const double ratio =
        length > numerical_tolerance
          ? std::clamp((projection->station - entry.reference_stations[end - 1]) / length, 0.0, 1.0)
          : 0.0;
      geometry_msgs::msg::Point reference_position;
      const auto & p0 = entry.reference[end - 1].pose.position;
      const auto & p1 = entry.reference[end].pose.position;
      reference_position.x = p0.x + ratio * (p1.x - p0.x);
      reference_position.y = p0.y + ratio * (p1.y - p0.y);
      reference_position.z = p0.z + ratio * (p1.z - p0.z);
      old_limit = velocity_limit(reference_position);
    }
    if (!same_limit(
          old_limit, velocity_limit(point.pose.position), parameters.limit_difference_threshold)) {
      return false;
    }
  }
  return true;
}

bool FixedVelocityLimitCache::apply_profile(
  const Entry & entry, TrajectoryPoints & points, const double elapsed,
  const LimitFunction & velocity_limit, const FixedVelocityLimitParameters & parameters)
{
  if (!valid_times(points) || !limits_are_consistent(entry, points, velocity_limit, parameters)) {
    return false;
  }
  const auto original = points;
  std::vector<double> times;
  times.reserve(points.size());
  bool modified = false;
  for (auto & point : points) {
    const double time = rclcpp::Duration(point.time_from_start).seconds();
    times.push_back(time);
    const auto state = sample(entry, elapsed + time);
    if (
      !std::isfinite(state.velocity) || !std::isfinite(state.acceleration) ||
      state.velocity > point.longitudinal_velocity_mps + parameters.limit_difference_threshold) {
      return false;
    }
    if (state.velocity < point.longitudinal_velocity_mps - parameters.limit_difference_threshold) {
      modified = true;
    }
    point.longitudinal_velocity_mps = static_cast<float>(
      std::min(state.velocity, static_cast<double>(point.longitudinal_velocity_mps)));
    point.acceleration_mps2 = static_cast<float>(state.acceleration);
  }
  if (!modified) {
    points = original;
    return false;
  }
  retime_velocity_profile(points, original, times);
  return limits_are_consistent(entry, points, velocity_limit, parameters);
}

void FixedVelocityLimitCache::extend_reference(
  Entry & entry, const TrajectoryPoints & points, const FixedVelocityLimitParameters & parameters)
{
  if (entry.reference.size() < 2) {
    return;
  }
  for (const auto & point : points) {
    const auto & last = entry.reference.back().pose.position;
    const auto & before = entry.reference[entry.reference.size() - 2].pose.position;
    const double gap = distance2d(last, point.pose.position);
    if (gap <= 0.05 || gap > 5.0) {
      continue;
    }
    const double old_heading = std::atan2(last.y - before.y, last.x - before.x);
    const double new_heading =
      std::atan2(point.pose.position.y - last.y, point.pose.position.x - last.x);
    if (heading_difference(old_heading, new_heading) > parameters.heading_deviation_threshold) {
      continue;
    }
    entry.reference_stations.push_back(entry.reference_stations.back() + gap);
    entry.reference.push_back(point);
  }
}

VelocityLimitResult FixedVelocityLimitCache::process(
  TrajectoryPoints & points, const TrajectoryModifierData & data, const FixedVelocityLimitKey & key,
  const LimitFunction & velocity_limit)
{
  const auto fallback = [&]() {
    ++calculation_count_;
    VelocityLimitOptions options;
    options.current_ego_velocity = data.current_odometry->twist.twist.linear.x;
    options.current_ego_acceleration = data.current_acceleration->accel.accel.linear.x;
    return apply_velocity_limits(points, key.deceleration, key.jerk, velocity_limit, options);
  };

  if (
    points.empty() || !data.current_odometry || !data.current_acceleration ||
    !valid_times(points)) {
    entries_.clear();
    return {ProcessingResult::Unchanged, "Fixed velocity profile requires valid inputs"};
  }
  const auto finite_point = [](const TrajectoryPoint & point) {
    const auto & position = point.pose.position;
    return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
           std::isfinite(point.longitudinal_velocity_mps) &&
           point.longitudinal_velocity_mps >= 0.0F && std::isfinite(point.acceleration_mps2);
  };
  if (
    !std::isfinite(key.deceleration) || key.deceleration < 0.0 || !std::isfinite(key.jerk) ||
    key.jerk < 0.0 || (key.uniform_limit && !valid_limit(key.uniform_limit)) ||
    !std::isfinite(data.current_odometry->twist.twist.linear.x) ||
    !std::isfinite(data.current_acceleration->accel.accel.linear.x) ||
    !std::all_of(points.begin(), points.end(), finite_point)) {
    entries_.clear();
    return {ProcessingResult::Unchanged, "Fixed velocity profile requires finite kinematic data"};
  }
  if (data.candidate_index == 0) {
    entries_.erase(
      std::remove_if(
        entries_.begin(), entries_.end(),
        [&](const Entry & entry) {
          return entry.last_seen_batch + 1 < data.candidate_batch_sequence;
        }),
      entries_.end());
  }
  if (
    !valid_stamps(data, parameters_) || data.candidate_batch_sequence == 0 ||
    (empty_uuid(data.candidate_generator_id) && data.candidate_count != 1)) {
    entries_.clear();
    return fallback();
  }
  const auto stamp = rclcpp::Time(data.current_odometry->header.stamp, RCL_ROS_TIME).nanoseconds();

  // Keep candidates with a shared generator UUID separate. An ordinal match is accepted only
  // when geometry agrees; a reordered batch can match a unique geometrically close entry.
  std::optional<std::size_t> matched_index;
  double best_score = std::numeric_limits<double>::infinity();
  double runner_up = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < entries_.size(); ++index) {
    const auto & entry = entries_[index];
    if (
      entry.last_seen_batch == data.candidate_batch_sequence ||
      entry.generator_id != data.candidate_generator_id ||
      entry.frame_id != data.candidate_header.frame_id) {
      continue;
    }
    double score = 0.0;
    bool valid = true;
    for (const std::size_t point_index :
         std::array<std::size_t, 3>{0, points.size() / 4, points.size() / 2}) {
      const auto projection = project(entry, points[point_index].pose.position);
      if (!projection || projection->separation > parameters_.lateral_deviation_threshold) {
        valid = false;
        break;
      }
      score += projection->separation;
    }
    if (!valid) {
      continue;
    }
    if (entry.candidate_index == data.candidate_index) {
      score -= 0.05;
    }
    if (score < best_score) {
      runner_up = best_score;
      best_score = score;
      matched_index = index;
    } else {
      runner_up = std::min(runner_up, score);
    }
  }
  if (runner_up - best_score < 0.1) {
    matched_index.reset();
  }

  if (matched_index) {
    auto & entry = entries_[*matched_index];
    const double elapsed = static_cast<double>(stamp - entry.anchor_nanoseconds) * 1e-9;
    if (
      same_key(entry.key, key, parameters_) && stamp > entry.last_odometry_nanoseconds &&
      elapsed >= 0.0 && limits_are_consistent(entry, points, velocity_limit, parameters_)) {
      if (tracking_is_consistent(entry, data, elapsed, parameters_)) {
        auto candidate = points;
        if (apply_profile(entry, candidate, elapsed, velocity_limit, parameters_)) {
          extend_reference(entry, points, parameters_);
          points = std::move(candidate);
          entry.last_odometry_nanoseconds = stamp;
          entry.last_seen_batch = data.candidate_batch_sequence;
          entry.candidate_index = data.candidate_index;
          return {ProcessingResult::Modified, {}, true};
        }
      }
    }
    // An entry that failed any validation must not survive a failed rebuild.
    entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(*matched_index));
    matched_index.reset();
  }

  const auto original = points;
  const auto result = fallback();
  if (!result.profile_generated || original.size() < 2 || !valid_stamps(data, parameters_)) {
    return result;
  }
  Entry new_entry;
  new_entry.generator_id = data.candidate_generator_id;
  new_entry.candidate_index = data.candidate_index;
  new_entry.last_seen_batch = data.candidate_batch_sequence;
  new_entry.anchor_nanoseconds = stamp;
  new_entry.last_odometry_nanoseconds = stamp;
  new_entry.frame_id = data.candidate_header.frame_id;
  new_entry.key = key;
  new_entry.reference = original;
  new_entry.reference_stations.resize(original.size(), 0.0);
  for (std::size_t i = 1; i < original.size(); ++i) {
    new_entry.reference_stations[i] =
      new_entry.reference_stations[i - 1] +
      distance2d(original[i - 1].pose.position, original[i].pose.position);
  }
  const auto anchor = project(new_entry, data.current_odometry->pose.pose.position);
  if (!anchor || anchor->separation > parameters_.lateral_deviation_threshold) {
    return result;
  }
  new_entry.anchor_station = anchor->station;
  new_entry.terminal_limit = valid_limit(velocity_limit(original.back().pose.position));
  if (!build_profile(
        new_entry, points, data.current_odometry->twist.twist.linear.x,
        data.current_acceleration->accel.accel.linear.x)) {
    return result;
  }
  auto fixed_output = original;
  if (!apply_profile(new_entry, fixed_output, 0.0, velocity_limit, parameters_)) {
    return result;
  }
  points = std::move(fixed_output);
  ++profile_build_count_;
  if (entries_.size() < maximum_cached_candidates) {
    entries_.push_back(std::move(new_entry));
  }
  return {ProcessingResult::Modified, {}, true};
}

}  // namespace autoware::trajectory_modifier::plugin::detail
