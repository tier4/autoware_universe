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

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spherical_linear_interpolation.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin::detail
{
namespace
{
constexpr double k_time_step = 0.01;      // [s] integration step of the forward simulation
constexpr double k_tracking_gain = 10.0;  // [1/s] gain of the release-aware velocity tracking
constexpr double k_braking_margin = 0.5;  // [m] added to braking distances before triggering
// Braking for a bound starts when this ratio of the deceleration limit is needed, which leaves a
// reserve to correct deviations without exceeding the limit. Since every cycle re-plans without
// memory, braking continues while the ego decelerates and the lower ratio is needed; the wide band
// keeps a braking manoeuvre from toggling on and off between cycles.
constexpr double k_start_ratio = 0.95;
constexpr double k_continue_ratio = 0.5;
constexpr double k_decelerating = -0.05;    // [m/s^2] acceleration below which the ego decelerates
constexpr double k_limit_tolerance = 1e-3;  // [m/s] input velocity tolerated above a limit
constexpr double k_limit_change_epsilon = 1e-6;  // [m/s] negligible map limit change

struct LongitudinalState
{
  double s{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

/// @brief Velocity reached after releasing the acceleration to zero with the given jerk.
double release_velocity(const double velocity, const double acceleration, const double jerk)
{
  return velocity + acceleration * std::abs(acceleration) / (2.0 * jerk);
}

/// @brief Constant-jerk motion over a duration.
LongitudinalState integrate(
  const LongitudinalState & state, const double jerk, const double duration)
{
  const double t = duration;
  return {
    state.s + state.velocity * t + 0.5 * state.acceleration * t * t + jerk * t * t * t / 6.0,
    state.velocity + state.acceleration * t + 0.5 * jerk * t * t, state.acceleration + jerk * t};
}

using LimitFunction = std::function<std::optional<double>(const geometry_msgs::msg::Point &)>;

double resolve_limit(
  const LimitFunction & velocity_limit, const geometry_msgs::msg::Point & position)
{
  const auto limit = velocity_limit(position);
  return limit && std::isfinite(*limit) && *limit >= 0.0 ? *limit
                                                         : std::numeric_limits<double>::infinity();
}

bool limits_differ(const double lhs, const double rhs)
{
  // Resolved limits are finite and non-negative, or positive infinity for an absent limit.
  if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
    return std::isfinite(lhs) != std::isfinite(rhs);
  }
  return std::abs(lhs - rhs) > k_limit_change_epsilon;
}

/// @brief Ratio along a segment where the limit changes from its value at the segment start.
double find_limit_change(
  const LimitFunction & velocity_limit, const geometry_msgs::msg::Point & from,
  const geometry_msgs::msg::Point & to, const double start_limit)
{
  double low = 0.0;
  double high = 1.0;
  // Twelve bisections locate the change within 1/4096 of the segment length.
  for (int iteration = 0; iteration < 12; ++iteration) {
    const double middle = 0.5 * (low + high);
    geometry_msgs::msg::Point position;
    position.x = autoware::interpolation::lerp(from.x, to.x, middle);
    position.y = autoware::interpolation::lerp(from.y, to.y, middle);
    position.z = autoware::interpolation::lerp(from.z, to.z, middle);
    (limits_differ(resolve_limit(velocity_limit, position), start_limit) ? high : low) = middle;
  }
  return high;
}

/// @brief Bounds along the input polyline, sampled at nodes: the input points plus the positions
/// where the limit changes between two points, located by bisection. The limit of a node holds
/// until the next node. A velocity drop at the last input point is ignored (horizon end, not a
/// stop).
struct PathBounds
{
  std::vector<double> s;
  std::vector<double> input;
  std::vector<double> input_acceleration;  ///< Constant acceleration of the input after a node.
  std::vector<double> limit;
  std::vector<double> lowest_limit_after;
  /// Nodes where the cap stops decreasing (zone entries, stops, bottoms of dips). Braking ends
  /// there, while a decreasing stretch before one is satisfied by braking for it or followed.
  std::vector<std::uint8_t> local_minimum;

  [[nodiscard]] std::size_t size() const { return s.size(); }

  /// @brief Lower of the input velocity and the limit at a node; must hold from that node on.
  [[nodiscard]] double cap(const std::size_t index) const
  {
    return std::min(input[index], limit[index]);
  }

  [[nodiscard]] double input_at(const double arc_length, const std::size_t node) const
  {
    if (node + 1 >= size()) {
      return input.back();
    }
    const double length = s[node + 1] - s[node];
    const double ratio = length > 0.0 ? std::clamp((arc_length - s[node]) / length, 0.0, 1.0) : 1.0;
    return autoware::interpolation::lerp(input[node], input[node + 1], ratio);
  }

  void add_node(
    const double arc_length, const double velocity, const double acceleration, const double max)
  {
    s.push_back(arc_length);
    input.push_back(velocity);
    input_acceleration.push_back(acceleration);
    limit.push_back(max);
  }
};

PathBounds make_path_bounds(
  const TrajectoryPoints & points, const std::vector<double> & limits,
  const LimitFunction & velocity_limit)
{
  const auto count = points.size();
  std::vector<double> arc_lengths(count, 0.0);
  std::vector<double> velocities(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    if (i > 0) {
      const auto & p0 = points[i - 1].pose.position;
      const auto & p1 = points[i].pose.position;
      arc_lengths[i] = arc_lengths[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    }
    velocities[i] = std::max(0.0, static_cast<double>(points[i].longitudinal_velocity_mps));
  }
  if (count > 1) {
    velocities[count - 1] = std::max(velocities[count - 1], velocities[count - 2]);
  }

  PathBounds bounds;
  bounds.s.reserve(count * 2);
  bounds.input.reserve(count * 2);
  bounds.input_acceleration.reserve(count * 2);
  bounds.limit.reserve(count * 2);
  for (std::size_t i = 0; i < count; ++i) {
    const double length = i + 1 < count ? arc_lengths[i + 1] - arc_lengths[i] : 0.0;
    const double acceleration =
      length > 1e-6
        ? (velocities[i + 1] * velocities[i + 1] - velocities[i] * velocities[i]) / (2.0 * length)
        : 0.0;
    bounds.add_node(arc_lengths[i], velocities[i], acceleration, limits[i]);
    if (i + 1 < count && limits_differ(limits[i], limits[i + 1])) {
      const double ratio = find_limit_change(
        velocity_limit, points[i].pose.position, points[i + 1].pose.position, limits[i]);
      bounds.add_node(
        arc_lengths[i] + ratio * length,
        autoware::interpolation::lerp(velocities[i], velocities[i + 1], ratio), acceleration,
        limits[i + 1]);
    }
  }

  const auto nodes = bounds.size();
  bounds.lowest_limit_after.assign(nodes, std::numeric_limits<double>::infinity());
  for (std::size_t i = nodes - 1; i > 0; --i) {
    bounds.lowest_limit_after[i - 1] = std::min(bounds.limit[i], bounds.lowest_limit_after[i]);
  }
  constexpr double epsilon = 1e-6;
  bounds.local_minimum.assign(nodes, 0);
  for (std::size_t i = 1; i < nodes; ++i) {
    const bool decreasing_into = bounds.cap(i) < bounds.cap(i - 1) - epsilon;
    const bool not_decreasing_after = i + 1 == nodes || bounds.cap(i + 1) > bounds.cap(i) - epsilon;
    bounds.local_minimum[i] = decreasing_into && not_decreasing_after;
  }
  return bounds;
}

/// @brief Forward simulation of the limited profile. Each step follows the input and the limit at
/// the current position and, for each local minimum of the bounds ahead, starts braking at the
/// latest step that still reaches it with the deceleration and jerk limits. A triggered minimum
/// stays latched until it is passed. The deceleration limit only bounds the braking for the
/// limits: following the input, the profile brakes as hard as the input does. The jerk limit
/// always holds.
class LimitedProfileSimulation
{
public:
  LimitedProfileSimulation(
    const PathBounds & bounds, const VelocityLimitConstraints & constraints,
    const LongitudinalState & initial)
  : bounds_{bounds}, constraints_{constraints}, state_{initial}, latched_(bounds.size(), 0)
  {
  }

  [[nodiscard]] const LongitudinalState & state() const { return state_; }

  void advance(const double dt)
  {
    while (segment_ + 1 < bounds_.size() && bounds_.s[segment_ + 1] <= state_.s) {
      ++segment_;
    }
    // The hardest input braking stays allowed until the profile stops braking, so that following
    // it does not weaken past the input braking (e.g. after a stop that the jerk limit overshoots).
    const double input_deceleration =
      std::max(constraints_.max_deceleration, -bounds_.input_acceleration[segment_]);
    input_deceleration_ = state_.acceleration < k_decelerating
                            ? std::max(input_deceleration_, input_deceleration)
                            : input_deceleration;
    double command = local_command();
    const auto predicted = step(command, dt);
    command = std::min(command, lookahead_command(predicted));
    state_ = step(command, dt);
  }

private:
  /// @brief Brake above the bounds at the current position, hold between them and the lowest
  /// limit ahead, and accelerate below it. Following the input brakes as hard as the input does
  /// when that exceeds the deceleration limit.
  [[nodiscard]] double local_command() const
  {
    const double input = std::clamp(
      std::max(input_command(), release_command()), -input_deceleration_,
      constraints_.max_acceleration);
    const double limit =
      std::clamp(limit_command(), -constraints_.max_deceleration, constraints_.max_acceleration);
    return std::min(input, limit);
  }

  /// @brief Follow the input velocity at the current position, with its acceleration as
  /// feedforward. The release term is relative to that acceleration so catching up does not
  /// overshoot.
  [[nodiscard]] double input_command() const
  {
    const double acceleration = bounds_.input_acceleration[segment_];
    const double relative = state_.acceleration - acceleration;
    const double velocity =
      state_.velocity + relative * std::abs(relative) / (2.0 * constraints_.max_jerk);
    return acceleration + k_tracking_gain * (bounds_.input_at(state_.s, segment_) - velocity);
  }

  /// @brief Where the input braking ends before a release of the braking with the jerk limit
  /// would complete, land on the lowest input velocity until then instead of following the input
  /// braking to its end and falling below the input while releasing. None otherwise.
  [[nodiscard]] double release_command() const
  {
    constexpr double none = -std::numeric_limits<double>::infinity();
    if (state_.acceleration >= 0.0) {
      return none;
    }
    const double jerk = constraints_.max_jerk;
    const double velocity = release_velocity(state_.velocity, state_.acceleration, jerk);
    const double duration = -state_.acceleration / jerk;
    // The ego stops before the release completes when the release velocity is negative.
    const double end =
      velocity > 0.0 ? integrate(state_, jerk, duration).s : state_.s + state_.velocity * duration;
    std::size_t node = segment_;
    double lowest = bounds_.input_at(state_.s, node);
    while (node + 1 < bounds_.size() && bounds_.s[node + 1] <= end) {
      ++node;
      lowest = std::min(lowest, bounds_.input[node]);
    }
    // While the input keeps braking past the release, following it leads the release.
    if (bounds_.input_acceleration[node] < k_decelerating) {
      return none;
    }
    lowest = std::min(lowest, bounds_.input_at(end, node));
    return k_tracking_gain * (lowest - velocity);
  }

  /// @brief Brake above the limit at the current position, do not accelerate above the lowest
  /// limit ahead, and leave the acceleration free below it.
  [[nodiscard]] double limit_command() const
  {
    const double limit = bounds_.limit[segment_];
    const double ceiling = std::min(limit, bounds_.lowest_limit_after[segment_]);
    const double velocity =
      release_velocity(state_.velocity, state_.acceleration, constraints_.max_jerk);
    if (velocity > limit) {
      return k_tracking_gain * (limit - velocity);
    }
    return velocity >= ceiling ? 0.0 : k_tracking_gain * (ceiling - velocity);
  }

  /// @brief Lowest braking command of the bounds ahead that require braking now. Braking starts
  /// as if limited by the deceleration limit; when that is not enough to reach a bound set by the
  /// input, the braking may be as hard as the input's own braking before that bound.
  [[nodiscard]] double lookahead_command(const LongitudinalState & predicted)
  {
    const auto braking_distance = [this, &predicted](
                                    const double target, const double deceleration) {
      return jerk_limited_braking_distance(
        predicted.velocity, predicted.acceleration, target, deceleration, constraints_.max_jerk);
    };
    const double trigger_deceleration =
      (state_.acceleration < k_decelerating ? k_continue_ratio : k_start_ratio) *
      constraints_.max_deceleration;
    const double reach = braking_distance(0.0, constraints_.max_deceleration) + k_braking_margin;
    double command = std::numeric_limits<double>::infinity();
    double lowest = std::numeric_limits<double>::infinity();
    double input_deceleration = input_deceleration_;  // Hardest input braking up to j.
    for (std::size_t j = segment_ + 1; j < bounds_.size(); ++j) {
      input_deceleration = std::max(input_deceleration, -bounds_.input_acceleration[j - 1]);
      if (!bounds_.local_minimum[j] && !latched_[j]) {
        continue;
      }
      const double cap = bounds_.cap(j);
      if (!latched_[j]) {
        // A bound beyond the stopping distance, or above a lower bound before it, cannot require
        // braking yet. Latched bounds are always nearer than the stopping distance when latched.
        if (bounds_.s[j] - predicted.s > reach && !any_latched_after(j)) {
          break;
        }
        if (cap >= lowest) {
          continue;
        }
        lowest = cap;
        const double distance = braking_distance(cap, trigger_deceleration);
        if (distance <= 0.0 || distance + k_braking_margin < bounds_.s[j] - predicted.s) {
          continue;
        }
        latched_[j] = 1;
      }
      lowest = std::min(lowest, cap);
      const bool input_bound = bounds_.input[j] <= bounds_.limit[j];
      command = std::min(
        command,
        braking_command(j, input_bound ? input_deceleration : constraints_.max_deceleration));
    }
    return command;
  }

  /// @brief Decelerate as much as needed, up to `max_deceleration`, to reach a bound where it
  /// begins, then release onto it.
  [[nodiscard]] double braking_command(const std::size_t index, const double max_deceleration) const
  {
    const double target = bounds_.cap(index);
    const double available = bounds_.s[index] - state_.s - k_braking_margin;
    return std::max(
      -required_deceleration(target, available, max_deceleration),
      landing_command(target, max_deceleration));
  }

  /// @brief Smallest deceleration, up to `max_deceleration`, whose jerk-limited braking profile
  /// from the current state reaches the target within the available distance.
  [[nodiscard]] double required_deceleration(
    const double target, const double available, const double max_deceleration) const
  {
    const auto distance = [this, target](const double deceleration) {
      return jerk_limited_braking_distance(
        state_.velocity, state_.acceleration, target, deceleration, constraints_.max_jerk);
    };
    double high = max_deceleration;
    if (available <= 0.0 || distance(high) >= available) {
      return high;
    }
    double low = 0.0;
    // Twenty bisections leave at most max_deceleration / 2^20 of uncertainty.
    for (int iteration = 0; iteration < 20; ++iteration) {
      const double middle = 0.5 * (low + high);
      (distance(middle) <= available ? high : low) = middle;
    }
    return high;
  }

  [[nodiscard]] bool any_latched_after(const std::size_t index) const
  {
    return std::any_of(
      latched_.begin() + static_cast<std::ptrdiff_t>(index), latched_.end(),
      [](const std::uint8_t latched) { return latched != 0; });
  }

  /// @brief Brake toward a target, up to `max_deceleration`, and release so that the acceleration
  /// reaches zero there.
  [[nodiscard]] double landing_command(const double target, const double max_deceleration) const
  {
    const double velocity =
      release_velocity(state_.velocity, state_.acceleration, constraints_.max_jerk);
    return std::clamp(k_tracking_gain * (target - velocity), -max_deceleration, 0.0);
  }

  /// @brief State after one step toward the commanded acceleration under the jerk limit.
  [[nodiscard]] LongitudinalState step(const double command, const double dt) const
  {
    if (
      state_.velocity <= 0.0 && state_.acceleration <= 0.0 &&
      (state_.acceleration < 0.0 || command <= 0.0)) {
      // Keep the vehicle stopped while releasing residual braking acceleration, or while no
      // positive acceleration is commanded. Once released, a positive command may start it.
      return {state_.s, 0.0, std::min(0.0, state_.acceleration + constraints_.max_jerk * dt)};
    }
    const double jerk_step = constraints_.max_jerk * dt;
    const double acceleration =
      std::clamp(command, state_.acceleration - jerk_step, state_.acceleration + jerk_step);
    auto next = integrate(state_, (acceleration - state_.acceleration) / dt, dt);
    if (next.velocity < 0.0) {
      next.velocity = 0.0;
    }
    return next;
  }

  const PathBounds & bounds_;
  const VelocityLimitConstraints & constraints_;
  LongitudinalState state_;
  std::vector<std::uint8_t> latched_;
  std::size_t segment_{0};
  double input_deceleration_{constraints_.max_deceleration};  ///< Allowed to follow the input.
};

/// @brief Move representable poses along the input polyline and return their count.
std::size_t resample_poses(
  TrajectoryPoints & points, const std::vector<geometry_msgs::msg::Pose> & original,
  const std::vector<double> & times)
{
  const auto count = points.size();
  std::vector<double> original_s(original.size(), 0.0);
  for (std::size_t i = 1; i < original.size(); ++i) {
    const auto & p0 = original[i - 1].position;
    const auto & p1 = original[i].position;
    original_s[i] = original_s[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
  }

  std::vector<double> new_s(count, 0.0);
  for (std::size_t i = 1; i < count; ++i) {
    const double dt = times[i] - times[i - 1];
    new_s[i] =
      new_s[i - 1] +
      0.5 * (points[i - 1].longitudinal_velocity_mps + points[i].longitudinal_velocity_mps) * dt;
  }

  // The polyline cannot represent motion beyond its end. Leave those points to the caller's
  // stop or moving-horizon policy instead of pinning moving points to the last pose.
  std::size_t valid_count = count;
  for (std::size_t i = 1; i < count; ++i) {
    if (new_s[i] > original_s.back() + 1e-3) {
      valid_count = i;
      break;
    }
  }

  std::size_t segment = 0;
  for (std::size_t i = 1; i < valid_count; ++i) {
    while (segment + 1 < original.size() && original_s[segment + 1] <= new_s[i]) {
      ++segment;
    }

    if (segment + 1 == original.size()) {
      points[i].pose = original.back();
    } else {
      const double segment_length = original_s[segment + 1] - original_s[segment];
      const double ratio = (new_s[i] - original_s[segment]) / std::max(segment_length, 1e-6);

      const auto & p0 = original[segment];
      const auto & p1 = original[segment + 1];
      auto & pose = points[i].pose;

      pose.position.x = autoware::interpolation::lerp(p0.position.x, p1.position.x, ratio);
      pose.position.y = autoware::interpolation::lerp(p0.position.y, p1.position.y, ratio);
      pose.position.z = autoware::interpolation::lerp(p0.position.z, p1.position.z, ratio);
      pose.orientation =
        autoware::interpolation::lerpOrientation(p0.orientation, p1.orientation, ratio);
    }
  }
  return valid_count;
}
}  // namespace

VelocityLimitOptions make_velocity_limit_options(const TrajectoryModifierData & data)
{
  VelocityLimitOptions options;
  if (data.current_odometry) {
    options.current_ego_velocity = data.current_odometry->twist.twist.linear.x;
  }
  if (data.current_acceleration) {
    options.current_ego_acceleration = data.current_acceleration->accel.accel.linear.x;
  }
  return options;
}

double jerk_limited_braking_distance(
  const double velocity, const double acceleration, const double target_velocity,
  const double max_deceleration, const double max_jerk)
{
  if (release_velocity(velocity, acceleration, max_jerk) <= target_velocity) {
    return 0.0;
  }
  // Triangular profile: jerk toward the peak deceleration, then release to zero at the target.
  // (a0^2 - 2 peak^2) / (2 jerk) = target - velocity, saturated at the deceleration limit.
  const double peak = std::max(
    -max_deceleration,
    -std::sqrt(
      0.5 * (acceleration * acceleration + 2.0 * max_jerk * (velocity - target_velocity))));
  LongitudinalState state{0.0, velocity, acceleration};
  state = integrate(
    state, peak < acceleration ? -max_jerk : max_jerk, std::abs(peak - acceleration) / max_jerk);
  const double release_loss = peak * peak / (2.0 * max_jerk);
  const double hold = std::max(0.0, (state.velocity - release_loss - target_velocity) / -peak);
  state = integrate(state, 0.0, hold);
  state = integrate(state, max_jerk, -peak / max_jerk);
  return state.s;
}

VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, const VelocityLimitConstraints & constraints,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit,
  const VelocityLimitOptions & options)
{
  if (points.empty()) {
    return {};
  }
  const bool valid_constraints =
    std::isfinite(constraints.max_acceleration) && constraints.max_acceleration >= 0.0 &&
    std::isfinite(constraints.max_deceleration) && constraints.max_deceleration > 0.0 &&
    std::isfinite(constraints.max_jerk) && constraints.max_jerk > 0.0;
  if (!valid_constraints) {
    return VelocityLimitResult{
      ProcessingResult::Unchanged,
      "Velocity limiting requires a finite non-negative acceleration and positive deceleration "
      "and jerk"};
  }

  const auto count = points.size();
  std::vector<double> times(count, 0.0);
  for (std::size_t i = 0; i < count; ++i) {
    times[i] = rclcpp::Duration(points[i].time_from_start).seconds();
    if (!std::isfinite(times[i]) || times[i] < 0.0 || (i > 0 && times[i] <= times[i - 1])) {
      return VelocityLimitResult{
        ProcessingResult::Unchanged,
        "Velocity limiting requires finite, non-negative, strictly increasing timestamps"};
    }
  }

  // Resolve the limits on the input path before any pose changes.
  std::vector<double> limits(count, std::numeric_limits<double>::infinity());
  bool needs_limiting = false;
  for (std::size_t i = 0; i < count; ++i) {
    limits[i] = resolve_limit(velocity_limit, points[i].pose.position);
    needs_limiting |= points[i].longitudinal_velocity_mps > limits[i] + k_limit_tolerance;
  }
  if (!needs_limiting) {
    return {ProcessingResult::Unchanged, {}};
  }

  const auto bounds = make_path_bounds(points, limits, velocity_limit);
  LongitudinalState initial;
  initial.velocity = std::max(
    0.0, options.current_ego_velocity && std::isfinite(*options.current_ego_velocity)
           ? *options.current_ego_velocity
           : static_cast<double>(points.front().longitudinal_velocity_mps));
  // A measured acceleration outside the limits (e.g. noise) is clamped so that the profile never
  // violates them. Braking as hard as the input at its start is kept.
  const double initial_deceleration =
    std::max(constraints.max_deceleration, -bounds.input_acceleration.front());
  initial.acceleration =
    options.current_ego_acceleration.value_or(points.front().acceleration_mps2);
  initial.acceleration =
    std::isfinite(initial.acceleration)
      ? std::clamp(initial.acceleration, -initial_deceleration, constraints.max_acceleration)
      : 0.0;

  std::vector<geometry_msgs::msg::Pose> original_poses;
  original_poses.reserve(count);
  std::optional<double> input_stop_s;
  double input_path_s = 0.0;
  for (std::size_t i = 1; i < count; ++i) {
    const auto & p0 = points[i - 1].pose.position;
    const auto & p1 = points[i].pose.position;
    input_path_s += std::hypot(p1.x - p0.x, p1.y - p0.y, p1.z - p0.z);
    if (
      !input_stop_s && i + 1 < count &&
      points[i - 1].longitudinal_velocity_mps > k_fixed_profile_stop_velocity_threshold_mps &&
      points[i].longitudinal_velocity_mps <= k_fixed_profile_stop_velocity_threshold_mps) {
      input_stop_s = input_path_s;
    }
  }
  const bool has_zero_limit = std::any_of(limits.begin(), limits.end(), [](const double limit) {
    return limit <= k_fixed_profile_stop_velocity_threshold_mps;
  });
  for (const auto & point : points) {
    original_poses.push_back(point.pose);
  }
  // Build the proposed profile separately. An infeasible stop or horizon must leave the plugin
  // input byte-for-byte unchanged when the limit is skipped.
  auto output = points;
  LimitedProfileSimulation simulation(bounds, constraints, initial);
  double time = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    while (time < times[i] - 1e-9) {
      const double dt = std::min(k_time_step, times[i] - time);
      simulation.advance(dt);
      time += dt;
    }
    output[i].longitudinal_velocity_mps = static_cast<float>(simulation.state().velocity);
    output[i].acceleration_mps2 = static_cast<float>(simulation.state().acceleration);
  }

  const auto valid_count = resample_poses(output, original_poses, times);
  double planned_travel = 0.0;
  for (std::size_t i = 1; i < count; ++i) {
    planned_travel +=
      0.5 * (output[i - 1].longitudinal_velocity_mps + output[i].longitudinal_velocity_mps) *
      (times[i] - times[i - 1]);
  }
  const bool missed_input_stop = input_stop_s && planned_travel > *input_stop_s + 0.1;
  if (
    missed_input_stop ||
    (valid_count < count && (input_stop_s || has_zero_limit || valid_count < 3))) {
    // The requested profile does not fit the available input geometry. Do not publish any part
    // of it; the input's stop and horizon are owned by the upstream planner.
    return {
      ProcessingResult::Unchanged,
      "Velocity profile needs " + std::to_string(planned_travel) + " m but the input has " +
        std::to_string(input_stop_s.value_or(input_path_s)) +
        (input_stop_s ? " m to its stop" : " m of path") + "; leaving input unchanged"};
  }
  if (valid_count < count) {
    output.resize(valid_count);
    points.swap(output);
    return {ProcessingResult::Modified, "Velocity profile truncated at the input path horizon"};
  }
  points.swap(output);
  return {ProcessingResult::Modified, {}};
}

namespace
{
struct PlanSample
{
  double velocity{0.0};
  double acceleration{0.0};
  geometry_msgs::msg::Point position;
};

/// @brief State of a plan `elapsed` seconds after the ego state it was planned from; none beyond
/// its last point. Where the velocities follow the trapezoidal integral of the accelerations, the
/// motion between points has a constant jerk; otherwise the velocity is linear.
std::optional<PlanSample> sample_plan(const TrajectoryPoints & plan, const double elapsed)
{
  const auto time_of = [](const auto & point) {
    return rclcpp::Duration(point.time_from_start).seconds();
  };
  if (plan.empty() || elapsed > time_of(plan.back())) {
    return std::nullopt;
  }
  if (elapsed <= time_of(plan.front())) {
    return PlanSample{
      plan.front().longitudinal_velocity_mps, plan.front().acceleration_mps2,
      plan.front().pose.position};
  }
  for (std::size_t i = 0; i + 1 < plan.size(); ++i) {
    const double t0 = time_of(plan[i]);
    const double t1 = time_of(plan[i + 1]);
    if (elapsed > t1) {
      continue;
    }
    const double h = t1 - t0;
    if (h <= 1e-6) {
      return PlanSample{
        plan[i + 1].longitudinal_velocity_mps, plan[i + 1].acceleration_mps2,
        plan[i + 1].pose.position};
    }
    const double u = elapsed - t0;
    const double v0 = plan[i].longitudinal_velocity_mps;
    const double v1 = plan[i + 1].longitudinal_velocity_mps;
    const double a0 = plan[i].acceleration_mps2;
    const double a1 = plan[i + 1].acceleration_mps2;
    const double jerk = (a1 - a0) / h;
    const bool constant_jerk = std::abs(v1 - v0 - 0.5 * (a0 + a1) * h) <= 5e-3;
    const auto travelled = [&](const double time) {
      return constant_jerk ? v0 * time + 0.5 * a0 * time * time + jerk * time * time * time / 6.0
                           : v0 * time + 0.5 * (v1 - v0) / h * time * time;
    };
    PlanSample sample;
    sample.velocity = constant_jerk ? v0 + a0 * u + 0.5 * jerk * u * u : v0 + u / h * (v1 - v0);
    sample.acceleration = a0 + u / h * (a1 - a0);
    const double length = travelled(h);
    const double ratio = length > 1e-6 ? std::clamp(travelled(u) / length, 0.0, 1.0) : u / h;
    const auto & p0 = plan[i].pose.position;
    const auto & p1 = plan[i + 1].pose.position;
    sample.position.x = autoware::interpolation::lerp(p0.x, p1.x, ratio);
    sample.position.y = autoware::interpolation::lerp(p0.y, p1.y, ratio);
    sample.position.z = autoware::interpolation::lerp(p0.z, p1.z, ratio);
    return sample;
  }
  return std::nullopt;
}

/// @brief Whether the input asks for a velocity lower than a plan by more than `margin`, `elapsed`
/// seconds after the ego state the plan was planned from. Each input point is compared with the
/// plan where it projects onto the plan polyline, searching forward from the plan segment at
/// `elapsed`. Input points beyond the plan are ignored, as is the last input point (horizon end,
/// not a stop).
bool input_below_plan(
  const TrajectoryPoints & plan, const double elapsed, const TrajectoryPoints & input,
  const double margin)
{
  if (plan.size() < 2) {
    return false;
  }
  std::size_t segment = 0;
  while (segment + 2 < plan.size() &&
         rclcpp::Duration(plan[segment + 1].time_from_start).seconds() <= elapsed) {
    ++segment;
  }
  // Ratio of the projection along a segment; a segment without length (stopped plan) is skipped.
  const auto project = [&plan](const std::size_t index, const geometry_msgs::msg::Point & point) {
    const auto & p0 = plan[index].pose.position;
    const auto & p1 = plan[index + 1].pose.position;
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    const double dz = p1.z - p0.z;
    const double squared_length = dx * dx + dy * dy + dz * dz;
    return squared_length > 1e-12
             ? ((point.x - p0.x) * dx + (point.y - p0.y) * dy + (point.z - p0.z) * dz) /
                 squared_length
             : std::numeric_limits<double>::infinity();
  };
  for (std::size_t i = 0; i + 1 < input.size(); ++i) {
    const auto & position = input[i].pose.position;
    double ratio = project(segment, position);
    while (ratio > 1.0 && segment + 2 < plan.size()) {
      ++segment;
      ratio = project(segment, position);
    }
    if (ratio > 1.0) {
      return false;  // This input point and the following ones are beyond the plan.
    }
    const double planned = autoware::interpolation::lerp(
      static_cast<double>(plan[segment].longitudinal_velocity_mps),
      static_cast<double>(plan[segment + 1].longitudinal_velocity_mps), std::max(0.0, ratio));
    if (input[i].longitudinal_velocity_mps < planned - margin) {
      return true;
    }
  }
  return false;
}

/// @brief Measurement time of the ego state of a plugin input; none when unset.
std::optional<double> measurement_time(const TrajectoryModifierData & data)
{
  if (!data.current_odometry) {
    return std::nullopt;
  }
  const auto & stamp = data.current_odometry->header.stamp;
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    return std::nullopt;
  }
  return rclcpp::Time(stamp).seconds();
}
}  // namespace

bool operator==(const FixedProfileParameters & lhs, const FixedProfileParameters & rhs)
{
  return lhs.enable == rhs.enable && lhs.reset_velocity_deviation == rhs.reset_velocity_deviation &&
         lhs.reset_acceleration_deviation == rhs.reset_acceleration_deviation &&
         lhs.reset_distance_deviation == rhs.reset_distance_deviation &&
         lhs.reset_time_gap == rhs.reset_time_gap &&
         lhs.reset_on_new_stop == rhs.reset_on_new_stop &&
         lhs.new_stop_match_distance == rhs.new_stop_match_distance &&
         lhs.reset_on_lower_input == rhs.reset_on_lower_input &&
         lhs.lower_input_velocity_margin == rhs.lower_input_velocity_margin;
}

FixedProfileStopState make_fixed_profile_stop_state(
  const TrajectoryPoints & input, const bool zero_speed_limit)
{
  FixedProfileStopState state;
  state.zero_speed_limit = zero_speed_limit;
  // A zero only at the horizon end is not a stop in apply_velocity_limits().
  for (std::size_t i = 0; i + 1 < input.size(); ++i) {
    const double velocity = input[i].longitudinal_velocity_mps;
    const bool stopped =
      std::isfinite(velocity) && std::abs(velocity) <= k_fixed_profile_stop_velocity_threshold_mps;
    const bool previous_stopped = i > 0 && std::isfinite(input[i - 1].longitudinal_velocity_mps) &&
                                  std::abs(input[i - 1].longitudinal_velocity_mps) <=
                                    k_fixed_profile_stop_velocity_threshold_mps;
    const auto & position = input[i].pose.position;
    if (
      stopped && !previous_stopped && std::isfinite(position.x) && std::isfinite(position.y) &&
      std::isfinite(position.z)) {
      state.input_stops.push_back(input[i].pose.position);
    }
  }
  return state;
}

VelocityLimitConstraints make_velocity_limit_constraints(const TrajectoryModifierParams & params)
{
  return {
    params.velocity_limits.max_acceleration, params.velocity_limits.max_deceleration,
    params.velocity_limits.max_jerk};
}

FixedProfileParameters make_fixed_profile_parameters(const TrajectoryModifierParams & params)
{
  const auto & fixed = params.velocity_limits.fixed_profile;
  return {
    fixed.enable,
    fixed.reset_velocity_deviation,
    fixed.reset_acceleration_deviation,
    fixed.reset_distance_deviation,
    fixed.reset_time_gap,
    fixed.reset_on_new_stop,
    fixed.new_stop_match_distance,
    fixed.reset_on_lower_input,
    fixed.lower_input_velocity_margin};
}

const char * to_string(const ProfileStart start)
{
  switch (start) {
    case ProfileStart::Measured:
      return "measured";
    case ProfileStart::NoPreviousPlan:
      return "no_previous_plan";
    case ProfileStart::PreviousPlan:
      return "previous_plan";
    case ProfileStart::ResetTimeGap:
      return "reset_time_gap";
    case ProfileStart::ResetVelocity:
      return "reset_velocity";
    case ProfileStart::ResetAcceleration:
      return "reset_acceleration";
    case ProfileStart::ResetDistance:
      return "reset_distance";
    case ProfileStart::ResetNewStop:
      return "reset_new_stop";
    case ProfileStart::ResetLowerInput:
      return "reset_lower_input";
  }
  return "unknown";
}

bool is_reset(const ProfileStart start)
{
  return start == ProfileStart::ResetTimeGap || start == ProfileStart::ResetVelocity ||
         start == ProfileStart::ResetAcceleration || start == ProfileStart::ResetDistance ||
         start == ProfileStart::ResetNewStop || start == ProfileStart::ResetLowerInput;
}

void FixedProfileMemory::set_parameters(const FixedProfileParameters & parameters)
{
  if (!(parameters == parameters_)) {
    parameters_ = parameters;
    clear();
  }
}

ProfileStartResult FixedProfileMemory::start(
  const std::size_t candidate, const std::optional<double> & time,
  const geometry_msgs::msg::Point & position, const VelocityLimitOptions & measured,
  const FixedProfileStopState & stop_state, const TrajectoryPoints & input) const
{
  if (!parameters_.enable) {
    return {measured, ProfileStart::Measured};
  }
  if (!time || candidate >= plans_.size() || !plans_[candidate]) {
    return {measured, ProfileStart::NoPreviousPlan};
  }
  const auto & previous = *plans_[candidate];
  const double elapsed = *time - previous.time;
  const auto sample = elapsed >= 0.0 && elapsed <= parameters_.reset_time_gap
                        ? sample_plan(previous.points, elapsed)
                        : std::nullopt;
  if (!sample) {
    return {measured, ProfileStart::ResetTimeGap};
  }
  // A stopped previous plan is not a valid starting state while the ego is still creeping.
  // The ordinary 0.5 m/s tracking tolerance would otherwise keep publishing an all-zero plan.
  if (
    stop_state.zero_speed_limit && measured.current_ego_velocity &&
    sample->velocity <= k_fixed_profile_stop_velocity_threshold_mps &&
    *measured.current_ego_velocity > k_fixed_profile_stop_velocity_threshold_mps) {
    return {measured, ProfileStart::ResetVelocity};
  }
  if (
    measured.current_ego_velocity && std::abs(*measured.current_ego_velocity - sample->velocity) >
                                       parameters_.reset_velocity_deviation) {
    return {measured, ProfileStart::ResetVelocity};
  }
  if (
    measured.current_ego_acceleration &&
    std::abs(*measured.current_ego_acceleration - sample->acceleration) >
      parameters_.reset_acceleration_deviation) {
    return {measured, ProfileStart::ResetAcceleration};
  }
  const double distance = std::hypot(
    position.x - sample->position.x, position.y - sample->position.y,
    position.z - sample->position.z);
  if (distance > parameters_.reset_distance_deviation) {
    return {measured, ProfileStart::ResetDistance};
  }
  if (parameters_.reset_on_new_stop) {
    const auto & previous_stops = previous.stop_state.input_stops;
    const bool new_input_stop = std::any_of(
      stop_state.input_stops.begin(), stop_state.input_stops.end(), [&](const auto & stop) {
        return std::none_of(previous_stops.begin(), previous_stops.end(), [&](const auto & prior) {
          return std::hypot(stop.x - prior.x, stop.y - prior.y, stop.z - prior.z) <=
                 parameters_.new_stop_match_distance;
        });
      });
    if (new_input_stop || (stop_state.zero_speed_limit && !previous.stop_state.zero_speed_limit)) {
      return {measured, ProfileStart::ResetNewStop};
    }
  }
  if (
    parameters_.reset_on_lower_input &&
    input_below_plan(previous.points, elapsed, input, parameters_.lower_input_velocity_margin)) {
    return {measured, ProfileStart::ResetLowerInput};
  }
  VelocityLimitOptions options;
  options.current_ego_velocity = sample->velocity;
  options.current_ego_acceleration = sample->acceleration;
  return {options, ProfileStart::PreviousPlan};
}

ProfileStartResult FixedProfileMemory::start(
  const TrajectoryModifierData & data, const FixedProfileStopState & stop_state,
  const TrajectoryPoints & input) const
{
  const auto measured = make_velocity_limit_options(data);
  if (!data.current_odometry) {
    return {measured, parameters_.enable ? ProfileStart::NoPreviousPlan : ProfileStart::Measured};
  }
  return start(
    data.candidate_index, measurement_time(data), data.current_odometry->pose.pose.position,
    measured, stop_state, input);
}

void FixedProfileMemory::store(
  const std::size_t candidate, const std::size_t candidate_count,
  const std::optional<double> & time, const TrajectoryPoints & plan,
  const FixedProfileStopState & stop_state)
{
  // Candidates beyond the current count belong to an earlier batch.
  plans_.resize(std::max(candidate_count, candidate + 1));
  if (time && !plan.empty() && parameters_.enable) {
    plans_[candidate] = Plan{*time, plan, stop_state};
  } else {
    plans_[candidate].reset();
  }
}

void FixedProfileMemory::store(
  const TrajectoryModifierData & data, const TrajectoryPoints & plan,
  const FixedProfileStopState & stop_state)
{
  store(data.candidate_index, data.candidate_count, measurement_time(data), plan, stop_state);
}

void FixedProfileMemory::clear()
{
  plans_.clear();
}

}  // namespace autoware::trajectory_modifier::plugin::detail
