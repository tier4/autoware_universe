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

#include "autoware/mppi_optimizer/detail/trajectory_utils.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace autoware::mppi_optimizer::detail
{

namespace
{

geometry_msgs::msg::Quaternion quaternionFromYaw(const float yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace

bool isOptimizationRequired(const Trajectory & trajectory, const double min_length)
{
  const bool is_stopping = std::any_of(
    trajectory.points.begin(), trajectory.points.end(),
    [](const auto & point) { return point.longitudinal_velocity_mps < 0.02F; });

  double length = 0.0;
  for (std::size_t i = 0; i + 1U < trajectory.points.size(); ++i) {
    length += std::hypot(
      trajectory.points[i].pose.position.x - trajectory.points[i + 1U].pose.position.x,
      trajectory.points[i].pose.position.y - trajectory.points[i + 1U].pose.position.y);
  }
  return !is_stopping || !(length < min_length);
}

void setInitialEngageVelocity(Trajectory & trajectory, const std::optional<float> & max_velocity)
{
  constexpr float engage_velocity = 0.25F;
  constexpr float engage_acceleration = 0.25F;
  const float bounded_engage_velocity =
    max_velocity ? std::min(engage_velocity, *max_velocity) : engage_velocity;
  const float bounded_engage_acceleration =
    max_velocity && max_velocity == 0.0 ? 0.0 : engage_acceleration;
  if (trajectory.points.size() < 3U) {
    return;
  }
  const auto first_moving_it = std::find_if(
    trajectory.points.begin(), trajectory.points.end(),
    [](const auto & point) { return point.longitudinal_velocity_mps > engage_velocity; });
  if (first_moving_it == trajectory.points.end()) return;
  for (auto it = trajectory.points.begin(); it != first_moving_it; ++it) {
    it->longitudinal_velocity_mps = bounded_engage_velocity;
    it->acceleration_mps2 = bounded_engage_acceleration;
  }
}

InitialState makeInitialState(
  const Odometry & odometry,
  const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
  const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params)
{
  InitialState state;
  state.x = static_cast<float>(odometry.pose.pose.position.x);
  state.y = static_cast<float>(odometry.pose.pose.position.y);
  state.yaw = static_cast<float>(tf2::getYaw(odometry.pose.pose.orientation));
  state.velocity = static_cast<float>(odometry.twist.twist.linear.x);
  const float acceleration_value =
    acceleration.has_value() ? static_cast<float>(acceleration->accel.accel.linear.x) : 0.0F;
  state.acceleration =
    std::clamp(acceleration_value, vehicle_params.min_accel(), vehicle_params.max_accel());
  const float steering_value =
    steering_status.has_value() ? steering_status->steering_tire_angle : 0.0F;
  state.steering =
    std::clamp(steering_value, -vehicle_params.max_steer_angle, vehicle_params.max_steer_angle);
  return state;
}

std::vector<float> computeCumulativeChordLength(const Trajectory & trajectory)
{
  const auto & points = trajectory.points;
  std::vector<float> arc_length_s(points.size(), 0.0F);
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto & prev = points[i - 1U].pose.position;
    const auto & curr = points[i].pose.position;
    const float segment = static_cast<float>(std::hypot(curr.x - prev.x, curr.y - prev.y));
    arc_length_s[i] = arc_length_s[i - 1U] + segment;
  }
  return arc_length_s;
}

std::vector<ReferenceSample> buildReferenceHorizon(
  const Trajectory & trajectory, const InitialState & ego, const int horizon, const float dt,
  const size_t start_idx, const std::vector<float> * cumulative_chord_length_s,
  const std::vector<std::optional<float>> * maximum_velocities)
{
  const size_t sample_count = std::max(0, horizon);
  std::vector<ReferenceSample> reference(static_cast<std::size_t>(sample_count));

  if (trajectory.points.empty()) {
    for (auto & reference_sample : reference) {
      reference_sample.x = ego.x;
      reference_sample.y = ego.y;
      reference_sample.yaw = ego.yaw;
      reference_sample.velocity = ego.velocity;
      reference_sample.arc_length_s = 0.0F;
    }
    return reference;
  }

  std::vector<float> owned_chord_length;
  const std::vector<float> * chord_length_ptr = cumulative_chord_length_s;
  if (chord_length_ptr == nullptr) {
    owned_chord_length = computeCumulativeChordLength(trajectory);
    chord_length_ptr = &owned_chord_length;
  }
  const std::vector<float> & chord_length_s = *chord_length_ptr;

  for (std::size_t k = 0; k < sample_count; ++k) {
    auto & sample = reference[k];
    sample.time = static_cast<float>(k + 1U) * dt;

    const std::size_t source_idx = std::min(k + start_idx, trajectory.points.size() - 1U);
    const auto & point = trajectory.points[source_idx];

    sample.x = static_cast<float>(point.pose.position.x);
    sample.y = static_cast<float>(point.pose.position.y);
    sample.yaw = static_cast<float>(tf2::getYaw(point.pose.orientation));
    sample.velocity = point.longitudinal_velocity_mps;
    if (maximum_velocities && source_idx < maximum_velocities->size()) {
      sample.max_velocity = (*maximum_velocities)[source_idx];
      if (sample.max_velocity) {
        sample.velocity = std::clamp(sample.velocity, 0.0F, *sample.max_velocity);
      }
    }
    sample.arc_length_s = source_idx < chord_length_s.size() ? chord_length_s[source_idx] : 0.0F;
  }
  return reference;
}

std::vector<std::optional<float>> buildEffectiveMaximumVelocityProfile(
  const std::size_t point_count, const FirstOrderDubinsMppiKinematicLimits & limits)
{
  const bool valid_external =
    limits.max_velocity && std::isfinite(*limits.max_velocity) && *limits.max_velocity >= 0.0F;
  std::vector<std::optional<float>> result(
    point_count, valid_external ? limits.max_velocity : std::nullopt);

  for (std::size_t index = 0;
       index < point_count && !limits.max_velocity_by_reference_point.empty(); ++index) {
    const std::size_t map_index =
      std::min(index, limits.max_velocity_by_reference_point.size() - 1U);
    const auto map_limit = limits.max_velocity_by_reference_point[map_index];
    if (!map_limit || !std::isfinite(*map_limit) || *map_limit < 0.0F) {
      continue;
    }
    result[index] = result[index] ? std::min(*result[index], *map_limit) : map_limit;
  }
  return result;
}

std::optional<float> getUniformMaximumVelocity(
  const std::vector<std::optional<float>> & maximum_velocities)
{
  if (maximum_velocities.empty() || !maximum_velocities.front()) {
    return std::nullopt;
  }
  const float first = *maximum_velocities.front();
  const bool uniform =
    std::all_of(maximum_velocities.begin(), maximum_velocities.end(), [first](const auto & value) {
      constexpr float kUniformToleranceMps = 1.0E-6F;
      return value && std::abs(*value - first) <= kUniformToleranceMps;
    });
  return uniform ? std::make_optional(first) : std::nullopt;
}

ActiveVelocityLimitProfile buildActiveVelocityLimitProfile(
  const std::vector<FirstOrderDubinsMppiControl> & controls, const InitialState & initial_state,
  const FirstOrderDubinsMppiKinematicLimits & limits,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, const int acceleration_delay_steps,
  const std::vector<float> & acceleration_delay_buffer, const float dt, const bool keep_active,
  const std::vector<float> & reference_velocities)
{
  ActiveVelocityLimitProfile profile;
  profile.controls = controls;

  const auto maximum_velocities = buildEffectiveMaximumVelocityProfile(controls.size(), limits);
  profile.maximum_velocities = maximum_velocities;
  const auto uniform_maximum_velocity = getUniformMaximumVelocity(maximum_velocities);
  const bool has_pointwise_velocity_limit = std::any_of(
    maximum_velocities.begin(), maximum_velocities.end(),
    [](const auto & value) { return value.has_value(); });
  const bool has_variable_velocity_limit =
    has_pointwise_velocity_limit && !uniform_maximum_velocity;

  constexpr float kActivationToleranceMps = 1.0E-3F;
  if (
    !has_pointwise_velocity_limit || !std::isfinite(initial_state.velocity) || controls.empty() ||
    !std::isfinite(dt) || dt <= 0.0F) {
    return profile;
  }

  const bool has_acceleration_limits =
    limits.min_longitudinal_acceleration && limits.max_longitudinal_acceleration &&
    std::isfinite(*limits.min_longitudinal_acceleration) &&
    std::isfinite(*limits.max_longitudinal_acceleration) &&
    *limits.min_longitudinal_acceleration <= 0.0F &&
    *limits.max_longitudinal_acceleration >= 0.0F &&
    *limits.min_longitudinal_acceleration <= *limits.max_longitudinal_acceleration;
  const bool has_jerk_limits =
    limits.min_longitudinal_jerk && limits.max_longitudinal_jerk &&
    std::isfinite(*limits.min_longitudinal_jerk) && std::isfinite(*limits.max_longitudinal_jerk) &&
    *limits.min_longitudinal_jerk <= 0.0F && *limits.max_longitudinal_jerk >= 0.0F &&
    *limits.min_longitudinal_jerk <= *limits.max_longitudinal_jerk;

  const float minimum_acceleration = std::max(
    vehicle_params.min_accel(),
    has_acceleration_limits ? *limits.min_longitudinal_acceleration : vehicle_params.min_accel());
  const float maximum_acceleration = std::min(
    vehicle_params.max_accel(),
    has_acceleration_limits ? *limits.max_longitudinal_acceleration : vehicle_params.max_accel());
  if (minimum_acceleration >= 0.0F || minimum_acceleration > maximum_acceleration) {
    return profile;
  }

  const float safe_dt = std::max(dt, 1.0E-4F);
  const float acceleration_time_constant = std::isfinite(vehicle_params.acc_time_constant)
                                             ? std::max(vehicle_params.acc_time_constant, 1.0E-4F)
                                             : 1.0E-4F;
  const float time_horizon = std::max(0.5F, safe_dt + acceleration_time_constant);
  const float minimum_jerk =
    has_jerk_limits ? *limits.min_longitudinal_jerk : -std::numeric_limits<float>::infinity();
  const float maximum_jerk =
    has_jerk_limits ? *limits.max_longitudinal_jerk : std::numeric_limits<float>::infinity();

  // A varying maximum is converted into a backwards reachable envelope. This starts braking
  // before a lower future limit instead of waiting until the first already-limited point.
  auto admissible_maximum_velocities = maximum_velocities;
  if (has_variable_velocity_limit) {
    std::optional<float> next_maximum;
    for (std::size_t offset = 0; offset < admissible_maximum_velocities.size(); ++offset) {
      const std::size_t index = admissible_maximum_velocities.size() - 1U - offset;
      auto & maximum = admissible_maximum_velocities[index];
      if (next_maximum) {
        const float reachable_maximum = *next_maximum - minimum_acceleration * safe_dt;
        maximum =
          maximum ? std::min(*maximum, reachable_maximum) : std::make_optional(reachable_maximum);
      }
      if (maximum) {
        next_maximum = maximum;
      }
    }
  }

  bool has_restrictive_velocity_limit = keep_active;
  if (uniform_maximum_velocity) {
    has_restrictive_velocity_limit =
      has_restrictive_velocity_limit ||
      initial_state.velocity > *uniform_maximum_velocity + kActivationToleranceMps ||
      (*uniform_maximum_velocity <= kActivationToleranceMps &&
       initial_state.velocity >= -kActivationToleranceMps);
  } else {
    float unbraked_velocity = initial_state.velocity;
    for (const auto & maximum : admissible_maximum_velocities) {
      unbraked_velocity = std::max(0.0F, unbraked_velocity + initial_state.acceleration * safe_dt);
      if (maximum && unbraked_velocity > *maximum + kActivationToleranceMps) {
        has_restrictive_velocity_limit = true;
        break;
      }
    }
  }
  if (!has_restrictive_velocity_limit) {
    return profile;
  }

  const auto command_for_jerk = [&](
                                  const float acceleration, const float jerk, const bool braking) {
    const float unconstrained = std::isfinite(jerk)
                                  ? acceleration + jerk * acceleration_time_constant
                                  : (braking ? minimum_acceleration : 0.0F);
    return std::clamp(
      unconstrained, minimum_acceleration,
      braking ? maximum_acceleration : std::min(maximum_acceleration, 0.0F));
  };
  const auto advance_plant =
    [&](const float applied_command, float & velocity, float & acceleration) {
      velocity = std::max(0.0F, velocity + acceleration * safe_dt);
      const float jerk = (applied_command - acceleration) / acceleration_time_constant;
      acceleration =
        std::clamp(acceleration + jerk * safe_dt, minimum_acceleration, maximum_acceleration);
    };
  const auto release_velocity_loss = [&](const float application_acceleration) {
    if (application_acceleration >= 0.0F) {
      return 0.0F;
    }
    if (has_jerk_limits && maximum_jerk <= 0.0F) {
      return std::numeric_limits<float>::infinity();
    }

    float acceleration = application_acceleration;
    float velocity_loss = 0.0F;
    constexpr int kMaximumReleaseSteps = 1000;
    for (int step = 0; step < kMaximumReleaseSteps && acceleration < -1.0E-4F; ++step) {
      velocity_loss -= acceleration * safe_dt;
      const float command = command_for_jerk(acceleration, maximum_jerk, false);
      const float jerk = (command - acceleration) / acceleration_time_constant;
      const float next_acceleration =
        std::clamp(acceleration + jerk * safe_dt, minimum_acceleration, maximum_acceleration);
      if (next_acceleration <= acceleration + 1.0E-6F) {
        return std::numeric_limits<float>::infinity();
      }
      acceleration = next_acceleration;
    }
    return velocity_loss;
  };

  std::deque<float> pending_commands;
  const int delay_steps = std::max(0, acceleration_delay_steps);
  for (int step = 0; step < delay_steps; ++step) {
    const float fallback = initial_state.acceleration;
    const float command = static_cast<std::size_t>(step) < acceleration_delay_buffer.size()
                            ? acceleration_delay_buffer[static_cast<std::size_t>(step)]
                            : fallback;
    pending_commands.push_back(
      std::clamp(command, vehicle_params.min_accel(), vehicle_params.max_accel()));
  }

  profile.active = true;
  if (uniform_maximum_velocity) {
    profile.target_velocity = *uniform_maximum_velocity;
  } else {
    const auto last_maximum = std::find_if(
      maximum_velocities.rbegin(), maximum_velocities.rend(),
      [](const auto & value) { return value.has_value(); });
    profile.target_velocity = last_maximum != maximum_velocities.rend() ? **last_maximum : 0.0F;
  }
  profile.velocities.reserve(controls.size());
  profile.accelerations.reserve(controls.size());
  float velocity = initial_state.velocity;
  // Preserve the measured initial state even if a newly received bound is already violated.
  // The rollout model applies the new acceleration-state bounds after the first integration step.
  float acceleration =
    std::clamp(initial_state.acceleration, vehicle_params.min_accel(), vehicle_params.max_accel());

  for (std::size_t index = 0; index < profile.controls.size(); ++index) {
    // Select the command for its delayed application state, not the current issue-time state.
    float application_velocity = velocity;
    float application_acceleration = acceleration;
    for (const float pending_command : pending_commands) {
      advance_plant(pending_command, application_velocity, application_acceleration);
    }

    const std::size_t application_index =
      std::min(index + pending_commands.size(), admissible_maximum_velocities.size() - 1U);
    const auto target_velocity = uniform_maximum_velocity
                                   ? uniform_maximum_velocity
                                   : admissible_maximum_velocities[application_index];
    const float remaining_velocity =
      target_velocity ? std::max(0.0F, application_velocity - *target_velocity) : 0.0F;
    const float release_loss = release_velocity_loss(application_acceleration);
    const bool release_brake =
      !target_velocity || remaining_velocity <= release_loss + kActivationToleranceMps;
    float command = release_brake ? command_for_jerk(application_acceleration, maximum_jerk, false)
                                  : command_for_jerk(application_acceleration, minimum_jerk, true);
    bool accelerate_to_reference = false;
    if (
      has_variable_velocity_limit && release_brake && target_velocity &&
      application_acceleration >= -kActivationToleranceMps &&
      application_index < reference_velocities.size()) {
      const float desired_velocity =
        std::min(std::max(0.0F, reference_velocities[application_index]), *target_velocity);
      if (application_velocity < desired_velocity - kActivationToleranceMps) {
        accelerate_to_reference = true;
        const float desired_accel = (desired_velocity - application_velocity) / time_horizon;
        const float unconstrained = std::clamp(
          desired_accel, application_acceleration + minimum_jerk * acceleration_time_constant,
          application_acceleration + maximum_jerk * acceleration_time_constant);
        command =
          std::clamp(unconstrained, std::max(0.0F, minimum_acceleration), maximum_acceleration);
      }
    }
    if (
      !accelerate_to_reference && target_velocity &&
      application_velocity <= *target_velocity + kActivationToleranceMps &&
      application_acceleration >= -kActivationToleranceMps) {
      command = 0.0F;
    }
    profile.controls[index].accel_cmd = command;

    float applied_command = command;
    if (!pending_commands.empty()) {
      applied_command = pending_commands.front();
      pending_commands.pop_front();
      pending_commands.push_back(command);
    }
    advance_plant(applied_command, velocity, acceleration);
    profile.velocities.push_back(velocity);
    profile.accelerations.push_back(acceleration);
  }
  return profile;
}

void applyActiveVelocityLimitProfile(
  Trajectory & trajectory, const ActiveVelocityLimitProfile & profile)
{
  if (!profile.active) {
    return;
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    auto & point = trajectory.points[index];
    if (index < profile.velocities.size()) {
      point.longitudinal_velocity_mps = profile.velocities[index];
      point.acceleration_mps2 = profile.accelerations[index];
    } else {
      point.longitudinal_velocity_mps = profile.target_velocity;
      point.acceleration_mps2 = 0.0F;
    }
  }
}

float computeMengerCurvatureWithMinChord(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points,
  const std::size_t target_idx, const float min_chord_length_m) noexcept
{
  constexpr double kSideLengthEpsilon = 1.0E-4;
  constexpr double kMinimumDenominator = 1.0E-8;
  if (points.size() < 3U || target_idx >= points.size()) {
    return 0.0F;
  }

  const double minimum_chord = std::max(0.0, static_cast<double>(min_chord_length_m));
  const auto & center = points[target_idx].pose.position;
  const auto distance_from_center = [&center](const auto & point) {
    return std::hypot(point.pose.position.x - center.x, point.pose.position.y - center.y);
  };

  std::size_t backward_idx = 0U;
  for (std::size_t candidate = target_idx; candidate > 0U; --candidate) {
    const std::size_t index = candidate - 1U;
    if (distance_from_center(points[index]) >= minimum_chord) {
      backward_idx = index;
      break;
    }
  }

  std::size_t forward_idx = points.size() - 1U;
  for (std::size_t index = target_idx + 1U; index < points.size(); ++index) {
    if (distance_from_center(points[index]) >= minimum_chord) {
      forward_idx = index;
      break;
    }
  }

  if (backward_idx == target_idx || forward_idx == target_idx) {
    return 0.0F;
  }

  const auto & first = points[backward_idx].pose.position;
  const auto & last = points[forward_idx].pose.position;
  const double first_to_center = std::hypot(center.x - first.x, center.y - first.y);
  const double center_to_last = std::hypot(last.x - center.x, last.y - center.y);
  const double first_to_last = std::hypot(last.x - first.x, last.y - first.y);
  const double denominator = first_to_center * center_to_last * first_to_last;
  if (
    first_to_center < kSideLengthEpsilon || center_to_last < kSideLengthEpsilon ||
    first_to_last < kSideLengthEpsilon || denominator < kMinimumDenominator) {
    return 0.0F;
  }

  const double cross =
    (center.x - first.x) * (last.y - first.y) - (center.y - first.y) * (last.x - first.x);
  return static_cast<float>(2.0 * cross / denominator);
}

std::vector<FirstOrderDubinsMppiControl> buildDiffusionNominalControl(
  const Trajectory & reference, const std::size_t start_idx,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, const int horizon,
  const float min_chord_length_m)
{
  const int control_count = std::max(0, horizon);
  std::vector<FirstOrderDubinsMppiControl> nominal(static_cast<std::size_t>(control_count));
  if (reference.points.empty()) {
    return nominal;
  }

  for (int t = 0; t < control_count; ++t) {
    const std::size_t index =
      std::min(start_idx + static_cast<std::size_t>(t), reference.points.size() - 1U);
    const auto & point = reference.points[index];
    auto & control = nominal[static_cast<std::size_t>(t)];
    control.accel_cmd =
      std::clamp(point.acceleration_mps2, vehicle_params.min_accel(), vehicle_params.max_accel());
    float steering = point.front_wheel_angle_rad;
    if (std::abs(steering) <= 1.0E-6F) {
      const float curvature =
        computeMengerCurvatureWithMinChord(reference.points, index, min_chord_length_m);
      if (std::isfinite(curvature)) {
        steering = std::atan(vehicle_params.wheel_base * curvature);
      }
    }
    control.steer_cmd =
      std::clamp(steering, -vehicle_params.max_steer_angle, vehicle_params.max_steer_angle);
  }
  return nominal;
}

std::vector<FirstOrderDubinsMppiControl> buildForcedNominalControl(
  const std::vector<float> & acceleration_commands, const std::vector<float> & steering_commands,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, const int horizon)
{
  const int control_count = std::max(0, horizon);
  std::vector<FirstOrderDubinsMppiControl> nominal(static_cast<std::size_t>(control_count));
  for (int t = 0; t < control_count; ++t) {
    const auto index = static_cast<std::size_t>(t);
    const float acceleration =
      index < acceleration_commands.size()
        ? acceleration_commands[index]
        : (acceleration_commands.empty() ? 0.0F : acceleration_commands.back());
    const float steering = index < steering_commands.size()
                             ? steering_commands[index]
                             : (steering_commands.empty() ? 0.0F : steering_commands.back());
    nominal[index].accel_cmd =
      std::clamp(acceleration, vehicle_params.min_accel(), vehicle_params.max_accel());
    nominal[index].steer_cmd =
      std::clamp(steering, -vehicle_params.max_steer_angle, vehicle_params.max_steer_angle);
  }
  return nominal;
}

std::vector<FirstOrderDubinsMppiControl> filterNominalControlWithKinematicLimits(
  const std::vector<FirstOrderDubinsMppiControl> & nominal, const InitialState & initial_state,
  const FirstOrderDubinsMppiKinematicLimits & limits,
  const FirstOrderDubinsMppiVehicleParams & vehicle_params, const int acceleration_delay_steps,
  const std::vector<float> & acceleration_delay_buffer, const float dt)
{
  auto maximum_velocities = buildEffectiveMaximumVelocityProfile(nominal.size(), limits);
  const auto uniform_maximum_velocity = getUniformMaximumVelocity(maximum_velocities);
  const bool has_velocity_limit = std::any_of(
    maximum_velocities.begin(), maximum_velocities.end(),
    [](const auto & value) { return value.has_value(); });
  const bool has_acceleration_limits =
    limits.min_longitudinal_acceleration && limits.max_longitudinal_acceleration &&
    std::isfinite(*limits.min_longitudinal_acceleration) &&
    std::isfinite(*limits.max_longitudinal_acceleration) &&
    *limits.min_longitudinal_acceleration <= 0.0F &&
    *limits.max_longitudinal_acceleration >= 0.0F &&
    *limits.min_longitudinal_acceleration <= *limits.max_longitudinal_acceleration;
  const bool has_jerk_limits =
    limits.min_longitudinal_jerk && limits.max_longitudinal_jerk &&
    std::isfinite(*limits.min_longitudinal_jerk) && std::isfinite(*limits.max_longitudinal_jerk) &&
    *limits.min_longitudinal_jerk <= 0.0F && *limits.max_longitudinal_jerk >= 0.0F &&
    *limits.min_longitudinal_jerk <= *limits.max_longitudinal_jerk;
  if (!has_velocity_limit && !has_acceleration_limits && !has_jerk_limits) {
    return nominal;
  }

  const float safe_dt = std::isfinite(dt) ? std::max(dt, 1.0E-4F) : kMppiDt;
  const float acceleration_time_constant = std::isfinite(vehicle_params.acc_time_constant)
                                             ? std::max(vehicle_params.acc_time_constant, 1.0E-4F)
                                             : 1.0E-4F;
  const float time_horizon = std::max(0.5F, safe_dt + acceleration_time_constant);
  const float minimum_acceleration = std::max(
    vehicle_params.min_accel(),
    has_acceleration_limits ? *limits.min_longitudinal_acceleration : vehicle_params.min_accel());
  const float maximum_acceleration = std::min(
    vehicle_params.max_accel(),
    has_acceleration_limits ? *limits.max_longitudinal_acceleration : vehicle_params.max_accel());
  if (minimum_acceleration > maximum_acceleration) {
    return nominal;
  }

  const float minimum_jerk =
    has_jerk_limits ? *limits.min_longitudinal_jerk : -std::numeric_limits<float>::infinity();
  const float maximum_jerk =
    has_jerk_limits ? *limits.max_longitudinal_jerk : std::numeric_limits<float>::infinity();
  if (has_velocity_limit && !uniform_maximum_velocity) {
    std::optional<float> next_maximum;
    for (std::size_t offset = 0; offset < maximum_velocities.size(); ++offset) {
      const std::size_t index = maximum_velocities.size() - 1U - offset;
      auto & maximum = maximum_velocities[index];
      if (next_maximum) {
        const float reachable_maximum = *next_maximum - minimum_acceleration * safe_dt;
        maximum =
          maximum ? std::min(*maximum, reachable_maximum) : std::make_optional(reachable_maximum);
      }
      if (maximum) {
        next_maximum = maximum;
      }
    }
  }

  const int delay_steps = std::max(0, acceleration_delay_steps);
  std::deque<float> pending_commands;
  for (int i = 0; i < delay_steps; ++i) {
    const float fallback = initial_state.acceleration;
    const float command = static_cast<std::size_t>(i) < acceleration_delay_buffer.size()
                            ? acceleration_delay_buffer[static_cast<std::size_t>(i)]
                            : fallback;
    pending_commands.push_back(
      std::clamp(command, vehicle_params.min_accel(), vehicle_params.max_accel()));
  }

  auto filtered = nominal;
  float velocity = initial_state.velocity;
  float acceleration =
    std::clamp(initial_state.acceleration, vehicle_params.min_accel(), vehicle_params.max_accel());

  const auto advance_plant =
    [&](const float applied_command, float & predicted_velocity, float & predicted_acceleration) {
      predicted_velocity += predicted_acceleration * safe_dt;
      const float jerk = (applied_command - predicted_acceleration) / acceleration_time_constant;
      predicted_acceleration = std::clamp(
        predicted_acceleration + jerk * safe_dt, vehicle_params.min_accel(),
        vehicle_params.max_accel());
    };

  for (std::size_t i = 0; i < filtered.size(); ++i) {
    // Predict the state at which the command issued now will reach the plant.
    float application_velocity = velocity;
    float application_acceleration = acceleration;
    for (const float pending_command : pending_commands) {
      advance_plant(pending_command, application_velocity, application_acceleration);
    }

    float command = std::clamp(nominal[i].accel_cmd, minimum_acceleration, maximum_acceleration);
    const float next_application_velocity =
      application_velocity + application_acceleration * safe_dt;
    const std::size_t application_index =
      std::min(i + pending_commands.size(), maximum_velocities.size() - 1U);
    const auto maximum_velocity =
      uniform_maximum_velocity ? uniform_maximum_velocity : maximum_velocities[application_index];
    if (maximum_velocity && next_application_velocity > *maximum_velocity) {
      const float braking_command = std::clamp(
        (*maximum_velocity - next_application_velocity) / time_horizon, minimum_acceleration,
        maximum_acceleration);
      command = std::min(command, braking_command);
    } else if (has_velocity_limit && next_application_velocity < 0.0F) {
      const float recovery_command = std::clamp(
        -next_application_velocity / time_horizon, minimum_acceleration, maximum_acceleration);
      command = std::max(command, recovery_command);
    }

    // Jerk is da/dt=(u_applied-a)/tau in the rollout model. Limit the command using the
    // acceleration predicted at its application time, not the state at its issue time.
    command = std::clamp(
      command, application_acceleration + minimum_jerk * acceleration_time_constant,
      application_acceleration + maximum_jerk * acceleration_time_constant);
    command = std::clamp(command, minimum_acceleration, maximum_acceleration);
    filtered[i].accel_cmd = command;

    float applied_command = command;
    if (!pending_commands.empty()) {
      applied_command = pending_commands.front();
      pending_commands.pop_front();
      pending_commands.push_back(command);
    }
    advance_plant(applied_command, velocity, acceleration);
  }
  return filtered;
}

std::vector<FirstOrderDubinsMppiControl> shiftNominalControl(
  const std::vector<FirstOrderDubinsMppiControl> & previous, const int horizon)
{
  const int control_count = std::max(0, horizon);
  std::vector<FirstOrderDubinsMppiControl> nominal(static_cast<std::size_t>(control_count));
  if (previous.empty()) {
    return nominal;
  }
  for (int t = 0; t < control_count; ++t) {
    const std::size_t source = std::min(static_cast<std::size_t>(t) + 1U, previous.size() - 1U);
    nominal[static_cast<std::size_t>(t)] = previous[source];
  }
  return nominal;
}

Trajectory buildOptimizedTrajectory(
  const Trajectory & input, const std::vector<OptimizedState> & post_step_states,
  const std::vector<FirstOrderDubinsMppiControl> & controls)
{
  Trajectory output = input;
  const std::size_t optimized_count =
    std::min({output.points.size(), post_step_states.size(), controls.size()});
  for (std::size_t i = 0; i < optimized_count; ++i) {
    const auto & state = post_step_states[i];
    const auto & input_point = input.points[i];
    auto & output_point = output.points[i];
    output_point.pose.position.x = state.x;
    output_point.pose.position.y = state.y;
    output_point.pose.position.z = input_point.pose.position.z;
    output_point.pose.orientation = quaternionFromYaw(state.yaw);
    output_point.longitudinal_velocity_mps = state.velocity;
    // Plant longitudinal accel / tire angle (lag states), not undelayed cmds.
    // output_point.acceleration_mps2 = state.acceleration;
    // output_point.front_wheel_angle_rad = state.steering;
    output_point.acceleration_mps2 = controls[i].accel_cmd;
    output_point.front_wheel_angle_rad = controls[i].steer_cmd;
  }
  return output;
}

}  // namespace autoware::mppi_optimizer::detail
