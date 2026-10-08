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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier::plugin::detail
{

struct VelocityLimitResult
{
  ProcessingResult status{ProcessingResult::Unchanged};
  std::string error;
  bool profile_generated{false};
};

struct VelocityLimitOptions
{
  std::optional<double> current_ego_velocity;
  std::optional<double> current_ego_acceleration;
};

// Retimes along the input polyline, keeping its first pose and every timestamp. The callback
// permits resolving either a spatially varying map limit or one external limit for every point.
// Feasible mode anchors the velocity profile to the current ego velocity at t=0. Its forward
// reachability pass can exceed an input speed if that speed is physically unreachable.
VelocityLimitResult apply_velocity_limits(
  TrajectoryPoints & points, const double deceleration, const double jerk,
  const std::function<std::optional<double>(const geometry_msgs::msg::Point &)> & velocity_limit,
  const VelocityLimitOptions & options = {});

// Re-evaluate poses on the current input polyline after velocities have been assigned.
void retime_velocity_profile(
  TrajectoryPoints & points, const TrajectoryPoints & original, const std::vector<double> & times);

struct FixedVelocityLimitKey
{
  std::optional<double> uniform_limit;
  double deceleration{};
  double jerk{};
  std::uint64_t context_revision{};
  bool spatial{false};
};

struct FixedVelocityLimitParameters
{
  double speed_deviation_threshold{0.3};
  double acceleration_deviation_threshold{0.3};
  double longitudinal_deviation_threshold{0.5};
  double lateral_deviation_threshold{0.5};
  double heading_deviation_threshold{0.17};
  double stamp_difference_threshold{0.2};
  double limit_difference_threshold{1e-3};
  double constraint_difference_threshold{1e-6};
};

FixedVelocityLimitParameters get_fixed_velocity_limit_parameters(
  const TrajectoryModifierParams & params);

// A plugin instance owns one cache. The same generator UUID may name several candidates, so
// entries are matched one-to-one by candidate index and path continuity within each callback.
class FixedVelocityLimitCache
{
public:
  using LimitFunction = std::function<std::optional<double>(const geometry_msgs::msg::Point &)>;

  VelocityLimitResult process(
    TrajectoryPoints & points, const TrajectoryModifierData & data,
    const FixedVelocityLimitKey & key, const LimitFunction & velocity_limit);

  void set_parameters(const FixedVelocityLimitParameters & parameters);
  void clear() { entries_.clear(); }
  [[nodiscard]] std::uint64_t profile_build_count() const { return profile_build_count_; }
  [[nodiscard]] std::uint64_t calculation_count() const { return calculation_count_; }

private:
  struct Segment
  {
    double start_time{};
    double duration{};
    double initial_velocity{};
    double initial_acceleration{};
    double initial_distance{};
    double jerk{};
  };

  struct ProfileState
  {
    double velocity{};
    double acceleration{};
    double distance{};
  };

  struct Projection
  {
    double station{};
    double separation{};
    double heading{};
  };

  struct Entry
  {
    unique_identifier_msgs::msg::UUID generator_id{};
    std::size_t candidate_index{};
    std::uint64_t last_seen_batch{};
    std::int64_t anchor_nanoseconds{};
    std::int64_t last_odometry_nanoseconds{};
    std::string frame_id;
    FixedVelocityLimitKey key;
    TrajectoryPoints reference;
    std::vector<double> reference_stations;
    double anchor_station{};
    std::optional<double> terminal_limit;
    std::vector<Segment> segments;
    ProfileState terminal_state;
    double terminal_time{};
  };

  static std::optional<Projection> project(
    const Entry & entry, const geometry_msgs::msg::Point & point);
  static ProfileState sample(const Entry & entry, double time);
  static bool build_profile(
    Entry & entry, const TrajectoryPoints & limited, double initial_velocity,
    double initial_acceleration);
  static bool tracking_is_consistent(
    const Entry & entry, const TrajectoryModifierData & data, double elapsed,
    const FixedVelocityLimitParameters & parameters);
  static bool limits_are_consistent(
    const Entry & entry, const TrajectoryPoints & points, const LimitFunction & velocity_limit,
    const FixedVelocityLimitParameters & parameters);
  static bool apply_profile(
    const Entry & entry, TrajectoryPoints & points, double elapsed,
    const LimitFunction & velocity_limit, const FixedVelocityLimitParameters & parameters);
  static void extend_reference(
    Entry & entry, const TrajectoryPoints & points,
    const FixedVelocityLimitParameters & parameters);

  FixedVelocityLimitParameters parameters_;
  std::vector<Entry> entries_;
  std::uint64_t profile_build_count_{0U};
  std::uint64_t calculation_count_{0U};
};

}  // namespace autoware::trajectory_modifier::plugin::detail

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_PLUGINS__VELOCITY_LIMITS_HPP_
