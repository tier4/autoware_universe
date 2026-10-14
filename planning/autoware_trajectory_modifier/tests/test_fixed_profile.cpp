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

#include <gtest/gtest.h>

#include <cstddef>
#include <optional>

namespace
{
namespace detail = autoware::trajectory_modifier::plugin::detail;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;
using detail::FixedProfileMemory;
using detail::FixedProfileParameters;
using detail::FixedProfileStopState;
using detail::ProfileStart;
using detail::VelocityLimitConstraints;
using detail::VelocityLimitOptions;

constexpr double dt = 0.1;
constexpr double cruise = 16.6667;
constexpr double zone_start = 125.0;
constexpr double zone_limit = 8.3333;
constexpr VelocityLimitConstraints constraints{1.0, 1.0, 3.0};

/// @brief Constant-speed input sampled every dt along +x from `x0`.
TrajectoryPoints make_input(const double x0, const std::size_t count = 120)
{
  TrajectoryPoints points(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double t = dt * static_cast<double>(i);
    points[i].time_from_start = rclcpp::Duration::from_seconds(t);
    points[i].pose.position.x = x0 + cruise * t;
    points[i].pose.orientation.w = 1.0;
    points[i].longitudinal_velocity_mps = static_cast<float>(cruise);
  }
  return points;
}

std::optional<double> zone(const geometry_msgs::msg::Point & position)
{
  return position.x >= zone_start ? zone_limit : cruise;
}

VelocityLimitOptions make_options(const double velocity, const double acceleration)
{
  VelocityLimitOptions options;
  options.current_ego_velocity = velocity;
  options.current_ego_acceleration = acceleration;
  return options;
}

/// @brief Plan that brakes for the zone ahead, from a cruising ego at x = 40 m.
TrajectoryPoints make_plan()
{
  auto plan = make_input(40.0);
  const auto result =
    detail::apply_velocity_limits(plan, constraints, zone, make_options(cruise, 0.0));
  EXPECT_EQ(result.status, ProcessingResult::Modified) << result.error;
  return plan;
}

geometry_msgs::msg::Point point_at(const double x)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  return point;
}

class FixedProfile : public ::testing::Test
{
protected:
  void SetUp() override
  {
    plan_ = make_plan();
    memory_.set_parameters(FixedProfileParameters{});
    memory_.store(0, 1, 10.0, plan_);
  }

  /// @brief Start at t = 10.1 s from a measurement offset from the plan's second point.
  [[nodiscard]] detail::ProfileStartResult start_with_offsets(
    const double velocity, const double acceleration, const double distance,
    const double time = 10.1, const FixedProfileStopState & stop_state = {}) const
  {
    const auto & planned = plan_[1];
    return memory_.start(
      0, time, point_at(planned.pose.position.x + distance),
      make_options(
        planned.longitudinal_velocity_mps + velocity, planned.acceleration_mps2 + acceleration),
      stop_state);
  }

  /// @brief Input from the plan's second point, with the plan's velocities shifted by `offset`.
  [[nodiscard]] TrajectoryPoints input_along_plan(const double offset) const
  {
    TrajectoryPoints input(plan_.begin() + 1, plan_.end());
    for (std::size_t i = 0; i < input.size(); ++i) {
      input[i].time_from_start = rclcpp::Duration::from_seconds(dt * static_cast<double>(i));
      input[i].longitudinal_velocity_mps += static_cast<float>(offset);
    }
    return input;
  }

  /// @brief Start at t = 10.1 s from the plan's second point with the given input.
  [[nodiscard]] detail::ProfileStartResult start_with_input(const TrajectoryPoints & input) const
  {
    const auto & planned = plan_[1];
    return memory_.start(
      0, 10.1, planned.pose.position,
      make_options(planned.longitudinal_velocity_mps, planned.acceleration_mps2), {}, input);
  }

  TrajectoryPoints plan_;
  FixedProfileMemory memory_;
};

TEST_F(FixedProfile, StartsFromTheMeasuredStateWhenDisabled)
{
  FixedProfileParameters parameters;
  parameters.enable = false;
  memory_.set_parameters(parameters);
  memory_.store(0, 1, 10.0, plan_);

  const auto start = start_with_offsets(0.2, 0.0, 0.0);
  EXPECT_EQ(start.start, ProfileStart::Measured);
  EXPECT_DOUBLE_EQ(*start.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps + 0.2);
}

TEST_F(FixedProfile, StartsFromTheMeasuredStateWithoutPreviousPlan)
{
  EXPECT_EQ(
    memory_.start(1, 10.1, point_at(0.0), make_options(10.0, 0.0)).start,
    ProfileStart::NoPreviousPlan);
  EXPECT_EQ(
    memory_.start(0, std::nullopt, point_at(0.0), make_options(10.0, 0.0)).start,
    ProfileStart::NoPreviousPlan);
}

TEST_F(FixedProfile, ContinuesThePreviousPlanWhileTheEgoTracksIt)
{
  const auto start = start_with_offsets(0.4, -0.4, 0.8);
  ASSERT_EQ(start.start, ProfileStart::PreviousPlan);
  EXPECT_NEAR(*start.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps, 1e-4);
  EXPECT_NEAR(*start.options.current_ego_acceleration, plan_[1].acceleration_mps2, 1e-4);
}

TEST_F(FixedProfile, RestartsFromTheMeasuredStateWhenTheEgoDeviates)
{
  EXPECT_EQ(start_with_offsets(0.6, 0.0, 0.0).start, ProfileStart::ResetVelocity);
  EXPECT_EQ(start_with_offsets(-0.6, 0.0, 0.0).start, ProfileStart::ResetVelocity);
  EXPECT_EQ(start_with_offsets(0.0, 0.6, 0.0).start, ProfileStart::ResetAcceleration);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 1.5).start, ProfileStart::ResetDistance);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0, 10.6).start, ProfileStart::ResetTimeGap);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0, 9.9).start, ProfileStart::ResetTimeGap);

  const auto start = start_with_offsets(0.6, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(*start.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps + 0.6);
}

TEST_F(FixedProfile, KeepsCandidatesApart)
{
  auto other = make_input(40.0);
  const auto result = detail::apply_velocity_limits(
    other, constraints, [](const auto &) { return std::optional<double>{5.0}; },
    make_options(cruise, 0.0));
  ASSERT_EQ(result.status, ProcessingResult::Modified);
  memory_.store(0, 2, 10.0, plan_);
  memory_.store(1, 2, 10.0, other);

  const auto first = memory_.start(
    0, 10.1, plan_[1].pose.position,
    make_options(plan_[1].longitudinal_velocity_mps, plan_[1].acceleration_mps2));
  const auto second = memory_.start(
    1, 10.1, other[1].pose.position,
    make_options(other[1].longitudinal_velocity_mps, other[1].acceleration_mps2));
  ASSERT_EQ(first.start, ProfileStart::PreviousPlan);
  ASSERT_EQ(second.start, ProfileStart::PreviousPlan);
  EXPECT_NEAR(*first.options.current_ego_acceleration, plan_[1].acceleration_mps2, 1e-4);
  EXPECT_NEAR(*second.options.current_ego_acceleration, other[1].acceleration_mps2, 1e-4);

  // A batch with a single candidate forgets the second one.
  memory_.store(0, 1, 10.1, plan_);
  EXPECT_EQ(
    memory_.start(1, 10.2, other[1].pose.position, make_options(cruise, 0.0)).start,
    ProfileStart::NoPreviousPlan);
}

TEST_F(FixedProfile, ParameterChangeForgetsPreviousPlans)
{
  memory_.set_parameters(FixedProfileParameters{});
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0).start, ProfileStart::PreviousPlan);

  FixedProfileParameters parameters;
  parameters.reset_velocity_deviation = 0.3;
  memory_.set_parameters(parameters);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0).start, ProfileStart::NoPreviousPlan);
}

TEST(FixedProfileStops, ExtractsNewStopsButIgnoresTheTerminalHorizonPoint)
{
  auto input = make_input(40.0, 12);
  input[4].longitudinal_velocity_mps = 0.0F;
  input[5].longitudinal_velocity_mps = 0.0F;
  input[8].longitudinal_velocity_mps = 0.0F;
  input.back().longitudinal_velocity_mps = 0.0F;

  const auto stops = detail::make_fixed_profile_stop_state(input);
  ASSERT_EQ(stops.input_stops.size(), 2U);
  EXPECT_DOUBLE_EQ(stops.input_stops[0].x, input[4].pose.position.x);
  EXPECT_DOUBLE_EQ(stops.input_stops[1].x, input[8].pose.position.x);

  auto terminal_only = make_input(40.0, 12);
  terminal_only.back().longitudinal_velocity_mps = 0.0F;
  EXPECT_TRUE(detail::make_fixed_profile_stop_state(terminal_only).input_stops.empty());

  auto already_stopped = make_input(40.0, 12);
  for (auto & point : already_stopped) {
    point.longitudinal_velocity_mps = 0.0F;
  }
  const auto stopped_state = detail::make_fixed_profile_stop_state(already_stopped);
  ASSERT_EQ(stopped_state.input_stops.size(), 1U);
  EXPECT_DOUBLE_EQ(stopped_state.input_stops.front().x, already_stopped.front().pose.position.x);
}

TEST_F(FixedProfile, NewInputStopResetsOnlyOnceAcrossMovingHorizons)
{
  auto input = make_input(40.0);
  input[50].longitudinal_velocity_mps = 0.0F;
  const auto stop_state = detail::make_fixed_profile_stop_state(input);
  ASSERT_EQ(stop_state.input_stops.size(), 1U);

  const auto reset = start_with_offsets(0.2, 0.0, 0.0, 10.1, stop_state);
  ASSERT_EQ(reset.start, ProfileStart::ResetNewStop);
  EXPECT_DOUBLE_EQ(*reset.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps + 0.2);
  EXPECT_TRUE(detail::is_reset(reset.start));
  EXPECT_STREQ(detail::to_string(reset.start), "reset_new_stop");

  memory_.store(0, 1, 10.1, plan_, stop_state);
  auto shifted_input = make_input(40.0 + cruise * dt);
  shifted_input[49].longitudinal_velocity_mps = 0.0F;
  const auto shifted_stops = detail::make_fixed_profile_stop_state(shifted_input);
  EXPECT_EQ(
    start_with_offsets(0.2, 0.0, 0.0, 10.2, shifted_stops).start, ProfileStart::PreviousPlan);
}

TEST_F(FixedProfile, ResetsWhenAnAdditionalOrMovedStopAppears)
{
  auto input = make_input(40.0);
  input[50].longitudinal_velocity_mps = 0.0F;
  const auto original_stops = detail::make_fixed_profile_stop_state(input);
  memory_.store(0, 1, 10.0, plan_, original_stops);

  input[60].longitudinal_velocity_mps = 0.0F;
  EXPECT_EQ(
    start_with_offsets(0.0, 0.0, 0.0, 10.1, detail::make_fixed_profile_stop_state(input)).start,
    ProfileStart::ResetNewStop);

  auto moved = make_input(40.0);
  moved[50].longitudinal_velocity_mps = 0.0F;
  moved[50].pose.position.x += 1.5;
  EXPECT_EQ(
    start_with_offsets(0.0, 0.0, 0.0, 10.1, detail::make_fixed_profile_stop_state(moved)).start,
    ProfileStart::ResetNewStop);

  moved[50].pose.position.x = input[50].pose.position.x + 0.5;
  EXPECT_EQ(
    start_with_offsets(0.0, 0.0, 0.0, 10.1, detail::make_fixed_profile_stop_state(moved)).start,
    ProfileStart::PreviousPlan);
}

TEST_F(FixedProfile, NewZeroSpeedExternalLimitResetsOnce)
{
  const auto zero_limit = detail::make_fixed_profile_stop_state(make_input(40.0), true);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0, 10.1, zero_limit).start, ProfileStart::ResetNewStop);
  memory_.store(0, 1, 10.1, plan_, zero_limit);
  EXPECT_EQ(start_with_offsets(0.0, 0.0, 0.0, 10.2, zero_limit).start, ProfileStart::PreviousPlan);
}

TEST_F(FixedProfile, StoppedPlanRestartsWhileEgoStillMoves)
{
  auto stopped_plan = make_input(40.0, 4);
  for (auto & point : stopped_plan) {
    point.pose.position.x = 40.0;
    point.longitudinal_velocity_mps = 0.0F;
    point.acceleration_mps2 = 0.0F;
  }
  FixedProfileStopState zero_limit;
  zero_limit.zero_speed_limit = true;
  memory_.store(0, 1, 10.0, stopped_plan, zero_limit);

  const auto moving = memory_.start(0, 10.1, point_at(40.0), make_options(0.12, 0.0), zero_limit);
  EXPECT_EQ(moving.start, ProfileStart::ResetVelocity);
  EXPECT_DOUBLE_EQ(*moving.options.current_ego_velocity, 0.12);
  auto moving_input = make_input(40.0);
  const auto result = detail::apply_velocity_limits(
    moving_input, {1.0, 2.0, 1.0}, [](const auto &) { return std::optional<double>{0.0}; },
    moving.options);
  ASSERT_EQ(result.status, ProcessingResult::Modified) << result.error;
  EXPECT_GT(moving_input.front().longitudinal_velocity_mps, 0.05F);
  EXPECT_EQ(
    memory_.start(0, 10.1, point_at(40.0), make_options(0.03, 0.0), zero_limit).start,
    ProfileStart::PreviousPlan);

  memory_.store(0, 1, 10.1, TrajectoryPoints{}, zero_limit);
  EXPECT_EQ(
    memory_.start(0, 10.2, point_at(40.0), make_options(0.12, 0.0), zero_limit).start,
    ProfileStart::NoPreviousPlan);
}

TEST_F(FixedProfile, CanDisableTheNewStopReset)
{
  FixedProfileParameters parameters;
  parameters.reset_on_new_stop = false;
  memory_.set_parameters(parameters);
  memory_.store(0, 1, 10.0, plan_);
  auto input = make_input(40.0);
  input[50].longitudinal_velocity_mps = 0.0F;
  EXPECT_EQ(
    start_with_offsets(0.0, 0.0, 0.0, 10.1, detail::make_fixed_profile_stop_state(input)).start,
    ProfileStart::PreviousPlan);
}

TEST_F(FixedProfile, RestartsWhenTheInputDropsBelowThePlan)
{
  // An input at or above the plan continues it.
  EXPECT_EQ(start_with_input(input_along_plan(0.0)).start, ProfileStart::PreviousPlan);
  EXPECT_EQ(start_with_input(input_along_plan(0.4)).start, ProfileStart::PreviousPlan);
  EXPECT_EQ(
    start_with_input(make_input(plan_[1].pose.position.x)).start, ProfileStart::PreviousPlan);

  // A drop ahead within the margin is ignored; a larger one restarts from the measured state.
  auto input = input_along_plan(0.0);
  input[50].longitudinal_velocity_mps -= 0.4F;
  EXPECT_EQ(start_with_input(input).start, ProfileStart::PreviousPlan);
  input[50].longitudinal_velocity_mps -= 0.2F;
  const auto reset = start_with_input(input);
  ASSERT_EQ(reset.start, ProfileStart::ResetLowerInput);
  EXPECT_DOUBLE_EQ(*reset.options.current_ego_velocity, plan_[1].longitudinal_velocity_mps);
  EXPECT_TRUE(detail::is_reset(reset.start));
  EXPECT_STREQ(detail::to_string(reset.start), "reset_lower_input");
}

TEST_F(FixedProfile, LowerInputIgnoresTheHorizonEndAndPointsBeyondThePlan)
{
  auto input = input_along_plan(0.0);
  input.back().longitudinal_velocity_mps = 0.0F;
  EXPECT_EQ(start_with_input(input).start, ProfileStart::PreviousPlan);

  // The cruising input reaches beyond the end of the plan, which brakes for the zone.
  auto longer = make_input(plan_[1].pose.position.x, 2 * plan_.size());
  for (auto & point : longer) {
    if (point.pose.position.x > plan_.back().pose.position.x + 1.0) {
      point.longitudinal_velocity_mps = 1.0F;
    }
  }
  EXPECT_EQ(start_with_input(longer).start, ProfileStart::PreviousPlan);
}

TEST_F(FixedProfile, LowerInputResetFollowsItsParameters)
{
  FixedProfileParameters parameters;
  parameters.lower_input_velocity_margin = 0.3;
  memory_.set_parameters(parameters);
  memory_.store(0, 1, 10.0, plan_);
  EXPECT_EQ(start_with_input(input_along_plan(-0.4)).start, ProfileStart::ResetLowerInput);

  parameters.reset_on_lower_input = false;
  memory_.set_parameters(parameters);
  memory_.store(0, 1, 10.0, plan_);
  EXPECT_EQ(start_with_input(input_along_plan(-1.0)).start, ProfileStart::PreviousPlan);
}

TEST(FixedProfileReplanning, StartingFromThePreviousPlanReproducesIt)
{
  // Every cycle, a new input starts where the previous plan says the ego is, and the measured
  // state is off the plan by a small error. Starting from the previous plan, the successive plans
  // continue the first one, including its braking for the zone, which keeps them below the input.
  const auto first = make_plan();
  FixedProfileMemory memory;
  memory.set_parameters(FixedProfileParameters{});
  memory.store(0, 1, 0.0, first);
  auto previous = first;
  for (std::size_t cycle = 1; cycle <= 40; ++cycle) {
    const double time = dt * static_cast<double>(cycle);
    const auto & planned = previous[1];
    const double error = cycle % 2 == 0 ? 0.2 : -0.2;
    auto plan = make_input(planned.pose.position.x);
    const auto start = memory.start(
      0, time, planned.pose.position,
      make_options(planned.longitudinal_velocity_mps + error, planned.acceleration_mps2 - error),
      {}, plan);
    ASSERT_EQ(start.start, ProfileStart::PreviousPlan) << cycle;

    detail::apply_velocity_limits(plan, constraints, zone, start.options);
    memory.store(0, 1, time, plan);
    for (std::size_t i = 0; i + cycle < first.size(); ++i) {
      EXPECT_NEAR(
        plan[i].longitudinal_velocity_mps, first[i + cycle].longitudinal_velocity_mps, 2e-2)
        << "cycle " << cycle << " point " << i;
    }
    previous = plan;
  }
}

}  // namespace
