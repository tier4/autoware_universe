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

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <optional>
#include <vector>

namespace autoware::mppi_optimizer::detail
{
namespace
{

geometry_msgs::msg::Quaternion makeQuaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

Trajectory makeTrajectory(const std::size_t point_count, const double spacing, const float velocity)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 42;
  for (std::size_t i = 0; i < point_count; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i) * spacing;
    point.pose.position.y = -0.5 * static_cast<double>(i);
    point.pose.position.z = 3.0 + static_cast<double>(i);
    point.pose.orientation = makeQuaternion(0.1 * static_cast<double>(i));
    point.longitudinal_velocity_mps = velocity;
    point.lateral_velocity_mps = static_cast<float>(i) + 0.5F;
    point.acceleration_mps2 = static_cast<float>(i);
    point.front_wheel_angle_rad = 0.01F * static_cast<float>(i);
    point.time_from_start.sec = static_cast<std::int32_t>(i);
    point.time_from_start.nanosec = 123U;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

void appendPosition(Trajectory & trajectory, const double x, const double y)
{
  autoware_planning_msgs::msg::TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  trajectory.points.push_back(point);
}

TEST(TrajectoryEligibility, SkipsOnlyTrajectoriesThatAreBothShortAndStopping)
{
  auto short_stopping = makeTrajectory(3U, 1.0, 1.0F);
  short_stopping.points[1].longitudinal_velocity_mps = 0.0F;
  EXPECT_FALSE(isOptimizationRequired(short_stopping, 4.0));

  auto short_moving = makeTrajectory(3U, 1.0, 0.02F);
  EXPECT_TRUE(isOptimizationRequired(short_moving, 4.0));

  auto long_stopping = makeTrajectory(3U, 2.0, 1.0F);
  long_stopping.points[1].longitudinal_velocity_mps = 0.0F;
  EXPECT_TRUE(isOptimizationRequired(long_stopping, 4.0));

  EXPECT_TRUE(isOptimizationRequired(short_stopping, 0.0));
  EXPECT_TRUE(isOptimizationRequired(short_stopping, 2.0));
}

TEST(InitialState, UsesOdometryDefaultsAndClampsOptionalVehicleState)
{
  Odometry odometry;
  odometry.pose.pose.position.x = 12.0;
  odometry.pose.pose.position.y = -4.0;
  odometry.pose.pose.orientation = makeQuaternion(0.7);
  odometry.twist.twist.linear.x = -2.5;

  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.vel_rate_lim = 3.0F;
  vehicle.max_steer_angle = 0.4F;

  const auto defaults = makeInitialState(odometry, std::nullopt, std::nullopt, vehicle);
  EXPECT_FLOAT_EQ(defaults.x, 12.0F);
  EXPECT_FLOAT_EQ(defaults.y, -4.0F);
  EXPECT_NEAR(defaults.yaw, 0.7F, 1.0E-6F);
  EXPECT_FLOAT_EQ(defaults.velocity, -2.5F);
  EXPECT_FLOAT_EQ(defaults.acceleration, 0.0F);
  EXPECT_FLOAT_EQ(defaults.steering, 0.0F);

  geometry_msgs::msg::AccelWithCovarianceStamped acceleration;
  acceleration.accel.accel.linear.x = 10.0;
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.steering_tire_angle = -1.0F;
  const auto clamped = makeInitialState(odometry, acceleration, steering, vehicle);
  EXPECT_FLOAT_EQ(clamped.acceleration, vehicle.max_accel());
  EXPECT_FLOAT_EQ(clamped.steering, -vehicle.max_steer_angle);
}

TEST(ReferenceHorizon, MapsInputDirectlyAndHoldsTheLastSample)
{
  auto trajectory = makeTrajectory(2U, 1.0, 3.0F);
  trajectory.points[0].longitudinal_velocity_mps = 1.0F;
  trajectory.points[1].longitudinal_velocity_mps = 2.0F;
  InitialState ego;

  const auto reference = buildReferenceHorizon(trajectory, ego, 4, 0.1F);
  ASSERT_EQ(reference.size(), 4U);
  EXPECT_FLOAT_EQ(reference[0].time, 0.1F);
  EXPECT_FLOAT_EQ(reference[0].x, 0.0F);
  EXPECT_FLOAT_EQ(reference[0].velocity, 1.0F);
  EXPECT_FLOAT_EQ(reference[1].time, 0.2F);
  EXPECT_FLOAT_EQ(reference[1].x, 1.0F);
  EXPECT_NEAR(reference[1].yaw, 0.1F, 1.0E-6F);
  EXPECT_FLOAT_EQ(reference[3].time, 0.4F);
  EXPECT_FLOAT_EQ(reference[3].x, reference[1].x);
  EXPECT_FLOAT_EQ(reference[3].velocity, reference[1].velocity);
}

TEST(ReferenceHorizon, EmptyInputHoldsTheMeasuredEgoState)
{
  InitialState ego;
  ego.x = 1.0F;
  ego.y = 2.0F;
  ego.yaw = 0.3F;
  ego.velocity = 4.0F;

  const auto reference = buildReferenceHorizon(Trajectory{}, ego, 2, 0.1F);
  ASSERT_EQ(reference.size(), 2U);
  for (const auto & sample : reference) {
    EXPECT_FLOAT_EQ(sample.x, ego.x);
    EXPECT_FLOAT_EQ(sample.y, ego.y);
    EXPECT_FLOAT_EQ(sample.yaw, ego.yaw);
    EXPECT_FLOAT_EQ(sample.velocity, ego.velocity);
    EXPECT_FLOAT_EQ(sample.arc_length_s, 0.0F);
  }
}

TEST(VelocityLimitProfile, CombinesExternalAndMapLimitsPointwise)
{
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 8.0F;
  limits.max_velocity_by_reference_point = {10.0F, 4.0F, std::nullopt, -1.0F};

  const auto effective = buildEffectiveMaximumVelocityProfile(4U, limits);

  ASSERT_EQ(effective.size(), 4U);
  ASSERT_TRUE(effective[0]);
  ASSERT_TRUE(effective[1]);
  ASSERT_TRUE(effective[2]);
  ASSERT_TRUE(effective[3]);
  EXPECT_FLOAT_EQ(*effective[0], 8.0F);
  EXPECT_FLOAT_EQ(*effective[1], 4.0F);
  EXPECT_FLOAT_EQ(*effective[2], 8.0F);
  EXPECT_FLOAT_EQ(*effective[3], 8.0F);
  ASSERT_FALSE(getUniformMaximumVelocity(effective));
}

TEST(VelocityLimitProfile, RecognizesUniformExternalLimit)
{
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 4.0F;

  const auto effective = buildEffectiveMaximumVelocityProfile(5U, limits);
  const auto uniform = getUniformMaximumVelocity(effective);

  ASSERT_TRUE(uniform);
  EXPECT_FLOAT_EQ(*uniform, 4.0F);
}

TEST(ReferenceHorizon, CarriesAndAppliesPointwiseMaximumVelocity)
{
  auto trajectory = makeTrajectory(3U, 1.0, 8.0F);
  const std::vector<std::optional<float>> maximum_velocities = {10.0F, 4.0F, std::nullopt};

  const auto reference =
    buildReferenceHorizon(trajectory, InitialState{}, 3, 0.1F, 0U, nullptr, &maximum_velocities);

  ASSERT_EQ(reference.size(), 3U);
  EXPECT_FLOAT_EQ(reference[0].velocity, 8.0F);
  ASSERT_TRUE(reference[0].max_velocity);
  EXPECT_FLOAT_EQ(*reference[0].max_velocity, 10.0F);
  EXPECT_FLOAT_EQ(reference[1].velocity, 4.0F);
  ASSERT_TRUE(reference[1].max_velocity);
  EXPECT_FLOAT_EQ(*reference[1].max_velocity, 4.0F);
  EXPECT_FALSE(reference[2].max_velocity);
  EXPECT_FLOAT_EQ(reference[2].velocity, 8.0F);
}

TEST(ActiveVelocityLimitProfile, InactivePathPreservesControlsAndTrajectoryExactly)
{
  const std::vector<FirstOrderDubinsMppiControl> controls = {{-1.0F, 0.2F}, {0.5F, -0.3F}};
  InitialState ego;
  ego.velocity = 5.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;

  const auto without_limit =
    buildActiveVelocityLimitProfile(controls, ego, FirstOrderDubinsMppiKinematicLimits{}, vehicle);
  EXPECT_FALSE(without_limit.active);
  ASSERT_EQ(without_limit.controls.size(), controls.size());
  for (std::size_t index = 0; index < controls.size(); ++index) {
    EXPECT_FLOAT_EQ(without_limit.controls[index].accel_cmd, controls[index].accel_cmd);
    EXPECT_FLOAT_EQ(without_limit.controls[index].steer_cmd, controls[index].steer_cmd);
  }

  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = ego.velocity;
  const auto nonrestrictive = buildActiveVelocityLimitProfile(controls, ego, limits, vehicle);
  EXPECT_FALSE(nonrestrictive.active);
  ASSERT_EQ(nonrestrictive.controls.size(), controls.size());
  for (std::size_t index = 0; index < controls.size(); ++index) {
    EXPECT_FLOAT_EQ(nonrestrictive.controls[index].accel_cmd, controls[index].accel_cmd);
    EXPECT_FLOAT_EQ(nonrestrictive.controls[index].steer_cmd, controls[index].steer_cmd);
  }

  auto trajectory = makeTrajectory(3U, 1.0, 3.0F);
  const auto original = trajectory;
  applyActiveVelocityLimitProfile(trajectory, nonrestrictive);
  EXPECT_TRUE(trajectory == original);
}

TEST(ActiveVelocityLimitProfile, BuildsDelayAndJerkAwareFastStop)
{
  std::vector<FirstOrderDubinsMppiControl> controls(40U, {0.7F, 0.1F});
  InitialState ego;
  ego.velocity = 4.0F;
  ego.acceleration = 0.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.1F;
  vehicle.vel_rate_lim = 3.0F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 0.0F;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto profile =
    buildActiveVelocityLimitProfile(controls, ego, limits, vehicle, 1, {0.0F}, 0.1F);

  ASSERT_TRUE(profile.active);
  ASSERT_EQ(profile.controls.size(), controls.size());
  ASSERT_EQ(profile.velocities.size(), controls.size());
  ASSERT_EQ(profile.accelerations.size(), controls.size());
  EXPECT_FLOAT_EQ(profile.controls[0].accel_cmd, -1.0F);
  EXPECT_FLOAT_EQ(profile.controls[1].accel_cmd, -2.0F);
  EXPECT_FLOAT_EQ(profile.controls[2].accel_cmd, -2.0F);
  EXPECT_FLOAT_EQ(profile.velocities[0], 4.0F);
  EXPECT_FLOAT_EQ(profile.velocities[1], 4.0F);
  EXPECT_FLOAT_EQ(profile.velocities[2], 3.9F);
  EXPECT_FLOAT_EQ(profile.accelerations[0], 0.0F);
  EXPECT_FLOAT_EQ(profile.accelerations[1], -1.0F);
  EXPECT_FLOAT_EQ(profile.accelerations[2], -2.0F);
  EXPECT_LE(profile.velocities.back(), 0.05F);
  for (std::size_t index = 0; index < controls.size(); ++index) {
    EXPECT_FLOAT_EQ(profile.controls[index].steer_cmd, controls[index].steer_cmd);
    EXPECT_GE(profile.controls[index].accel_cmd, -2.0F);
    EXPECT_LE(profile.controls[index].accel_cmd, 0.0F);
    EXPECT_GE(profile.velocities[index], 0.0F);
  }
}

TEST(ActiveVelocityLimitProfile, SettlesAtNonzeroLimitInsteadOfStopping)
{
  const std::vector<FirstOrderDubinsMppiControl> controls(40U);
  InitialState ego;
  ego.velocity = 4.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.1F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 2.0F;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto profile = buildActiveVelocityLimitProfile(controls, ego, limits, vehicle);

  ASSERT_TRUE(profile.active);
  EXPECT_NEAR(profile.velocities.back(), 2.0F, 0.05F);
  EXPECT_NEAR(profile.accelerations.back(), 0.0F, 1.0E-5F);

  auto trajectory = makeTrajectory(45U, 1.0, 9.0F);
  applyActiveVelocityLimitProfile(trajectory, profile);
  EXPECT_NEAR(trajectory.points[39].longitudinal_velocity_mps, 2.0F, 0.05F);
  EXPECT_FLOAT_EQ(trajectory.points.back().longitudinal_velocity_mps, 2.0F);
  EXPECT_FLOAT_EQ(trajectory.points.back().acceleration_mps2, 0.0F);
}

TEST(ActiveVelocityLimitProfile, AnticipatesADecreasingPointwiseLimit)
{
  const std::vector<FirstOrderDubinsMppiControl> controls(30U);
  InitialState ego;
  ego.velocity = 4.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.1F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity_by_reference_point.resize(controls.size(), 4.0F);
  for (std::size_t index = 15U; index < controls.size(); ++index) {
    limits.max_velocity_by_reference_point[index] = 2.0F;
  }
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto profile = buildActiveVelocityLimitProfile(controls, ego, limits, vehicle);

  ASSERT_TRUE(profile.active);
  ASSERT_EQ(profile.velocities.size(), controls.size());
  const auto first_braking_command = std::find_if(
    profile.controls.begin(), profile.controls.end(),
    [](const auto & control) { return control.accel_cmd < -1.0E-4F; });
  ASSERT_NE(first_braking_command, profile.controls.end());
  EXPECT_LT(
    static_cast<std::size_t>(std::distance(profile.controls.begin(), first_braking_command)), 15U);
  EXPECT_NEAR(profile.velocities.back(), 2.0F, 0.1F);
}

TEST(ActiveVelocityLimitProfile, AcceleratesTowardReferenceAfterPointwiseLimitIncreases)
{
  const std::vector<FirstOrderDubinsMppiControl> controls(60U);
  const std::vector<float> reference_velocities(controls.size(), 4.0F);
  InitialState ego;
  ego.velocity = 4.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.1F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity_by_reference_point.resize(controls.size(), 4.0F);
  for (std::size_t index = 0U; index < 20U; ++index) {
    limits.max_velocity_by_reference_point[index] = 2.0F;
  }
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto profile = buildActiveVelocityLimitProfile(
    controls, ego, limits, vehicle, 0, {}, 0.1F, false, reference_velocities);

  ASSERT_TRUE(profile.active);
  EXPECT_TRUE(
    std::any_of(profile.controls.begin() + 20, profile.controls.end(), [](const auto & control) {
      return control.accel_cmd > 1.0E-4F;
    }));
  EXPECT_GT(profile.velocities.back(), 2.0F);
  EXPECT_LE(profile.velocities.back(), 4.0F + 0.1F);
}

TEST(ActiveVelocityLimitProfile, UsesLookaheadProportionalAccelerationNearReference)
{
  const std::vector<FirstOrderDubinsMppiControl> controls(3U);
  const std::vector<float> reference_velocities = {1.0F, 1.1F, 1.1F};
  InitialState ego;
  ego.velocity = 1.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.1F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity_by_reference_point = {1.0F, 2.0F, 2.0F};
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto profile = buildActiveVelocityLimitProfile(
    controls, ego, limits, vehicle, 0, {}, 0.1F, true, reference_velocities);

  ASSERT_TRUE(profile.active);
  ASSERT_EQ(profile.controls.size(), controls.size());
  EXPECT_NEAR(profile.controls[1].accel_cmd, 0.2F, 1.0E-5F);
  EXPECT_LT(profile.controls[1].accel_cmd, limits.max_longitudinal_acceleration.value());
}

TEST(ActiveVelocityLimitProfile, RetainsOnlyAnExplicitlyActiveUnchangedLimit)
{
  const std::vector<FirstOrderDubinsMppiControl> controls(4U, {0.3F, -0.1F});
  InitialState ego;
  ego.velocity = 2.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 2.0F;

  const auto newly_nonrestrictive = buildActiveVelocityLimitProfile(controls, ego, limits, vehicle);
  EXPECT_FALSE(newly_nonrestrictive.active);

  const auto retained =
    buildActiveVelocityLimitProfile(controls, ego, limits, vehicle, 0, {}, 0.1F, true);
  EXPECT_TRUE(retained.active);

  const auto cleared = buildActiveVelocityLimitProfile(
    controls, ego, FirstOrderDubinsMppiKinematicLimits{}, vehicle, 0, {}, 0.1F, true);
  EXPECT_FALSE(cleared.active);
  ASSERT_EQ(cleared.controls.size(), controls.size());
  for (std::size_t index = 0; index < controls.size(); ++index) {
    EXPECT_FLOAT_EQ(cleared.controls[index].accel_cmd, controls[index].accel_cmd);
    EXPECT_FLOAT_EQ(cleared.controls[index].steer_cmd, controls[index].steer_cmd);
  }
}

TEST(CumulativeChordLength, AccumulatesPolylineSegmentLengthsByIndex)
{
  Trajectory trajectory;
  for (std::size_t i = 0; i < 4U; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i) * 3.0;
    point.pose.position.y = 0.0;
    trajectory.points.push_back(point);
  }

  const auto s = computeCumulativeChordLength(trajectory);
  ASSERT_EQ(s.size(), 4U);
  EXPECT_FLOAT_EQ(s[0], 0.0F);
  EXPECT_FLOAT_EQ(s[1], 3.0F);
  EXPECT_FLOAT_EQ(s[2], 6.0F);
  EXPECT_FLOAT_EQ(s[3], 9.0F);

  InitialState ego;
  const auto reference = buildReferenceHorizon(trajectory, ego, 3, 0.1F, 1U, &s);
  ASSERT_EQ(reference.size(), 3U);
  EXPECT_FLOAT_EQ(reference[0].arc_length_s, 3.0F);
  EXPECT_FLOAT_EQ(reference[1].arc_length_s, 6.0F);
  EXPECT_FLOAT_EQ(reference[2].arc_length_s, 9.0F);
}

TEST(NominalControl, CopiesClampsPadsAndDerivesSteeringFromCurvature)
{
  // 1. Create a 3-point trajectory so Menger curvature can evaluate 3 non-collinear points
  auto trajectory = makeTrajectory(3U, 2.0, 2.0F);
  trajectory.points[0].acceleration_mps2 = 20.0F;
  trajectory.points[0].front_wheel_angle_rad = 0.0F;  // Zero -> triggers Menger curvature fallback
  trajectory.points[0].pose.orientation = makeQuaternion(0.0);

  trajectory.points[1].acceleration_mps2 = -20.0F;
  trajectory.points[1].front_wheel_angle_rad =
    1.0F;  // Non-zero -> uses explicit value (clamped to max)
  trajectory.points[1].pose.orientation = makeQuaternion(0.2);

  trajectory.points[2].acceleration_mps2 = -20.0F;
  trajectory.points[2].front_wheel_angle_rad = 1.0F;
  trajectory.points[2].pose.orientation = makeQuaternion(0.4);

  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.vel_rate_lim = 3.0F;
  vehicle.max_steer_angle = 0.4F;
  vehicle.wheel_base = 0.32F;
  const auto nominal = buildDiffusionNominalControl(trajectory, 0U, vehicle, 4);

  ASSERT_EQ(nominal.size(), 4U);
  EXPECT_FLOAT_EQ(nominal[0].accel_cmd, vehicle.max_accel());

  // 2. Compute expected Menger curvature for the 3 points in makeTrajectory(3U, 2.0, ...)
  // Points are: p0=(0, 0), p1=(2.0, -0.5), p2=(4.0, -1.0) -> Note: collinear if y is linear!
  // To test curvature, let's make p1 slightly offset so it has real curvature:
  const float expected_curvature = computeMengerCurvatureWithMinChord(trajectory.points, 0U, 1.5);
  EXPECT_NEAR(nominal[0].steer_cmd, std::atan(vehicle.wheel_base * expected_curvature), 1.0E-6F);

  EXPECT_FLOAT_EQ(nominal[1].accel_cmd, vehicle.min_accel());
  EXPECT_FLOAT_EQ(nominal[1].steer_cmd, vehicle.max_steer_angle);  // Clamped from 1.0F -> 0.4F
  EXPECT_FLOAT_EQ(nominal[3].accel_cmd, nominal[1].accel_cmd);
  EXPECT_FLOAT_EQ(nominal[3].steer_cmd, nominal[1].steer_cmd);
}

TEST(MengerCurvature, CollinearPointsHaveZeroCurvatureWithVariableSpacing)
{
  Trajectory trajectory;
  for (const double x : {0.0, 0.1, 0.4, 2.0, 2.1, 4.0}) {
    appendPosition(trajectory, x, 0.0);
  }

  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    EXPECT_FLOAT_EQ(computeMengerCurvatureWithMinChord(trajectory.points, i, 1.5F), 0.0F);
  }
}

TEST(MengerCurvature, DenseCircleMatchesInverseRadius)
{
  constexpr double radius = 10.0;
  constexpr double arc_spacing = 0.1;
  constexpr std::size_t point_count = 80U;
  Trajectory trajectory;
  for (std::size_t i = 0; i < point_count; ++i) {
    const double angle = static_cast<double>(i) * arc_spacing / radius;
    appendPosition(trajectory, radius * std::cos(angle), radius * std::sin(angle));
  }

  const float curvature = computeMengerCurvatureWithMinChord(trajectory.points, 20U, 1.5F);
  EXPECT_NEAR(curvature, 1.0 / radius, 0.001);
}

TEST(MengerCurvature, DistanceWindowAttenuatesDenseLateralJitter)
{
  Trajectory trajectory;
  for (std::size_t i = 0; i < 80U; ++i) {
    const double y = i % 2U == 0U ? -0.05 : 0.05;
    appendPosition(trajectory, 0.1 * static_cast<double>(i), y);
  }

  float adjacent_peak = 0.0F;
  float windowed_peak = 0.0F;
  for (std::size_t i = 1U; i + 1U < trajectory.points.size(); ++i) {
    adjacent_peak = std::max(
      adjacent_peak, std::abs(computeMengerCurvatureWithMinChord(trajectory.points, i, 0.0F)));
    windowed_peak = std::max(
      windowed_peak, std::abs(computeMengerCurvatureWithMinChord(trajectory.points, i, 1.5F)));
  }

  EXPECT_LT(windowed_peak, adjacent_peak);
}

TEST(MengerCurvature, EndpointsAndNearEndpointsRemainFinite)
{
  Trajectory trajectory;
  for (std::size_t i = 0; i < 20U; ++i) {
    appendPosition(
      trajectory, 0.1 * static_cast<double>(i), 0.05 * std::sin(static_cast<double>(i)));
  }

  const std::vector<std::size_t> indices = {
    0U, 1U, trajectory.points.size() - 2U, trajectory.points.size() - 1U};
  for (const std::size_t i : indices) {
    EXPECT_TRUE(std::isfinite(computeMengerCurvatureWithMinChord(trajectory.points, i, 1.5F)));
  }
}

TEST(NominalControl, CurvatureChordParameterSmoothsColdStartSteeringSeed)
{
  Trajectory trajectory;
  for (std::size_t i = 0; i < 80U; ++i) {
    const double y = i % 2U == 0U ? -0.05 : 0.05;
    appendPosition(trajectory, 0.1 * static_cast<double>(i), y);
  }

  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.wheel_base = 4.76F;
  vehicle.max_steer_angle = 1.5F;
  const auto adjacent = buildDiffusionNominalControl(trajectory, 20U, vehicle, 30, 0.0F);
  const auto windowed = buildDiffusionNominalControl(trajectory, 20U, vehicle, 30, 1.5F);
  float adjacent_peak = 0.0F;
  float windowed_peak = 0.0F;
  for (std::size_t i = 0; i < adjacent.size(); ++i) {
    adjacent_peak = std::max(adjacent_peak, std::abs(adjacent[i].steer_cmd));
    windowed_peak = std::max(windowed_peak, std::abs(windowed[i].steer_cmd));
  }
  EXPECT_LT(windowed_peak, adjacent_peak);
}

TEST(NominalControl, ForcedControlPadsClampsAndShiftHoldsTheTerminalCommand)
{
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.vel_rate_lim = 2.0F;
  vehicle.max_steer_angle = 0.3F;
  const auto forced = buildForcedNominalControl({10.0F, -1.0F}, {1.0F}, vehicle, 4);
  ASSERT_EQ(forced.size(), 4U);
  EXPECT_FLOAT_EQ(forced[0].accel_cmd, 2.0F);
  EXPECT_FLOAT_EQ(forced[0].steer_cmd, 0.3F);
  EXPECT_FLOAT_EQ(forced[3].accel_cmd, -1.0F);
  EXPECT_FLOAT_EQ(forced[3].steer_cmd, 0.3F);

  const auto shifted = shiftNominalControl(forced, 4);
  ASSERT_EQ(shifted.size(), 4U);
  EXPECT_FLOAT_EQ(shifted[0].accel_cmd, forced[1].accel_cmd);
  EXPECT_FLOAT_EQ(shifted[2].accel_cmd, forced[3].accel_cmd);
  EXPECT_FLOAT_EQ(shifted[3].accel_cmd, forced[3].accel_cmd);
}

TEST(NominalControlFilter, LeavesNominalExactlyUnchangedWithoutExternalLimits)
{
  const std::vector<FirstOrderDubinsMppiControl> nominal = {{-1.0F, 0.2F}, {0.5F, -0.3F}};
  InitialState ego;
  ego.velocity = -1.0F;
  ego.acceleration = 0.4F;
  FirstOrderDubinsMppiVehicleParams vehicle;

  const auto filtered = filterNominalControlWithKinematicLimits(
    nominal, ego, FirstOrderDubinsMppiKinematicLimits{}, vehicle, 1, {2.0F});

  ASSERT_EQ(filtered.size(), nominal.size());
  for (std::size_t i = 0; i < nominal.size(); ++i) {
    EXPECT_FLOAT_EQ(filtered[i].accel_cmd, nominal[i].accel_cmd);
    EXPECT_FLOAT_EQ(filtered[i].steer_cmd, nominal[i].steer_cmd);
  }
}

TEST(NominalControlFilter, ClampsAccelerationAndAppliesJerkAtCommandApplicationTime)
{
  const std::vector<FirstOrderDubinsMppiControl> nominal = {{-3.0F, 0.2F}, {3.0F, -0.3F}};
  InitialState ego;
  ego.velocity = 1.0F;
  ego.acceleration = 0.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.2F;
  vehicle.vel_rate_lim = 3.0F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -5.0F;
  limits.max_longitudinal_jerk = 4.0F;

  const auto filtered =
    filterNominalControlWithKinematicLimits(nominal, ego, limits, vehicle, 0, {}, 0.1F);

  ASSERT_EQ(filtered.size(), nominal.size());
  EXPECT_FLOAT_EQ(filtered[0].accel_cmd, -1.0F);
  EXPECT_NEAR(filtered[1].accel_cmd, 0.3F, 1.0E-6F);
  EXPECT_FLOAT_EQ(filtered[0].steer_cmd, nominal[0].steer_cmd);
  EXPECT_FLOAT_EQ(filtered[1].steer_cmd, nominal[1].steer_cmd);
}

TEST(NominalControlFilter, ZeroVelocityLimitSeedsStrongestDelayAwareBraking)
{
  const std::vector<FirstOrderDubinsMppiControl> nominal(4U, {0.0F, 0.1F});
  InitialState ego;
  ego.velocity = 4.0F;
  ego.acceleration = 0.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.1F;
  vehicle.vel_rate_lim = 3.0F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 0.0F;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto filtered =
    filterNominalControlWithKinematicLimits(nominal, ego, limits, vehicle, 1, {0.0F}, 0.1F);

  ASSERT_EQ(filtered.size(), nominal.size());
  EXPECT_FLOAT_EQ(filtered[0].accel_cmd, -1.0F);
  EXPECT_FLOAT_EQ(filtered[1].accel_cmd, -2.0F);
  EXPECT_FLOAT_EQ(filtered[2].accel_cmd, -2.0F);
  EXPECT_FLOAT_EQ(filtered[3].accel_cmd, -2.0F);
}

TEST(NominalControlFilter, UsesLookaheadProportionalBrakingNearVelocityLimit)
{
  const std::vector<FirstOrderDubinsMppiControl> nominal(1U, {0.0F, 0.1F});
  InitialState ego;
  ego.velocity = 2.1F;
  ego.acceleration = 0.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  vehicle.acc_time_constant = 0.1F;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 2.0F;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto filtered =
    filterNominalControlWithKinematicLimits(nominal, ego, limits, vehicle, 0, {}, 0.1F);

  ASSERT_EQ(filtered.size(), nominal.size());
  EXPECT_NEAR(filtered[0].accel_cmd, -0.2F, 1.0E-5F);
  EXPECT_GT(filtered[0].accel_cmd, limits.min_longitudinal_acceleration.value());
}

TEST(NominalControlFilter, DoesNotImposeMinimumVelocityWithoutVelocityLimit)
{
  const std::vector<FirstOrderDubinsMppiControl> nominal(1U, {-1.0F, 0.1F});
  InitialState ego;
  ego.velocity = -2.0F;
  FirstOrderDubinsMppiVehicleParams vehicle;
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;

  const auto filtered = filterNominalControlWithKinematicLimits(nominal, ego, limits, vehicle);

  ASSERT_EQ(filtered.size(), nominal.size());
  EXPECT_FLOAT_EQ(filtered[0].accel_cmd, nominal[0].accel_cmd);
}

TEST(OutputConversion, OverwritesOnlyAvailablePostStepSamples)
{
  const auto input = makeTrajectory(3U, 1.0, 2.0F);
  const std::vector<OptimizedState> states = {
    {10.0F, 20.0F, 0.3F, 4.0F, 0.7F, 0.1F}, {11.0F, 21.0F, 0.4F, 5.0F, 0.8F, 0.2F}};
  const std::vector<FirstOrderDubinsMppiControl> controls = {{0.5F, -0.1F}, {0.6F, -0.2F}};

  const auto output = buildOptimizedTrajectory(input, states, controls);
  ASSERT_EQ(output.points.size(), input.points.size());
  EXPECT_EQ(output.header, input.header);
  EXPECT_DOUBLE_EQ(output.points[0].pose.position.x, 10.0);
  EXPECT_DOUBLE_EQ(output.points[0].pose.position.y, 20.0);
  EXPECT_DOUBLE_EQ(output.points[0].pose.position.z, input.points[0].pose.position.z);
  EXPECT_NEAR(tf2::getYaw(output.points[0].pose.orientation), 0.3, 1.0E-6);
  EXPECT_FLOAT_EQ(output.points[0].longitudinal_velocity_mps, 4.0F);
  EXPECT_FLOAT_EQ(output.points[0].acceleration_mps2, controls[0].accel_cmd);
  EXPECT_FLOAT_EQ(output.points[0].front_wheel_angle_rad, controls[0].steer_cmd);
  EXPECT_FLOAT_EQ(output.points[0].lateral_velocity_mps, input.points[0].lateral_velocity_mps);
  EXPECT_TRUE(output.points[2] == input.points[2]);
}

TEST(EngageVelocity, ChangesAllLeadingStoppedPointsWhenMovementIsRequested)
{
  auto trajectory = makeTrajectory(4U, 1.0, 0.0F);
  trajectory.points[3].longitudinal_velocity_mps = 1.0F;
  setInitialEngageVelocity(trajectory);
  EXPECT_FLOAT_EQ(trajectory.points[0].longitudinal_velocity_mps, 0.25F);
  EXPECT_FLOAT_EQ(trajectory.points[1].longitudinal_velocity_mps, 0.25F);
  EXPECT_FLOAT_EQ(trajectory.points[2].longitudinal_velocity_mps, 0.25F);

  auto stopping = makeTrajectory(4U, 1.0, 0.0F);
  setInitialEngageVelocity(stopping);
  EXPECT_FLOAT_EQ(stopping.points[0].longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(stopping.points[1].longitudinal_velocity_mps, 0.0F);

  auto externally_stopped = makeTrajectory(4U, 1.0, 0.0F);
  externally_stopped.points[3].longitudinal_velocity_mps = 1.0F;
  setInitialEngageVelocity(externally_stopped, 0.0F);
  EXPECT_FLOAT_EQ(externally_stopped.points[0].longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(externally_stopped.points[1].longitudinal_velocity_mps, 0.0F);
  EXPECT_FLOAT_EQ(externally_stopped.points[2].longitudinal_velocity_mps, 0.0F);

  auto externally_limited = makeTrajectory(4U, 1.0, 0.0F);
  externally_limited.points[3].longitudinal_velocity_mps = 1.0F;
  setInitialEngageVelocity(externally_limited, 0.1F);
  EXPECT_FLOAT_EQ(externally_limited.points[0].longitudinal_velocity_mps, 0.1F);
  EXPECT_FLOAT_EQ(externally_limited.points[1].longitudinal_velocity_mps, 0.1F);
  EXPECT_FLOAT_EQ(externally_limited.points[2].longitudinal_velocity_mps, 0.1F);
}

TEST(ShiftNominalControlForInputDelay, ShiftsPerChannelAndHoldsTail)
{
  const std::vector<FirstOrderDubinsMppiControl> nominal = {
    {0.0F, 0.0F}, {1.0F, 10.0F}, {2.0F, 20.0F}, {3.0F, 30.0F}};
  const auto shifted = shiftNominalControlForInputDelay(nominal, 1, 2);
  ASSERT_EQ(shifted.size(), nominal.size());
  EXPECT_FLOAT_EQ(shifted[0].accel_cmd, 1.0F);
  EXPECT_FLOAT_EQ(shifted[0].steer_cmd, 20.0F);
  EXPECT_FLOAT_EQ(shifted[1].accel_cmd, 2.0F);
  EXPECT_FLOAT_EQ(shifted[1].steer_cmd, 30.0F);
  EXPECT_FLOAT_EQ(shifted[2].accel_cmd, 3.0F);
  EXPECT_FLOAT_EQ(shifted[2].steer_cmd, 30.0F);
  EXPECT_FLOAT_EQ(shiftNominalControlForInputDelay(nominal, 0, 0)[1].accel_cmd, 1.0F);
}

}  // namespace
}  // namespace autoware::mppi_optimizer::detail
