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
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_kinematic_limits.cuh>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{

Trajectory makeStraightTrajectory(const std::size_t point_count)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 123;
  for (std::size_t i = 0; i < point_count; ++i) {
    autoware_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = 0.2 * static_cast<double>(i + 1U);
    point.pose.position.y = 0.0;
    point.pose.position.z = 1.0 + static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 2.0F;
    point.time_from_start.sec = static_cast<std::int32_t>(i / 10U);
    point.time_from_start.nanosec = static_cast<std::uint32_t>((i % 10U) * 100000000U);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

Odometry makeOdometry()
{
  Odometry odometry;
  odometry.header.frame_id = "map";
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = 2.0;
  return odometry;
}

FirstOrderDubinsMppiOptimizationResult optimize(
  FirstOrderDubinsMppiInterface & interface, const Trajectory & trajectory,
  const Odometry & odometry = makeOdometry(),
  const TrackedObjects & tracked_objects = TrackedObjects{},
  const std::vector<Segment> & road_borders = {},
  const FirstOrderDubinsMppiKinematicLimits & kinematic_limits = {})
{
  return interface.optimizeTrajectory(
    trajectory, odometry, std::nullopt, std::nullopt, tracked_objects, road_borders, {},
    kinematic_limits);
}

TEST(KinematicLimitCost, NormalizesIntervalsAndCapsAggregate)
{
  const FirstOrderDubinsBicycleKinematicLimitData inactive_limits;
  const auto inactive =
    computeCappedKinematicIntervalCost(inactive_limits, 10.0F, 100.0F, -100.0F, 100.0F, -100.0F);
  EXPECT_FLOAT_EQ(inactive.total, 0.0F);

  FirstOrderDubinsBicycleKinematicLimitData limits;
  limits.active_mask = kVelocityLimitActive | kAccelerationLimitActive | kJerkLimitActive;
  limits.min_velocity = 0.0F;
  limits.max_velocity = 2.0F;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 2.0F;
  limits.min_longitudinal_jerk = -3.0F;
  limits.max_longitudinal_jerk = 3.0F;

  const auto inside = computeCappedKinematicIntervalCost(limits, 10.0F, 100.0F, 1.0F, 0.0F, 0.0F);
  EXPECT_FLOAT_EQ(inside.total, 0.0F);

  const auto lower_velocity =
    computeCappedKinematicIntervalCost(limits, 10.0F, 100.0F, -1.0F, 0.0F, 0.0F);
  EXPECT_FLOAT_EQ(lower_velocity.velocity, 10.0F);
  EXPECT_FLOAT_EQ(lower_velocity.total, 10.0F);

  // Each raw violation normalizes to one velocity-equivalent unit before squaring:
  // velocity: 3 - 2 = 1; acceleration: (4 - 2) * 0.5 = 1;
  // jerk: (8 - 3) * 0.2 = 1.
  const auto normalized =
    computeCappedKinematicIntervalCost(limits, 10.0F, 100.0F, 3.0F, 4.0F, 8.0F);
  EXPECT_FLOAT_EQ(normalized.velocity, 10.0F);
  EXPECT_FLOAT_EQ(normalized.acceleration, 10.0F);
  EXPECT_FLOAT_EQ(normalized.jerk, 10.0F);
  EXPECT_FLOAT_EQ(normalized.total, 30.0F);

  const auto capped = computeCappedKinematicIntervalCost(limits, 10.0F, 15.0F, 3.0F, 4.0F, 8.0F);
  EXPECT_NEAR(capped.velocity, 5.0F, 1.0E-5F);
  EXPECT_NEAR(capped.acceleration, 5.0F, 1.0E-5F);
  EXPECT_NEAR(capped.jerk, 5.0F, 1.0E-5F);
  EXPECT_LE(capped.total, 15.0F);

  const auto extreme =
    computeCappedKinematicIntervalCost(limits, 1.0E8F, 100000.0F, 1.0E30F, 1.0E30F, 1.0E30F);
  EXPECT_TRUE(std::isfinite(extreme.total));
  EXPECT_LE(extreme.total, 100000.0F);
}

TrackedObjects makeStationaryBoxObstacle(
  const double x, const double y, const double length, const double width)
{
  TrackedObjects objects;
  objects.objects.emplace_back();
  auto & object = objects.objects.back();
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.position.y = y;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.dimensions.x = length;
  object.shape.dimensions.y = width;
  return objects;
}

TEST(FirstOrderDubinsMppiInterface, SkippedInputsDoNotInitializeCuda)
{
  FirstOrderDubinsMppiInterface interface;
  const auto empty_result = optimize(interface, Trajectory{});
  EXPECT_TRUE(empty_result.trajectory.points.empty());
  EXPECT_FALSE(interface.isInitialized());

  FirstOrderDubinsMppiRuntimeOptions options;
  options.min_optimization_length = 4.0F;
  interface.setRuntimeOptions(options);

  auto short_stopping = makeStraightTrajectory(3U);
  short_stopping.points[1].longitudinal_velocity_mps = 0.0F;
  const auto stopping_result = optimize(interface, short_stopping);
  EXPECT_TRUE(stopping_result.trajectory == short_stopping);
  EXPECT_TRUE(stopping_result.debug.reference_trajectory == short_stopping);
  EXPECT_TRUE(stopping_result.debug.optimized_trajectory == short_stopping);
  EXPECT_FALSE(interface.isInitialized());
}

class FirstOrderDubinsMppiInterfaceGpuTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    const cudaError_t error = cudaGetDeviceCount(&device_count);
    if (error != cudaSuccess || device_count == 0) {
      GTEST_SKIP() << "A CUDA device is required for the MPPI integration test";
    }
    interface_ = std::make_unique<FirstOrderDubinsMppiInterface>();

    // Keep tests that are unrelated to actuator delay aligned with the raw odometry state.
    FirstOrderDubinsMppiVehicleParams vehicle_params;
    vehicle_params.acc_time_delay = 0.0F;
    vehicle_params.steer_time_delay = 0.0F;
    interface_->setVehicleParams(vehicle_params);
  }

  std::unique_ptr<FirstOrderDubinsMppiInterface> interface_;
};

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, ProducesFinitePostStepTrajectoryAndPreservesSuffix)
{
  const auto input = makeStraightTrajectory(85U);
  const auto result = optimize(*interface_, input);

  ASSERT_TRUE(interface_->isInitialized());
  ASSERT_EQ(result.trajectory.points.size(), input.points.size());
  EXPECT_EQ(result.trajectory.header, input.header);
  EXPECT_TRUE(result.debug.reference_trajectory == input);
  EXPECT_TRUE(result.debug.optimized_trajectory == result.trajectory);
  ASSERT_EQ(result.debug.rollouts.size(), 128U);
  for (const auto & rollout : result.debug.rollouts) {
    EXPECT_EQ(rollout.points.size(), static_cast<std::size_t>(detail::kMppiHorizon));
    EXPECT_TRUE(std::isfinite(rollout.cost));
  }
  EXPECT_TRUE(std::isfinite(result.debug.baseline_cost));
  // Full host cost reconstruction is intentionally skipped unless debug logging is enabled.
  EXPECT_EQ(result.debug.cost_breakdown.evaluated_timesteps, 0U);
  EXPECT_EQ(result.debug.nominal_cost_breakdown.evaluated_timesteps, 0U);
  // baseline_cost is the best sampled rollout before MPPI's distribution update and smoothing;
  // it is intentionally not asserted equal to the reconstructed selected trajectory cost.
  EXPECT_TRUE(result.debug.validation.isValid());
  EXPECT_FALSE(result.debug.was_rejected);
  EXPECT_EQ(result.debug.optimal_horizon.size(), static_cast<std::size_t>(detail::kMppiHorizon));
  EXPECT_FLOAT_EQ(result.debug.nominal_control_profile.time_step_s, detail::kMppiDt);
  EXPECT_EQ(
    result.debug.nominal_control_profile.acceleration_commands_mps2.size(),
    static_cast<std::size_t>(detail::kMppiHorizon));
  EXPECT_EQ(
    result.debug.nominal_control_profile.steering_commands_rad.size(),
    static_cast<std::size_t>(detail::kMppiHorizon));

  const auto & first = result.trajectory.points.front();
  EXPECT_NEAR(first.pose.position.x, 0.2, 1.0E-5);
  EXPECT_NEAR(first.pose.position.y, 0.0, 1.0E-5);
  EXPECT_NEAR(first.longitudinal_velocity_mps, 2.0F, 1.0E-5F);
  EXPECT_DOUBLE_EQ(first.pose.position.z, input.points.front().pose.position.z);

  for (std::size_t i = 0; i < static_cast<std::size_t>(detail::kMppiHorizon); ++i) {
    const auto & point = result.trajectory.points[i];
    EXPECT_TRUE(std::isfinite(point.pose.position.x));
    EXPECT_TRUE(std::isfinite(point.pose.position.y));
    EXPECT_TRUE(std::isfinite(point.longitudinal_velocity_mps));
    EXPECT_TRUE(std::isfinite(point.acceleration_mps2));
    EXPECT_TRUE(std::isfinite(point.front_wheel_angle_rad));
    EXPECT_LE(std::abs(point.acceleration_mps2), 7.0F + 1.0E-5F);
    EXPECT_LE(std::abs(point.front_wheel_angle_rad), 0.45F + 1.0E-5F);
  }
  for (std::size_t i = static_cast<std::size_t>(detail::kMppiHorizon); i < input.points.size();
       ++i) {
    EXPECT_TRUE(result.trajectory.points[i] == input.points[i]);
  }
}

TEST_F(
  FirstOrderDubinsMppiInterfaceGpuTest,
  ActiveLimitSampleCostDistributionRemainsFiniteUnderExtremeViolations)
{
  FirstOrderDubinsMppiCostParams cost_params;
  cost_params.overlimit_coeff = 1.0E8F;
  cost_params.crash_contact_penalty = 100000.0F;
  interface_->setCostParams(cost_params);

  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 0.0F;
  limits.min_longitudinal_acceleration = -0.1F;
  limits.max_longitudinal_acceleration = 0.1F;
  limits.min_longitudinal_jerk = -0.1F;
  limits.max_longitudinal_jerk = 0.1F;

  auto odometry = makeOdometry();
  odometry.twist.twist.linear.x = 30.0;
  const auto result =
    optimize(*interface_, makeStraightTrajectory(85U), odometry, TrackedObjects{}, {}, limits);

  EXPECT_TRUE(std::isfinite(result.debug.baseline_cost));
  std::vector<float> costs;
  std::vector<float> weights;
  ASSERT_TRUE(interface_->copySampleCostDistribution(costs, weights));
  ASSERT_FALSE(weights.empty());
  ASSERT_EQ(costs.size(), weights.size());
  float weight_sum = 0.0F;
  for (std::size_t index = 0U; index < weights.size(); ++index) {
    EXPECT_TRUE(std::isfinite(costs[index]));
    EXPECT_TRUE(std::isfinite(weights[index]));
    weight_sum += weights[index];
  }
  EXPECT_NEAR(weight_sum, 1.0F, 1.0E-3F);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, InactiveVelocityLimitProducesIdenticalOutput)
{
  FirstOrderDubinsMppiVehicleParams vehicle_params;
  vehicle_params.acc_time_delay = 0.0F;
  vehicle_params.steer_time_delay = 0.0F;
  const auto input = makeStraightTrajectory(85U);
  FirstOrderDubinsMppiOptimizationResult unrestricted_result;
  FirstOrderDubinsMppiOptimizationResult nonrestrictive_result;
  std::vector<float> unrestricted_acceleration;
  std::vector<float> unrestricted_steering;
  std::vector<float> nonrestrictive_acceleration;
  std::vector<float> nonrestrictive_steering;

  {
    FirstOrderDubinsMppiInterface unrestricted;
    unrestricted.setVehicleParams(vehicle_params);
    unrestricted_result = optimize(unrestricted, input);
    ASSERT_TRUE(
      unrestricted.copyLastOptimizedControl(unrestricted_acceleration, unrestricted_steering));
  }
  {
    FirstOrderDubinsMppiInterface nonrestrictive;
    nonrestrictive.setVehicleParams(vehicle_params);
    FirstOrderDubinsMppiKinematicLimits limits;
    limits.max_velocity = 30.0F;
    nonrestrictive_result =
      optimize(nonrestrictive, input, makeOdometry(), TrackedObjects{}, {}, limits);
    ASSERT_TRUE(nonrestrictive.copyLastOptimizedControl(
      nonrestrictive_acceleration, nonrestrictive_steering));
  }

  EXPECT_TRUE(nonrestrictive_result.trajectory == unrestricted_result.trajectory);
  EXPECT_FALSE(unrestricted_result.debug.external_velocity_limit_active);
  EXPECT_FALSE(nonrestrictive_result.debug.external_velocity_limit_active);
  EXPECT_TRUE(
    nonrestrictive_result.debug.nominal_trajectory == unrestricted_result.debug.nominal_trajectory);
  ASSERT_EQ(nonrestrictive_acceleration.size(), unrestricted_acceleration.size());
  ASSERT_EQ(nonrestrictive_steering.size(), unrestricted_steering.size());
  for (size_t i = 0; i < unrestricted_acceleration.size(); ++i) {
    EXPECT_NEAR(nonrestrictive_acceleration[i], unrestricted_acceleration[i], 1.0E-4F);
    EXPECT_NEAR(nonrestrictive_steering[i], unrestricted_steering[i], 1.0E-4F);
  }
}

TEST_F(
  FirstOrderDubinsMppiInterfaceGpuTest,
  PointwiseLimitsAboveExternalLimitPreserveExternalOutputExactly)
{
  FirstOrderDubinsMppiVehicleParams vehicle_params;
  vehicle_params.acc_time_constant = 0.1F;
  vehicle_params.acc_time_delay = 0.0F;
  vehicle_params.steer_time_delay = 0.0F;
  const auto input = makeStraightTrajectory(85U);
  FirstOrderDubinsMppiKinematicLimits external_limits;
  external_limits.max_velocity = 1.0F;
  external_limits.min_longitudinal_acceleration = -2.0F;
  external_limits.max_longitudinal_acceleration = 1.0F;
  external_limits.min_longitudinal_jerk = -10.0F;
  external_limits.max_longitudinal_jerk = 10.0F;
  auto combined_limits = external_limits;
  combined_limits.max_velocity_by_reference_point.resize(input.points.size(), 4.0F);

  FirstOrderDubinsMppiOptimizationResult external_result;
  FirstOrderDubinsMppiOptimizationResult combined_result;
  std::vector<float> external_acceleration;
  std::vector<float> external_steering;
  std::vector<float> combined_acceleration;
  std::vector<float> combined_steering;
  {
    FirstOrderDubinsMppiInterface external;
    external.setVehicleParams(vehicle_params);
    external_result =
      optimize(external, input, makeOdometry(), TrackedObjects{}, {}, external_limits);
    ASSERT_TRUE(external.copyLastOptimizedControl(external_acceleration, external_steering));
  }
  {
    FirstOrderDubinsMppiInterface combined;
    combined.setVehicleParams(vehicle_params);
    combined_result =
      optimize(combined, input, makeOdometry(), TrackedObjects{}, {}, combined_limits);
    ASSERT_TRUE(combined.copyLastOptimizedControl(combined_acceleration, combined_steering));
  }

  EXPECT_TRUE(combined_result.trajectory == external_result.trajectory);
  EXPECT_TRUE(combined_result.debug.nominal_trajectory == external_result.debug.nominal_trajectory);
  EXPECT_EQ(combined_acceleration, external_acceleration);
  EXPECT_EQ(combined_steering, external_steering);
}

TEST_F(
  FirstOrderDubinsMppiInterfaceGpuTest, ActiveZeroLimitProjectsPostSmootherLongitudinalSequence)
{
  FirstOrderDubinsMppiVehicleParams vehicle_params;
  vehicle_params.acc_time_constant = 0.1F;
  vehicle_params.acc_time_delay = 0.1F;
  vehicle_params.steer_time_delay = 0.0F;
  vehicle_params.vel_rate_lim = 3.0F;
  interface_->setVehicleParams(vehicle_params);

  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 0.0F;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;
  auto odometry = makeOdometry();
  odometry.twist.twist.linear.x = 4.0;

  const auto result =
    optimize(*interface_, makeStraightTrajectory(85U), odometry, TrackedObjects{}, {}, limits);

  EXPECT_TRUE(result.debug.external_velocity_limit_active);
  std::vector<float> acceleration;
  std::vector<float> steering;
  ASSERT_TRUE(interface_->copyLastOptimizedControl(acceleration, steering));
  ASSERT_GE(acceleration.size(), 8U);
  for (std::size_t index = 0U; index < 8U; ++index) {
    EXPECT_FLOAT_EQ(acceleration[index], -2.0F);
  }
  ASSERT_EQ(result.trajectory.points.size(), 85U);
  EXPECT_FLOAT_EQ(result.trajectory.points[0].acceleration_mps2, -1.0F);
  EXPECT_FLOAT_EQ(result.trajectory.points[1].acceleration_mps2, -2.0F);
  EXPECT_FLOAT_EQ(result.trajectory.points[0].longitudinal_velocity_mps, 4.0F);
  EXPECT_FLOAT_EQ(result.trajectory.points[1].longitudinal_velocity_mps, 4.0F);
  EXPECT_FLOAT_EQ(result.trajectory.points[2].longitudinal_velocity_mps, 3.9F);
  EXPECT_FLOAT_EQ(result.trajectory.points.back().longitudinal_velocity_mps, 0.0F);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, AppliesADecreasingPointwiseVelocityLimit)
{
  const auto input = makeStraightTrajectory(85U);
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity_by_reference_point.resize(input.points.size(), 2.0F);
  for (std::size_t index = 20U; index < input.points.size(); ++index) {
    limits.max_velocity_by_reference_point[index] = 0.5F;
  }
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;

  const auto result = optimize(*interface_, input, makeOdometry(), TrackedObjects{}, {}, limits);

  EXPECT_TRUE(result.debug.map_velocity_limit_active);
  EXPECT_TRUE(result.debug.velocity_limit_profile_active);
  EXPECT_FALSE(result.debug.external_velocity_limit_active);
  ASSERT_EQ(result.debug.effective_max_velocity_by_reference_point.size(), input.points.size());
  ASSERT_TRUE(result.debug.effective_max_velocity_by_reference_point[19]);
  ASSERT_TRUE(result.debug.effective_max_velocity_by_reference_point[20]);
  EXPECT_FLOAT_EQ(*result.debug.effective_max_velocity_by_reference_point[19], 2.0F);
  EXPECT_FLOAT_EQ(*result.debug.effective_max_velocity_by_reference_point[20], 0.5F);
  EXPECT_LT(result.trajectory.points[19].longitudinal_velocity_mps, 2.0F);
  EXPECT_NEAR(result.trajectory.points.back().longitudinal_velocity_mps, 0.5F, 0.1F);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, UsesMinimumOfExternalAndPointwiseVelocityLimits)
{
  const auto input = makeStraightTrajectory(85U);
  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 1.5F;
  limits.max_velocity_by_reference_point.resize(input.points.size(), 4.0F);
  limits.max_velocity_by_reference_point[10] = 0.75F;

  const auto result = optimize(*interface_, input, makeOdometry(), TrackedObjects{}, {}, limits);

  ASSERT_EQ(result.debug.effective_max_velocity_by_reference_point.size(), input.points.size());
  ASSERT_TRUE(result.debug.effective_max_velocity_by_reference_point[0]);
  ASSERT_TRUE(result.debug.effective_max_velocity_by_reference_point[10]);
  EXPECT_FLOAT_EQ(*result.debug.effective_max_velocity_by_reference_point[0], 1.5F);
  EXPECT_FLOAT_EQ(*result.debug.effective_max_velocity_by_reference_point[10], 0.75F);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, RejectedActiveLimitRetainsLongitudinalFallback)
{
  FirstOrderDubinsMppiRuntimeOptions options;
  options.skip_if_invalid = true;
  interface_->setRuntimeOptions(options);

  FirstOrderDubinsMppiKinematicLimits limits;
  limits.max_velocity = 0.0F;
  limits.min_longitudinal_acceleration = -2.0F;
  limits.max_longitudinal_acceleration = 1.0F;
  limits.min_longitudinal_jerk = -10.0F;
  limits.max_longitudinal_jerk = 10.0F;
  const auto input = makeStraightTrajectory(30U);
  const auto objects = makeStationaryBoxObstacle(0.4, 0.41, 0.2, 0.2);

  const auto result = optimize(*interface_, input, makeOdometry(), objects, {}, limits);

  EXPECT_TRUE(result.debug.was_rejected);
  EXPECT_TRUE(result.debug.external_velocity_limit_active);
  EXPECT_EQ(result.trajectory.points.size(), input.points.size());
  EXPECT_LT(
    result.trajectory.points.back().longitudinal_velocity_mps,
    input.points.back().longitudinal_velocity_mps);
  EXPECT_TRUE(result.debug.optimized_trajectory == result.trajectory);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, AppliesPerChannelActuatorDelayWithoutRefShift)
{
  FirstOrderDubinsMppiVehicleParams vehicle_params;
  vehicle_params.acc_time_delay = 0.10F;    // N_acc = ceil(0.10/0.1) = 1 at t=0.
  vehicle_params.steer_time_delay = 0.24F;  // N_steer = ceil(0.24/0.1) = 3 at t=0.
  interface_->setVehicleParams(vehicle_params);

  const auto input = makeStraightTrajectory(85U);
  const auto result = optimize(*interface_, input);

  ASSERT_TRUE(interface_->isInitialized());
  ASSERT_EQ(result.trajectory.points.size(), input.points.size());

  // Delay is applied in dynamics from the measured ego IC (no host pre-roll / ref shift),
  // so the first published post-step state stays near the undelayed one-step motion.
  const auto & first = result.trajectory.points.front();
  EXPECT_NEAR(first.pose.position.x, 0.2, 1.0E-5);
  EXPECT_NEAR(first.pose.position.y, 0.0, 1.0E-5);
  EXPECT_NEAR(first.longitudinal_velocity_mps, 2.0F, 1.0E-5F);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, RejectsBeyondLateralBoundaryThreshold)
{
  FirstOrderDubinsMppiCostParams cost_params;
  cost_params.boundary_threshold = 0.5F;
  interface_->setCostParams(cost_params);
  FirstOrderDubinsMppiRuntimeOptions options;
  options.skip_if_invalid = true;
  interface_->setRuntimeOptions(options);

  const auto input = makeStraightTrajectory(30U);
  auto odometry = makeOdometry();
  odometry.pose.pose.position.y = 0.6;
  const auto result = optimize(*interface_, input, odometry);

  EXPECT_TRUE(interface_->isInitialized());
  EXPECT_TRUE(result.trajectory == input);
  EXPECT_TRUE(result.debug.reference_trajectory == input);
  EXPECT_TRUE(result.debug.optimized_trajectory == input);
  EXPECT_TRUE(result.debug.was_rejected);
  EXPECT_TRUE(hasInvalidityReason(
    result.debug.validation.reasons, FirstOrderDubinsMppiInvalidityReason::lateral_boundary));
  ASSERT_TRUE(result.debug.validation.first_invalid_index.has_value());
  EXPECT_EQ(result.debug.validation.first_invalid_index.value(), 0U);
  EXPECT_TRUE(std::isfinite(result.debug.baseline_cost));
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, RejectsRoadBorderInsideConfiguredMargin)
{
  FirstOrderDubinsMppiCostParams cost_params;
  cost_params.boundary_threshold = 100.0F;
  cost_params.road_border_collision_margin = 0.2F;
  interface_->setCostParams(cost_params);
  FirstOrderDubinsMppiRuntimeOptions options;
  options.skip_if_invalid = true;
  interface_->setRuntimeOptions(options);

  const auto input = makeStraightTrajectory(30U);
  const Segment border_outside_physical_footprint{-1.0F, 0.31F, 2.0F, 0.31F};
  const auto result = optimize(
    *interface_, input, makeOdometry(), TrackedObjects{}, {border_outside_physical_footprint});

  EXPECT_TRUE(result.trajectory == input);
  EXPECT_TRUE(result.debug.was_rejected);
  EXPECT_TRUE(hasInvalidityReason(
    result.debug.validation.reasons, FirstOrderDubinsMppiInvalidityReason::road_border));
  ASSERT_TRUE(result.debug.validation.first_invalid_index.has_value());
  EXPECT_EQ(result.debug.validation.first_invalid_index.value(), 0U);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, RejectsObjectInsideConfiguredMargin)
{
  FirstOrderDubinsMppiCostParams cost_params;
  cost_params.boundary_threshold = 100.0F;
  cost_params.obstacle_collision_margin = 0.2F;
  interface_->setCostParams(cost_params);
  FirstOrderDubinsMppiRuntimeOptions options;
  options.skip_if_invalid = true;
  interface_->setRuntimeOptions(options);

  const auto input = makeStraightTrajectory(30U);
  const auto objects = makeStationaryBoxObstacle(0.4, 0.41, 0.2, 0.2);
  const auto result = optimize(*interface_, input, makeOdometry(), objects);

  EXPECT_TRUE(result.trajectory == input);
  EXPECT_TRUE(result.debug.was_rejected);
  EXPECT_TRUE(hasInvalidityReason(
    result.debug.validation.reasons, FirstOrderDubinsMppiInvalidityReason::obstacle));
  ASSERT_TRUE(result.debug.validation.first_invalid_index.has_value());
  EXPECT_EQ(result.debug.validation.first_invalid_index.value(), 0U);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, DoesNotRejectInvalidOutputWhenOptionIsDisabled)
{
  FirstOrderDubinsMppiCostParams cost_params;
  cost_params.boundary_threshold = 0.5F;
  interface_->setCostParams(cost_params);
  FirstOrderDubinsMppiRuntimeOptions options;
  options.skip_if_invalid = false;
  interface_->setRuntimeOptions(options);

  const auto input = makeStraightTrajectory(30U);
  auto odometry = makeOdometry();
  odometry.pose.pose.position.y = 0.6;
  const auto result = optimize(*interface_, input, odometry);

  EXPECT_FALSE(result.debug.validation.isValid());
  EXPECT_TRUE(hasInvalidityReason(
    result.debug.validation.reasons, FirstOrderDubinsMppiInvalidityReason::lateral_boundary));
  EXPECT_FALSE(result.debug.was_rejected);
  EXPECT_FALSE(result.trajectory == input);
  EXPECT_TRUE(result.debug.optimized_trajectory == result.trajectory);
}

TEST_F(FirstOrderDubinsMppiInterfaceGpuTest, HandlesInitialOffsetsAcrossThresholdRange)
{
  FirstOrderDubinsMppiRuntimeOptions options;
  options.skip_if_invalid = true;
  interface_->setRuntimeOptions(options);

  struct TestCase
  {
    float threshold;
    float initial_y_offset;
    bool expect_rejected;
  };

  const std::vector<TestCase> test_cases = {
    {0.50F, 0.30F, false}, {0.50F, -0.30F, false}, {0.50F, 0.60F, true},
    {0.50F, -0.60F, true}, {0.10F, 0.11F, true},
  };

  for (std::size_t idx = 0; idx < test_cases.size(); ++idx) {
    const auto & tc = test_cases[idx];
    FirstOrderDubinsMppiCostParams cost_params;
    cost_params.boundary_threshold = tc.threshold;
    interface_->setCostParams(cost_params);

    const auto input = makeStraightTrajectory(30U);
    auto odometry = makeOdometry();
    odometry.pose.pose.position.y = tc.initial_y_offset;

    const auto result = optimize(*interface_, input, odometry, TrackedObjects{}, {});

    EXPECT_EQ(result.debug.was_rejected, tc.expect_rejected)
      << "Failed at test case index " << idx << " (threshold=" << tc.threshold
      << ", init_y=" << tc.initial_y_offset << ")";

    EXPECT_EQ(result.debug.validation.isValid(), !tc.expect_rejected)
      << "Validation mismatch at test case index " << idx;

    if (tc.expect_rejected) {
      EXPECT_TRUE(result.trajectory == input)
        << "Rejected trajectory was not properly reverted to input at index " << idx;
      EXPECT_TRUE(hasInvalidityReason(
        result.debug.validation.reasons, FirstOrderDubinsMppiInvalidityReason::lateral_boundary))
        << "Missing lateral_boundary reason bit at index " << idx;
    }
  }
}

}  // namespace
}  // namespace autoware::mppi_optimizer
