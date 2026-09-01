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

#include "autoware/mppi_optimizer/detail/trajectory_validator.hpp"

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{

constexpr int kTestHorizon = detail::kMppiHorizon;
using TestCost = FirstOrderDubinsBicycleCost<kTestHorizon>;
using TestCostParams = FirstOrderDubinsBicycleCostParams<kTestHorizon>;
using OutputIndex = FirstOrderDubinsBicycleParams::OutputIndex;
using ControlIndex = FirstOrderDubinsBicycleParams::ControlIndex;

class TrajectoryValidatorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    const cudaError_t error = cudaGetDeviceCount(&device_count);
    if (error != cudaSuccess || device_count == 0) {
      GTEST_SKIP() << "A CUDA device is required by the MPPI cost object";
    }
    cost_ = std::make_unique<TestCost>();
    cost_->GPUSetup();
  }

  void TearDown() override
  {
    if (cost_) {
      cost_->freeCudaMem();
    }
  }

  TestCostParams makeParams() const
  {
    TestCostParams params;
    params.boundary_threshold = 100.0F;
    params.ego_length = 0.825F;
    params.ego_width = 0.42F;
    params.ego_axle_to_box_center = 0.2F;
    params.obstacle_collision_margin = 0.0F;
    params.road_border_collision_margin = 0.0F;
    params.lateral_boundary_barrier_weight =
      params.crash_contact_penalty /
      (params.lateral_boundary_soft_margin * params.lateral_boundary_soft_margin);
    params.obstacle_barrier_weight =
      params.crash_contact_penalty / (params.obstacle_safe_margin * params.obstacle_safe_margin);
    params.road_border_barrier_weight =
      params.crash_contact_penalty /
      (params.road_border_safe_margin * params.road_border_safe_margin);
    return params;
  }

  void setStraightReference(const float * terminal_reference = nullptr)
  {
    std::array<float, kTestHorizon> x{};
    std::array<float, kTestHorizon> y{};
    std::array<float, kTestHorizon> velocity{};
    std::array<float, kTestHorizon> yaw{};
    for (int i = 0; i < kTestHorizon; ++i) {
      // Line along y = 0 so polyline lateral distance equals |y| for nearby states.
      x[static_cast<size_t>(i)] = 0.20F * static_cast<float>(i);
      y[static_cast<size_t>(i)] = 0.0F;
      velocity[static_cast<size_t>(i)] = 2.0F;
      yaw[static_cast<size_t>(i)] = 0.0F;
    }
    cost_->setReferenceTrajectory(
      x.data(), y.data(), velocity.data(), kTestHorizon, yaw.data(), nullptr, nullptr,
      terminal_reference);
  }

  detail::OptimizedState makeFirstPostStepState(const float y = 0.0F) const
  {
    detail::OptimizedState state;
    state.x = 0.2F;
    state.y = y;
    state.yaw = 0.0F;
    state.velocity = 2.0F;
    return state;
  }

  std::unique_ptr<TestCost> cost_;
};

TEST_F(TrajectoryValidatorTest, ReportsRunningCostComponentsWithoutChangingTheirSum)
{
  auto params = makeParams();
  params.spatial_overspeed_coeff = 0.0F;
  params.track_coeff = 2.0F;
  params.track_terminal_scale = 0.0F;
  params.heading_coeff = 0.0F;
  params.lateral_distance_coeff = 0.0F;
  params.lateral_yaw_error_coeff = 0.0F;
  params.track_center_coeff = 3.0F;
  params.corner_buffer_coeff = 0.0F;
  params.accel_cmd_coeff = 4.0F;
  params.steer_cmd_coeff = 0.0F;
  params.steer_rate_coeff = 5.0F;
  params.lateral_acceleration_coeff = 0.0F;
  params.lateral_jerk_coeff = 0.0F;
  params.longitudinal_jerk_coeff = 0.0F;
  params.steer_time_constant = 0.1F;
  params.drivable_area_barrier_weight = 0.0F;
  cost_->setParams(params);
  setStraightReference();

  TestCost::output_array output = TestCost::output_array::Zero();
  output(static_cast<int>(OutputIndex::BASELINK_POS_I_X)) = 1.0F;
  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 2.0F;
  TestCost::control_array control = TestCost::control_array::Zero();
  control(static_cast<int>(ControlIndex::ACCELERATION_CMD)) = 2.0F;
  control(static_cast<int>(ControlIndex::STEER_CMD)) = 0.2F;
  int crash_status = 0;

  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  int direct_crash_status = 0;
  const float direct_total = cost_->computeRunningCost(output, control, 0, &direct_crash_status);

  EXPECT_FLOAT_EQ(breakdown.track, 2.0F);
  EXPECT_NEAR(breakdown.track_center, 3.6F, 1.0E-6F);
  EXPECT_FLOAT_EQ(breakdown.acceleration_command, 16.0F);
  EXPECT_NEAR(breakdown.steering_rate, 20.0F, 1.0E-5F);
  EXPECT_NEAR(breakdown.running_total, 41.6F, 1.0E-5F);
  EXPECT_NEAR(breakdown.componentTotal(), breakdown.total, 1.0E-5F);
  EXPECT_NEAR(breakdown.total, direct_total, 1.0E-5F);
  EXPECT_EQ(crash_status, 0);
  EXPECT_EQ(direct_crash_status, 0);
}

TEST_F(TrajectoryValidatorTest, EvaluatesIndependentTerminalPositionAndHeadingErrors)
{
  auto params = makeParams();
  params.spatial_overspeed_coeff = 0.0F;
  params.track_coeff = 0.0F;
  params.track_terminal_scale = 0.0F;
  params.heading_coeff = 0.0F;
  params.terminal_error_coeff = 2.0F;
  params.terminal_heading_coeff = 3.0F;
  params.lateral_distance_coeff = 0.0F;
  params.lateral_yaw_error_coeff = 0.0F;
  params.remaining_distance_coeff = 0.0F;
  params.path_overshoot_coeff = 0.0F;
  params.track_center_coeff = 0.0F;
  params.corner_buffer_coeff = 0.0F;
  params.drivable_area_barrier_weight = 0.0F;
  params.obstacle_barrier_weight = 0.0F;
  params.road_border_barrier_weight = 0.0F;
  params.lateral_boundary_barrier_weight = 0.0F;
  cost_->setParams(params);
  const float terminal_reference[3] = {20.0F, 1.0F, 0.0F};
  setStraightReference(terminal_reference);

  constexpr float two_pi = 6.2831853071795864769F;
  TestCost::output_array output = TestCost::output_array::Zero();
  output(static_cast<int>(OutputIndex::BASELINK_POS_I_X)) = terminal_reference[0] - 1.0F;
  output(static_cast<int>(OutputIndex::BASELINK_POS_I_Y)) = terminal_reference[1] + 2.0F;
  output(static_cast<int>(OutputIndex::YAW)) = two_pi - 0.5F;

  const auto breakdown = cost_->computeTerminalCostBreakdown(output);

  EXPECT_FLOAT_EQ(breakdown.terminal_error, 10.0F);
  EXPECT_NEAR(breakdown.terminal_heading, 0.75F, 1.0E-5F);
  EXPECT_NEAR(breakdown.terminal_total, 10.75F, 1.0E-5F);
  EXPECT_NEAR(breakdown.componentTotal(), breakdown.total, 1.0E-5F);
}

TEST_F(TrajectoryValidatorTest, PenalizesSpatialOverspeedUsingInterpolatedVelocityAndProgress)
{
  auto params = makeParams();
  params.spatial_overspeed_coeff = 10.0F;
  params.track_coeff = 0.0F;
  params.heading_coeff = 0.0F;
  params.lateral_distance_coeff = 0.0F;
  params.lateral_yaw_error_coeff = 0.0F;
  params.remaining_distance_coeff = 0.0F;
  params.path_overshoot_coeff = 0.0F;
  params.track_center_coeff = 0.0F;
  params.corner_buffer_coeff = 0.0F;
  params.lateral_boundary_barrier_weight = 0.0F;
  params.lateral_acceleration_coeff = 0.0F;
  params.lateral_jerk_coeff = 0.0F;
  params.longitudinal_jerk_coeff = 0.0F;
  params.drivable_area_barrier_weight = 0.0F;
  params.obstacle_barrier_weight = 0.0F;
  params.road_border_barrier_weight = 0.0F;
  cost_->setParams(params);
  setStraightReference();

  const std::array<float, 2> corridor_x{0.0F, 10.0F};
  const std::array<float, 2> corridor_y{0.0F, 0.0F};
  const std::array<float, 2> corridor_s{0.0F, 10.0F};
  const std::array<float, 2> corridor_ref_velocity{2.0F, 4.0F};
  cost_->setLateralCorridor(
    corridor_x.data(), corridor_y.data(), static_cast<int>(corridor_x.size()), corridor_s.data(),
    corridor_ref_velocity.data());

  TestCost::output_array output = TestCost::output_array::Zero();
  output(static_cast<int>(OutputIndex::BASELINK_POS_I_X)) = 5.0F;
  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 5.0F;
  TestCost::control_array control = TestCost::control_array::Zero();
  int crash_status = 0;

  const auto spatial_metrics = cost_->computeLateralPathMetrics(5.0F, 0.0F, 0.0F);
  EXPECT_FLOAT_EQ(spatial_metrics.spatial_s, 5.0F);
  EXPECT_FLOAT_EQ(spatial_metrics.spatial_ref_velocity, 3.0F);

  const auto at_half_progress =
    cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  int direct_crash_status = 0;
  const float direct_total = cost_->computeRunningCost(output, control, 0, &direct_crash_status);
  // v_ref(5 m) = 3 m/s, progress = 0.5: 10 * 0.5 * (5 - 3)^2 = 20.
  EXPECT_FLOAT_EQ(at_half_progress.spatial_overspeed, 20.0F);
  EXPECT_FLOAT_EQ(at_half_progress.total, 20.0F);
  EXPECT_FLOAT_EQ(direct_total, 20.0F);
  EXPECT_EQ(direct_crash_status, 0);

  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 2.5F;
  const auto below_reference =
    cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  EXPECT_FLOAT_EQ(below_reference.spatial_overspeed, 0.0F);

  output(static_cast<int>(OutputIndex::BASELINK_POS_I_X)) = 0.0F;
  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 5.0F;
  const auto at_start = cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  EXPECT_FLOAT_EQ(at_start.spatial_overspeed, 0.0F);
}

TEST_F(TrajectoryValidatorTest, UsesPointwiseMaximumVelocityForEachRunningCostStep)
{
  auto params = makeParams();
  params.spatial_overspeed_coeff = 0.0F;
  params.track_coeff = 0.0F;
  params.heading_coeff = 0.0F;
  params.lateral_distance_coeff = 0.0F;
  params.lateral_yaw_error_coeff = 0.0F;
  params.remaining_distance_coeff = 0.0F;
  params.path_overshoot_coeff = 0.0F;
  params.track_center_coeff = 0.0F;
  params.corner_buffer_coeff = 0.0F;
  params.accel_cmd_coeff = 0.0F;
  params.steer_cmd_coeff = 0.0F;
  params.steer_rate_coeff = 0.0F;
  params.lateral_acceleration_coeff = 0.0F;
  params.lateral_jerk_coeff = 0.0F;
  params.longitudinal_jerk_coeff = 0.0F;
  params.drivable_area_barrier_weight = 0.0F;
  params.overlimit_coeff = 10.0F;
  cost_->setParams(params);

  std::array<float, kTestHorizon> x{};
  std::array<float, kTestHorizon> y{};
  std::array<float, kTestHorizon> velocity{};
  std::array<float, kTestHorizon> yaw{};
  std::array<float, kTestHorizon> maximum_velocity{};
  std::array<std::uint8_t, kTestHorizon> velocity_limit_active{};
  velocity.fill(2.0F);
  maximum_velocity.fill(3.0F);
  velocity_limit_active.fill(1U);
  maximum_velocity[1] = 1.0F;
  cost_->setReferenceTrajectory(
    x.data(), y.data(), velocity.data(), kTestHorizon, yaw.data(), maximum_velocity.data(),
    velocity_limit_active.data());

  TestCost::output_array output = TestCost::output_array::Zero();
  output(static_cast<int>(OutputIndex::BASELINK_VEL_B_X)) = 2.0F;
  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 2.0F;
  TestCost::control_array control = TestCost::control_array::Zero();
  int crash_status = 0;

  const auto step_zero = cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  const auto step_one = cost_->computeRunningCostBreakdown(output, control, 1, &crash_status);

  EXPECT_FLOAT_EQ(step_zero.kinematic_velocity_overlimit, 0.0F);
  EXPECT_FLOAT_EQ(step_one.kinematic_velocity_overlimit, 10.0F);
}

TEST_F(TrajectoryValidatorTest, SmoothBarrierCostRampsUpQuadratically)
{
  constexpr float safe_margin = 1.0F;
  constexpr float precomputed_weight = 2000.0F;

  EXPECT_FLOAT_EQ(computeSmoothBarrierCost(safe_margin, safe_margin, precomputed_weight), 0.0F);
  EXPECT_FLOAT_EQ(
    computeSmoothBarrierCost(safe_margin - 0.5F, safe_margin, precomputed_weight),
    precomputed_weight * 0.25F);

  auto params = makeParams();
  params.obstacle_safe_margin = safe_margin;
  params.obstacle_barrier_weight = precomputed_weight;
  cost_->setParams(params);
  setStraightReference();
  constexpr float obstacle_y = 0.0F;
  constexpr float obstacle_yaw = 0.0F;
  constexpr float obstacle_half_length = 0.1F;
  constexpr float obstacle_half_width = 0.1F;
  float obstacle_x = 1.7125F;  // Exactly 1.0 m beyond the ego front contour.
  cost_->setOrientedBoxObstacles(
    &obstacle_x, &obstacle_y, &obstacle_yaw, &obstacle_half_length, &obstacle_half_width, 1);

  TestCost::output_array output = TestCost::output_array::Zero();
  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 2.0F;
  TestCost::control_array control = TestCost::control_array::Zero();
  int crash_status = 0;
  EXPECT_NEAR(
    cost_->computeRunningCostBreakdown(output, control, 0, &crash_status).obstacle, 0.0F, 1.0E-5F);

  obstacle_x = 1.2125F;  // Clearance is 0.5 m, so margin violation is exactly 0.5 m.
  cost_->setOrientedBoxObstacles(
    &obstacle_x, &obstacle_y, &obstacle_yaw, &obstacle_half_length, &obstacle_half_width, 1);
  EXPECT_NEAR(
    cost_->computeRunningCostBreakdown(output, control, 0, &crash_status).obstacle,
    precomputed_weight * 0.25F, 1.0E-3F);
}

TEST_F(TrajectoryValidatorTest, LateralBoundaryBarrierActivatesInsideThreshold)
{
  auto params = makeParams();
  params.spatial_overspeed_coeff = 0.0F;
  params.track_coeff = 0.0F;
  params.heading_coeff = 0.0F;
  params.lateral_distance_coeff = 0.0F;
  params.lateral_yaw_error_coeff = 0.0F;
  params.remaining_distance_coeff = 0.0F;
  params.path_overshoot_coeff = 0.0F;
  params.track_center_coeff = 0.0F;
  params.corner_buffer_coeff = 0.0F;
  params.accel_cmd_coeff = 0.0F;
  params.steer_cmd_coeff = 0.0F;
  params.steer_rate_coeff = 0.0F;
  params.lateral_acceleration_coeff = 0.0F;
  params.lateral_jerk_coeff = 0.0F;
  params.longitudinal_jerk_coeff = 0.0F;
  params.drivable_area_barrier_weight = 0.0F;
  params.obstacle_barrier_weight = 0.0F;
  params.road_border_barrier_weight = 0.0F;
  params.boundary_threshold = 0.8F;
  params.lateral_boundary_soft_margin = 0.2F;
  params.crash_contact_penalty = 100000.0F;
  params.lateral_boundary_barrier_weight =
    params.crash_contact_penalty /
    (params.lateral_boundary_soft_margin * params.lateral_boundary_soft_margin);
  cost_->setParams(params);
  setStraightReference();

  TestCost::output_array output = TestCost::output_array::Zero();
  TestCost::control_array control = TestCost::control_array::Zero();
  int crash_status = 0;

  output(static_cast<int>(OutputIndex::BASELINK_POS_I_Y)) = 0.5F;
  EXPECT_FLOAT_EQ(
    cost_->computeRunningCostBreakdown(output, control, 0, &crash_status).lateral_boundary, 0.0F);

  output(static_cast<int>(OutputIndex::BASELINK_POS_I_Y)) = 0.7F;
  const auto inside = cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  EXPECT_NEAR(inside.lateral_boundary, 25000.0F, 1.0F);
  EXPECT_NEAR(inside.total, inside.lateral_boundary, 1.0E-3F);

  output(static_cast<int>(OutputIndex::BASELINK_POS_I_Y)) = 0.8F;
  EXPECT_NEAR(
    cost_->computeRunningCostBreakdown(output, control, 0, &crash_status).lateral_boundary,
    params.crash_contact_penalty, 1.0F);
}

TEST_F(TrajectoryValidatorTest, SmoothBarrierCostGrowsBeyondContactPenaltyForPenetration)
{
  constexpr float safe_margin = 0.5F;
  constexpr float nominal_contact_penalty = 100000.0F;
  constexpr float precomputed_weight = nominal_contact_penalty / (safe_margin * safe_margin);
  const float contact_cost = computeSmoothBarrierCost(0.0F, safe_margin, precomputed_weight);
  const float deep_penetration_cost =
    computeSmoothBarrierCost(-10.0F, safe_margin, precomputed_weight);

  EXPECT_FLOAT_EQ(contact_cost, nominal_contact_penalty);
  EXPECT_GT(deep_penetration_cost, nominal_contact_penalty);
  EXPECT_TRUE(std::isfinite(deep_penetration_cost));

  auto params = makeParams();
  params.obstacle_safe_margin = safe_margin;
  params.obstacle_barrier_weight = precomputed_weight;
  params.crash_contact_penalty = nominal_contact_penalty;
  cost_->setParams(params);
  setStraightReference();
  constexpr float obstacle_x = 0.2F;
  constexpr float obstacle_y = 0.0F;
  constexpr float obstacle_yaw = 0.0F;
  constexpr float obstacle_half_length = 20.0F;
  constexpr float obstacle_half_width = 20.0F;
  cost_->setOrientedBoxObstacles(
    &obstacle_x, &obstacle_y, &obstacle_yaw, &obstacle_half_length, &obstacle_half_width, 1);

  EXPECT_LT(cost_->distanceToClosestObstacle(0.0F, 0.0F, 0.0F, 0), -10.0F);
  TestCost::output_array output = TestCost::output_array::Zero();
  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 2.0F;
  TestCost::control_array control = TestCost::control_array::Zero();
  int crash_status = 0;
  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  EXPECT_GT(breakdown.obstacle, nominal_contact_penalty);
  EXPECT_TRUE(std::isfinite(breakdown.total));
}

TEST_F(TrajectoryValidatorTest, ExcludesMovingObjectsFromGradualObstacleCost)
{
  auto params = makeParams();
  params.obstacle_safe_margin = 0.5F;
  params.obstacle_barrier_weight = 2000.0F;
  cost_->setParams(params);
  setStraightReference();

  std::array<float, kTestHorizon> obstacle_x{};
  std::array<float, kTestHorizon> obstacle_y{};
  std::array<float, kTestHorizon> obstacle_yaw{};
  for (int t = 0; t < kTestHorizon; ++t) {
    obstacle_x[static_cast<size_t>(t)] = 0.2F + 0.1F * static_cast<float>(t);
  }
  constexpr float obstacle_half_length = 0.1F;
  constexpr float obstacle_half_width = 0.1F;
  cost_->setOrientedBoxObstacleTrajectories(
    obstacle_x.data(), obstacle_y.data(), obstacle_yaw.data(), &obstacle_half_length,
    &obstacle_half_width, 1, kTestHorizon);

  // Moving objects remain available to the hard output validator.
  EXPECT_TRUE(cost_->egoIntersectsObstacleAtStep(0.0F, 0.0F, 0.0F, 0));

  TestCost::output_array output = TestCost::output_array::Zero();
  TestCost::control_array control = TestCost::control_array::Zero();
  int crash_status = 0;
  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  EXPECT_FLOAT_EQ(breakdown.obstacle, 0.0F);
}

TEST_F(TrajectoryValidatorTest, GradualConstraintCostsAreIncludedInBreakdownTotal)
{
  auto params = makeParams();
  params.spatial_overspeed_coeff = 0.0F;
  params.track_coeff = 0.0F;
  params.heading_coeff = 0.0F;
  params.track_center_coeff = 0.0F;
  params.corner_buffer_coeff = 0.0F;
  params.accel_cmd_coeff = 0.0F;
  params.steer_cmd_coeff = 0.0F;
  params.steer_rate_coeff = 0.0F;
  params.lateral_acceleration_coeff = 0.0F;
  params.lateral_jerk_coeff = 0.0F;
  params.longitudinal_jerk_coeff = 0.0F;
  params.obstacle_safe_margin = 0.5F;
  params.obstacle_barrier_weight = 2000.0F;
  params.road_border_safe_margin = 0.3F;
  params.road_border_barrier_weight = 2000.0F;
  params.drivable_area_safe_margin = 0.0F;
  params.drivable_area_barrier_weight = 2000.0F;
  params.crash_contact_penalty = 100000.0F;
  cost_->setParams(params);
  setStraightReference();

  constexpr float obstacle_x = 0.2F;
  constexpr float obstacle_y = 0.0F;
  constexpr float obstacle_yaw = 0.0F;
  constexpr float obstacle_half_length = 0.1F;
  constexpr float obstacle_half_width = 0.1F;
  cost_->setOrientedBoxObstacles(
    &obstacle_x, &obstacle_y, &obstacle_yaw, &obstacle_half_length, &obstacle_half_width, 1);
  cost_->setRoadBorderSegments({Segment{-2.0F, 0.4F, 2.0F, 0.4F}});
  cost_->setDrivableAreaSegments({Segment{-2.0F, 0.0F, 2.0F, 0.0F}});

  TestCost::output_array output = TestCost::output_array::Zero();
  output(static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 2.0F;
  TestCost::control_array control = TestCost::control_array::Zero();
  int crash_status = 0;

  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 0, &crash_status);
  const float gradual_cost_sum =
    breakdown.drivable_area + breakdown.obstacle + breakdown.road_border;

  EXPECT_GT(breakdown.drivable_area, 0.0F);
  EXPECT_GT(breakdown.obstacle, 0.0F);
  EXPECT_GT(breakdown.road_border, 0.0F);
  EXPECT_NEAR(breakdown.total, gradual_cost_sum, 1.0E-4F);
  EXPECT_NEAR(breakdown.componentTotal(), breakdown.total, 1.0E-4F);
  EXPECT_EQ(crash_status, 0);
}

TEST_F(TrajectoryValidatorTest, AppliesBoundaryThresholdSymmetricallyAndInclusively)
{
  auto params = makeParams();
  params.boundary_threshold = 0.5F;
  cost_->setParams(params);
  setStraightReference();

  struct Case
  {
    float lateral_offset;
    bool valid;
  };
  const std::vector<Case> cases = {{0.49F, true},  {0.5F, false},  {0.51F, false},
                                   {-0.49F, true}, {-0.5F, false}, {-0.51F, false}};

  for (const auto & test_case : cases) {
    const auto result = detail::validateOptimizedTrajectory(
      *cost_,
      std::vector<detail::OptimizedState>{makeFirstPostStepState(test_case.lateral_offset)});
    EXPECT_EQ(result.isValid(), test_case.valid) << "offset=" << test_case.lateral_offset;
    EXPECT_EQ(
      hasInvalidityReason(result.reasons, FirstOrderDubinsMppiInvalidityReason::lateral_boundary),
      !test_case.valid)
      << "offset=" << test_case.lateral_offset;
  }
}

TEST_F(TrajectoryValidatorTest, RoadBorderMarginInflatesTheEgoFootprint)
{
  auto params = makeParams();
  cost_->setParams(params);
  setStraightReference();
  cost_->setRoadBorderSegments({Segment{-1.0F, 0.31F, 2.0F, 0.31F}});
  const std::vector<detail::OptimizedState> states{makeFirstPostStepState()};

  const auto without_margin = detail::validateOptimizedTrajectory(*cost_, states);
  EXPECT_TRUE(without_margin.isValid());

  params.road_border_collision_margin = 0.2F;
  cost_->setParams(params);
  const auto with_margin = detail::validateOptimizedTrajectory(*cost_, states);
  EXPECT_FALSE(with_margin.isValid());
  EXPECT_TRUE(
    hasInvalidityReason(with_margin.reasons, FirstOrderDubinsMppiInvalidityReason::road_border));
  ASSERT_TRUE(with_margin.first_invalid_index.has_value());
  EXPECT_EQ(with_margin.first_invalid_index.value(), 0U);
}

TEST_F(TrajectoryValidatorTest, ObstacleMarginInflatesTheEgoOrientedBox)
{
  auto params = makeParams();
  cost_->setParams(params);
  setStraightReference();
  constexpr float obstacle_x = 0.4F;
  constexpr float obstacle_y = 0.41F;
  constexpr float obstacle_yaw = 0.0F;
  constexpr float obstacle_half_length = 0.1F;
  constexpr float obstacle_half_width = 0.1F;
  cost_->setOrientedBoxObstacles(
    &obstacle_x, &obstacle_y, &obstacle_yaw, &obstacle_half_length, &obstacle_half_width, 1);
  const std::vector<detail::OptimizedState> states{makeFirstPostStepState()};

  const auto without_margin = detail::validateOptimizedTrajectory(*cost_, states);
  EXPECT_TRUE(without_margin.isValid());

  params.obstacle_collision_margin = 0.2F;
  cost_->setParams(params);
  const auto with_margin = detail::validateOptimizedTrajectory(*cost_, states);
  EXPECT_FALSE(with_margin.isValid());
  EXPECT_TRUE(
    hasInvalidityReason(with_margin.reasons, FirstOrderDubinsMppiInvalidityReason::obstacle));
  ASSERT_TRUE(with_margin.first_invalid_index.has_value());
  EXPECT_EQ(with_margin.first_invalid_index.value(), 0U);
}

TEST_F(TrajectoryValidatorTest, DetectsLateralBoundaryViolationsAcrossHorizonAndProfiles)
{
  auto params = makeParams();
  params.boundary_threshold = 0.50F;
  cost_->setParams(params);
  setStraightReference();  // Reference trajectory is along y = 0.0

  struct DeviationCase
  {
    std::string name;
    std::vector<float> y_profile;
    bool expected_valid;
    std::optional<std::size_t> expected_first_invalid_idx;
  };

  std::vector<DeviationCase> test_cases;

  // Case 1: Strictly within threshold across the entire horizon -> Valid
  test_cases.push_back({"all_valid", std::vector<float>(kTestHorizon, 0.40F), true, std::nullopt});

  // Case 2: Constant violation from step 0 (Left and Right) -> Invalid at index 0
  test_cases.push_back(
    {"invalid_at_start_right", std::vector<float>(kTestHorizon, 0.60F), false, 0U});
  test_cases.push_back(
    {"invalid_at_start_left", std::vector<float>(kTestHorizon, -0.60F), false, 0U});

  // Case 3: Progressive drift that starts valid (y=0) and breaches 0.50F at mid-horizon (step 5)
  {
    std::vector<float> drift(kTestHorizon, 0.0F);
    for (int i = 0; i < kTestHorizon; ++i) {
      drift[static_cast<std::size_t>(i)] =
        0.10F * static_cast<float>(i);  // Breaches 0.50F at i=5 (0.50F)
    }
    test_cases.push_back({"progressive_drift_mid_horizon", drift, false, 5U});
  }

  // Case 4: Single-step spike at the tail end of the horizon (step 79)
  {
    std::vector<float> tail_spike(kTestHorizon, 0.0F);
    tail_spike.back() = 0.55F;
    test_cases.push_back(
      {"tail_step_violation", tail_spike, false, static_cast<std::size_t>(kTestHorizon - 1)});
  }

  // Case 5: Exact boundary edge (0.50F is inclusive rejection: offset >= threshold)
  test_cases.push_back(
    {"exact_threshold_edge", std::vector<float>(kTestHorizon, 0.50F), false, 0U});

  for (const auto & tc : test_cases) {
    std::vector<detail::OptimizedState> states(kTestHorizon);
    for (std::size_t i = 0; i < static_cast<std::size_t>(kTestHorizon); ++i) {
      states[i].x = 0.20F * static_cast<float>(i + 1U);
      states[i].y = tc.y_profile[i];
      states[i].yaw = 0.0F;
      states[i].velocity = 2.0F;
    }
    const auto result = detail::validateOptimizedTrajectory(*cost_, states);
    EXPECT_EQ(result.isValid(), tc.expected_valid) << "Failed case: " << tc.name;
    if (!tc.expected_valid) {
      EXPECT_TRUE(
        hasInvalidityReason(result.reasons, FirstOrderDubinsMppiInvalidityReason::lateral_boundary))
        << "Failed case: " << tc.name;
      ASSERT_TRUE(result.first_invalid_index.has_value()) << "Failed case: " << tc.name;
      EXPECT_EQ(result.first_invalid_index.value(), tc.expected_first_invalid_idx.value())
        << "Failed case: " << tc.name;
    }
  }
}

TEST_F(TrajectoryValidatorTest, LateralCorridorIncludesGeometryBeforeDelayShiftedRef)
{
  // Delay-shifted tracking ref starts at x=2. Past-start cross-track to the extended tip
  // already ignores along-track undershoot; a curved corridor still needs the full DP polyline
  // when the extended first segment does not pass near ego.
  auto params = makeParams();
  params.boundary_threshold = 0.8F;
  cost_->setParams(params);

  std::array<float, kTestHorizon> ref_x{};
  std::array<float, kTestHorizon> ref_y{};
  std::array<float, kTestHorizon> ref_v{};
  std::array<float, kTestHorizon> ref_yaw{};
  for (int i = 0; i < kTestHorizon; ++i) {
    // Path going north from (2,2), so extending the first segment does not pass through (0,0.4).
    ref_x[static_cast<size_t>(i)] = 2.0F;
    ref_y[static_cast<size_t>(i)] = 2.0F + static_cast<float>(i);
    ref_v[static_cast<size_t>(i)] = 1.0F;
    ref_yaw[static_cast<size_t>(i)] = 1.5707963F;
  }
  cost_->setReferenceTrajectory(
    ref_x.data(), ref_y.data(), ref_v.data(), kTestHorizon, ref_yaw.data());

  detail::OptimizedState near_ego;
  near_ego.x = 0.0F;
  near_ego.y = 0.4F;
  near_ego.yaw = 0.0F;
  near_ego.velocity = 1.0F;
  near_ego.steering = 0.0F;

  // Without full corridor: far from the tracking path → crash.
  EXPECT_FALSE(detail::validateOptimizedTrajectory(*cost_, {near_ego}).isValid());

  // Full DP corridor along y=0.4 from x=0.. includes ego → valid.
  constexpr int kCorridor = 16;
  std::array<float, kCorridor> corridor_x{};
  std::array<float, kCorridor> corridor_y{};
  for (int i = 0; i < kCorridor; ++i) {
    corridor_x[static_cast<size_t>(i)] = static_cast<float>(i);
    corridor_y[static_cast<size_t>(i)] = 0.4F;
  }
  cost_->setLateralCorridor(corridor_x.data(), corridor_y.data(), kCorridor);
  EXPECT_TRUE(detail::validateOptimizedTrajectory(*cost_, {near_ego}).isValid());

  near_ego.y = 1.5F;
  EXPECT_FALSE(detail::validateOptimizedTrajectory(*cost_, {near_ego}).isValid());
}

TEST_F(TrajectoryValidatorTest, PastPolylineEndUsesCrossTrackNotEndpointDistance)
{
  // Ref along x from 0..10. A point past the tip (x=11.5) with tiny cross-track must not
  // crash: clamped segment distance to the endpoint would be ~1.5 m and falsely fail.
  auto params = makeParams();
  params.boundary_threshold = 0.8F;
  cost_->setParams(params);

  constexpr int kCorridor = 11;
  std::array<float, kCorridor> corridor_x{};
  std::array<float, kCorridor> corridor_y{};
  for (int i = 0; i < kCorridor; ++i) {
    corridor_x[static_cast<size_t>(i)] = static_cast<float>(i);
    corridor_y[static_cast<size_t>(i)] = 0.0F;
  }
  cost_->setLateralCorridor(corridor_x.data(), corridor_y.data(), kCorridor);
  // Also need a reference for the cost object; corridor drives lateral checks.
  setStraightReference();
  cost_->setLateralCorridor(corridor_x.data(), corridor_y.data(), kCorridor);

  detail::OptimizedState past_end;
  past_end.x = 11.5F;
  past_end.y = 0.05F;
  past_end.yaw = 0.0F;
  past_end.velocity = 1.0F;
  past_end.steering = 0.0F;

  EXPECT_NEAR(cost_->computeLateralDistanceValue(past_end.x, past_end.y), 0.05F, 1.0E-4F);
  EXPECT_TRUE(detail::validateOptimizedTrajectory(*cost_, {past_end}).isValid());

  past_end.y = 1.0F;
  EXPECT_NEAR(cost_->computeLateralDistanceValue(past_end.x, past_end.y), 1.0F, 1.0E-4F);
  EXPECT_FALSE(detail::validateOptimizedTrajectory(*cost_, {past_end}).isValid());
}

}  // namespace
}  // namespace autoware::mppi_optimizer
