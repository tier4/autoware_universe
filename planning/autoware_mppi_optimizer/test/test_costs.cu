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
#include <fstream>
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

class RawCostEvaluationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    int device_count = 0;
    if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
      GTEST_SKIP() << "A CUDA device is required";
    }
    cost_ = std::make_unique<TestCost>();
    cost_->GPUSetup();
    cost_->setParams(makeRawParams());
    setStraightReference();
  }

  void TearDown() override
  {
    if (cost_) cost_->freeCudaMem();
  }

  TestCostParams makeRawParams() const
  {
    TestCostParams params;
    params.accel_cmd_coeff = 1.0F;
    params.steer_cmd_coeff = 1.0F;
    params.steer_rate_coeff = 1.0F;
    params.accel_cmd_rate_coeff = 1.0F;
    params.steer_cmd_rate_coeff = 1.0F;
    params.initial_steer_rate_coeff = 1.0F;
    params.lateral_acceleration_coeff = 1.0F;
    params.lateral_jerk_coeff = 1.0F;
    params.longitudinal_jerk_coeff = 1.0F;
    params.track_center_coeff = 1.0F;
    params.spatial_overspeed_coeff = 1.0F;
    params.track_coeff = 1.0F;
    params.track_terminal_scale = 1.0F;
    params.heading_coeff = 1.0F;
    params.lateral_distance_coeff = 1.0F;
    params.lateral_yaw_error_coeff = 1.0F;
    params.terminal_error_coeff = 1.0F;
    params.terminal_heading_coeff = 1.0F;
    params.boundary_threshold = 100.0F;
    params.ego_length = 0.825F;
    params.ego_width = 0.42F;
    params.ego_axle_to_box_center = 0.2F;
    params.obstacle_safe_margin = 1.0F;
    params.obstacle_barrier_weight = 1.0F;
    params.wheel_base = 2.0F;
    return params;
  }

  void setStraightReference()
  {
    std::array<float, kTestHorizon> x{}, y{}, velocity{}, yaw{};
    for (int i = 0; i < kTestHorizon; ++i) {
      x[static_cast<size_t>(i)] = 0.20F * static_cast<float>(i);
      velocity[static_cast<size_t>(i)] = 2.0F;
    }
    cost_->setReferenceTrajectory(x.data(), y.data(), velocity.data(), kTestHorizon, yaw.data());
  }

  // Exports a full trajectory and cost breakdown to CSV for Python reporting
  void exportTrajectoryReport(
    const std::string & test_name, const std::vector<TestCost::output_array> & outputs,
    const std::vector<TestCost::control_array> & controls)
  {
    if (std::getenv("MPPI_REPORT") == nullptr) return;

    std::ofstream out("/tmp/mppi_report_" + test_name + ".csv");
    out << "step,x,y,yaw,v,v_ref,steer_cmd,accel_cmd,"
        << "cost_track,cost_track_center,cost_heading,cost_lateral_distance,cost_lateral_yaw,"
        << "cost_obstacle,cost_road_border,cost_drivable_area,cost_lateral_boundary,"
        << "cost_spatial_overspeed,cost_steering_rate,cost_lateral_acceleration,"
        << "cost_lateral_jerk,cost_longitudinal_jerk,cost_accel_command_rate,"
        << "cost_steer_command_rate,cost_total\n";

    for (size_t i = 0; i < outputs.size(); ++i) {
      int crash = 0;
      auto breakdown = cost_->computeRunningCostBreakdown(outputs[i], controls[i], i, &crash);

      const float x = outputs[i](static_cast<int>(OutputIndex::BASELINK_POS_I_X));
      const float y = outputs[i](static_cast<int>(OutputIndex::BASELINK_POS_I_Y));
      const float yaw = outputs[i](static_cast<int>(OutputIndex::YAW));
      const float v = outputs[i](static_cast<int>(OutputIndex::TOTAL_VELOCITY));

      // Extract the reference velocity interpolated for the ego's current progress
      const auto metrics = cost_->computeLateralPathMetrics(x, y, yaw);

      out << i << "," << x << "," << y << "," << yaw << "," << v << ","
          << metrics.spatial_ref_velocity << ","
          << controls[i](static_cast<int>(ControlIndex::STEER_CMD)) << ","
          << controls[i](static_cast<int>(ControlIndex::ACCELERATION_CMD)) << "," << breakdown.track
          << "," << breakdown.track_center << "," << breakdown.heading << ","
          << breakdown.lateral_distance << "," << breakdown.lateral_yaw_error << ","
          << breakdown.obstacle << "," << breakdown.road_border << "," << breakdown.drivable_area
          << "," << breakdown.lateral_boundary << "," << breakdown.spatial_overspeed << ","
          << breakdown.steering_rate << "," << breakdown.lateral_acceleration << ","
          << breakdown.lateral_jerk << "," << breakdown.longitudinal_jerk << ","
          << breakdown.acceleration_command_rate << "," << breakdown.steering_command_rate << ","
          << breakdown.total << "\n";
    }
  }

  std::unique_ptr<TestCost> cost_;
};

// --- NEW EXPANDED TESTS ---

TEST_F(RawCostEvaluationTest, EvaluatesRawSpatialOverspeedPenalty)
{
  std::vector<TestCost::output_array> outputs(kTestHorizon, TestCost::output_array::Zero());
  std::vector<TestCost::control_array> controls(kTestHorizon, TestCost::control_array::Zero());

  // Set ego velocity to 5.0 m/s. Reference is 2.0 m/s.
  outputs[0](static_cast<int>(OutputIndex::TOTAL_VELOCITY)) = 5.0F;

  int crash_status = 0;
  const auto breakdown =
    cost_->computeRunningCostBreakdown(outputs[0], controls[0], 0, &crash_status);

  // spatial_overspeed mathematically computes: progress_weight * (v - v_ref)^2
  // progress_weight at t=0 is (horizon - 0) / horizon = 1.0.
  // (5.0 - 2.0)^2 = 9.0
  EXPECT_FLOAT_EQ(breakdown.spatial_overspeed, 9.0F);

  exportTrajectoryReport("spatial_overspeed", outputs, controls);
}

TEST_F(RawCostEvaluationTest, EvaluatesRawTerminalError)
{
  TestCost::output_array output = TestCost::output_array::Zero();

  // Terminal reference is at x = 15.8 (0.2 * 79 steps)
  output(static_cast<int>(OutputIndex::BASELINK_POS_I_X)) = 15.8F;
  output(static_cast<int>(OutputIndex::BASELINK_POS_I_Y)) = 3.0F;  // 3m offset at terminal state

  const auto breakdown = cost_->computeTerminalCostBreakdown(output);

  // (3.0)^2 = 9.0
  EXPECT_FLOAT_EQ(breakdown.terminal_error, 9.0F);
}

TEST_F(RawCostEvaluationTest, EvaluatesRawSteeringRatePenalty)
{
  TestCost::output_array output = TestCost::output_array::Zero();
  TestCost::control_array control = TestCost::control_array::Zero();

  output(static_cast<int>(OutputIndex::STEERING_RATE)) = 1.5F;

  int crash_status = 0;
  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 1, &crash_status);

  // (1.5 rad/s)^2 = 2.25
  EXPECT_FLOAT_EQ(breakdown.steering_rate, 2.25F);
}

TEST_F(RawCostEvaluationTest, EvaluatesRawLateralAccelerationPenalty)
{
  TestCost::output_array output = TestCost::output_array::Zero();
  TestCost::control_array control = TestCost::control_array::Zero();

  output(static_cast<int>(OutputIndex::BASELINK_VEL_B_X)) = 2.0F;
  output(static_cast<int>(OutputIndex::STEER_ANGLE)) = 0.1F;

  int crash_status = 0;
  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 1, &crash_status);

  // lat_accel = v^2 * tan(steer) / L = 4.0 * tan(0.1) / 2.0
  const float expected_lat_accel = 2.0F * std::tan(0.1F);
  EXPECT_FLOAT_EQ(breakdown.lateral_acceleration, expected_lat_accel * expected_lat_accel);
}

TEST_F(RawCostEvaluationTest, EvaluatesRawTrackCenterError)
{
  TestCost::output_array output = TestCost::output_array::Zero();
  TestCost::control_array control = TestCost::control_array::Zero();

  output(static_cast<int>(OutputIndex::BASELINK_POS_I_X)) = 0.0F;
  output(static_cast<int>(OutputIndex::BASELINK_POS_I_Y)) = 1.5F;  // 1.5m lateral offset

  int crash_status = 0;
  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 1, &crash_status);

  // (1.5m)^2 = 2.25
  EXPECT_FLOAT_EQ(breakdown.track_center, 2.25F);
}

TEST_F(RawCostEvaluationTest, EvaluatesRawCommandRegularization)
{
  TestCost::output_array output = TestCost::output_array::Zero();
  TestCost::control_array control = TestCost::control_array::Zero();

  output(static_cast<int>(OutputIndex::ACCEL_COMMAND_RATE)) = 5.0F;

  int crash_status = 0;
  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 1, &crash_status);

  // (5.0 m/s^3)^2 = 25.0
  EXPECT_FLOAT_EQ(breakdown.acceleration_command_rate, 25.0F);
}

TEST_F(RawCostEvaluationTest, EvaluatesRawObstacleMarginPenalty)
{
  float obstacle_x = 1.2125F;  // Ego front contour is ~0.7125. Clearance = 0.5m.
  float obstacle_y = 0.0F;
  float obstacle_yaw = 0.0F;
  float obstacle_half_length = 0.01F;
  float obstacle_half_width = 0.01F;

  cost_->setOrientedBoxObstacles(
    &obstacle_x, &obstacle_y, &obstacle_yaw, &obstacle_half_length, &obstacle_half_width, 1);

  TestCost::output_array output = TestCost::output_array::Zero();
  TestCost::control_array control = TestCost::control_array::Zero();

  int crash_status = 0;
  const auto breakdown = cost_->computeRunningCostBreakdown(output, control, 1, &crash_status);

  // margin = 1.0, clearance = 0.5. Penetration = 0.5. Quadratic cost: (0.5)^2 = 0.25
  EXPECT_NEAR(breakdown.obstacle, 0.25F, 1.0E-4F);
}

TEST_F(RawCostEvaluationTest, SimulatesRealisticTrajectory)
{
  // 1. Initialize the physical vehicle model
  FirstOrderDubinsBicycleParams model_params;
  FirstOrderDubinsBicycle model(model_params);

  auto state = model.getZeroState();
  auto next = model.getZeroState();
  auto derivative = model.getZeroState();

  // Start the vehicle at 2.0 m/s
  state(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)) = 2.0F;

  std::vector<TestCost::output_array> outputs(kTestHorizon);
  std::vector<TestCost::control_array> controls(kTestHorizon);

  // 2. Simulate 80 steps of driving
  for (int i = 0; i < kTestHorizon; ++i) {
    // Command a constant acceleration (0.5 m/s^2) and a gentle steering angle (0.05 rad)
    controls[i](static_cast<int>(ControlIndex::ACCELERATION_CMD)) = 0.5F;
    controls[i](static_cast<int>(ControlIndex::STEER_CMD)) = 0.05F;

    // Step the physics engine forward by dt (0.1s)
    model.step(state, next, derivative, controls[i], outputs[i], i * 0.1F, 0.1F);

    // Advance the state for the next loop
    state = next;
  }

  // 3. Export the physically accurate trajectory
  exportTrajectoryReport("realistic_swerve", outputs, controls);
}
}  // namespace
}  // namespace autoware::mppi_optimizer
