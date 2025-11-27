// Copyright 2025 TIER IV, Inc.
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

#include "stop_state_machine.hpp"
#include "test_utils_plotter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{

class StopStateTest : public ::testing::Test
{
protected:
  StopStateMachine sm;
  SplineData spline;
  nav_msgs::msg::Odometry odom;
  rclcpp::Time clock{0};

  // Simulation Constants
  const double sim_dt = 0.05;          // 20Hz
  const double stop_vel_thresh = 0.1;  // Must match StopStateMachine internal threshold

  // Physics State
  double current_x = 0.0;
  double current_v = 0.0;

  // Logs for Plotting
  std::vector<double> t_log;
  std::vector<double> x_log;
  std::vector<double> v_log;
  std::vector<double> wait_log;

  void SetUp() override
  {
    // Path: 0m -> 10m -> 20m. Stop at 10m (5s wait).
    spline.s = {0.0, 10.0, 20.0};
    spline.x = {0.0, 10.0, 20.0};
    spline.y = {0.0, 0.0, 0.0};
    spline.wait_times = {0.0, 5.0, 0.0};

    // Initialize
    current_x = 0.0;
    current_v = 10.0;  // Start at cruising speed

    // Clear logs
    t_log.clear();
    x_log.clear();
    v_log.clear();
    wait_log.clear();
  }

  void drive(double duration, double accel)
  {
    // Calculate exact number of steps to avoid rounding drift
    int steps = static_cast<int>(std::ceil(duration / sim_dt));

    for (int i = 0; i < steps; ++i) {
      // 1. Update Physics (Euler Integration)
      current_x += current_v * sim_dt + 0.5 * accel * sim_dt * sim_dt;
      current_v += accel * sim_dt;

      // Clamp for sanity (no reversing in these tests unless intended)
      if (current_v < 0.0) current_v = 0.0;

      // 2. Update ROS Msgs
      clock += rclcpp::Duration::from_seconds(sim_dt);
      odom.pose.pose.position.x = current_x;
      odom.twist.twist.linear.x = current_v;

      // 3. Update State Machine
      sm.update(spline, odom, clock);

      // 4. Log Data
      t_log.push_back(clock.seconds());
      x_log.push_back(current_x);
      v_log.push_back(current_v);
      wait_log.push_back(sm.get_state().accumulated_wait);
    }
  }

  // Helper to dynamically calculate how much time the state machine will "leak"
  // (accumulate) while the velocity ramps up/down through the [0, 0.1] range.
  [[nodiscard]] double compute_expected_leakage(double accel_magnitude) const
  {
    int leakage_steps = 0;
    double v_sim = 0.0;

    // Simulate just the velocity ramp to count steps below threshold
    while (v_sim < stop_vel_thresh - 1e-9) {
      leakage_steps++;
      v_sim += accel_magnitude * sim_dt;
    }
    return leakage_steps * sim_dt;
  }
};

TEST_F(StopStateTest, RealisticStop)
{
  // Scenario: Cruising at 10m/s, then braking to stop at 10m.
  const double initial_v = 10.0;
  const double stop_dist = 10.0;
  const double target_wait = 5.0;

  // Calculate required deceleration: a = -v^2 / 2s
  const double braking_accel = -(initial_v * initial_v) / (2.0 * stop_dist);  // -5.0
  const double braking_time = std::abs(initial_v / braking_accel);            // 2.0s

  // 1. Brake to stop
  drive(braking_time, braking_accel);

  // Verify physics result
  EXPECT_NEAR(current_x, stop_dist, 0.1);
  EXPECT_NEAR(current_v, 0.0, 1e-3);

  // Verify State Machine Latch
  EXPECT_EQ(sm.get_state().knot_index, 1);
  EXPECT_FALSE(sm.get_state().is_completed);

  // 2. Wait part of duration
  const double partial_wait = 2.0;
  drive(partial_wait, 0.0);
  EXPECT_NEAR(sm.get_state().accumulated_wait, partial_wait, 0.1);

  // 3. Wait remaining (add buffer for float noise)
  const double remaining_wait = target_wait - partial_wait + 0.1;
  drive(remaining_wait, 0.0);
  EXPECT_TRUE(sm.get_state().is_completed);

  plot_stop_state_results(
    "RealisticStop", t_log, x_log, v_log, wait_log, stop_dist, target_wait, stop_vel_thresh);
}

TEST_F(StopStateTest, RealisticOverrun)
{
  // Scenario: Brake too soft, causing overrun.
  const double initial_v = 10.0;
  const double soft_accel = -4.0;  // Softer than needed (-5.0)

  // Expected Physics: s = -v^2 / 2a = 100 / 8 = 12.5m
  const double expected_dist = -(initial_v * initial_v) / (2.0 * soft_accel);
  const double stop_time = std::abs(initial_v / soft_accel);
  const double stop_dist = 10.0;

  drive(stop_time + 0.1, soft_accel);  // +0.1 to ensure v=0

  // Verify we stopped past the line
  EXPECT_NEAR(current_x, expected_dist, 0.1);
  EXPECT_GT(current_x, stop_dist);

  // Tolerance is +3.0m (up to 13.0m). 12.5m is valid.
  EXPECT_EQ(sm.get_state().knot_index, 1) << "Should detect stop despite overrun";

  // Verify wait starts accumulating
  drive(1.0, 0.0);
  EXPECT_GT(sm.get_state().accumulated_wait, 0.5);

  plot_stop_state_results(
    "RealisticOverrun", t_log, x_log, v_log, wait_log, stop_dist, std::nullopt, stop_vel_thresh);
}

TEST_F(StopStateTest, RealisticCreep)
{
  // 1. Initial Hard Stop
  const double initial_v = 10.0;
  const double stop_accel = -5.0;
  const double stop_dist = 10.0;
  drive(std::abs(initial_v / stop_accel), stop_accel);

  // 2. Initial Wait
  const double initial_wait = 2.0;
  drive(initial_wait, 0.0);

  double wait_snapshot = sm.get_state().accumulated_wait;
  EXPECT_NEAR(wait_snapshot, initial_wait, 0.1);

  // 3. Creep Phase
  const double creep_accel = 1.0;
  const double creep_brake_accel = -1.0;
  const double creep_duration = 0.5;

  // A. Creep Forward (Accel)
  drive(creep_duration, creep_accel);
  EXPECT_GT(current_v, stop_vel_thresh);

  // B. Brake Again (Decel)
  drive(creep_duration, creep_brake_accel);
  EXPECT_NEAR(current_v, 0.0, 1e-3);

  // 4. Final Wait
  const double final_wait = 1.0;
  drive(final_wait, 0.0);

  // 5. Verify Total Time using Computed Leakage
  double accel_leak = compute_expected_leakage(std::abs(creep_accel));
  double decel_leak = compute_expected_leakage(std::abs(creep_brake_accel));
  double total_expected = initial_wait + accel_leak + decel_leak + final_wait;

  // Now we can use a very tight tolerance because the math is exact
  EXPECT_NEAR(sm.get_state().accumulated_wait, total_expected, 1e-4);

  plot_stop_state_results(
    "RealisticCreep", t_log, x_log, v_log, wait_log, stop_dist, std::nullopt, stop_vel_thresh);
}

TEST_F(StopStateTest, FullCycle_MoveStopWaitMove)
{
  // ---------------------------------------------------------
  // 1. SETUP: Physics & Requirements
  // ---------------------------------------------------------
  const double initial_v = 10.0;
  const double stop_dist = 10.0;    // Stop line at 10.0m
  const double target_wait = 5.0;   // Required wait time
  const double depart_accel = 2.0;  // Acceleration to leave

  // Calculate braking physics: v^2 = u^2 + 2as -> a = -v^2 / 2s
  const double braking_accel = -(initial_v * initial_v) / (2.0 * stop_dist);
  const double braking_time = std::abs(initial_v / braking_accel);

  // ---------------------------------------------------------
  // 2. PHASE 1: APPROACH & STOP
  // ---------------------------------------------------------
  // Drive exactly enough to stop at 10m
  drive(braking_time, braking_accel);

  // Verification: We are stopped at the line
  EXPECT_NEAR(current_x, stop_dist, 0.1);
  EXPECT_NEAR(current_v, 0.0, 1e-3);

  // State Machine: Should be "Latched" (Index 1) but "Not Completed"
  EXPECT_EQ(sm.get_state().knot_index, 1);
  EXPECT_FALSE(sm.get_state().is_completed);

  // ---------------------------------------------------------
  // 3. PHASE 2: WAIT (Simulate 5.1 seconds)
  // ---------------------------------------------------------
  // We stay at v=0. The state machine should accumulate time.
  drive(target_wait + 0.1, 0.0);

  // Verification: Wait is complete
  EXPECT_GE(sm.get_state().accumulated_wait, target_wait);
  EXPECT_TRUE(sm.get_state().is_completed) << "State machine did not authorize departure!";

  // ---------------------------------------------------------
  // 4. PHASE 3: DEPARTURE (Move Again)
  // ---------------------------------------------------------
  // Now that is_completed is true, the car (physically) moves away.
  // We accelerate for 3 seconds.
  drive(3.0, depart_accel);

  // Physics Check:
  // Distance moved = 0.5 * a * t^2 = 0.5 * 2.0 * 9.0 = 9.0m
  // Position = 10.0m (start) + 9.0m = 19.0m
  EXPECT_GT(current_x, 15.0);
  EXPECT_GT(current_v, 5.0);

  // ---------------------------------------------------------
  // 5. PHASE 4: RESET VERIFICATION
  // ---------------------------------------------------------
  // The stop fence is +3.0m (13.0m total). We are at ~19.0m.
  // The state machine should have lost the target and reset.
  EXPECT_EQ(sm.get_state().knot_index, -1)
    << "State machine should reset after leaving the stop zone (passed fence)";

  // ---------------------------------------------------------
  // 6. VISUALIZE
  // ---------------------------------------------------------
  plot_stop_state_results(
    "FullCycle_MoveStopWaitMove", t_log, x_log, v_log, wait_log, stop_dist, target_wait,
    stop_vel_thresh);
}
}  // namespace autoware::time_to_space_trajectory_converter
