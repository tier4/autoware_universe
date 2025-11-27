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

#include "spatial_preprocessor.hpp"
#include "test_utils.hpp"
#include "test_utils_plotter.hpp"

#include <fmt/format.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{

class SpatialPreprocessorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Default config
    config_.min_knot_dist = 1.0;
    config_.stop_v_thresh = 0.1;
  }

  SpatialPreprocessorConfig config_;
  SplineData output_;
};

// Standard baseline: Perfect Stop -> Move -> Stop -> Move -> Stop
TEST_F(SpatialPreprocessorTest, HandlePerfectStopGo)
{
  output_ = {};

  TrajectoryBuilder builder(0.0, 0.1);  // No noise, dt=0.1
  builder.add_wait(0.1).add_wait(2.0).add_move(2.0, 5.0).add_wait(2.0).add_move(2.0, 5.0).add_wait(
    0.1);

  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_perfect_stop_go.png");

  ASSERT_FALSE(result.has_value()) << result.value();

  // 1. Initial Stop
  EXPECT_NEAR(output_.s[0], 0.0, 1e-3);
  EXPECT_NEAR(output_.wait_times[0], 2.0, 0.15);

  // 2. Middle Stop (approx 10m mark)
  bool found_middle_stop = false;
  for (size_t i = 0; i < output_.s.size(); ++i) {
    if (std::abs(output_.s[i] - 10.0) < 0.5 && output_.wait_times[i] > 1.0) {
      EXPECT_NEAR(output_.wait_times[i], 2.0, 0.15);
      EXPECT_NEAR(output_.v[i], 0.0, 1e-3);
      found_middle_stop = true;
    }
  }
  EXPECT_TRUE(found_middle_stop) << "Failed to detect the 2.0s stop at 10m mark";

  // 3. Total Length (10m + 10m)
  EXPECT_NEAR(output_.s.back(), 20.0, 0.5);
}

// Tests robustness against GPS/Localization jitter
TEST_F(SpatialPreprocessorTest, HandleNoisyStopGo)
{
  TrajectoryBuilder builder(0.1, 0.1);  // 10cm noise
  builder.add_wait(0.1)
    .add_wait(2.0)
    .without_noise()
    .add_move(2.0, 5.0)
    .with_noise()
    .add_wait(2.0)
    .without_noise()
    .add_move(2.0, 5.0)
    .add_wait(0.1);

  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_noisy_stop_go.png");
  ASSERT_FALSE(result.has_value());

  // 1. Total Distance (allow larger tolerance for noise accumulation)
  EXPECT_NEAR(output_.s.back(), 20.0, 1.0);

  // 2. Stop Detection (Should trust v=0 over noisy position)
  bool found_stop = false;
  for (size_t i = 0; i < output_.s.size(); ++i) {
    if (std::abs(output_.s[i] - 10.0) < 2.0 && output_.wait_times[i] > 1.5) {
      found_stop = true;
      EXPECT_NEAR(output_.v[i], 0.0, 1e-3);
    }
  }
  EXPECT_TRUE(found_stop) << "Failed to identify stop amidst GPS noise.";
}

// Tests manual sparse input construction
TEST_F(SpatialPreprocessorTest, HandleSparseTrajectory)
{
  std::vector<PlannerPoint> input;
  input.push_back({0.0, {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0});    // Stop
  input.push_back({1.0, {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0});    // Wait 1s
  input.push_back({2.0, {10.0, 0.0, 0.0}, 10.0, 0.0, 0.0});  // Move to 10m
  input.push_back({3.0, {10.0, 0.0, 0.0}, 0.0, 0.0, 0.0});   // Stop

  auto result = SpatialPreprocessor::process(input, output_, config_);
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_sparse_trajectory.png");
  ASSERT_FALSE(result.has_value());

  ASSERT_EQ(output_.s.size(), 2);  // Expect 2 knots: 0m and 10m
  EXPECT_NEAR(output_.wait_times[0], 1.0, 1e-3);
  EXPECT_NEAR(output_.s[1], 10.0, 1e-3);
  EXPECT_NEAR(output_.wait_times[1], 1.0, 1e-3);
}

// Tests stationary vehicle logic
TEST_F(SpatialPreprocessorTest, HandleStopOnly)
{
  TrajectoryBuilder builder;
  builder.add_wait(0.1).add_wait(3.0);

  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_stop_only.png");
  ASSERT_FALSE(result.has_value());

  ASSERT_EQ(output_.s.size(), 1);
  EXPECT_NEAR(output_.wait_times[0], 3.0, 0.15);
  EXPECT_NEAR(output_.v[0], 0.0, 1e-3);
}

// Stop -> Move -> Stop
TEST_F(SpatialPreprocessorTest, HandleStopMoveStop)
{
  TrajectoryBuilder builder;
  builder.set_dt(0.1).set_noise(0.0).add_wait(0.1).add_wait(1.0).add_move(3.0, 5.0).add_wait(1.0);

  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_stop_move_stop.png");
  ASSERT_FALSE(result.has_value());

  EXPECT_NEAR(output_.s.back(), 15.0, 0.5);  // 3s * 5m/s
  EXPECT_NEAR(output_.wait_times.front(), 1.0, 0.15);
  EXPECT_NEAR(output_.wait_times.back(), 1.0, 0.15);
}

// Replanning while already moving (10m/s -> Brake -> Stop)
TEST_F(SpatialPreprocessorTest, HandleMovingDecelStop)
{
  TrajectoryBuilder builder;
  builder.set_dt(0.1)
    .set_start_state(0.0, 0.0, 10.0)
    .add_move(2.0, 10.0)  // Cruise
    .add_ramp(5.0, 0.0)   // Brake to 0
    .add_wait(2.0);       // Stop

  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_moving_decel_stop.png");
  ASSERT_FALSE(result.has_value());

  EXPECT_NEAR(output_.s.back(), 45.0, 1.0);  // 20m + 25m
  EXPECT_NEAR(output_.v[0], 10.0, 1e-3);     // Moving start
  EXPECT_NEAR(output_.wait_times[0], 0.0, 1e-3);
  EXPECT_NEAR(output_.wait_times.back(), 2.0, 0.15);
}

// Tests unstable update rates (dt jitter)
TEST_F(SpatialPreprocessorTest, HandleVariableTimeSteps)
{
  TrajectoryBuilder builder;
  builder.set_dt(0.1).set_time_jitter(0.02).add_move(3.0, 5.0).add_wait(2.0);

  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  ASSERT_FALSE(result.has_value());
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_variable_time_step.png");

  double expected_dist = input.back().pos.x;
  EXPECT_NEAR(output_.s.back(), expected_dist, 0.5);

  double actual_wait = 0.0;
  for (auto w : output_.wait_times) actual_wait += w;
  EXPECT_NEAR(actual_wait, 2.0, 0.2);
}

// "Emergency Brake" Scenario: High speed (20m/s) -> Instant Stop
TEST_F(SpatialPreprocessorTest, HandleInstantStop)
{
  TrajectoryBuilder builder;
  builder.set_dt(0.1).add_move(2.0, 20.0).add_wait(2.0);

  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  ASSERT_FALSE(result.has_value());
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_instant_stop.png");

  // Tolerance relaxed to 1.0m to account for Spline Gibbs Phenomenon (overshoot)
  // caused by the physically impossible "brick wall" stop (20m/s -> 0m/s in 0s).
  EXPECT_NEAR(output_.s.back(), 40.0, 1.0);
  EXPECT_GT(output_.wait_times.back(), 1.0);
}

// 2D Figure-8 path to test Euclidean integration
TEST_F(SpatialPreprocessorTest, HandleInfinityLoop)
{
  config_.min_knot_dist = 1.0;
  SpatialPreprocessor preprocessor(config_);
  auto input = generate_infinity_loop(5.0, 10.0);

  auto result = SpatialPreprocessor::process(input, output_, config_);
  ASSERT_FALSE(result.has_value());
  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_infinity_loop.png");

  // S must always increase
  for (size_t i = 1; i < output_.s.size(); ++i) {
    EXPECT_GT(output_.s[i], output_.s[i - 1] + g_geom_eps);
  }

  // Check bounds to ensure figure-8 shape was traced
  double min_x = 100;
  double max_x = -100;
  double min_y = 100;
  double max_y = -100;
  for (auto x : output_.x) {
    if (x < min_x) min_x = x;
    if (x > max_x) max_x = x;
  }
  for (auto y : output_.y) {
    if (y < min_y) min_y = y;
    if (y > max_y) max_y = y;
  }

  EXPECT_LT(min_x, -9.0);
  EXPECT_GT(max_x, 9.0);
}

// Very slow movement on a curve
TEST_F(SpatialPreprocessorTest, HandleCreepingOnCurve)
{
  config_.stop_v_thresh = 1.0 / 3.6;  // Keep your low threshold

  constexpr double creep_velocity = 1.1 / 3.6;
  constexpr double duration = 50.0;
  constexpr double wavelength = 10.0;
  constexpr double dt = 0.5;

  auto input = generate_sinusoidal_creep(creep_velocity, duration, wavelength, dt);
  auto result = SpatialPreprocessor::process(input, output_, config_);

  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_creeping_on_curve.png");

  const double linear_dist = creep_velocity * duration;  // 1.0m

  const double expected_arc_dist = compute_sine_arc_length(linear_dist, wavelength);

  // The distance the car moves in one simulation tick.
  // The preprocessor cannot see details smaller than this gap.
  const double step_size = creep_velocity * dt;  // 0.05 * 0.1 = 0.005m

  // We allow an error of roughly 1 simulation step.
  // We clamp it to a minimum of 1cm (0.01) to handle floating point noise at very low speeds.
  const double tolerance = std::max(0.01, step_size * 1.5);
  EXPECT_NEAR(output_.s.back(), expected_arc_dist, tolerance);

  for (size_t i = 0; i < output_.s.size(); ++i) {
    double expected_y = 1.0 * std::sin(output_.x[i] * 2.0 * M_PI / wavelength);
    EXPECT_NEAR(output_.y[i], expected_y, 0.05);  // Relaxed geometry tolerance slightly
  }
}
// ---------------------------------------------------------
// TEST CASE: Constant Jerk on Curved Path (Aggressive vs Non-Aggressive)
// ---------------------------------------------------------
TEST_F(SpatialPreprocessorTest, HandleConstantJerkProfiles)
{
  TrajectoryBuilder builder;
  builder.set_dt(0.1);

  // 1. Setup Curve: Driving on a 50m radius circle
  builder.set_circular_path(50.0);

  // ==========================================
  // PROFILE 1: NON-AGGRESSIVE (Gentle City Driving)
  // Target: Accel to ~5 m/s smoothly, then stop.
  // ==========================================

  // A. Smooth Acceleration
  // Jerk +1.0 for 2s -> Accel reaches 2.0 m/s^2
  builder.add_constant_jerk(2.0, 1.0);
  // Jerk -1.0 for 2s -> Accel drops to 0.0 m/s^2. Velocity is smooth.
  builder.add_constant_jerk(2.0, -1.0);

  // B. Cruise (Constant Velocity)
  // Maintain the velocity we reached (~4-5 m/s) for 3 seconds
  builder.add_move(3.0, builder.get_current_velocity());

  // C. Smooth Deceleration
  // Jerk -1.0 for 2s -> Accel goes to -2.0 m/s^2
  builder.add_constant_jerk(2.0, -1.0);
  // Jerk +1.0 for 2s -> Accel returns to 0. V should be near 0.
  // Note: We might need to clamp if math isn't perfect, but let's trust the builder.
  builder.add_constant_jerk(2.0, 1.0);

  // D. Stop
  builder.add_wait(2.0);

  // ==========================================
  // PROFILE 2: AGGRESSIVE (Emergency / Sport)
  // Target: Hard Accel to high speed, then Emergency Brake
  // ==========================================

  // A. Hard Acceleration (Launch)
  // Jerk +5.0 for 1s -> Accel hits 5 m/s^2 instantly
  builder.add_constant_jerk(1.0, 5.0);
  // Jerk -5.0 for 1s -> Accel back to 0
  builder.add_constant_jerk(1.0, -5.0);

  // B. Short Cruise (High Speed)
  builder.add_move(2.0, builder.get_current_velocity());

  // C. Hard Braking (Emergency Stop)
  // Jerk -10.0 for 0.5s -> Decel hits -5 m/s^2 very fast
  builder.add_constant_jerk(0.5, -10.0);
  // Hold that hard deceleration (Linear Ramp logic would be needed for constant decel,
  // but here we use jerk to "ease out" the brake just before stop?)
  // Let's just Jerk back up to 0 to simulate releasing the brake at the stop line.
  builder.add_constant_jerk(0.5, 10.0);

  // Force stop
  builder.add_wait(2.0);

  // --- EXECUTION ---
  auto input = builder.get_result();
  auto result = SpatialPreprocessor::process(input, output_, config_);
  ASSERT_FALSE(result.has_value());

  plot_and_save(input, output_, "test_spatial_preprocessor", "handle_constant_jerk_profiles.png");

  // --- VERIFICATION ---

  // 1. Geometry Check
  // Since we drove on a circle, X and Y should effectively trace an arc.
  // We can verify that Y is not 0 (which would imply straight line).
  bool traveled_curve = false;
  for (auto y : output_.y) {
    if (std::abs(y) > 5.0) traveled_curve = true;
  }
  EXPECT_TRUE(traveled_curve) << "Trajectory did not follow the curved path";

  // 2. Stop Detection
  // We have at least 2 stops: One after Gentle, One after Aggressive.
  int stop_count = 0;
  for (double w : output_.wait_times) {
    if (w > 1.0) stop_count++;
  }
  EXPECT_GE(stop_count, 2);

  // 3. Continuity
  // Ensure the aggressive jerk didn't break the spline continuity (gaps in S)
  for (size_t i = 1; i < output_.s.size(); ++i) {
    EXPECT_GT(output_.s[i], output_.s[i - 1] - 1e-4);
  }
}

}  // namespace autoware::time_to_space_trajectory_converter
