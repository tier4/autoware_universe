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

#include "test_utils.hpp"

#include <fmt/format.h>

#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
TrajectoryBuilder & TrajectoryBuilder::set_dt(double dt)
{
  dt_ = dt;
  return *this;
}

TrajectoryBuilder & TrajectoryBuilder::set_time_jitter(double std_dev)
{
  time_jitter_ = std_dev;
  return *this;
}

TrajectoryBuilder & TrajectoryBuilder::set_path_function(PathFunc func)
{
  path_func_ = std::move(func);
  return *this;
}

TrajectoryBuilder & TrajectoryBuilder::with_noise()
{
  noise_active_ = true;
  return *this;
}

// Disable noise for subsequent commands (Perfect data)
TrajectoryBuilder & TrajectoryBuilder::without_noise()
{
  noise_active_ = false;
  return *this;
}
TrajectoryBuilder & TrajectoryBuilder::set_noise(double std_dev)
{
  global_noise_dev_ = std_dev;
  using param_t = std::normal_distribution<>::param_type;
  noise_dist_.param(param_t(0.0, std_dev));

  noise_active_ = std_dev > g_math_eps;
  return *this;
}

TrajectoryBuilder & TrajectoryBuilder::set_start_state(double t, double s, double v)
{
  current_t_ = t;
  current_s_ = s;
  current_v_ = v;
  return *this;
}

TrajectoryBuilder & TrajectoryBuilder::add_wait(double duration_s)
{
  int steps = static_cast<int>(std::ceil(duration_s / dt_ - g_math_eps));
  current_v_ = 0.0;

  for (int i = 0; i < steps; ++i) {
    add_point_and_integrate(0.0, 0.0);
  }
  return *this;
}

// Command: "MOVE, Duration, Velocity"
TrajectoryBuilder & TrajectoryBuilder::add_move(double duration_s, double velocity, double accel)
{
  int steps = static_cast<int>(std::ceil(duration_s / dt_ - g_math_eps));
  current_v_ = velocity;

  for (int i = 0; i < steps; ++i) {
    add_point_and_integrate(velocity, accel);
  }
  return *this;
}

// Command: "RAMP, Duration, Target Velocity" (Linear Acceleration)
TrajectoryBuilder & TrajectoryBuilder::add_ramp(double duration_s, double target_v)
{
  // 1. Fix Floating Point Ceiling (prevents extra step)
  int steps = static_cast<int>(std::ceil(duration_s / dt_ - 1e-6));
  if (steps <= 0) return *this;

  // 2. Fix Physics Consistency
  // Calculate acceleration based on the ACTUAL simulated time (steps * dt)
  // NOT the requested duration. This ensures V hits target_v exactly.
  double actual_duration = steps * dt_;
  double accel = (target_v - current_v_) / actual_duration;

  for (int i = 0; i < steps; ++i) {
    add_point_and_integrate(current_v_, accel);
    current_v_ += accel * dt_;
  }

  return *this;
}

TrajectoryBuilder & TrajectoryBuilder::add_constant_jerk(double duration_s, double jerk)
{
  int steps = static_cast<int>(std::ceil(duration_s / dt_ - 1e-6));
  if (steps <= 0) return *this;

  for (int i = 0; i < steps; ++i) {
    // Record state at start of step
    add_point_and_integrate(current_v_, current_a_);

    // Update Kinematics for NEXT step
    // v = v0 + a*t + 0.5*j*t^2
    current_v_ += current_a_ * dt_ + 0.5 * jerk * dt_ * dt_;

    // a = a0 + j*t
    current_a_ += jerk * dt_;
  }
  return *this;
}

TrajectoryBuilder & TrajectoryBuilder::set_circular_path(double radius)
{
  // Lambda to map arc-length 's' to (x, y) on a circle
  auto circle_func = [radius](double s) -> std::pair<double, double> {
    double theta = s / radius;
    // Standard circle starting at (0,0) and moving along +X initially approx
    // x = R * sin(theta)
    // y = R * (1 - cos(theta))
    // (Or any parameterization you prefer. This starts at 0,0 heading North-ish?
    // Let's use standard unit circle shifted: x=R*cos, y=R*sin.
    // But we want start at 0,0 heading +X.
    // x = R * sin(theta)
    // y = R * (1 - cos(theta)) -> derivative at 0 is dx/dth=R, dy/dth=0. Perfect.)
    return {radius * std::sin(theta), radius * (1.0 - std::cos(theta))};
  };

  path_func_ = circle_func;
  return *this;
}

void TrajectoryBuilder::add_point_and_integrate(double v_command, double a_command)
{
  PlannerPoint p{};
  p.t = current_t_;
  p.v = v_command;
  p.a = a_command;

  // 1. Current Geometry
  auto [x_ideal, y_ideal] = path_func_(current_s_);

  // 2. Apply Noise
  double noise_x = 0.0;
  double noise_y = 0.0;
  if (global_noise_dev_ > g_math_eps && noise_active_) {
    noise_x = noise_dist_(gen_);
    noise_y = noise_dist_(gen_);
  }
  p.pos = {x_ideal + noise_x, y_ideal + noise_y, 0.0};

  // 3. Heading (Lookahead)
  auto [x_next, y_next] = path_func_(current_s_ + 0.01);
  p.yaw = std::atan2(y_next - y_ideal, x_next - x_ideal);

  points_.push_back(p);

  // 4. Integrate for NEXT step

  // Calculate Time Step (Jitter)
  double step_dt = dt_;
  if (time_jitter_ > g_math_eps) {
    std::normal_distribution<> t_dist(0.0, time_jitter_);
    step_dt += t_dist(gen_);
    if (step_dt < 0.001) step_dt = 0.001;
  }

  // Kinematic Integration: s += v*t + 0.5*a*t^2
  // Using the ACTUAL randomized step_dt to ensure physics consistency
  double ds = v_command * step_dt + 0.5 * a_command * step_dt * step_dt;

  current_s_ += ds;
  current_t_ += step_dt;

  // Note: current_v_ is updated by the caller (add_ramp/add_move) based on COMMAND logic,
  // not physics integration, because we simulate a perfect controller following the command
  // profile.
}

[[nodiscard]] std::vector<PlannerPoint> TrajectoryBuilder::get_result() const
{
  return points_;
}

[[nodiscard]] double TrajectoryBuilder::get_current_velocity() const
{
  return current_v_;
}
std::vector<PlannerPoint> generate_stop_go_trajectory(double noise_std_dev)
{
  constexpr double dt = 0.1;
  TrajectoryBuilder builder(noise_std_dev, dt);

  builder
    .add_wait(0.1)       // Initial stop point
    .add_wait(2.0)       // Phase 1: Wait 2s
    .add_move(2.0, 5.0)  // Phase 2: Move 2s @ 5m/s
    .add_wait(2.0)       // Phase 3: Wait 2s
    .add_move(2.0, 5.0)  // Phase 4: Move 2s @ 5m/s
    .add_wait(0.1);      // Final stop point

  return builder.get_result();
}

/**
 * @brief Generates an Infinity (Lemniscate) trajectory.
 * @param velocity Constant travel speed
 * @param scale Size of the loop (radius approx)
 */
std::vector<PlannerPoint> generate_infinity_loop(double velocity, double scale)
{
  // Lemniscate of Bernoulli (Parametric using s approximation)
  // Note: True arc-length parameterization of a Lemniscate is complex (Elliptic integrals).
  // For testing, we use 't' parameter as a proxy for 's' by scaling it.
  // Loop repeats every 2*PI parameter units.

  auto infinity_shape = [scale](double s) -> std::pair<double, double> {
    // Map s -> theta. Assume 1 "lap" is roughly 6 * scale meters.
    double theta = s / scale;

    // Parametric Formula
    double den = 1 + std::sin(theta) * std::sin(theta);
    double x = scale * std::cos(theta) / den;
    double y = scale * std::sin(theta) * std::cos(theta) / den;
    return {x, y};
  };

  // Calculate total duration for one full loop
  // Total S approx 60m if scale=10. Velocity=5.
  double duration = (6.28 * scale) / velocity;

  return TrajectoryBuilder()
    .set_dt(0.1)
    .set_path_function(infinity_shape)
    .add_move(duration, velocity)  // Drive the full loop
    .get_result();
}

std::vector<PlannerPoint> generate_sinusoidal_creep(
  double creep_velocity, double duration, double wavelength, double dt)
{
  // Parametric Wavenumber k
  const double k = 2.0 * M_PI / wavelength;

  // Capture 'k' in the lambda
  auto sine_shape = [k](double s) -> std::pair<double, double> {
    // Amplitude fixed at 1.0m for simplicity
    return {s, 1.0 * std::sin(k * s)};
  };

  // TrajectoryBuilder builder(0.0, 0.5); // (Optional: verify your constructor signature)
  TrajectoryBuilder builder;
  builder.set_dt(dt).set_path_function(sine_shape);

  builder.add_move(duration, creep_velocity);

  return builder.get_result();
}

/**
 * @brief Numerically integrates the arc length of y = A * sin(kx)
 * @param linear_dist The straight-line distance traveled (v * t)
 * @param wavelength The wave length of the sine wave
 * @param amplitude The amplitude of the sine wave (default 1.0 in your generator)
 * @return The actual distance traveled along the curve
 */
double compute_sine_arc_length(double linear_dist, double wavelength, double amplitude)
{
  const double k = 2.0 * M_PI / wavelength;
  const int steps = 1000;  // 1000 steps is accurate enough for <1e-6 error
  const double h = linear_dist / steps;
  double sum = 0.0;

  // Derivative of A*sin(kx) is A*k*cos(kx)
  // Arc Length Integrand: sqrt(1 + (dy/dx)^2)
  auto integrand = [&](double x) {
    double dydx = amplitude * k * std::cos(k * x);
    return std::sqrt(1.0 + dydx * dydx);
  };

  // Trapezoidal Rule Integration
  sum += 0.5 * integrand(0.0);
  sum += 0.5 * integrand(linear_dist);
  for (int i = 1; i < steps; ++i) {
    sum += integrand(i * h);
  }

  return sum * h;
}
}  // namespace autoware::time_to_space_trajectory_converter
