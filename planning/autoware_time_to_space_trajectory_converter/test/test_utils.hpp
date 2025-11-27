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

#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include "data_types.hpp"

#include <autoware/pyplot/patches.hpp>
#include <autoware/pyplot/pyplot.hpp>

#include <random>
#include <string>
#include <utility>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{

/**
 * @brief Helper class to manage trajectory state and noise generation.
 * Keeps the main test logic clean.
 */
class TrajectoryBuilder
{
public:
  explicit TrajectoryBuilder(double noise_std_dev = 0.0, double dt = 0.1, int seed = 12345)
  : dt_(dt), gen_(seed), noise_dist_(0.0, noise_std_dev), global_noise_dev_(noise_std_dev)
  {
    if (noise_std_dev > g_math_eps) {
      noise_active_ = true;
    }
  }

  TrajectoryBuilder & set_dt(double dt);
  TrajectoryBuilder & set_time_jitter(double std_dev);

  using PathFunc = std::function<std::pair<double, double>(double)>;
  TrajectoryBuilder & set_path_function(PathFunc func);

  TrajectoryBuilder & with_noise();
  TrajectoryBuilder & without_noise();
  TrajectoryBuilder & set_noise(double std_dev);
  TrajectoryBuilder & set_start_state(double t, double s, double v);
  TrajectoryBuilder & add_wait(double duration_s);
  TrajectoryBuilder & add_move(double duration_s, double velocity, double accel = 0.0);
  TrajectoryBuilder & add_ramp(double duration_s, double target_v);
  TrajectoryBuilder & add_constant_jerk(double duration_s, double jerk);
  TrajectoryBuilder & set_circular_path(double radius);

  void add_point_and_integrate(double v_command, double a_command);

  [[nodiscard]] std::vector<PlannerPoint> get_result() const;
  [[nodiscard]] double get_current_velocity() const;

private:
  std::vector<PlannerPoint> points_;
  double current_t_ = 0.0;
  double current_s_ = 0.0;
  double current_v_ = 0.0;
  double current_a_ = 0.0;
  double dt_ = 0.1;
  double time_jitter_{0.0};
  PathFunc path_func_ = [](double s) { return std::make_pair(s, 0.0); };

  std::mt19937 gen_;
  std::normal_distribution<> noise_dist_;
  double global_noise_dev_{0.0};
  bool noise_active_{false};
};

/**
 * @brief Generates the specific Stop-Go-Stop scenario using the builder.
 */
std::vector<PlannerPoint> generate_stop_go_trajectory(double noise_std_dev = 0.0);

/**
 * @brief Generates an Infinity (Lemniscate) trajectory.
 * @param velocity Constant travel speed
 * @param scale Size of the loop (radius approx)
 */
std::vector<PlannerPoint> generate_infinity_loop(double velocity = 5.0, double scale = 10.0);
std::vector<PlannerPoint> generate_sinusoidal_creep(
  double creep_velocity, double duration, double wavelength, double dt);

/**
 * @brief Numerically integrates the arc length of y = A * sin(kx)
 * @param linear_dist The straight-line distance traveled (v * t)
 * @param wavelength The wave length of the sine wave
 * @param amplitude The amplitude of the sine wave (default 1.0 in your generator)
 * @return The actual distance traveled along the curve
 */
double compute_sine_arc_length(double linear_dist, double wavelength, double amplitude = 1.0);
}  // namespace autoware::time_to_space_trajectory_converter
#endif  // TEST_UTILS_HPP_
