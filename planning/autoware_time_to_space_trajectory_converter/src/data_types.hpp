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

#ifndef DATA_TYPES_HPP_
#define DATA_TYPES_HPP_

#include "hermite_spline.hpp"

#include <cmath>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
constexpr double g_math_eps = 1e-9;
constexpr double g_geom_eps = 1e-4;

struct Vec3
{
  double x;
  double y;
  double z;
  [[nodiscard]] double norm2d() const { return std::hypot(x, y); }

  Vec3 operator-(const Vec3 & other) const { return {x - other.x, y - other.y, z - other.z}; }
};

struct PlannerPoint
{
  double t;
  Vec3 pos;
  double v;
  double a;
  double yaw;
};

enum class MotionState { STOPPED, CREEPING, MOVING };

struct SplineData
{
  std::vector<double> s;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> t;

  std::vector<double> wait_times;
  double stop_v_thresh = 0.001;

  [[nodiscard]] MotionState get_state(size_t i) const;
  void add_point(double new_s, const PlannerPoint & p, double wait_t);
  void reset(size_t capacity);  // clear and reserve
  void set_config(double v_thresh);
  void smooth_acceleration(double resolution);
};

struct SplineSet
{
  HermiteSpline sx;
  HermiteSpline sy;
  HermiteSpline sz;
  HermiteSpline sv;
  HermiteSpline sa;
  bool use_original_accel = false;

  static SplineSet build(const SplineData & input, bool recompute_accel = false);
  double compute_acceleration(double s, double v, double total_s) const;
};
}  // namespace autoware::time_to_space_trajectory_converter

#endif  // DATA_TYPES_HPP_
