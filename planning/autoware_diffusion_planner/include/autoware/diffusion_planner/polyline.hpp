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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POLYLINE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POLYLINE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

namespace autoware::diffusion_planner
{
constexpr size_t POINT_STATE_DIM = 3;

struct LanePoint
{
  // Construct a new instance filling all elements by `0.0f`.
  LanePoint() : data_({0.0}) {}

  /**
   * @brief Construct a new instance with specified values.
   *
   * @param x X position.
   * @param y Y position.
   * @param z Z position.
   */
  LanePoint(const double x, const double y, const double z) : data_({x, y, z}), x_(x), y_(y), z_(z)
  {
  }

  // Construct a new instance filling all elements by `0.0f`.
  static LanePoint empty() noexcept { return {}; }

  // Return the point state dimensions `D`.
  static size_t dim() { return POINT_STATE_DIM; }

  // Return the x position of the point.
  [[nodiscard]] double x() const { return x_; }

  // Return the y position of the point.
  [[nodiscard]] double y() const { return y_; }

  // Return the z position of the point.
  [[nodiscard]] double z() const { return z_; }

  /**
   * @brief Return the distance between myself and another one.
   *
   * @param other Another point.
   * @return double Distance between myself and another one.
   */
  [[nodiscard]] double distance(const LanePoint & other) const
  {
    const double diff_x = x_ - other.x();
    const double diff_y = y_ - other.y();
    const double diff_z = z_ - other.z();
    return std::sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);
  }

  [[nodiscard]] LanePoint lerp(const LanePoint & other, double t) const
  {
    // Interpolate position
    const double new_x = x_ + t * (other.x_ - x_);
    const double new_y = y_ + t * (other.y_ - y_);
    const double new_z = z_ + t * (other.z_ - z_);
    return LanePoint{new_x, new_y, new_z};
  }

private:
  std::array<double, POINT_STATE_DIM> data_;
  double x_{0.0}, y_{0.0}, z_{0.0};
};

using Polyline = std::vector<LanePoint>;

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__POLYLINE_HPP_
