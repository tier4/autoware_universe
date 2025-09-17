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
constexpr size_t POINT_STATE_DIM = 7;

enum PolylineLabel { LANE = 0, ROAD_LINE = 1, ROAD_EDGE = 2, CROSSWALK = 3 };

// Normalize a 3D direction vector (shared utility)
inline void normalize_direction(double & dx, double & dy, double & dz)
{
  const double magnitude = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (magnitude > 1e-6f) {
    dx /= magnitude;
    dy /= magnitude;
    dz /= magnitude;
  }
}

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
   * @param dx Normalized delta x.
   * @param dy Normalized delta y.
   * @param dz Normalized delta z.
   * @param label Label.
   */
  LanePoint(
    const double x, const double y, const double z, const double dx, const double dy,
    const double dz)
  : data_({x, y, z, dx, dy, dz, 0.0}), x_(x), y_(y), z_(z), dx_(dx), dy_(dy), dz_(dz), label_(0.0)
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
    double new_x = x_ + t * (other.x_ - x_);
    double new_y = y_ + t * (other.y_ - y_);
    double new_z = z_ + t * (other.z_ - z_);

    // Calculate direction vector from interpolated positions
    double new_dx = other.x_ - x_;
    double new_dy = other.y_ - y_;
    double new_dz = other.z_ - z_;

    // Check if points are too close
    const double magnitude_sq = new_dx * new_dx + new_dy * new_dy + new_dz * new_dz;
    if (magnitude_sq < 1e-12) {
      // If points are too close, use the first point's direction
      new_dx = dx_;
      new_dy = dy_;
      new_dz = dz_;
    } else {
      normalize_direction(new_dx, new_dy, new_dz);
    }

    return LanePoint{new_x, new_y, new_z, new_dx, new_dy, new_dz};
  }

private:
  std::array<double, POINT_STATE_DIM> data_;
  double x_{0.0}, y_{0.0}, z_{0.0}, dx_{0.0}, dy_{0.0}, dz_{0.0}, label_{0.0};
};

using Polyline = std::vector<LanePoint>;

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__POLYLINE_HPP_
