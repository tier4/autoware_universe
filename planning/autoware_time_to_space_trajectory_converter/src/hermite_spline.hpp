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

#ifndef HERMITE_SPLINE_HPP_
#define HERMITE_SPLINE_HPP_

#include <Eigen/Core>

#include <optional>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{
class HermiteSpline
{
public:
  struct Coefficients
  {
    Eigen::VectorXd a, b, c, d;
  };

  enum class SplineType {
    Monotone,  // PCHIP: Safe, no overshoot
    Akima,     // Smoother, local, needs N>=5
    Linear     // Piecewise Linear (C0), used for Z-axis or raw data
  };

  HermiteSpline() = default;

  // Unified Build Interface
  bool build_impl(
    const std::vector<double> & bases, const std::vector<double> & values,
    const std::optional<SplineType> type = std::nullopt);

  // Overload for rvalue
  bool build_impl(
    const std::vector<double> & bases, std::vector<double> && values,
    const std::optional<SplineType> type = std::nullopt);

  // Computation Interface (Identical for both)
  double compute(const double s) const;
  double compute_first_derivative(const double s) const;
  double compute_second_derivative(const double s) const;

private:
  // The specific math implementations
  static Eigen::VectorXd compute_monotone_tangents(
    const Eigen::VectorXd & h, const Eigen::VectorXd & delta);

  static Eigen::VectorXd compute_akima_tangents(
    const Eigen::VectorXd & h, const Eigen::VectorXd & delta);

  static Coefficients compute_coefficients(
    const Eigen::Ref<const Eigen::VectorXd> & bases,
    const Eigen::Ref<const Eigen::VectorXd> & values, SplineType type);

  int32_t get_index(const double s) const;

  std::vector<double> bases_;
  Coefficients coeffs_;
  mutable int32_t last_idx_ = 0;
};

}  // namespace autoware::time_to_space_trajectory_converter
#endif  // HERMITE_SPLINE_HPP_
