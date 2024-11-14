// Copyright 2021 Tier IV, Inc.
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

#include "interpolation/spline_interpolation.hpp"

#include "interpolation/interpolation_utils.hpp"

#include <algorithm>

Eigen::VectorXd solve_tridiagonal_matrix_algorithm(
  const Eigen::Ref<const Eigen::VectorXd> & a, const Eigen::Ref<const Eigen::VectorXd> & b,
  const Eigen::Ref<const Eigen::VectorXd> & c, const Eigen::Ref<const Eigen::VectorXd> & d)
{
  auto n = d.size();

  if (n == 1) {
    return d.array() / b.array();
  }

  Eigen::VectorXd c_prime = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd d_prime = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);

  // Forward sweep
  c_prime(0) = c(0) / b(0);
  d_prime(0) = d(0) / b(0);

  for (auto i = 1; i < n; i++) {
    double m = 1.0 / (b(i) - a(i - 1) * c_prime(i - 1));
    c_prime(i) = i < n - 1 ? c(i) * m : 0;
    d_prime(i) = (d(i) - a(i - 1) * d_prime(i - 1)) * m;
  }

  // Back substitution
  x(n - 1) = d_prime(n - 1);

  for (auto i = n - 2; i >= 0; i--) {
    x(i) = d_prime(i) - c_prime(i) * x(i + 1);
  }

  return x;
}

namespace interpolation
{
std::vector<double> slerp(
  const std::vector<double> & base_keys, const std::vector<double> & base_values,
  const std::vector<double> & query_keys)
{
  SplineInterpolation interpolator;

  // calculate spline coefficients
  interpolator.calcSplineCoefficients(base_keys, base_values);

  // interpolate base_keys at query_keys
  return interpolator.getSplineInterpolatedValues(query_keys);
}
}  // namespace interpolation

void SplineInterpolation::calcSplineCoefficients(
  const std::vector<double> & base_keys, const std::vector<double> & base_values)
{
  // throw exceptions for invalid arguments
  interpolation_utils::validateKeysAndValues(base_keys, base_values);

  Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(
    base_keys.data(), static_cast<Eigen::Index>(base_keys.size()));
  Eigen::VectorXd y = Eigen::Map<const Eigen::VectorXd>(
    base_values.data(), static_cast<Eigen::Index>(base_values.size()));

  const auto n = x.size();

  if (n == 2) {
    a_ = Eigen::VectorXd::Zero(1);
    b_ = Eigen::VectorXd::Zero(1);
    c_ = Eigen::VectorXd::Zero(1);
    d_ = Eigen::VectorXd::Zero(1);
    c_[0] = (y[1] - y[0]) / (x[1] - x[0]);
    d_[0] = y[0];
    base_keys_ = base_keys;
    return;
  }

  // Create Tridiagonal matrix
  Eigen::VectorXd v(n);
  Eigen::VectorXd h = x.segment(1, n - 1) - x.segment(0, n - 1);
  Eigen::VectorXd a = h.segment(1, n - 3);
  Eigen::VectorXd b = 2 * (h.segment(0, n - 2) + h.segment(1, n - 2));
  Eigen::VectorXd c = h.segment(1, n - 3);
  Eigen::VectorXd y_diff = y.segment(1, n - 1) - y.segment(0, n - 1);
  Eigen::VectorXd d = 6 * (y_diff.segment(1, n - 2).array() / h.tail(n - 2).array() -
                           y_diff.segment(0, n - 2).array() / h.head(n - 2).array());

  // Solve tridiagonal matrix
  v.segment(1, n - 2) = solve_tridiagonal_matrix_algorithm(a, b, c, d);
  v[0] = 0;
  v[n - 1] = 0;

  // Calculate spline coefficients
  a_ = (v.tail(n - 1) - v.head(n - 1)).array() / 6.0 / (x.tail(n - 1) - x.head(n - 1)).array();
  b_ = v.segment(0, n - 1) / 2.0;
  c_ = (y.tail(n - 1) - y.head(n - 1)).array() / (x.tail(n - 1) - x.head(n - 1)).array() -
       (x.tail(n - 1) - x.head(n - 1)).array() *
         (2 * v.segment(0, n - 1).array() + v.segment(1, n - 1).array()) / 6.0;
  d_ = y.head(n - 1);
  base_keys_ = base_keys;
}

Eigen::Index SplineInterpolation::get_index(double key) const
{
  auto it = std::lower_bound(base_keys_.begin(), base_keys_.end(), key);
  return std::clamp(
    static_cast<int>(std::distance(base_keys_.begin(), it)) - 1, 0,
    static_cast<int>(base_keys_.size()) - 2);
}

std::vector<double> SplineInterpolation::getSplineInterpolatedValues(
  const std::vector<double> & query_keys) const
{
  // throw exceptions for invalid arguments
  interpolation_utils::validateKeys(base_keys_, query_keys);

  std::vector<double> interpolated_values;
  interpolated_values.reserve(query_keys.size());

  for (const auto & key : query_keys) {
    const auto idx = get_index(key);
    const auto dx = key - base_keys_[idx];
    interpolated_values.emplace_back(
      a_[idx] * dx * dx * dx + b_[idx] * dx * dx + c_[idx] * dx + d_[idx]);
  }

  return interpolated_values;
}

std::vector<double> SplineInterpolation::getSplineInterpolatedDiffValues(
  const std::vector<double> & query_keys) const
{
  // throw exceptions for invalid arguments
  interpolation_utils::validateKeys(base_keys_, query_keys);

  std::vector<double> interpolated_diff_values;
  interpolated_diff_values.reserve(query_keys.size());

  for (const auto & key : query_keys) {
    const auto idx = get_index(key);
    const auto dx = key - base_keys_[idx];
    interpolated_diff_values.emplace_back(3 * a_[idx] * dx * dx + 2 * b_[idx] * dx + c_[idx]);
  }

  return interpolated_diff_values;
}
