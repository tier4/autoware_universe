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

#include "hermite_spline.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace autoware::time_to_space_trajectory_converter
{

// -------------------------------------------------------------------------
// Core Math Logic
// -------------------------------------------------------------------------

Eigen::VectorXd HermiteSpline::compute_monotone_tangents(
  const Eigen::VectorXd & h, const Eigen::VectorXd & delta)
{
  const auto n = static_cast<int32_t>(h.size());

  // m size is n+1 (number of points)
  Eigen::VectorXd m(n + 1);

  // Endpoints (One-sided difference)
  m(0) = delta(0);
  m(n) = delta(n - 1);

  // Internal points (Harmonic Mean)
  for (int32_t i = 1; i < n; ++i) {
    if (delta(i - 1) * delta(i) <= 0.0) {
      m(i) = 0.0;  // Peak/Valley
    } else {
      double w1 = 2.0 * h(i) + h(i - 1);
      double w2 = h(i) + 2.0 * h(i - 1);
      m(i) = (w1 + w2) / (w1 / delta(i - 1) + w2 / delta(i));
    }
  }
  return m;
}

Eigen::VectorXd HermiteSpline::compute_akima_tangents(
  const Eigen::VectorXd & h, const Eigen::VectorXd & delta)
{
  const auto n = static_cast<int32_t>(h.size());

  // Akima needs ghost points. We create a temporary augmented vector for slopes.
  // We need 2 ghost deltas on left, 2 ghost deltas on right.
  Eigen::VectorXd delta_aug(n + 4);
  delta_aug.segment(2, n) = delta;

  // Extrapolate Ghost Deltas (Linear assumption)
  delta_aug(1) = 2.0 * delta(0) - delta(1);
  delta_aug(0) = 2.0 * delta_aug(1) - delta(0);
  delta_aug(n + 2) = 2.0 * delta(n - 1) - delta(n - 2);
  delta_aug(n + 3) = 2.0 * delta_aug(n + 2) - delta(n - 1);

  // Calculate Tangents
  // m(i) corresponds to the knot at index i.
  // This depends on deltas: i-2, i-1, i, i+1 relative to the knot index.
  // In our augmented array starting at -2, the indices shift.
  Eigen::VectorXd m(n + 1);

  for (int32_t i = 0; i <= n; ++i) {
    double m1 = delta_aug(i);      // delta_{i-2}
    double m2 = delta_aug(i + 1);  // delta_{i-1}
    double m3 = delta_aug(i + 2);  // delta_{i}
    double m4 = delta_aug(i + 3);  // delta_{i+1}

    double w1 = std::abs(m4 - m3);
    double w2 = std::abs(m2 - m1);

    if (std::abs(w1 + w2) < 1e-9) {
      m(i) = (m2 + m3) / 2.0;
    } else {
      m(i) = (w1 * m2 + w2 * m3) / (w1 + w2);
    }
  }
  return m;
}
HermiteSpline::Coefficients HermiteSpline::compute_coefficients(
  const Eigen::Ref<const Eigen::VectorXd> & bases, const Eigen::Ref<const Eigen::VectorXd> & values,
  SplineType type)
{
  const auto num_points = static_cast<int32_t>(bases.size());
  const auto n = num_points - 1;

  Eigen::VectorXd h = bases.tail(n) - bases.head(n);
  Eigen::VectorXd delta = (values.tail(n) - values.head(n)).cwiseQuotient(h);

  Coefficients res;
  res.a = values;

  if (type == SplineType::Linear) {
    res.b = delta;  // For Linear, slope 'b' is just the segment delta. c and d are 0.
    res.c = Eigen::VectorXd::Zero(n);
    res.d = Eigen::VectorXd::Zero(n);
    return res;
  }

  Eigen::VectorXd m = (type == SplineType::Akima) ? compute_akima_tangents(h, delta)
                                                  : compute_monotone_tangents(h, delta);

  res.b = m.head(n);
  res.c.resize(n);
  res.d.resize(n);

  for (int32_t i = 0; i < n; ++i) {
    res.c(i) = (3.0 * delta(i) - 2.0 * m(i) - m(i + 1)) / h(i);
    res.d(i) = (m(i) + m(i + 1) - 2.0 * delta(i)) / (h(i) * h(i));
  }

  return res;
}

// -------------------------------------------------------------------------
// Implementation Boilerplate
// -------------------------------------------------------------------------

bool HermiteSpline::build_impl(
  const std::vector<double> & bases, const std::vector<double> & values,
  const std::optional<SplineType> type)
{
  if (bases.size() != values.size() || bases.size() < 2) return false;
  const SplineType actual_type =
    type.value_or((bases.size() >= 5) ? SplineType::Akima : SplineType::Monotone);

  Coefficients new_coeffs = compute_coefficients(
    Eigen::Map<const Eigen::VectorXd>(bases.data(), static_cast<int32_t>(bases.size())),
    Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<int32_t>(values.size())),
    actual_type);

  this->bases_ = bases;
  this->coeffs_ = std::move(new_coeffs);
  this->last_idx_ = 0;

  return true;
}

bool HermiteSpline::build_impl(
  const std::vector<double> & bases, std::vector<double> && values,
  const std::optional<SplineType> type)
{
  if (bases.size() != values.size() || bases.size() < 2) return false;

  const SplineType actual_type =
    type.value_or((bases.size() >= 5) ? SplineType::Akima : SplineType::Monotone);

  Coefficients new_coeffs = compute_coefficients(
    Eigen::Map<const Eigen::VectorXd>(bases.data(), static_cast<int32_t>(bases.size())),
    Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<int32_t>(values.size())),
    actual_type);

  this->bases_ = bases;
  this->coeffs_ = std::move(new_coeffs);
  this->last_idx_ = 0;

  return true;
}

double HermiteSpline::compute(const double s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->bases_.at(i);
  return coeffs_.a(i) + coeffs_.b(i) * dx + coeffs_.c(i) * dx * dx + coeffs_.d(i) * dx * dx * dx;
}

double HermiteSpline::compute_first_derivative(const double s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->bases_.at(i);
  return coeffs_.b(i) + 2 * coeffs_.c(i) * dx + 3 * coeffs_.d(i) * dx * dx;
}

double HermiteSpline::compute_second_derivative(const double s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->bases_.at(i);
  return 2 * coeffs_.c(i) + 6 * coeffs_.d(i) * dx;
}

int32_t HermiteSpline::get_index(const double s) const
{
  if (bases_.empty()) return 0;
  if (s <= bases_.front()) return 0;
  if (s >= bases_.back() - 1e-9) return static_cast<int32_t>(bases_.size()) - 2;

  if (s >= bases_[last_idx_] && s < bases_[last_idx_ + 1]) return last_idx_;
  if (
    last_idx_ + 2 < static_cast<int32_t>(bases_.size()) && s >= bases_[last_idx_ + 1] &&
    s < bases_[last_idx_ + 2]) {
    last_idx_++;
    return last_idx_;
  }
  auto it = std::upper_bound(bases_.begin(), bases_.end(), s);
  last_idx_ = std::max(0, static_cast<int32_t>(std::distance(bases_.begin(), it)) - 1);
  return last_idx_;
}
}  // namespace autoware::time_to_space_trajectory_converter
