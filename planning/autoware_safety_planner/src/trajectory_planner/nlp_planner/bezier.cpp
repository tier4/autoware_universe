// Copyright 2026 TIER IV, Inc.
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

#include "bezier.hpp"

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

//! Binomial coefficient C(n, k), built as a plain product since n stays around 10
double binomial(const int n, const int k)
{
  if (k < 0 || k > n) {
    return 0.0;
  }
  double value = 1.0;
  for (int i = 0; i < k; ++i) {
    value = value * static_cast<double>(n - i) / static_cast<double>(i + 1);
  }
  return value;
}

}  // namespace

double bernstein(const int m, const int i, const double u)
{
  if (i < 0 || i > m) {
    return 0.0;
  }
  return binomial(m, i) * std::pow(u, i) * std::pow(1.0 - u, m - i);
}

std::vector<std::vector<double>> hodograph_matrix(const int m, const int k)
{
  // D_0 = I, then D_k = Delta_k D_(k-1), Delta_k being the forward difference times (m - k + 1)
  std::vector<std::vector<double>> d(
    static_cast<std::size_t>(m + 1), std::vector<double>(static_cast<std::size_t>(m + 1), 0.0));
  for (int i = 0; i <= m; ++i) {
    d[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] = 1.0;
  }

  for (int step = 1; step <= k; ++step) {
    const int rows = m - step + 1;  // size of q^(step)
    const double factor = static_cast<double>(m - step + 1);
    std::vector<std::vector<double>> next(
      static_cast<std::size_t>(rows), std::vector<double>(static_cast<std::size_t>(m + 1), 0.0));
    for (int i = 0; i < rows; ++i) {
      for (int c = 0; c <= m; ++c) {
        next[static_cast<std::size_t>(i)][static_cast<std::size_t>(c)] =
          factor * (d[static_cast<std::size_t>(i + 1)][static_cast<std::size_t>(c)] -
                    d[static_cast<std::size_t>(i)][static_cast<std::size_t>(c)]);
      }
    }
    d = std::move(next);
  }
  return d;
}

std::vector<std::vector<double>> jerk_hessian(const int m)
{
  // With y'''(u) = sum_i q_i^(3) b_i^(m-3)(u),
  //   int (y''')^2 du = q' M q,   M_ij = int b_i^(m-3) b_j^(m-3) du
  // and the integral of a product of Bernstein polynomials is closed form:
  //   int_0^1 b_i^a b_j^c du = C(a,i) C(c,j) / ((a+c+1) C(a+c, i+j))
  const int k = 3;
  const int a = m - k;
  const auto d = hodograph_matrix(m, k);

  std::vector<std::vector<double>> mass(
    static_cast<std::size_t>(a + 1), std::vector<double>(static_cast<std::size_t>(a + 1), 0.0));
  for (int i = 0; i <= a; ++i) {
    for (int j = 0; j <= a; ++j) {
      mass[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
        binomial(a, i) * binomial(a, j) / (static_cast<double>(2 * a + 1) * binomial(2 * a, i + j));
    }
  }

  // Q = Dᵀ M D
  std::vector<std::vector<double>> q(
    static_cast<std::size_t>(m + 1), std::vector<double>(static_cast<std::size_t>(m + 1), 0.0));
  for (int r = 0; r <= m; ++r) {
    for (int c = 0; c <= m; ++c) {
      double sum = 0.0;
      for (int i = 0; i <= a; ++i) {
        for (int j = 0; j <= a; ++j) {
          sum += d[static_cast<std::size_t>(i)][static_cast<std::size_t>(r)] *
                 mass[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                 d[static_cast<std::size_t>(j)][static_cast<std::size_t>(c)];
        }
      }
      q[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = sum;
    }
  }
  return q;
}

double evaluate_derivative(const std::vector<double> & p, const int k, const double u)
{
  if (p.empty()) {
    return 0.0;
  }
  const int m = static_cast<int>(p.size()) - 1;
  if (k > m) {
    return 0.0;
  }
  const auto d = hodograph_matrix(m, k);
  double value = 0.0;
  for (int i = 0; i + k <= m; ++i) {
    double control = 0.0;
    for (int c = 0; c <= m; ++c) {
      control += d[static_cast<std::size_t>(i)][static_cast<std::size_t>(c)] *
                 p[static_cast<std::size_t>(c)];
    }
    value += control * bernstein(m - k, i, u);
  }
  return value;
}

}  // namespace autoware::safety_planner
