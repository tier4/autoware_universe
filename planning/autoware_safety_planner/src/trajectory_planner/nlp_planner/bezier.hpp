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

#ifndef AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__BEZIER_HPP_
#define AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__BEZIER_HPP_

// The mathematics of the piecewise Bezier basis, depending on neither ROS nor the constraint IR.
//
// One piece is a Bezier of degree m = BEZIER_DEGREE, scaled by the length alpha of the piece:
//
//     f(t) = alpha * sum_i p_i * b_i^m(u),   u = (t - t_start) / alpha in [0, 1]
//     b_i^m(u) = C(m,i) u^i (1-u)^(m-i)      (Bernstein basis)
//
// Scaling by alpha follows the SSC paper: it gives the control points the dimension of position per
// time, so their magnitudes stay comparable as the length of a piece changes. Two properties are
// used:
//
//   (P1) convex hull   sum b_i = 1 and b_i >= 0, so f is a convex combination of the control
//                      points: putting them in a box puts the **whole** curve in that box. This is
//                      sufficient, not necessary
//   (P2) hodograph     the k-th derivative is a Bezier as well, whose control points q^(k) are a
//                      **linear map** of p:
//                        q^(0) = p,  q_i^(k) = (m-k+1) * (q_(i+1)^(k-1) - q_i^(k-1))
//                      with the end values d^k f/dt^k(t_start) = alpha^(1-k) q_0^(k) and
//                                          d^k f/dt^k(t_end)   = alpha^(1-k) q_(m-k)^(k)
//
// Applying (P1) at every order of (P2) turns "the whole derivative profile stays in a box" into
// linear inequalities.

#include <cstddef>
#include <vector>

namespace autoware::safety_planner
{

//! Degree of the Bezier, 5 as in the SSC paper: the lowest degree that leaves the jerk, the third
//! derivative, as a quadratic Bezier
inline constexpr int BEZIER_DEGREE = 5;
//! number of control points per piece
inline constexpr int BEZIER_CONTROL_POINTS = BEZIER_DEGREE + 1;

//! Bernstein basis b_i^m(u)
double bernstein(int m, int i, double u);

//! The linear map from p to the control points q^(k) of the k-th derivative, of size
//! (m - k + 1) x (m + 1): q^(k) = D_k p. The scale is left out; the caller multiplies by
//! alpha^(1-k).
std::vector<std::vector<double>> hodograph_matrix(int m, int k);

//! The 6x6 Hessian Q of the squared jerk integral over the normalized interval [0, 1],
//! int (y'''(u))^2 du. The contribution of a real piece is (1/alpha^3) p' Q p.
std::vector<std::vector<double>> jerk_hessian(int m);

//! Evaluates the k-th derivative of the control points p (of length m + 1) at u. The value is the
//! one on the normalized interval, without the scale; in real time it is alpha^(1-k) times that.
double evaluate_derivative(const std::vector<double> & p, int k, double u);

}  // namespace autoware::safety_planner

#endif  // AUTOWARE__SAFETY_PLANNER__TRAJECTORY_PLANNER__NLP_PLANNER__BEZIER_HPP_
