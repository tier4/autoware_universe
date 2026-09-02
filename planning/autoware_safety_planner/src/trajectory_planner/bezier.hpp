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

#ifndef TRAJECTORY_PLANNER__BEZIER_HPP_
#define TRAJECTORY_PLANNER__BEZIER_HPP_

// 区分 Bézier の基底まわりの純粋な数学。ROS にも制約 IR にも依存しない
// (docs/safety_planner_arch_design/formulation_ssc_vs_poc.md §2.2)。
//
// 1 区間は次数 m = BEZIER_DEGREE の Bézier で、区間長 α でスケールして書く:
//
//     f(t) = α · Σ_i p_i · b_i^m(u),        u = (t − t_start) / α ∈ [0, 1]
//     b_i^m(u) = C(m,i) u^i (1−u)^{m−i}     (Bernstein 基底)
//
// この α 倍は SSC 論文の流儀。制御点が「位置 / 時間」の次元になり、区間長が変わっても
// 係数のオーダーが揃う。使う性質は 2 つ:
//
//   (P1) 凸包性       Σ b_i = 1, b_i ≥ 0 なので f は制御点の凸結合 → 制御点を箱に入れれば
//                     曲線**全体**が箱に入る (十分条件。逆は成り立たない)
//   (P2) hodograph 性 k 階微分もまた Bézier で、その制御点 q^{(k)} は p の**線形写像**:
//                     q^{(0)} = p,  q_i^{(k)} = (m−k+1) · (q_{i+1}^{(k-1)} − q_i^{(k-1)})
//                     端点値は d^k f/dt^k(t_start) = α^{1-k} q_0^{(k)}、
//                                d^k f/dt^k(t_end)   = α^{1-k} q_{m-k}^{(k)}
//
// (P1) を (P2) の各階に適用すると、微分プロファイル全体を箱に閉じ込める線形不等式になる。

#include <cstddef>
#include <vector>

namespace autoware::safety_planner
{

//! Bézier の次数 (SSC 論文と同じ 5 次。jerk = 3 階微分が 2 次 Bézier として残る最小次数)
inline constexpr int BEZIER_DEGREE = 5;
//! 1 区間あたりの制御点数
inline constexpr int BEZIER_CONTROL_POINTS = BEZIER_DEGREE + 1;

//! Bernstein 基底 b_i^m(u)
double bernstein(int m, int i, double u);

//! k 階微分の制御点 q^{(k)} を p から作る線形写像 (行数 = m − k + 1、列数 = m + 1)。
//! q^{(k)} = D_k · p。scale は掛けない (呼び出し側が α^{1-k} を掛ける)
std::vector<std::vector<double>> hodograph_matrix(int m, int k);

//! 正規化区間 [0, 1] 上の jerk 二乗積分 ∫ (y'''(u))² du を与える Hessian Q (6×6)。
//! 実区間の寄与は (1/α³) · pᵀ Q p (formulation_ssc_vs_poc.md §2.3)
std::vector<std::vector<double>> jerk_hessian(int m);

//! 制御点列 p (長さ m+1) の k 階微分を u で評価する。戻り値は α のスケールを含まない
//! 「正規化区間上の値」で、実時間の値は α^{1-k} を掛けたもの
double evaluate_derivative(const std::vector<double> & p, int k, double u);

}  // namespace autoware::safety_planner

#endif  // TRAJECTORY_PLANNER__BEZIER_HPP_
