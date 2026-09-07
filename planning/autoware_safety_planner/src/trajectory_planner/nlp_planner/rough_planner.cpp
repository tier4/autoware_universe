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

#include "rough_planner.hpp"

#include "../../utils/sl_view_utils.hpp"

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace autoware::safety_planner
{

namespace
{

// =============================================================================================
// 定数
// =============================================================================================

// 数値パラメータ (格子・重み・出力グリッド) は RoughPlannerParams (rough_planner.hpp) へ移した。
// ここに残るのは数値誤差の許容値だけ
constexpr double CURVATURE_EPS = 1e-6;
constexpr double EPS = 1e-9;

// =============================================================================================
// DP 内部型
// =============================================================================================

//! (s_i, l_j) ごとの格子幾何 (時間層・速度に依らない)
struct DpNodeGeometry
{
  Pose2d pose{};          //!< 世界座標。yaw は中心線接線 (dl/ds 補正は遷移側で加味。S3 §3.1)
  double curvature{0.0};  //!< [1/m] 中心線曲率 κ_ref(s_i) (l 補正なし)
};

//! DP 格子の 1 ノード (t_k, s_i, l_j, v_m)。4D 化で速度が格子軸になったため、
//! 幾何は DpNodeGeometry へ分離し、ノードは有効性と探索状態だけを持つ
struct DpNode
{
  // --- 有効性 (build_dp_grid の棄却工程が書く。S3 §3.2) ---
  //! false = A 到達可能バンド外 (s・v とも) or B 棄却 or 速度上限超過。探索から除外
  bool valid{true};

  // --- 探索状態 (search_dp_candidates が書く。S3 §3.3–3.4) ---
  double cost{INF};  //!< cost-to-come。INF = 未到達
  int parent_s{-1};  //!< 後退追跡用: 最良親の s index (-1 = 親なし)
  int parent_l{-1};  //!< 後退追跡用: 最良親の l index
  int parent_v{-1};  //!< 後退追跡用: 最良親の v index
};

//! (s, l, t, v) 4 次元の DP 格子。周期ごとに使い捨て
//! (S1 §1 原則「IR は毎周期使い捨て」と同じ扱い)
struct DpGrid
{
  std::vector<double> s_values;  //!< [m] reference_path 弧長 (ego 位置起点、s 昇順)
  std::vector<double> l_values;  //!< [m] 横オフセット (左 = 正、l 昇順)
  std::vector<double> t_values;  //!< [s] 層時刻 (t = 0 = 計画基準時刻。層 0 = ego 実状態)
  std::vector<double> v_values;  //!< [m/s] 速度 (0 起点、v 昇順)

  std::vector<DpNodeGeometry> geometries;  //!< size = s × l ((s_i, l_j) ごと)
  std::vector<DpNode> nodes;               //!< size = t × s × l × v (t-major)

  //! s 格子ごとの目標速度 (コストの参照。S3 §3.4)。mark_invalid_nodes が埋める
  std::vector<double> v_target;

  //! 層 k の s_i が B (幾何・停止線) で全 l 塞がっているか。size = t × s (t-major)。
  //! 壁 (制動可能性条件) の判定はこれだけを見る。A 到達可能バンド外・速度上限超過は
  //! 「まだ届かない」だけで物理的な障害ではないので含めない
  std::vector<std::uint8_t> b_blocked;

  double s_ego{0.0};  //!< [m] reference_path 上の ego 弧長 (格子の原点)
  double l_ego{0.0};  //!< [m] ego の横オフセット実値 (格子に丸めない。S3 §3.1)

  std::size_t geometry_index(const int s_index, const int l_index) const
  {
    return static_cast<std::size_t>(s_index) * l_values.size() + l_index;
  }
  DpNodeGeometry & geometry(const int s_index, const int l_index)
  {
    return geometries[geometry_index(s_index, l_index)];
  }
  const DpNodeGeometry & geometry(const int s_index, const int l_index) const
  {
    return geometries[geometry_index(s_index, l_index)];
  }

  std::size_t node_index(
    const int t_index, const int s_index, const int l_index, const int v_index) const
  {
    return ((static_cast<std::size_t>(t_index) * s_values.size() + s_index) * l_values.size() +
            l_index) *
             v_values.size() +
           v_index;
  }
  DpNode & node(const int t_index, const int s_index, const int l_index, const int v_index)
  {
    return nodes[node_index(t_index, s_index, l_index, v_index)];
  }
  const DpNode & node(
    const int t_index, const int s_index, const int l_index, const int v_index) const
  {
    return nodes[node_index(t_index, s_index, l_index, v_index)];
  }
};

//! DP 後退追跡で得た粗経路の 1 点 (層ごと 1 点、Δt_dp 刻み)
struct DpPathPoint
{
  double t{0.0};  //!< [s]
  double s{0.0};  //!< [m]
  double l{0.0};  //!< [m]
  double v{0.0};  //!< [m/s] 最良親由来の到達速度
};

//! DP 候補経路。cost は cost-to-come 合計 (候補間の順位付けに使う)
struct DpPath
{
  std::vector<DpPathPoint> points;
  double cost{0.0};
};

// =============================================================================================
// DP 段階の関数 (SpatiotemporalDpPlanner 相当。S3 §3)
// =============================================================================================

//! [DP 1a] 格子の幾何を張る。reference_path の ego 前方 [0, params.dp.s_max_m] を (s, l) 格子に、
//! [0, params.dp.horizon_s] を時間層にして、各ノードの世界座標 pose と中心線曲率を引き当てる
DpGrid build_grid_geometry(
  const RoughPlannerParams & params, const PlannerContext & context, const KinematicLimits & limits)
{
  DpGrid grid;
  const auto & path = context.reference_path;

  // ego の弧長と横オフセット (l_ego は格子に丸めず実値で持つ。S3 §3.1)
  const EgoFrenetState ego = compute_ego_frenet_state(context);
  grid.s_ego = ego.s;
  grid.l_ego = ego.l;

  // s 軸: ego 弧長を起点に Δs 刻みで reference_path 終端まで (最大 params.dp.s_max_m)
  const double s_end = std::min(path.length(), grid.s_ego + params.dp.s_max_m);
  const int num_s =
    std::max(static_cast<int>(std::floor((s_end - grid.s_ego) / params.dp.s_step_m)), 0) + 1;
  for (int i = 0; i < num_s; ++i) {
    grid.s_values.push_back(grid.s_ego + i * params.dp.s_step_m);
  }

  // l 軸: [-params.dp.l_range_m, +params.dp.l_range_m] / Δl 刻み (左 = 正)
  const int num_l_half = static_cast<int>(std::round(params.dp.l_range_m / params.dp.l_step_m));
  for (int j = -num_l_half; j <= num_l_half; ++j) {
    grid.l_values.push_back(j * params.dp.l_step_m);
  }

  // t 軸: [0, params.dp.horizon_s] / Δt 刻み (層 0 = ego 実状態)
  const int num_t = static_cast<int>(std::round(params.dp.horizon_s / params.dp.t_step_s)) + 1;
  for (int k = 0; k < num_t; ++k) {
    grid.t_values.push_back(k * params.dp.t_step_s);
  }

  // v 軸: [0, v_hard] / Δv 刻み
  const int num_v = static_cast<int>(std::floor(limits.v_hard / params.dp.v_step_mps)) + 1;
  for (int m = 0; m < num_v; ++m) {
    grid.v_values.push_back(m * params.dp.v_step_mps);
  }

  // (s, l) ごとの格子幾何 (時間層・速度で共通)
  grid.geometries.resize(grid.s_values.size() * grid.l_values.size());
  for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
    const double s = grid.s_values[i];
    const auto ref_position = path.compute(s).point.pose.position;
    const double ref_yaw = path.azimuth(s);
    const double ref_curvature = path.curvature(s);
    const double normal_x = -std::sin(ref_yaw);  // 中心線の左法線 (l 正方向)
    const double normal_y = std::cos(ref_yaw);
    for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
      const double l = grid.l_values[j];
      DpNodeGeometry & geometry = grid.geometry(i, j);
      geometry.pose.position =
        Point2d{ref_position.x + normal_x * l, ref_position.y + normal_y * l};
      geometry.pose.yaw = ref_yaw;  // dl/ds 補正は遷移側で加味 (S3 §3.1)
      geometry.curvature = ref_curvature;
    }
  }

  grid.nodes.resize(
    grid.t_values.size() * grid.s_values.size() * grid.l_values.size() * grid.v_values.size());
  return grid;
}

//! A 到達可能バンドの下限: 最大減速 (a_hard_min) で減速し続けた場合の走行距離
double min_reachable_distance(const double v0, const double t, const KinematicLimits & limits)
{
  const double decel = std::abs(limits.a_hard_min);
  const double t_stop = v0 / decel;
  if (t >= t_stop) {
    return v0 * v0 / (2.0 * decel);
  }
  return v0 * t - 0.5 * decel * t * t;
}

//! A 到達可能バンドの上限: 最大加速 (a_hard_max、v_hard で飽和) の走行距離
double max_reachable_distance(const double v0, const double t, const KinematicLimits & limits)
{
  const double t_saturate = std::max((limits.v_hard - v0) / limits.a_hard_max, 0.0);
  if (t <= t_saturate) {
    return v0 * t + 0.5 * limits.a_hard_max * t * t;
  }
  const double distance_to_saturate =
    v0 * t_saturate + 0.5 * limits.a_hard_max * t_saturate * t_saturate;
  return distance_to_saturate + limits.v_hard * (t - t_saturate);
}

//! s 格子ごとの速度上限 (v_upper = ハード条件) と目標速度 (v_target = コストの参照)。
//! v_upper = min(v_hard, v_legal, v_curve)、v_target = min(v_nom, v_upper, C の速度目標)
//! (S3 §3.3–3.4)。goal に接続していれば終端への停止包絡 (a_nom) も両方に掛ける
struct SpeedLimits
{
  std::vector<double> v_upper;
  std::vector<double> v_target;
};

SpeedLimits compute_speed_limits(
  const DpGrid & grid, const CompiledConstraints & compiled_constraints,
  const PlannerContext & context, const KinematicLimits & kinematic_limits)
{
  const bool stop_at_path_end = context.is_reference_path_connected_to_goal_pose();
  const double s_path_end = context.reference_path.length();

  SpeedLimits limits;
  limits.v_upper.reserve(grid.s_values.size());
  limits.v_target.reserve(grid.s_values.size());
  for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
    const double s = grid.s_values[i];
    const double curvature = std::abs(grid.geometry(i, 0).curvature);
    const double v_curve =
      std::sqrt(kinematic_limits.a_lat_nom / std::max(curvature, CURVATURE_EPS));

    double v_upper = std::min(kinematic_limits.v_hard, v_curve);
    double v_target = std::min(kinematic_limits.v_nom, v_curve);
    for (const auto & bound : compiled_constraints.scalar_bounds) {
      if (bound.quantity != BoundedQuantity::VELOCITY || s < bound.s0 || s > bound.s1) {
        continue;
      }
      // Tier 削除により区間 bound は全て v_legal (B の速度上限) として扱う
      v_target = std::min(v_target, bound.max);
      v_upper = std::min(v_upper, bound.max);
    }
    if (stop_at_path_end) {
      // goal で v = 0 に至る快適減速の包絡
      const double v_stop =
        std::sqrt(2.0 * std::abs(kinematic_limits.a_nom_min) * std::max(s_path_end - s, 0.0));
      v_upper = std::min(v_upper, v_stop);
      v_target = std::min(v_target, v_stop);
    }
    limits.v_upper.push_back(v_upper);
    limits.v_target.push_back(std::min(v_target, v_upper));
  }
  return limits;
}

//! [DP 1b] 制約からノードの valid フラグを落とす (S3 §3.2)
//! - A 到達可能バンド: 最大減速〜最大加速の s(t_k) 包絡の外を除外
//! - B 幾何 (静的): lateral_bounds は層に依らないので (s, l) ごとに 1 回だけ評価
//! - B 幾何 (動的): occupancies を層時刻 ±半窓で評価
//! - B 停止線: stop_bars の時間窓と層時刻が交わるノードの前端越えを棄却
//! C は棄却にもコストにも使わない (S3 §1 原則 5)。射影ビューで通って raw で落ちるのは許す
//! (compiler の射影規約。精密評価は下流の compiler / verification が raw で行う)
DpGrid mark_invalid_nodes(
  const RoughPlannerParams & params, DpGrid grid, const CompiledConstraints & compiled_constraints,
  const PlannerContext & context, const KinematicLimits & limits)
{
  //! 層時刻 t_k の周りに動的制約を評価する半窓 (層間を保守側に覆う。S3 §3.2)
  const double half_window = 0.5 * params.dp.t_step_s;
  const double v0 = std::max(context.odometry.twist.twist.linear.x, 0.0);
  const double band_tolerance = 0.5 * params.dp.s_step_m;  // 格子丸め分の余裕

  // 速度上限は 4D 化でノードの属性になった: v_m > v_upper(s_i) のノードを無効にする。
  // v_target はコストの参照として grid に持たせて探索へ渡す
  const SpeedLimits speed_limits =
    compute_speed_limits(grid, compiled_constraints, context, limits);
  grid.v_target = speed_limits.v_target;

  // Tier 削除により幾何エントリは全て棄却に使う (落とせる制約の区別は制約セット側で行う)

  // 静的 B (境界) は層・速度に依らない → (s, l) ごとに 1 回だけ評価
  std::vector<std::uint8_t> static_valid(grid.s_values.size() * grid.l_values.size(), 1);
  for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
    for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
      const SlBox box = footprint_sl_box(context.vehicle_info, grid.s_values[i], grid.l_values[j]);
      for (const auto & bound : compiled_constraints.lateral_bounds) {
        if (violates_lateral_bound(bound, box)) {
          static_valid[i * grid.l_values.size() + j] = 0;
          break;
        }
      }
    }
  }

  // 層ごと: A 到達可能バンド (s・v とも) + 動的 B (占有・停止線) + 速度上限
  grid.b_blocked.assign(grid.t_values.size() * grid.s_values.size(), 0);
  for (std::size_t k = 0; k < grid.t_values.size(); ++k) {
    const double t = grid.t_values[k];
    const double s_band_min = grid.s_ego + min_reachable_distance(v0, t, limits) - band_tolerance;
    const double s_band_max = grid.s_ego + max_reachable_distance(v0, t, limits) + band_tolerance;
    // v の到達可能バンド: 初速 v0 から最大加減速で到達できる速度範囲
    const double v_band_min = v0 + limits.a_hard_min * t - 0.5 * params.dp.v_step_mps;
    const double v_band_max = v0 + limits.a_hard_max * t + 0.5 * params.dp.v_step_mps;
    for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
      const double s = grid.s_values[i];
      const bool in_band = s >= s_band_min && s <= s_band_max;
      bool all_l_b_blocked = true;
      for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
        // (s, l, t) レベルの棄却判定を 1 回だけ行い、全 v へ反映する。
        // B 起因の棄却だけは別に集計する (壁判定の材料。grid.b_blocked)
        bool b_valid = static_valid[i * grid.l_values.size() + j] != 0;
        if (b_valid) {
          const SlBox box = footprint_sl_box(context.vehicle_info, s, grid.l_values[j]);
          for (const auto & occupancy : compiled_constraints.occupancies) {
            if (violates_occupancy(
                  occupancy, compiled_constraints, box, t - half_window, t + half_window)) {
              b_valid = false;
              break;
            }
          }
          if (b_valid) {
            for (const auto & stop_bar : compiled_constraints.stop_bars) {
              if (violates_stop_bar(stop_bar, box, t - half_window, t + half_window)) {
                b_valid = false;
                break;
              }
            }
          }
        }
        all_l_b_blocked = all_l_b_blocked && !b_valid;
        const bool base_valid = in_band && b_valid;
        for (std::size_t m = 0; m < grid.v_values.size(); ++m) {
          const double v = grid.v_values[m];
          const bool v_valid =
            v <= speed_limits.v_upper[i] + EPS && v >= v_band_min - EPS && v <= v_band_max + EPS;
          grid.node(k, i, j, m).valid = base_valid && v_valid;
        }
      }
      grid.b_blocked[k * grid.s_values.size() + i] = all_l_b_blocked ? 1 : 0;
    }
  }
  return grid;
}

//! [DP 1] 探索可能な格子の作成 = 幾何 + 有効性まで確定した使い捨て格子を返す
DpGrid build_dp_grid(
  const RoughPlannerParams & params, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints, const KinematicLimits & limits)
{
  return mark_invalid_nodes(
    params, build_grid_geometry(params, context, limits), compiled_constraints, context, limits);
}

//! DP 探索の結果。candidates が空 = B 充足経路無し (→ 編成が停止 rough_plan へ落とす)。
//! rejected は不成立理由 (debug.rejected へ連結する)
struct DpSearchResult
{
  std::vector<DpPath> candidates;  //!< コスト昇順
  std::vector<std::string> rejected;
};

//! [DP 3] コスト計算と候補抽出。前向き cost-to-come 掃引 (S3 §3.3–3.4) の後、
//! 終端ノードから後退追跡して候補を返す (S3 §3.5)。
//! prev_decisions はヒステリシス障壁 (S3 §5) の搬入用: side 反転経路への
//! w_side_switch 加算 (コスト側)、STOP→GO 保守窓・FOLLOW→LEAD ギャップ (棄却側)
DpSearchResult search_dp_candidates(
  const RoughPlannerParams & params, DpGrid grid, const CompiledConstraints & compiled_constraints,
  const PlannerContext & context, const Decisions & prev_decisions,
  const KinematicLimits & kinematic_limits)
{
  // grid は値で受ける: 探索状態 (cost / parent) をノードに書き込みながら掃引し、
  // 使い捨てる (呼び出し側に書き戻さない)
  // TODO(odashima): prev_decisions によるヒステリシス障壁 (S3 §5。w_side_switch /
  // STOP→GO 保守窓 / FOLLOW→LEAD ギャップ) は derive_decisions() 実装後に搬入する
  (void)prev_decisions;
  (void)compiled_constraints;

  DpSearchResult result;
  const std::size_t num_t = grid.t_values.size();
  const std::size_t num_s = grid.s_values.size();
  const std::size_t num_l = grid.l_values.size();
  const std::size_t num_v = grid.v_values.size();
  if (num_t < 2 || num_s == 0 || num_l == 0 || num_v == 0) {
    result.rejected.push_back("dp: degenerate grid");
    return result;
  }

  const double dt = params.dp.t_step_s;
  const double decel = std::abs(kinematic_limits.a_hard_min);
  const double v0 = std::clamp(context.odometry.twist.twist.linear.x, 0.0, kinematic_limits.v_hard);

  // 遷移コスト (S3 §3.4)。v_target は mark 側が grid に埋めた値を読む
  const auto edge_cost = [&](
                           const double ds, const double l_prev, const double l_next,
                           const double v_next, const double accel, const std::size_t i_next) {
    const double lateral_rate = (l_next - l_prev) / dt;
    const double v_error = v_next - grid.v_target[i_next];
    const double accel_over = std::max(std::abs(accel) - kinematic_limits.a_nom_max, 0.0);
    return -params.dp.weights.progress * ds + params.dp.weights.lateral * l_next * l_next * dt +
           params.dp.weights.lateral_rate * lateral_rate * lateral_rate * dt +
           params.dp.weights.velocity * v_error * v_error * dt +
           params.dp.weights.accel * accel * accel * dt +
           params.dp.weights.accel_nominal * accel_over * accel_over * dt;
  };

  //! 探索の元になる状態。s_index < 0 は ego 実状態 (格子外) を表す
  struct SourceState
  {
    double s{0.0};
    double l{0.0};
    double v{0.0};
    double cost{0.0};
    int s_index{-1};
    int l_index{-1};
    int v_index{-1};
  };

  const auto to_s_index = [&](const double s_query) {
    return (s_query - grid.s_ego) / params.dp.s_step_m;
  };
  const auto to_l_index = [&](const double l_query) {
    return (l_query + params.dp.l_range_m) / params.dp.l_step_m;
  };
  const auto to_v_index = [&](const double v_query) { return v_query / params.dp.v_step_mps; };

  const auto relax = [&](
                       const std::size_t layer_next, const std::size_t i_next,
                       const std::size_t j_next, const std::size_t m_next, const double cost_next,
                       const SourceState & src) {
    DpNode & node = grid.node(layer_next, i_next, j_next, m_next);
    if (!node.valid || cost_next >= node.cost) {
      return;
    }
    node.cost = cost_next;
    node.parent_s = src.s_index;
    node.parent_l = src.l_index;
    node.parent_v = src.v_index;
  };

  // 経路終端が goal に接続しているときだけ、終端を壁として扱う (下の distance_to_wall 参照)
  const double goal_wall_s =
    context.is_reference_path_connected_to_goal_pose() ? context.reference_path.length() : INF;

  // 層 k → k+1 の前向き掃引
  for (std::size_t k = 0; k + 1 < num_t; ++k) {
    const std::size_t layer_next = k + 1;

    // wall までの制動可能性 (S3 §3.3 保守側条件): 層 k+1 で「B (幾何・停止線) が全 l を
    // 塞ぐ」s を壁とみなし、壁までの残距離を最大減速で使い切って止まれる速度以下の
    // 遷移だけを許す。
    // A 到達可能バンド外・速度上限超過のノードは壁に数えない (バンド前端は「その時刻には
    // まだ届かない」だけで障害ではなく、壁にすると層 1 の目前に幻の壁が立って速度が頭打ちになる)。
    // 格子前端も、s_max_m / 前方長で経路が切れているだけのときは壁にしない (その先が
    // 見えていないだけ)。goal に接続しているときだけ goal 位置を壁にする
    std::vector<double> distance_to_wall(num_s, INF);
    {
      double wall_s = goal_wall_s;
      for (int i = static_cast<int>(num_s) - 1; i >= 0; --i) {
        if (grid.b_blocked[layer_next * num_s + i] != 0) {
          wall_s = grid.s_values[i];
        }
        distance_to_wall[i] = std::isinf(wall_s) ? INF : std::max(wall_s - grid.s_values[i], 0.0);
      }
    }

    const auto expand = [&](const SourceState & src) {
      const bool from_ego = src.s_index < 0;
      // ego ノードの格子スナップ余裕 (S3 §3.3): 初層のみ横移動条件に +0.5·Δl を許す
      const double snap_slack = from_ego ? 0.5 * params.dp.l_step_m : 0.0;

      // 4D: v' を軸から列挙し (加速度 box 内)、Δs = (v + v')/2·Δt は等加速度仮定から
      // 従属して決まる。s' は最近傍格子へ丸め、コストは実際の格子間距離で評価する。
      // v' = 0 も通常列挙に含まれるため停止遷移の別枠 (3D 版) は不要
      const int m_lo = std::max(
        static_cast<int>(std::ceil(to_v_index(src.v + kinematic_limits.a_hard_min * dt) - EPS)), 0);
      const int m_hi = std::min(
        static_cast<int>(std::floor(to_v_index(src.v + kinematic_limits.a_hard_max * dt) + EPS)),
        static_cast<int>(num_v) - 1);
      for (int m_next = m_lo; m_next <= m_hi; ++m_next) {
        const double v_next = grid.v_values[m_next];
        const double ds_ideal = 0.5 * (src.v + v_next) * dt;
        const int i_next = static_cast<int>(std::round(to_s_index(src.s + ds_ideal)));
        if (i_next < 0 || i_next >= static_cast<int>(num_s)) {
          continue;
        }
        const double ds = grid.s_values[i_next] - src.s;
        if (ds < -EPS) {
          continue;
        }
        if (v_next * v_next > 2.0 * decel * distance_to_wall[i_next] + EPS) {
          continue;  // 壁の手前で止まれない速度で入らない
        }
        const double accel = (v_next - src.v) / dt;

        // 横移動条件。停止 (Δs = 0) 中は横移動しない
        const double dl_max =
          (ds < EPS)
            ? 0.0
            : std::min(params.dp.lateral_slope_max * ds, params.dp.lateral_rate_max_mps * dt) +
                snap_slack;
        const int j_lo = std::max(static_cast<int>(std::ceil(to_l_index(src.l - dl_max) - EPS)), 0);
        const int j_hi = std::min(
          static_cast<int>(std::floor(to_l_index(src.l + dl_max) + EPS)),
          static_cast<int>(num_l) - 1);
        for (int j_next = j_lo; j_next <= j_hi; ++j_next) {
          const double l_next = grid.l_values[j_next];
          const double cost = src.cost + edge_cost(ds, src.l, l_next, v_next, accel, i_next);
          relax(layer_next, i_next, j_next, m_next, cost, src);
        }
      }
    };

    if (k == 0) {
      // 層 0 は ego の実状態 1 点 (格子に丸めない。S3 §3.1)
      SourceState ego;
      ego.s = grid.s_ego;
      ego.l = grid.l_ego;
      ego.v = v0;
      ego.cost = 0.0;
      expand(ego);
    } else {
      for (std::size_t i = 0; i < num_s; ++i) {
        for (std::size_t j = 0; j < num_l; ++j) {
          for (std::size_t m = 0; m < num_v; ++m) {
            const DpNode & node = grid.node(k, i, j, m);
            if (!node.valid || std::isinf(node.cost)) {
              continue;
            }
            SourceState src;
            src.s = grid.s_values[i];
            src.l = grid.l_values[j];
            src.v = grid.v_values[m];
            src.cost = node.cost;
            src.s_index = static_cast<int>(i);
            src.l_index = static_cast<int>(j);
            src.v_index = static_cast<int>(m);
            expand(src);
          }
        }
      }
    }
  }

  // 終端 (最終層) は cost-to-come 最小のノードを採用 (進行報酬が edge に入っているため
  // 終端項は不要。S3 §3.4)
  const std::size_t last_layer = num_t - 1;
  int best_i = -1;
  int best_j = -1;
  int best_m = -1;
  double best_cost = INF;
  for (std::size_t i = 0; i < num_s; ++i) {
    for (std::size_t j = 0; j < num_l; ++j) {
      for (std::size_t m = 0; m < num_v; ++m) {
        const DpNode & node = grid.node(last_layer, i, j, m);
        if (node.valid && node.cost < best_cost) {
          best_cost = node.cost;
          best_i = static_cast<int>(i);
          best_j = static_cast<int>(j);
          best_m = static_cast<int>(m);
        }
      }
    }
  }
  if (best_i < 0) {
    result.rejected.push_back("dp: no feasible path to horizon");
    return result;
  }

  // 後退追跡 (S3 §3.5)。当面 top-1 のみ (top-K 抽出は将来拡張)
  DpPath path;
  path.cost = best_cost;
  std::vector<DpPathPoint> reversed_points;
  int i_trace = best_i;
  int j_trace = best_j;
  int m_trace = best_m;
  for (int k = static_cast<int>(last_layer); k >= 1; --k) {
    if (i_trace < 0 || j_trace < 0 || m_trace < 0) {
      result.rejected.push_back("dp: broken parent chain");
      return result;
    }
    const DpNode & node = grid.node(k, i_trace, j_trace, m_trace);
    reversed_points.push_back(
      {grid.t_values[k], grid.s_values[i_trace], grid.l_values[j_trace], grid.v_values[m_trace]});
    i_trace = node.parent_s;
    j_trace = node.parent_l;
    m_trace = node.parent_v;
  }
  path.points.push_back({0.0, grid.s_ego, grid.l_ego, v0});  // 層 0 = ego 実状態
  path.points.insert(path.points.end(), reversed_points.rbegin(), reversed_points.rend());
  result.candidates.push_back(std::move(path));
  return result;
}

//! l(s) の cubic Hermite 内挿 (S3 §3.7 手順 1。線形だと折れ点の κ がスパイクする)。
//! 節点は s 狭義単調 (停止区間は 1 点に潰す)、勾配は中心差分・端点 0
class LateralOffsetSpline
{
public:
  explicit LateralOffsetSpline(const std::vector<DpPathPoint> & dp_points)
  {
    for (const auto & point : dp_points) {
      if (knot_s_.empty() || point.s > knot_s_.back() + EPS) {
        knot_s_.push_back(point.s);
        knot_l_.push_back(point.l);
      }
    }
    knot_slope_.assign(knot_s_.size(), 0.0);  // 端点勾配 0
    for (std::size_t i = 1; i + 1 < knot_s_.size(); ++i) {
      knot_slope_[i] = (knot_l_[i + 1] - knot_l_[i - 1]) / (knot_s_[i + 1] - knot_s_[i - 1]);
    }
  }

  double evaluate(const double s) const
  {
    if (knot_s_.empty()) {
      return 0.0;
    }
    if (s <= knot_s_.front()) {
      return knot_l_.front();
    }
    if (s >= knot_s_.back()) {
      return knot_l_.back();
    }
    std::size_t seg = 0;
    while (seg + 2 < knot_s_.size() && s > knot_s_[seg + 1]) {
      ++seg;
    }
    const double h = knot_s_[seg + 1] - knot_s_[seg];
    const double u = (s - knot_s_[seg]) / h;
    const double u2 = u * u;
    const double u3 = u2 * u;
    return (2.0 * u3 - 3.0 * u2 + 1.0) * knot_l_[seg] + (u3 - 2.0 * u2 + u) * h * knot_slope_[seg] +
           (-2.0 * u3 + 3.0 * u2) * knot_l_[seg + 1] + (u3 - u2) * h * knot_slope_[seg + 1];
  }

private:
  std::vector<double> knot_s_;
  std::vector<double> knot_l_;
  std::vector<double> knot_slope_;
};

//! (s, l) を世界座標の RoughPlanPoint に引き当てる (yaw は中心線接線で代表)
RoughPlanPoint to_rough_plan_point(
  const PathPointTrajectory & path, const double t, const double s, const double l, const double v,
  const double a)
{
  const auto ref_position = path.compute(s).point.pose.position;
  const double ref_yaw = path.azimuth(s);
  RoughPlanPoint point;
  point.t = t;
  point.pose.position =
    Point2d{ref_position.x - std::sin(ref_yaw) * l, ref_position.y + std::cos(ref_yaw) * l};
  point.pose.yaw = ref_yaw;
  point.kappa = path.curvature(s);
  point.v = v;
  point.a = a;
  return point;
}

//! [DP 4] DP 粗経路 (Δt_dp 刻み) を NLP ステージ (params.time_step_s × params.num_points)
//! へ持ち上げる (S3 §3.7)。手順 5.5 (躍度制限) は未実装 (TODO(odashima): NLP 接続時に追加)
RoughPlan lift_to_stage_grid(
  const RoughPlannerParams & params, const DpPath & dp_path,
  const CompiledConstraints & compiled_constraints, const PlannerContext & context,
  const KinematicLimits & kinematic_limits)
{
  RoughPlan plan;
  plan.source = RoughPlanSource::SPATIOTEMPORAL_DP;
  if (dp_path.points.size() < 2) {
    return plan;
  }
  const auto & path = context.reference_path;
  const double s_path_end = path.length();

  // --- 手順 1–2: 区間等加速度で v(t)・s(t) を params.time_step_s 刻みに展開 ---
  std::vector<double> v_fine(params.num_points);
  std::vector<double> s_fine(params.num_points);
  for (int k = 0; k < params.num_points; ++k) {
    const double t = k * params.time_step_s;
    const std::size_t seg =
      std::min(static_cast<std::size_t>(t / params.dp.t_step_s), dp_path.points.size() - 2);
    const auto & p0 = dp_path.points[seg];
    const auto & p1 = dp_path.points[seg + 1];
    const double tau = t - p0.t;
    const double accel = (p1.v - p0.v) / params.dp.t_step_s;
    v_fine[k] = std::max(p0.v + accel * tau, 0.0);
    s_fine[k] = std::min(p0.s + p0.v * tau + 0.5 * accel * tau * tau, s_path_end);
  }

  // --- 手順 3: 速度キャップ (0.1 s 解像度で曲率・速度制約・goal 停止包絡を再適用。
  //     DP の 2 m 格子が拾えない鋭い曲率を潰す) ---
  const bool stop_at_path_end = context.is_reference_path_connected_to_goal_pose();
  const auto v_cap_at = [&](const double s) {
    const double curvature = std::abs(path.curvature(s));
    double cap = std::min(
      kinematic_limits.v_nom,
      std::sqrt(kinematic_limits.a_lat_nom / std::max(curvature, CURVATURE_EPS)));
    for (const auto & bound : compiled_constraints.scalar_bounds) {
      if (bound.quantity == BoundedQuantity::VELOCITY && s >= bound.s0 && s <= bound.s1) {
        cap = std::min(cap, bound.max);
      }
    }
    if (stop_at_path_end) {
      cap = std::min(
        cap, std::sqrt(2.0 * std::abs(kinematic_limits.a_nom_min) * std::max(s_path_end - s, 0.0)));
    }
    return cap;
  };
  for (int k = 0; k < params.num_points; ++k) {
    v_fine[k] = std::min(v_fine[k], v_cap_at(s_fine[k]));
  }

  // --- 手順 4–5: forward / backward pass (a_nom = ±1.0)。
  //     初期は ego 実速度に接続する (jerk 制限接続は手順 5.5 と併せて TODO) ---
  v_fine[0] = std::clamp(context.odometry.twist.twist.linear.x, 0.0, v_fine[0]);
  for (int k = 0; k + 1 < params.num_points; ++k) {
    v_fine[k + 1] =
      std::min(v_fine[k + 1], v_fine[k] + kinematic_limits.a_nom_max * params.time_step_s);
  }
  for (int k = params.num_points - 2; k >= 0; --k) {
    v_fine[k] = std::min(
      v_fine[k], v_fine[k + 1] + std::abs(kinematic_limits.a_nom_min) * params.time_step_s);
  }

  // --- 手順 6: 平滑後の v で s(t) を積分し直す (1 回のみ) ---
  s_fine[0] = dp_path.points.front().s;
  for (int k = 0; k + 1 < params.num_points; ++k) {
    s_fine[k + 1] =
      std::min(s_fine[k] + 0.5 * (v_fine[k] + v_fine[k + 1]) * params.time_step_s, s_path_end);
    if (s_fine[k + 1] >= s_path_end - EPS && stop_at_path_end) {
      v_fine[k + 1] = 0.0;
    }
  }

  // --- 手順 7: l(s) 内挿と κ・a 付与 ---
  const LateralOffsetSpline lateral_spline(dp_path.points);
  plan.points.reserve(params.num_points);
  plan.s.reserve(params.num_points);
  for (int k = 0; k < params.num_points; ++k) {
    const double a =
      (k + 1 < params.num_points) ? (v_fine[k + 1] - v_fine[k]) / params.time_step_s : 0.0;
    plan.points.push_back(to_rough_plan_point(
      path, k * params.time_step_s, s_fine[k], lateral_spline.evaluate(s_fine[k]), v_fine[k], a));
    plan.s.push_back(s_fine[k]);
  }
  return plan;
}

// =============================================================================================
// 編成の他候補 (S3 §2.3)
// =============================================================================================

//! 前周期解再利用の試行結果。plan = nullopt なら不成立 (理由は rejected)
struct PreviousSolutionResult
{
  std::optional<RoughPlan> plan;
  std::vector<std::string> rejected;
};

//! [編成 1] 前周期解の再利用 (S3 §4)。
//! 成立時は §3.7 の速度平滑を通した RoughPlan を返す
//! ([[maybe_unused]] は DP 動作確認のための一時無効化中のみ。有効化時に外す)
[[maybe_unused]] PreviousSolutionResult try_previous_solution(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const PreviousPlanningResult & prev_planning_result)
{
  // TODO(odashima): S3 §4.2 の 6 検査 (品質 / 初期状態乖離 / B 幾何 / B 停止線 / B 速度 /
  // 新規物体の決定不能)
  (void)context;
  (void)compiled_constraints;
  (void)prev_planning_result;
  return {};
}

//! [編成 3] 停止 rough_plan (S3 §6)。無条件成立の最終手段。
//! ego の現横オフセットを保持して reference_path に平行に、快適減速で止まる。
//! TODO(odashima): S3 §6 の減速度段階選択 (障害の手前で止まれる最小の減速度)・
//! jerk 制限接続・blocked 判定
RoughPlan make_stop_plan(
  const RoughPlannerParams & params, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints, const KinematicLimits & kinematic_limits)
{
  (void)compiled_constraints;

  RoughPlan plan;
  plan.source = RoughPlanSource::STOP;
  const auto & path = context.reference_path;
  const double s_path_end = path.length();
  const EgoFrenetState ego = compute_ego_frenet_state(context);
  const double v0 = std::clamp(context.odometry.twist.twist.linear.x, 0.0, kinematic_limits.v_hard);

  plan.points.reserve(params.num_points);
  plan.s.reserve(params.num_points);
  double s = ego.s;
  for (int k = 0; k < params.num_points; ++k) {
    const double t = k * params.time_step_s;
    const double v = std::max(v0 + kinematic_limits.a_nom_min * t, 0.0);
    const double v_next = std::max(v0 + kinematic_limits.a_nom_min * (t + params.time_step_s), 0.0);
    const double a = (v > 0.0) ? kinematic_limits.a_nom_min : 0.0;
    plan.points.push_back(to_rough_plan_point(path, t, s, ego.l, v, a));
    plan.s.push_back(s);
    s = std::min(s + 0.5 * (v + v_next) * params.time_step_s, s_path_end);
  }
  return plan;
}

// =============================================================================================
// デバッグマーカー (可視化のみ。意味論には関与しない)
// =============================================================================================

//! DP 格子の可視化。格子点を世界座標 (ego 前方 = s、横 = l) に置き、高さで時間層を表す
//! (1 s = 1 m)。valid = 緑 / invalid = 赤の小球
MarkerArray make_grid_markers(const DpGrid & grid, const double z_base)
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  const auto stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  auto valid_marker = create_default_marker(
    "map", stamp, "grid_valid", 0, Marker::SPHERE_LIST, create_marker_scale(0.1, 0.1, 0.1),
    create_marker_color(0.0, 1.0, 0.0, 0.8));
  auto invalid_marker = create_default_marker(
    "map", stamp, "grid_invalid", 0, Marker::SPHERE_LIST, create_marker_scale(0.1, 0.1, 0.1),
    create_marker_color(1.0, 0.0, 0.0, 0.8));

  for (std::size_t k = 0; k < grid.t_values.size(); ++k) {
    for (std::size_t i = 0; i < grid.s_values.size(); ++i) {
      for (std::size_t j = 0; j < grid.l_values.size(); ++j) {
        // 4D 格子の v 軸は表示上潰す: ひとつでも valid な v があれば緑
        bool any_v_valid = false;
        for (std::size_t m = 0; m < grid.v_values.size(); ++m) {
          if (grid.node(k, i, j, m).valid) {
            any_v_valid = true;
            break;
          }
        }
        geometry_msgs::msg::Point point;
        point.x = grid.geometry(i, j).pose.position.x();
        point.y = grid.geometry(i, j).pose.position.y();
        point.z = z_base + grid.t_values[k];  // 1 s = 1 m
        (any_v_valid ? valid_marker : invalid_marker).points.push_back(point);
      }
    }
  }

  MarkerArray marker_array;
  if (!valid_marker.points.empty()) {
    marker_array.markers.push_back(valid_marker);
  }
  if (!invalid_marker.points.empty()) {
    marker_array.markers.push_back(invalid_marker);
  }
  return marker_array;
}

//! 出力候補経路の可視化。候補ごとにオレンジの LINE_STRIP を張り、高さは格子と同じ
//! 時間スケール (1 s = 1 m)。ns = "candidate_<優先順位>" で何番目の候補かを区別する
MarkerArray make_candidate_markers(const std::vector<RoughPlan> & plans, const double z_base)
{
  using autoware_utils_visualization::create_default_marker;
  using autoware_utils_visualization::create_marker_color;
  using autoware_utils_visualization::create_marker_scale;

  MarkerArray marker_array;
  const auto stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  for (std::size_t index = 0; index < plans.size(); ++index) {
    auto marker = create_default_marker(
      "map", stamp, "candidate_" + std::to_string(index), 0, Marker::LINE_STRIP,
      create_marker_scale(0.1, 0.0, 0.0), create_marker_color(1.0, 0.65, 0.0, 0.9));
    for (const auto & plan_point : plans[index].points) {
      geometry_msgs::msg::Point point;
      point.x = plan_point.pose.position.x();
      point.y = plan_point.pose.position.y();
      point.z = z_base + plan_point.t;  // 1 s = 1 m
      marker.points.push_back(point);
    }
    if (marker.points.size() >= 2) {
      marker_array.markers.push_back(marker);
    }
  }
  return marker_array;
}

//! 2 つの MarkerArray を連結して返す
MarkerArray merge_marker_arrays(MarkerArray first, const MarkerArray & second)
{
  first.markers.insert(first.markers.end(), second.markers.begin(), second.markers.end());
  return first;
}

// =============================================================================================
// 結果組み立て
// =============================================================================================

//! [組み立て] decisions を軌道の幾何から導出して plan を完成させる (S3 §2.4)
RoughPlan finalize_plan(
  RoughPlan plan, const CompiledConstraints & compiled_constraints,
  const Decisions & prev_decisions)
{
  plan.decisions = derive_decisions(plan, compiled_constraints, prev_decisions);
  return plan;
}

//! 編成 (S3 §2.3): 前周期解 → 時空間 DP → 停止 rough_plan を固定順に試し、最初に成立した段の
//! 候補列を返す。停止 rough_plan が無条件成立するため plans は必ず 1 本以上
RoughPlanResult make_plan_candidates(
  const RoughPlannerParams & params, const PlannerContext & context,
  const CompiledConstraints & compiled_constraints,
  const PreviousPlanningResult & prev_planning_result)
{
  RoughPlanResult result;
  const Decisions prev_decisions =
    prev_planning_result.plan ? prev_planning_result.plan->decisions : Decisions{};

  // [1] 前周期解の再利用 (ヒステリシス第一層: 通る限り決定は変わらない。S3 §5)
  // memo: DPの動作確認目的のためしばらくは無効化する
  // const auto previous = try_previous_solution(context, compiled_constraints,
  //                                             prev_planning_result);
  // result.debug.rejected = previous.rejected;
  // if (previous.plan) {
  //   result.plans.push_back(finalize_plan(*previous.plan, compiled_constraints, prev_decisions));
  //   return result;
  // }

  // 車両運動限界を IR (vehicle_kinematics プラグインの全域 ScalarBound) から集約する
  const KinematicLimits kinematic_limits = collect_kinematic_limits(compiled_constraints);

  // [2] 時空間 DP (格子作成 [幾何 + 有効性] → 探索・候補抽出 → ステージ持ち上げ)
  DpGrid grid = build_dp_grid(params, context, compiled_constraints, kinematic_limits);
  result.debug.debug_markers = make_grid_markers(grid, context.odometry.pose.pose.position.z);
  const DpSearchResult dp_result = search_dp_candidates(
    params, std::move(grid), compiled_constraints, context, prev_decisions, kinematic_limits);
  result.debug.rejected.insert(
    result.debug.rejected.end(), dp_result.rejected.begin(), dp_result.rejected.end());
  for (const DpPath & candidate : dp_result.candidates) {
    result.plans.push_back(finalize_plan(
      lift_to_stage_grid(params, candidate, compiled_constraints, context, kinematic_limits),
      compiled_constraints, prev_decisions));
  }
  if (!result.plans.empty()) {
    return result;
  }

  // [3] 停止 rough_plan (無条件成立)
  result.plans.push_back(finalize_plan(
    make_stop_plan(params, context, compiled_constraints, kinematic_limits), compiled_constraints,
    prev_decisions));
  return result;
}

}  // namespace

RoughPlanner::RoughPlanner(const RoughPlannerParams & params) : params_(params)
{
}

RoughPlanResult RoughPlanner::plan_rough_trajectories(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const PreviousPlanningResult & prev_planning_result) const
{
  const auto start_time = std::chrono::steady_clock::now();

  RoughPlanResult result =
    make_plan_candidates(params_, context, compiled_constraints, prev_planning_result);
  result.debug.debug_markers = merge_marker_arrays(
    std::move(result.debug.debug_markers),
    make_candidate_markers(result.plans, context.odometry.pose.pose.position.z));

  result.debug.elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
      .count();
  return result;
}

Decisions derive_decisions(
  const RoughPlan & plan, const CompiledConstraints & compiled_constraints,
  const Decisions & prev_decisions)
{
  // TODO(odashima): 軌道の幾何からの決定導出 (S3 §2.4)
  // - side: 占有の s 区間と軌道の s が重なる静的物体の、最接近点での左右
  // - lead_lag: コンフリクト s 区間を ego が先に抜けるか後か
  // - stop_go: 時間窓内に s_stop を越えるか
  (void)plan;
  (void)compiled_constraints;
  (void)prev_decisions;
  return {};
}

}  // namespace autoware::safety_planner
