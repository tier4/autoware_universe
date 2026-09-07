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

#include "nlp_trajectory_optimizer.hpp"

#include "../../utils/sl_view_utils.hpp"

#include <autoware/trajectory/utils/closest.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// 生成コードのマクロが acados / システム側の名前と衝突する
#ifdef MAX_ITER
#undef MAX_ITER
#endif

extern "C" {
#include "c_generated_code_nlp_time/acados_solver_rbp_nlp_time.h"
}

namespace autoware::safety_planner
{

// solve_stop_landing が使う (定義は下)
OptimizedTrajectoryPoint integrate_rk4(
  const OptimizedTrajectoryPoint & state, double w, double j, double dt);

namespace
{

// ---------------------------------------------------------------------------------------------
// 行レイアウト。generators/time_bicycle_model.py と**同じ順序**でなければならない
// (生成物と C++ の唯一の暗黙契約なので、下の static_assert で行数だけは機械的に守る)
// ---------------------------------------------------------------------------------------------
constexpr int ROW_STEER_RATE_HARD = 0;
constexpr int ROW_STEER_RATE_NOMINAL = 1;
constexpr int ROW_LATERAL_ACCEL = 2;
constexpr int ROW_ACCEL_COMFORT = 3;
constexpr int ROW_JERK_COMFORT = 4;
constexpr int ROW_VELOCITY_SECTION = 5;
constexpr int ROW_VELOCITY_NOMINAL = 6;
constexpr int NUM_CORRIDOR_ROWS = 2 * static_cast<int>(NUM_PLANES);
constexpr int FIRST_CORRIDOR_ROW_SAFETY = 7;

constexpr int ROW_E_LATERAL_ACCEL = 0;
constexpr int ROW_E_ACCEL_COMFORT = 1;
constexpr int ROW_E_VELOCITY_SECTION = 2;
constexpr int ROW_E_VELOCITY_NOMINAL = 3;
constexpr int FIRST_CORRIDOR_ROW_E_SAFETY = 4;

static_assert(
  FIRST_CORRIDOR_ROW_SAFETY + NUM_CORRIDOR_ROWS == RBP_NLP_TIME_NH, "h row layout mismatch");
static_assert(
  FIRST_CORRIDOR_ROW_E_SAFETY + NUM_CORRIDOR_ROWS == RBP_NLP_TIME_NHN,
  "terminal h row layout mismatch");
// 行 0 (ステアレートの hard) 以外がスラック付き (S5 §3.4: A はスラックなし)
static_assert(RBP_NLP_TIME_NSH == RBP_NLP_TIME_NH - 1, "every row but the hard one is slacked");
static_assert(RBP_NLP_TIME_NSHN == RBP_NLP_TIME_NHN, "every terminal row is slacked");
static_assert(RBP_NLP_TIME_NP == 5 + 3 * static_cast<int>(NUM_PLANES), "parameter layout mismatch");

//! 無効行の開き幅。生成側 (nlp_time_ocp.py の FREE_BOUND) と同値
constexpr double FREE_BOUND = 1.0e6;
constexpr double EPS = 1e-9;
//! 力学残差の許容値 ε_dyn (S5 §9.1)
constexpr double DYNAMICS_RESIDUAL_TOL = 1e-3;
//! A tier の相対許容 (S5 §9.2)
constexpr double VEHICLE_RELATIVE_TOL = 1e-3;
//! C tier スカラー行の相対許容 (S5 §9.2)
constexpr double COMFORT_RELATIVE_TOL = 0.05;
//! B 幾何の許容侵入 [m] (S5 §9.2)。sl_view の判定は bool なので、この分だけ box を縮めて測る
constexpr double SAFETY_GEOMETRY_TOL = 0.02;
//! B 区間速度の許容超過 [m/s] (S5 §9.2)
constexpr double SAFETY_VELOCITY_TOL = 0.1;
//! 停止と見なす速度 [m/s] (S5 §7.2 の ε_v)
constexpr double STOP_VELOCITY = 1e-3;

//! 弧長 s に効く区間速度上限。全域行 (v_hard / v_nom) は box 側で課すのでここでは読まない。
//! Tier 削除により区間速度は全て SAFETY 行として扱う (COMFORT 側の区間速度は発生しない)
double section_velocity_upper(const CompiledConstraints & compiled_constraints, const double s)
{
  double upper = INF;
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.quantity != BoundedQuantity::VELOCITY) {
      continue;
    }
    if (bound.s0 == -INF && bound.s1 == INF) {
      continue;
    }
    if (s < bound.s0 || s > bound.s1) {
      continue;
    }
    upper = std::min(upper, bound.max);
  }
  return upper;
}

//! 追従参照の終点を goal そのものへ差し替える開始ステージ (S5 §4.3)。
//! rough_plan の終端は DP の s 格子 (Δs = 2 m) で量子化されていて周期ごとに跳ねるので、
//! そのまま終端参照に使うとゴール手前で参照がチャタリングして止まりきらない。
//! goal が捕捉距離まで近づいた周期は、**末尾の停止区間**と **goal を越えたステージ**の
//! 参照を goal に差し替える (差し替えるのは yref だけ。コリドーの seed は rough_plan のまま)。
std::optional<std::size_t> goal_snap_index(
  const RoughPlan & plan, const double s_goal, const double capture_distance)
{
  if (capture_distance <= 0.0 || plan.s.size() != plan.points.size() || plan.points.empty()) {
    return std::nullopt;
  }
  // goal がまだホライゾンの外 (rough_plan の終端より捕捉距離以上先) なら差し替えない
  if (s_goal - plan.s.back() > capture_distance) {
    return std::nullopt;
  }
  // 末尾の停止区間の先頭 (rough_plan が既に止まっているステージ列はすべて goal へ寄せる)
  std::size_t index = plan.points.size();
  while (index > 0 && plan.points[index - 1].v <= STOP_VELOCITY) {
    --index;
  }
  // goal を越えたステージも差し替える (越えた先を追うとゴールを通り過ぎる)
  for (std::size_t k = 0; k < plan.s.size(); ++k) {
    if (plan.s[k] >= s_goal - EPS) {
      index = std::min(index, k);
      break;
    }
  }
  if (index >= plan.points.size()) {
    // 停止区間も goal 越えも無い = まだ走り抜けている。終端だけを goal に合わせる
    index = plan.points.size() - 1;
  }
  // stage 0 は pin 済みなので差し替えても意味がない。1 以上に丸める
  return std::max<std::size_t>(index, 1);
}

//! 参照姿勢の連続化。LINEAR_LS は θ をそのまま二乗で測るので、参照が ±π で折り返すと
//! 1 ステージだけ 2π ずれた目標を追ってしまう。初期姿勢から連続になるよう積み上げる
std::vector<double> unwrap_yaw(const std::vector<RoughPlanPoint> & points, const double yaw0)
{
  std::vector<double> yaw;
  yaw.reserve(points.size());
  double previous = yaw0;
  for (const auto & point : points) {
    previous += autoware_utils_math::normalize_radian(point.pose.yaw - previous);
    yaw.push_back(previous);
  }
  return yaw;
}

std::array<double, 6> to_array(const OptimizedTrajectoryPoint & point)
{
  return {point.pose.position.x(),
          point.pose.position.y(),
          point.pose.yaw,
          point.kappa,
          point.v,
          point.a};
}

OptimizedTrajectoryPoint from_array(const std::array<double, 6> & x, const double t)
{
  OptimizedTrajectoryPoint point;
  point.t = t;
  point.pose.position = Point2d{x[0], x[1]};
  point.pose.yaw = x[2];
  point.kappa = x[3];
  point.v = x[4];
  point.a = x[5];
  return point;
}

double steer_rate_of(const double w, const double kappa, const double wheel_base)
{
  const double scale = 1.0 + (kappa * wheel_base) * (kappa * wheel_base);
  return wheel_base * w / scale;
}

//! 停止着地の躍度列 (S5 §7.5 #1)。**状態を事後に書き換えず入力側で厳密に着地させる**
//! (書き換えると (x_k, u_k) が F を満たさなくなり、力学残差 0 の保証が崩れる)。
//! (v, a) 部分系は線形なので RK4 は厳密で、一定躍度 j を n ステップ入れると
//!   a_n = a + n j dt,   v_n = v + n a dt + j dt² n²/2
//! さらに最終 1 ステップで a を 0 に落とす (j2 = −a_n/dt) と v は a_n dt/2 だけ動くので、
//!   v + n a dt + j dt² n²/2 + (a + n j dt) dt/2 = 0
//! を j について解けばよい。入力 box と v ≥ 0 を満たす最小の n を採る
std::optional<std::vector<double>> solve_stop_landing(
  const OptimizedTrajectoryPoint & state, const double dt, const double j_hard,
  const std::size_t max_steps)
{
  for (std::size_t n = 1; n + 1 <= max_steps; ++n) {
    const double count = static_cast<double>(n);
    const double denominator = dt * dt * count * (count + 1.0) / 2.0;
    const double jerk = -(state.v + state.a * dt * (count + 0.5)) / denominator;
    const double terminal_jerk = -(state.a + count * jerk * dt) / dt;
    if (std::abs(jerk) > j_hard || std::abs(terminal_jerk) > j_hard) {
      continue;
    }
    std::vector<double> jerks(n, jerk);
    jerks.push_back(terminal_jerk);
    // v ≥ 0 は閉形式では保証されないので前進シミュレーションで確かめる
    auto probe = state;
    bool feasible = true;
    for (const double value : jerks) {
      probe = integrate_rk4(probe, 0.0, value, dt);
      if (probe.v < -STOP_VELOCITY) {
        feasible = false;
        break;
      }
    }
    if (feasible) {
      return jerks;
    }
  }
  return std::nullopt;
}

}  // namespace

// =============================================================================================
// 生成された acados ソルバーのラッパ
// =============================================================================================

class NlpTrajectoryOptimizer::Solver
{
public:
  static constexpr int N = RBP_NLP_TIME_N;
  static constexpr int NX = RBP_NLP_TIME_NX;
  static constexpr int NU = RBP_NLP_TIME_NU;
  static constexpr int NY = RBP_NLP_TIME_NY;
  static constexpr int NYN = RBP_NLP_TIME_NYN;
  static constexpr int NH = RBP_NLP_TIME_NH;
  static constexpr int NHN = RBP_NLP_TIME_NHN;
  static constexpr int NP = RBP_NLP_TIME_NP;
  static constexpr int NSH = RBP_NLP_TIME_NSH;
  static constexpr int NSHN = RBP_NLP_TIME_NSHN;
  static constexpr int NBX = RBP_NLP_TIME_NBX;

  struct Result
  {
    std::vector<std::array<double, NX>> x;  //!< N + 1 点
    std::vector<std::array<double, NU>> u;  //!< N 点
    int status{-1};                         //!< acados のステータス (0 = 収束、2 = 反復上限)
    int sqp_iterations{0};
    double solve_time_ms{0.0};
  };

  Solver()
  {
    capsule_ = rbp_nlp_time_acados_create_capsule();
    if (capsule_ == nullptr || rbp_nlp_time_acados_create(capsule_) != 0) {
      throw std::runtime_error("failed to create the acados solver for the NLP optimizer");
    }
    config_ = rbp_nlp_time_acados_get_nlp_config(capsule_);
    dims_ = rbp_nlp_time_acados_get_nlp_dims(capsule_);
    in_ = rbp_nlp_time_acados_get_nlp_in(capsule_);
    out_ = rbp_nlp_time_acados_get_nlp_out(capsule_);
    solver_ = rbp_nlp_time_acados_get_nlp_solver(capsule_);
    opts_ = rbp_nlp_time_acados_get_nlp_opts(capsule_);
  }

  ~Solver()
  {
    if (capsule_ != nullptr) {
      rbp_nlp_time_acados_free(capsule_);
      rbp_nlp_time_acados_free_capsule(capsule_);
    }
  }

  Solver(const Solver &) = delete;
  Solver & operator=(const Solver &) = delete;

  void set_max_iterations(const int max_iterations)
  {
    int value = std::max(1, max_iterations);
    ocp_nlp_solver_opts_set(config_, opts_, "max_iter", &value);
  }

  void set_parameters(const int stage, const std::array<double, NP> & values)
  {
    rbp_nlp_time_acados_update_params(capsule_, stage, const_cast<double *>(values.data()), NP);
  }

  //! 初期状態の pin (S5 §2)。stage 0 は全状態が box で固定される
  void set_initial_state(const std::array<double, NX> & x0)
  {
    set_constraint(0, "lbx", x0.data());
    set_constraint(0, "ubx", x0.data());
  }

  //! [κ, v, a] の box。stage 0 は pin 済みなので 1..N に課す
  void set_state_bounds(
    const std::array<double, NBX> & lower, const std::array<double, NBX> & upper)
  {
    for (int stage = 1; stage <= N; ++stage) {
      set_constraint(stage, "lbx", lower.data());
      set_constraint(stage, "ubx", upper.data());
    }
  }

  //! [w, j] の box
  void set_control_bounds(
    const std::array<double, NU> & lower, const std::array<double, NU> & upper)
  {
    for (int stage = 0; stage < N; ++stage) {
      set_constraint(stage, "lbu", lower.data());
      set_constraint(stage, "ubu", upper.data());
    }
  }

  //! 非線形行の上下限。生成側は stage 0 に h を持たない (pin 済みの初期状態で
  //! ハード行が破れていると実行不能になるため) ので 1..N-1 に課す
  void set_h_bounds(
    const int stage, const std::array<double, NH> & lower, const std::array<double, NH> & upper)
  {
    set_constraint(stage, "lh", lower.data());
    set_constraint(stage, "uh", upper.data());
  }

  void set_terminal_h_bounds(
    const std::array<double, NHN> & lower, const std::array<double, NHN> & upper)
  {
    set_constraint(N, "lh", lower.data());
    set_constraint(N, "uh", upper.data());
  }

  void set_stage_weights(const std::array<double, NY> & diagonal)
  {
    std::array<double, NY * NY> weight{};
    for (int i = 0; i < NY; ++i) {
      weight[i * NY + i] = diagonal[static_cast<std::size_t>(i)];
    }
    for (int stage = 0; stage < N; ++stage) {
      set_cost(stage, "W", weight.data());
    }
  }

  void set_terminal_weights(const std::array<double, NYN> & diagonal)
  {
    std::array<double, NYN * NYN> weight{};
    for (int i = 0; i < NYN; ++i) {
      weight[i * NYN + i] = diagonal[static_cast<std::size_t>(i)];
    }
    set_cost(N, "W", weight.data());
  }

  //! スラックの二次ペナルティ (S5 §3.4)。スラック添字 i は行 i+1 に対応する (行 0 はハード)
  void set_slack_weights(const std::array<double, NSH> & quadratic)
  {
    const std::array<double, NSH> linear{};  // L1 は使わない (S5 §3.4)
    for (int stage = 1; stage < N; ++stage) {
      set_cost(stage, "Zl", quadratic.data());
      set_cost(stage, "Zu", quadratic.data());
      set_cost(stage, "zl", linear.data());
      set_cost(stage, "zu", linear.data());
    }
  }

  void set_terminal_slack_weights(const std::array<double, NSHN> & quadratic)
  {
    const std::array<double, NSHN> linear{};
    set_cost(N, "Zl", quadratic.data());
    set_cost(N, "Zu", quadratic.data());
    set_cost(N, "zl", linear.data());
    set_cost(N, "zu", linear.data());
  }

  void set_reference(const int stage, const std::array<double, NY> & reference)
  {
    set_cost(stage, "yref", reference.data());
  }

  void set_terminal_reference(const std::array<double, NYN> & reference)
  {
    set_cost(N, "yref", reference.data());
  }

  void set_guess(
    const int stage, const std::array<double, NX> & x, const std::array<double, NU> & u)
  {
    ocp_nlp_out_set(config_, dims_, out_, in_, stage, "x", const_cast<double *>(x.data()));
    if (stage < N) {
      ocp_nlp_out_set(config_, dims_, out_, in_, stage, "u", const_cast<double *>(u.data()));
    }
  }

  Result solve()
  {
    Result result;
    result.status = rbp_nlp_time_acados_solve(capsule_);
    double time_tot = 0.0;
    ocp_nlp_get(solver_, "time_tot", &time_tot);
    result.solve_time_ms = 1.0e3 * time_tot;
    ocp_nlp_get(solver_, "sqp_iter", &result.sqp_iterations);

    result.x.resize(static_cast<std::size_t>(N) + 1);
    for (int stage = 0; stage <= N; ++stage) {
      ocp_nlp_out_get(
        config_, dims_, out_, stage, "x", result.x[static_cast<std::size_t>(stage)].data());
    }
    result.u.resize(static_cast<std::size_t>(N));
    for (int stage = 0; stage < N; ++stage) {
      ocp_nlp_out_get(
        config_, dims_, out_, stage, "u", result.u[static_cast<std::size_t>(stage)].data());
    }
    return result;
  }

private:
  void set_constraint(const int stage, const char * field, const double * value) const
  {
    ocp_nlp_constraints_model_set(
      config_, dims_, in_, out_, stage, field, const_cast<double *>(value));
  }

  void set_cost(const int stage, const char * field, const double * value) const
  {
    ocp_nlp_cost_model_set(config_, dims_, in_, stage, field, const_cast<double *>(value));
  }

  rbp_nlp_time_solver_capsule * capsule_{nullptr};
  ocp_nlp_config * config_{nullptr};
  ocp_nlp_dims * dims_{nullptr};
  ocp_nlp_in * in_{nullptr};
  ocp_nlp_out * out_{nullptr};
  ocp_nlp_solver * solver_{nullptr};
  void * opts_{nullptr};
};

// =============================================================================================
// 入力の前処理
// =============================================================================================

RoughPlan resample_rough_plan(const RoughPlan & plan, const std::size_t num_points)
{
  if (plan.points.size() == num_points || plan.points.size() < 2 || num_points < 2) {
    return plan;
  }
  // rough_planner の格子と codegen のステージ数が違う設定でも解けるようにする。
  // ホライゾン長 T は rough_plan のものを保ち、刻みを T/(num_points-1) に取り直す
  const double horizon = plan.points.back().t - plan.points.front().t;
  const double step = horizon / static_cast<double>(num_points - 1);
  const bool has_s = plan.s.size() == plan.points.size();

  RoughPlan resampled = plan;
  resampled.points.clear();
  resampled.s.clear();
  std::size_t index = 0;
  for (std::size_t k = 0; k < num_points; ++k) {
    const double t = plan.points.front().t + step * static_cast<double>(k);
    while (index + 2 < plan.points.size() && plan.points[index + 1].t < t) {
      ++index;
    }
    const auto & a = plan.points[index];
    const auto & b = plan.points[index + 1];
    const double span = b.t - a.t;
    const double ratio = span > EPS ? std::clamp((t - a.t) / span, 0.0, 1.0) : 0.0;

    RoughPlanPoint point;
    point.t = t;
    point.pose.position = Point2d{
      a.pose.position.x() + ratio * (b.pose.position.x() - a.pose.position.x()),
      a.pose.position.y() + ratio * (b.pose.position.y() - a.pose.position.y())};
    point.pose.yaw =
      a.pose.yaw + ratio * autoware_utils_math::normalize_radian(b.pose.yaw - a.pose.yaw);
    point.kappa = a.kappa + ratio * (b.kappa - a.kappa);
    point.v = a.v + ratio * (b.v - a.v);
    point.a = a.a + ratio * (b.a - a.a);
    resampled.points.push_back(point);
    if (has_s) {
      resampled.s.push_back(plan.s[index] + ratio * (plan.s[index + 1] - plan.s[index]));
    }
  }
  return resampled;
}

NlpLimits collect_nlp_limits(
  const CompiledConstraints & compiled_constraints, const VehicleInfo & vehicle_info)
{
  NlpLimits limits;
  limits.base = collect_kinematic_limits(compiled_constraints);
  // κ 上限はステア角上限から導出する (発行側が二重定義しない規約。vehicle_kinematics.cpp)
  double steer_angle_max = vehicle_info.max_steer_angle_rad;

  // Tier 削除により IR の全域 bound は全てハード上限として読む
  // (nominal 系 limits は既定値のまま。COMFORT 行は IR からは現状発生しない)
  for (const auto & bound : compiled_constraints.scalar_bounds) {
    if (bound.s0 != -INF || bound.s1 != INF) {
      continue;  // 区間限定は s ごとに読む (section_velocity_upper)
    }
    switch (bound.quantity) {
      case BoundedQuantity::LON_JERK:
        limits.j_hard = std::min(limits.j_hard, bound.max);
        break;
      case BoundedQuantity::STEER_RATE:
        limits.steer_rate_hard = std::min(limits.steer_rate_hard, bound.max);
        break;
      case BoundedQuantity::STEER_ANGLE:
        steer_angle_max = std::min(steer_angle_max, bound.max);
        break;
      case BoundedQuantity::LAT_ACCEL:
        limits.a_lat_hard = std::min(limits.a_lat_hard, bound.max);
        break;
      case BoundedQuantity::CURVATURE:
        limits.kappa_max = std::min(limits.kappa_max, bound.max);
        break;
      default:
        break;  // VELOCITY / LON_ACCEL は KinematicLimits が持つ
    }
  }
  limits.kappa_max = std::min(
    limits.kappa_max, std::tan(steer_angle_max) / std::max(vehicle_info.wheel_base_m, EPS));
  return limits;
}

// =============================================================================================
// 離散写像・初期状態・certificate (S5 §1.2 / §6 / §7)
// =============================================================================================

OptimizedTrajectoryPoint integrate_rk4(
  const OptimizedTrajectoryPoint & state, const double w, const double j, const double dt)
{
  const auto derivative = [w, j](const std::array<double, 6> & x) {
    return std::array<double, 6>{
      x[4] * std::cos(x[2]), x[4] * std::sin(x[2]), x[4] * x[3], w, x[5], j};
  };
  const auto advance =
    [](const std::array<double, 6> & x, const std::array<double, 6> & d, const double step) {
      std::array<double, 6> out{};
      for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = x[i] + step * d[i];
      }
      return out;
    };

  const auto x0 = to_array(state);
  const auto k1 = derivative(x0);
  const auto k2 = derivative(advance(x0, k1, 0.5 * dt));
  const auto k3 = derivative(advance(x0, k2, 0.5 * dt));
  const auto k4 = derivative(advance(x0, k3, dt));

  std::array<double, 6> next{};
  for (std::size_t i = 0; i < next.size(); ++i) {
    next[i] = x0[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
  return from_array(next, state.t + dt);
}

OptimizedTrajectoryPoint condition_initial_state(
  const OptimizedTrajectoryPoint & raw, const NlpLimits & limits)
{
  OptimizedTrajectoryPoint conditioned = raw;
  conditioned.t = 0.0;
  conditioned.v = std::clamp(raw.v, 0.0, limits.base.v_hard);
  conditioned.kappa = std::clamp(raw.kappa, -limits.kappa_max, limits.kappa_max);
  // a₀ の下限: 減速中の a を j_hard で 0 へ戻す間に v が a₀²/(2 j_hard) 下がるので、
  // これを超える減速から出発すると v ≥ 0 が回復不能になる (S5 §6 #3)。上限は対称
  const double lower =
    std::max(limits.base.a_hard_min, -std::sqrt(2.0 * limits.j_hard * conditioned.v));
  const double upper = std::min(
    limits.base.a_hard_max,
    std::sqrt(2.0 * limits.j_hard * std::max(0.0, limits.base.v_hard - conditioned.v)));
  conditioned.a = std::clamp(raw.a, lower, std::max(lower, upper));
  conditioned.w = 0.0;
  conditioned.j = 0.0;
  return conditioned;
}

OptimizedTrajectory make_certificate(
  const OptimizedTrajectoryPoint & initial, const NlpLimits & limits, const std::size_t num_points,
  const double dt)
{
  OptimizedTrajectory certificate;
  if (num_points == 0 || dt <= EPS) {
    return certificate;
  }
  certificate.points.reserve(num_points);

  //! 残りのステージを停留点 (v = 0, a = 0, u = 0) で埋める。F の停留点なので残差は 0
  const auto hold_until_end = [&certificate, num_points, dt](OptimizedTrajectoryPoint state) {
    state.v = 0.0;
    state.a = 0.0;
    state.w = 0.0;
    state.j = 0.0;
    while (certificate.points.size() < num_points) {
      state.t = dt * static_cast<double>(certificate.points.size());
      certificate.points.push_back(state);
    }
  };

  auto state = initial;
  state.t = 0.0;
  while (certificate.points.size() + 1 < num_points) {
    if (state.v <= STOP_VELOCITY && state.a <= 0.0) {
      hold_until_end(state);  // 既に停止している (発進の誘因はこの軌道に無い)
      return certificate;
    }

    // 回復境界 (下側): a を j_hard で 0 へ戻す余裕が尽きる前に停止へ着地する。
    // 通常規則の躍度は j_nom なので、この切替が無いと戻しきれず v < 0 になる (S5 §7.5 #3)
    const double margin = state.a * state.a / (2.0 * limits.j_hard) + std::abs(state.a) * dt;
    if (state.a < 0.0 && state.v <= margin) {
      const auto landing =
        solve_stop_landing(state, dt, limits.j_hard, num_points - certificate.points.size() - 1);
      if (landing) {
        for (const double jerk : *landing) {
          state.w = 0.0;
          state.j = jerk;
          certificate.points.push_back(state);
          state = integrate_rk4(state, 0.0, jerk, dt);
        }
        hold_until_end(state);
        return certificate;
      }
      // ⚠ 離散格子上、境界状態では v ≥ 0 を保ったまま (0, 0) へ厳密着地できない場合がある。
      // その周期は最大躍度で a を戻すだけに留める (S5 §7.5 の「残る既知の緩み」)
      state.w = 0.0;
      state.j = std::clamp(-state.a / dt, -limits.j_hard, limits.j_hard);
      certificate.points.push_back(state);
      state = integrate_rk4(state, 0.0, state.j, dt);
      continue;
    }

    // 通常規則: a を最大快適減速 a_nom_min へ寄せる。w = 0 (曲率保持) で、減速中に形状を
    // いじらない (S5 §7.2)。κ は clamp 済みで box 内なので w = 0 は box 内
    double jerk = std::clamp((limits.base.a_nom_min - state.a) / dt, -limits.j_nom, limits.j_nom);
    // 回復境界 (上側): a > 0 を戻しきる前に v が v_hard を越えないようにする (S5 §7.5 #2)
    if (state.a > 0.0 && (limits.base.v_hard - state.v) <= margin) {
      jerk = std::clamp(-state.a / dt, -limits.j_hard, limits.j_hard);
    }
    state.w = 0.0;
    state.j = jerk;
    certificate.points.push_back(state);
    state = integrate_rk4(state, 0.0, jerk, dt);
  }
  state.w = 0.0;
  state.j = 0.0;
  certificate.points.push_back(state);  // 終端に入力は無い
  return certificate;
}

// =============================================================================================
// コリドー: cube → ステージごとの半空間
// =============================================================================================

std::vector<StagePlanes> make_stage_planes(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const std::vector<SemanticCube> & cubes, const std::vector<CorridorSeedPoint> & seed,
  const SscCorridorParams & corridor)
{
  std::vector<StagePlanes> stage_planes;
  stage_planes.reserve(seed.size());
  if (cubes.empty()) {
    return stage_planes;
  }
  const auto & path = context.reference_path;
  const double length = path.length();
  const auto & vehicle = context.vehicle_info;

  // 前方カット (footprint 前端が越えてはいけない弧長) は cube の s1 を使わず、時刻 t_k に
  // 前方で効く占有・停止線から直接引く。cube の s1 は 1 s 窓の最悪値なので窓ごとに階段状に
  // 跳び、横境界の狭まりでも止まる (横は横 2 面が持つので前方面に載せない)。
  // 対象は、そのステージで車両が居られる横バンド (cube の l 範囲 + footprint) に掛かり、
  // seed の footprint 前端より先にあるもの。seed と重なる物体は彫り込みの seed 棄却で
  // 既に除かれているか、横面が受け持つ。何も無ければ cube 膨張の上限と同じ距離だけ先に置く
  // (面を 1e6 に飛ばすと行の値が lh = −1e6 を割ることがある)
  const auto forward_limit = [&](const CorridorSeedPoint & point, const SemanticCube & cube) {
    const SlBox band = footprint_sl_box(vehicle, SlBox{point.s, point.s, cube.l0, cube.l1});
    const double margin_m = corridor.margin_m;
    double limit = band.s_max + corridor.max_longitudinal_inflation_m;
    for (const auto & occupancy : compiled_constraints.occupancies) {
      const auto * keep_out =
        std::get_if<KeepOut>(&compiled_constraints.raw_constraints[occupancy.raw_index].payload);
      const double margin = (keep_out ? keep_out->margin_m : 0.0) + margin_m;
      for (const auto & slab : occupancy.slabs) {
        if (point.t < slab.t0 || point.t > slab.t1) {
          continue;
        }
        if (slab.l1 + margin < band.l_min || slab.l0 - margin > band.l_max) {
          continue;
        }
        const double front_limit = slab.s0 - margin;
        if (front_limit >= band.s_max - EPS) {
          limit = std::min(limit, front_limit);
        }
      }
    }
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (point.t < stop_bar.time.t0 || point.t > stop_bar.time.t1) {
        continue;
      }
      const double front_limit = stop_bar.s_stop - stop_bar.margin - margin_m;
      if (front_limit >= band.s_max - EPS) {
        limit = std::min(limit, front_limit);
      }
    }
    return limit;
  };

  for (const auto & point : seed) {
    // 時間分割は隙間なく連続 (ssc_corridor.hpp) なので、t を含む cube は一意に決まる。
    // 末尾の丸めで外れたときは最後の cube に寄せる
    std::size_t index = cubes.size() - 1;
    for (std::size_t j = 0; j < cubes.size(); ++j) {
      if (point.t <= cubes[j].t1 + EPS) {
        index = j;
        break;
      }
    }
    const auto & cube = cubes[index];

    // anchor はそのステージの seed の弧長。cube は Frenet の箱なので、世界座標の半空間に
    // 落とすときに曲率ぶんの近似が入る。anchor を「そのステージで車両が居るはずの場所」に
    // 取ることで近似誤差を最小にする (cube の中心に取ると、曲がっている区間で数十 cm ずれる)
    const double s_anchor = std::clamp(point.s, 0.0, length);
    const auto anchor = to_world_pose(path, s_anchor, 0.0);
    const double tangent_x = std::cos(anchor.yaw);
    const double tangent_y = std::sin(anchor.yaw);
    const double left_x = -tangent_y;
    const double left_y = tangent_x;
    const double px = anchor.position.x();
    const double py = anchor.position.y();

    // cube は **footprint 込みで後軸点**を縛る (SSC は ego を点として扱い、彫り込みが
    // footprint_sl_box で膨張を吸収している)。一方 NLP の行は footprint の支持関数を持つので、
    // cube をそのまま面にすると footprint を二重に数えて実行不能になる (半幅 2.5 m の車線で
    // C の追加マージンが構造的に破れた)。そこで cube を footprint の張り出し分だけ**広げて**
    // 「点の自由空間」に戻し、姿勢依存の張り出しは支持関数行に任せる。姿勢が参照に一致する
    // 場合はちょうど元の cube と同じ制約になり、ヘディング偏差があるぶんだけ厳しくなる
    const double l_upper = cube.l1 + vehicle.max_lateral_offset_m;
    const double l_lower = cube.l0 + vehicle.min_lateral_offset_m;

    StagePlanes planes;
    // 横 (左): l ≤ l1
    planes[0] = HalfPlane{left_x, left_y, left_x * px + left_y * py + l_upper};
    // 横 (右): l ≥ l0
    planes[1] = HalfPlane{-left_x, -left_y, -(left_x * px + left_y * py) - l_lower};
    // 縦 (前): footprint 前端 ≤ t_k の前方限界。後方カットは課さない (NUM_PLANES の注記)
    planes[2] = HalfPlane{
      tangent_x, tangent_y,
      tangent_x * px + tangent_y * py + (forward_limit(point, cube) - s_anchor)};
    stage_planes.push_back(planes);
  }
  return stage_planes;
}

// =============================================================================================
// 検証 (S5 §9)。ソルバーのステータスは成否判定に使わない
// =============================================================================================

bool NlpVerification::passes(const SolveLevel level) const
{
  if (!dynamics_ok || !vehicle_ok) {
    return false;  // iterate が軌道ですらない (S5 §9.3 の 1・2)
  }
  switch (level) {
    case SolveLevel::LEVEL_1:
      return safety_ok && comfort_ok;
    case SolveLevel::LEVEL_2:
      return safety_ok;
    case SolveLevel::LEVEL_3:
      return true;
  }
  return false;
}

NlpVerification verify_trajectory(
  const PlannerContext & context, const CompiledConstraints & compiled_constraints,
  const OptimizedTrajectory & trajectory, const NlpLimits & limits)
{
  NlpVerification verification;
  const auto & points = trajectory.points;
  if (points.size() < 2) {
    verification.message = "trajectory is too short to verify";
    return verification;
  }
  for (const auto & point : points) {
    if (
      !std::isfinite(point.pose.position.x()) || !std::isfinite(point.pose.position.y()) ||
      !std::isfinite(point.pose.yaw) || !std::isfinite(point.kappa) || !std::isfinite(point.v) ||
      !std::isfinite(point.a) || !std::isfinite(point.w) || !std::isfinite(point.j)) {
      verification.message = "iterate contains a non-finite value";
      return verification;
    }
  }

  // ---- 1. 力学残差 (正規化 ∞ ノルム。S5 §9.1) ----
  const std::array<double, 6> scale{
    1.0,
    1.0,
    1.0,
    limits.kappa_max,
    limits.base.v_hard,
    std::max(std::abs(limits.base.a_hard_min), std::abs(limits.base.a_hard_max))};
  for (std::size_t k = 0; k + 1 < points.size(); ++k) {
    const double dt = points[k + 1].t - points[k].t;
    if (dt <= EPS) {
      verification.message = "non-monotonic time grid";
      return verification;
    }
    const auto expected = to_array(integrate_rk4(points[k], points[k].w, points[k].j, dt));
    const auto actual = to_array(points[k + 1]);
    for (std::size_t i = 0; i < expected.size(); ++i) {
      // θ (i = 2) は法 2π の量なので、差は正規化してから測る。出力の yaw を [−π, π] へ
      // 畳んである分、素の差では折り返しの 1 ステージだけ 2π の残差に見えてしまう
      const double difference = (i == 2)
                                  ? autoware_utils_math::normalize_radian(actual[i] - expected[i])
                                  : actual[i] - expected[i];
      const double residual = std::abs(difference) / std::max(scale[i], EPS);
      verification.max_dynamics_residual = std::max(verification.max_dynamics_residual, residual);
    }
  }
  verification.dynamics_ok = verification.max_dynamics_residual <= DYNAMICS_RESIDUAL_TOL;
  if (!verification.dynamics_ok) {
    verification.message = "dynamics residual exceeds the tolerance";
    return verification;
  }

  // ---- 2. A tier (box + ステアレート hard)。スラックが無いので超過は打ち切り iterate の兆候 ----
  const double wheel_base = context.vehicle_info.wheel_base_m;
  const auto relative = [](const double value) {
    return VEHICLE_RELATIVE_TOL * std::max(std::abs(value), EPS);
  };
  verification.vehicle_ok = true;
  for (const auto & point : points) {
    const bool ok =
      std::abs(point.kappa) <= limits.kappa_max + relative(limits.kappa_max) &&
      point.v >= -relative(limits.base.v_hard) &&
      point.v <= limits.base.v_hard + relative(limits.base.v_hard) &&
      point.a >= limits.base.a_hard_min - relative(limits.base.a_hard_min) &&
      point.a <= limits.base.a_hard_max + relative(limits.base.a_hard_max) &&
      std::abs(point.j) <= limits.j_hard + relative(limits.j_hard) &&
      std::abs(steer_rate_of(point.w, point.kappa, wheel_base)) <=
        limits.steer_rate_hard + relative(limits.steer_rate_hard) &&
      point.v * point.v * std::abs(point.kappa) <= limits.a_lat_hard + relative(limits.a_lat_hard);
    if (!ok) {
      verification.vehicle_ok = false;
      verification.message = "vehicle (tier A) limit is violated";
      return verification;
    }
  }

  // ---- 3. B tier (コリドー幾何 + 区間速度) ----
  // 幾何は射影ビュー越しに測り直す (raw から作った (s, l) ビューが唯一の共通土台。sl_view)
  const auto & path = context.reference_path;
  verification.safety_ok = true;
  verification.comfort_ok = true;
  for (const auto & point : points) {
    const double s = experimental::trajectory::closest(path, [&] {
      geometry_msgs::msg::Point p;
      p.x = point.pose.position.x();
      p.y = point.pose.position.y();
      return p;
    }());
    const double l = lateral_offset_at(path, s, point.pose.position);
    auto box = footprint_sl_box(context.vehicle_info, s, l);
    // 許容侵入 2 cm ぶん box を縮めて測る (sl_view の判定は bool で余裕を返さない)
    box.s_min += SAFETY_GEOMETRY_TOL;
    box.s_max -= SAFETY_GEOMETRY_TOL;
    box.l_min += SAFETY_GEOMETRY_TOL;
    box.l_max -= SAFETY_GEOMETRY_TOL;

    // Tier 削除により全エントリを safety 行として検証する
    for (const auto & bound : compiled_constraints.lateral_bounds) {
      if (!violates_lateral_bound(bound, box)) {
        continue;
      }
      verification.safety_ok = false;
      verification.message = "safety (tier B) lateral bound is violated";
      return verification;
    }
    for (const auto & occupancy : compiled_constraints.occupancies) {
      if (!violates_occupancy(occupancy, compiled_constraints, box, point.t, point.t)) {
        continue;
      }
      verification.safety_ok = false;
      verification.message = "safety (tier B) occupancy is violated";
      return verification;
    }
    for (const auto & stop_bar : compiled_constraints.stop_bars) {
      if (!violates_stop_bar(stop_bar, box, point.t, point.t)) {
        continue;
      }
      verification.safety_ok = false;
      verification.message = "safety (tier B) stop bar is violated";
      return verification;
    }
    const double v_legal = section_velocity_upper(compiled_constraints, s);
    if (point.v > v_legal + SAFETY_VELOCITY_TOL) {
      verification.safety_ok = false;
      verification.message = "safety (tier B) section speed limit is violated";
      return verification;
    }

    // ---- 4. C tier (快適スカラー)。落とせる行なので判定は 5 % 許容 ----
    const double comfort_tol = COMFORT_RELATIVE_TOL;
    const double v_nom = limits.base.v_nom;
    const auto comfort_row = [&]() -> const char * {
      if (
        point.a < limits.base.a_nom_min * (1.0 + comfort_tol) - EPS ||
        point.a > limits.base.a_nom_max * (1.0 + comfort_tol) + EPS) {
        return "comfort (tier C) longitudinal acceleration";
      }
      if (std::abs(point.j) > limits.j_nom * (1.0 + comfort_tol)) {
        return "comfort (tier C) longitudinal jerk";
      }
      if (point.v * point.v * std::abs(point.kappa) > limits.base.a_lat_nom * (1.0 + comfort_tol)) {
        return "comfort (tier C) lateral acceleration";
      }
      if (
        std::abs(steer_rate_of(point.w, point.kappa, wheel_base)) >
        limits.steer_rate_nom * (1.0 + comfort_tol)) {
        return "comfort (tier C) steering rate";
      }
      if (point.v > v_nom * (1.0 + comfort_tol)) {
        return "comfort (tier C) nominal speed";
      }
      return nullptr;
    }();
    if (comfort_row != nullptr) {
      verification.comfort_ok = false;
      if (verification.message.empty()) {
        verification.message = std::string(comfort_row) + " is violated";
      }
    }
  }
  return verification;
}

// =============================================================================================
// プラグイン
// =============================================================================================

NlpTrajectoryOptimizer::NlpTrajectoryOptimizer() = default;
NlpTrajectoryOptimizer::~NlpTrajectoryOptimizer() = default;

NlpParams NlpTrajectoryOptimizer::read_params() const
{
  const auto & p = params_.trajectory_optimizer.nlp;
  NlpParams params;
  params.corridor.cube_duration_s = p.cube_duration_s;
  params.corridor.margin_m = p.margin_m;
  params.corridor.inflation_step_m = p.inflation_step_m;
  params.corridor.max_lateral_inflation_m = p.max_lateral_inflation_m;
  params.corridor.max_longitudinal_inflation_m = p.max_longitudinal_inflation_m;
  params.curvature_rate_max = p.curvature_rate_max;
  params.goal_capture_distance_m = p.goal_capture_distance_m;
  params.weight_pos = p.weight_pos;
  params.weight_yaw = p.weight_yaw;
  params.weight_v = p.weight_v;
  params.weight_kappa = p.weight_kappa;
  params.weight_w = p.weight_w;
  params.weight_a = p.weight_a;
  params.weight_j = p.weight_j;
  params.weight_terminal_v = p.weight_terminal_v;
  params.weight_terminal_a = p.weight_terminal_a;
  params.slack_safety = p.slack_safety;
  params.slack_comfort = p.slack_comfort;
  params.max_iter_level1 = static_cast<int>(p.max_iter_level1);
  params.max_iter_level2 = static_cast<int>(p.max_iter_level2);
  params.max_iter_level3 = static_cast<int>(p.max_iter_level3);
  return params;
}

TrajectoryOptimizerResult NlpTrajectoryOptimizer::optimize(const TrajectoryOptimizerInput & input)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  using Solver = NlpTrajectoryOptimizer::Solver;
  constexpr std::size_t NUM_POINTS = static_cast<std::size_t>(Solver::N) + 1;

  TrajectoryOptimizerResult result;
  const auto params = read_params();
  const auto limits = collect_nlp_limits(input.compiled_constraints, input.context.vehicle_info);

  if (input.rough_plan.points.size() < 2) {
    result.status = TrajectoryOptimizerStatus::INFEASIBLE;
    result.debug.message = "rough plan is too short to optimize";
    return result;
  }
  // 時間格子を codegen のステージ数へ合わせる (点数が一致していれば恒等写像)
  const auto plan = resample_rough_plan(input.rough_plan, NUM_POINTS);
  const double dt = (plan.points.back().t - plan.points.front().t) / (NUM_POINTS - 1);
  if (dt <= EPS) {
    result.status = TrajectoryOptimizerStatus::INFEASIBLE;
    result.debug.message = "rough plan has a degenerate time grid";
    return result;
  }

  // 初期状態: **曲率以外は ego の実測値**を使い、A の box へ clamp する (S5 §6)。
  // rough_plan の先頭は位置こそ ego 由来だが、姿勢は reference_path の方位、速度は
  // 速度平滑を通した後の値、加速度は隣接ステージの差分で、いずれも実状態と乖離しうる。
  // stage 0 は box で pin されるので、乖離はそのまま出力軌道の起点になる。
  // κ₀ は実測操舵角から自転車モデルの逆写像 κ = tan(δ) / L で作る (S5 §1.2 の δ = atan(κ L)
  // と同じ定義。生値をそのまま使うので、操舵の遅れ・ノイズはそのまま x0 に入る)。
  const auto & ego_pose = input.context.odometry.pose.pose;
  OptimizedTrajectoryPoint raw_ego;
  raw_ego.pose.position = Point2d{ego_pose.position.x, ego_pose.position.y};
  raw_ego.pose.yaw = autoware_utils_geometry::get_rpy(ego_pose).z;
  raw_ego.kappa =
    std::tan(input.context.steering.steering_tire_angle) / input.context.vehicle_info.wheel_base_m;
  raw_ego.v = input.context.odometry.twist.twist.linear.x;
  raw_ego.a = input.context.acceleration.accel.accel.linear.x;
  const auto x0 = condition_initial_state(raw_ego, limits);
  const auto certificate = make_certificate(x0, limits, NUM_POINTS, dt);

  // コリドー: SSC と同じ彫り込みを共用する (このファイル冒頭の注記)
  const auto seed = make_corridor_seed(input.context, plan);
  const auto cubes =
    generate_semantic_corridor(input.context, input.compiled_constraints, seed, params.corridor);
  const auto stage_planes =
    make_stage_planes(input.context, input.compiled_constraints, cubes, seed, params.corridor);
  const double z_base = input.context.odometry.pose.pose.position.z;
  result.debug.debug_markers = make_corridor_markers(input.context, cubes, z_base);

  if (!solver_) {
    // 構造は 1 回だけ作る (S6 §4.1 原則 1)。生成に失敗したら以降の周期も無理なので、
    // 例外にせず「ソルバーエラー」として consumer のフォールバックへ渡す
    try {
      solver_ = std::make_unique<Solver>();
    } catch (const std::runtime_error & error) {
      result.status = TrajectoryOptimizerStatus::SOLVER_ERROR;
      result.debug.message = error.what();
      return result;
    }
  }

  // ---- 周期ごとに変わらない注入 ----
  solver_->set_initial_state(to_array(x0));
  solver_->set_state_bounds(
    {-limits.kappa_max, 0.0, limits.base.a_hard_min},
    {limits.kappa_max, limits.base.v_hard, limits.base.a_hard_max});
  solver_->set_control_bounds(
    {-params.curvature_rate_max, -limits.j_hard}, {params.curvature_rate_max, limits.j_hard});
  solver_->set_stage_weights(
    {params.weight_pos, params.weight_pos, params.weight_yaw, params.weight_kappa, params.weight_v,
     params.weight_a, params.weight_w, params.weight_j});

  // モデルパラメータ: dt・車両諸元・ステージのコリドー面
  for (int stage = 0; stage <= Solver::N; ++stage) {
    std::array<double, Solver::NP> values{};
    values[0] = dt;
    values[1] = input.context.vehicle_info.wheel_base_m;
    values[2] = input.context.vehicle_info.min_longitudinal_offset_m;  // 後端 (負)
    values[3] = input.context.vehicle_info.max_longitudinal_offset_m;  // 前端
    values[4] = input.context.vehicle_info.max_lateral_offset_m;       // 片側幅
    const auto index = static_cast<std::size_t>(stage);
    if (index < stage_planes.size()) {
      for (std::size_t plane = 0; plane < NUM_PLANES; ++plane) {
        values[5 + 3 * plane] = stage_planes[index][plane].nx;
        values[6 + 3 * plane] = stage_planes[index][plane].ny;
        values[7 + 3 * plane] = stage_planes[index][plane].d;
      }
    } else {
      // コリドーが無い周期は幾何行を下の境界値 (±FREE_BOUND) で無効化するので、面は
      // 値が小さく留まるものなら何でもよい (d を 1e6 にすると行の値が lh を割りうる)
      for (std::size_t plane = 0; plane < NUM_PLANES; ++plane) {
        values[5 + 3 * plane] = 1.0;
        values[6 + 3 * plane] = 0.0;
        values[7 + 3 * plane] = 0.0;
      }
    }
    solver_->set_parameters(stage, values);
  }
  const bool corridor_available = stage_planes.size() == NUM_POINTS;

  // C 免除ステージ (S5 §9.4): certificate (最大快適減速) でも満たせないステージでは
  // C 行を課さない。恒常的に破れる行のスラック勾配が過激な復帰操作へ解を歪めるため
  const auto comfort_exempt_until = [&](const auto & satisfied) {
    std::size_t k_ex = 0;
    for (std::size_t k = certificate.points.size(); k > 0; --k) {
      if (!satisfied(certificate.points[k - 1])) {
        k_ex = k;
        break;
      }
    }
    return k_ex;
  };
  const double wheel_base = input.context.vehicle_info.wheel_base_m;
  const std::size_t exempt_v = comfort_exempt_until(
    [&](const OptimizedTrajectoryPoint & p) { return p.v <= limits.base.v_nom; });
  const std::size_t exempt_a = comfort_exempt_until([&](const OptimizedTrajectoryPoint & p) {
    return p.a >= limits.base.a_nom_min && p.a <= limits.base.a_nom_max;
  });
  const std::size_t exempt_j = comfort_exempt_until(
    [&](const OptimizedTrajectoryPoint & p) { return std::abs(p.j) <= limits.j_nom; });
  const std::size_t exempt_lat = comfort_exempt_until([&](const OptimizedTrajectoryPoint & p) {
    return p.v * p.v * std::abs(p.kappa) <= limits.base.a_lat_nom;
  });
  const std::size_t exempt_steer = comfort_exempt_until([&](const OptimizedTrajectoryPoint & p) {
    return std::abs(steer_rate_of(p.w, p.kappa, wheel_base)) <= limits.steer_rate_nom;
  });

  // 追従参照。姿勢は段によらず不変で、v_ref だけが段で切り替わる (S5 §4.2)。
  // ゴール手前では終点を goal そのものへ差し替える (S5 §4.3)
  auto reference_points = plan.points;
  const auto s_goal = input.context.goal_arc_length();
  const auto snap_index = s_goal ? goal_snap_index(plan, *s_goal, params.goal_capture_distance_m)
                                 : std::optional<std::size_t>{};
  if (snap_index) {
    const auto & goal = input.context.goal_pose;
    Pose2d goal_pose;
    goal_pose.position = Point2d{goal.position.x, goal.position.y};
    goal_pose.yaw = autoware_utils_geometry::get_rpy(goal).z;
    for (std::size_t k = *snap_index; k < reference_points.size(); ++k) {
      reference_points[k].pose = goal_pose;
      reference_points[k].v = 0.0;
      reference_points[k].a = 0.0;
    }
  }
  const auto yaw_reference = unwrap_yaw(reference_points, x0.pose.yaw);

  // ---- 三段フォールバック (S5 §8.1) ----
  const auto warm_start = input.prev_trajectory;
  bool previous_iterate_usable = false;  // 前段の iterate を seed に使えるか (S5 §8.5 I-6)
  for (const auto level : {SolveLevel::LEVEL_1, SolveLevel::LEVEL_2, SolveLevel::LEVEL_3}) {
    const bool comfort_active = level == SolveLevel::LEVEL_1;
    const bool safety_active = level != SolveLevel::LEVEL_3;
    const bool stop_mode = level == SolveLevel::LEVEL_3;
    if (!corridor_available && safety_active) {
      // コリドーが彫れなかった周期 (seed が塞がれている等) は B 行を課せない。
      // 段を飛ばして Level 3 (停止) へ落とす — 塞がれた seed を助けるのは上流の仕事 (S3)
      continue;
    }

    // 参照 (yref) と目的の重み
    for (int stage = 0; stage < Solver::N; ++stage) {
      const auto & reference = reference_points[static_cast<std::size_t>(stage)];
      const double v_ref =
        stop_mode ? certificate.points[static_cast<std::size_t>(stage)].v : reference.v;
      solver_->set_reference(
        stage, {reference.pose.position.x(), reference.pose.position.y(),
                yaw_reference[static_cast<std::size_t>(stage)], 0.0, v_ref, 0.0, 0.0, 0.0});
    }
    {
      const auto & reference = reference_points.back();
      const double v_ref = stop_mode ? 0.0 : reference.v;
      solver_->set_terminal_reference(
        {reference.pose.position.x(), reference.pose.position.y(), yaw_reference.back(), 0.0, v_ref,
         0.0});
      // Solve 3 は終端停止項を常に有効化し、重みを 1 桁強める (S5 §4.2)
      const double terminal_v = params.weight_terminal_v * (stop_mode ? 10.0 : 1.0);
      solver_->set_terminal_weights(
        {params.weight_pos, params.weight_pos, params.weight_yaw, params.weight_kappa, terminal_v,
         params.weight_terminal_a});
    }

    // スラック係数 (スラック添字 i = 行 i+1)
    std::array<double, Solver::NSH> slack{};
    for (int row = 1; row < Solver::NH; ++row) {
      const bool is_safety = row == ROW_VELOCITY_SECTION || row >= FIRST_CORRIDOR_ROW_SAFETY;
      slack[static_cast<std::size_t>(row - 1)] =
        is_safety ? params.slack_safety : params.slack_comfort;
    }
    solver_->set_slack_weights(slack);
    std::array<double, Solver::NSHN> terminal_slack{};
    for (int row = 0; row < Solver::NHN; ++row) {
      const bool is_safety = row == ROW_E_VELOCITY_SECTION || row >= FIRST_CORRIDOR_ROW_E_SAFETY;
      terminal_slack[static_cast<std::size_t>(row)] =
        is_safety ? params.slack_safety : params.slack_comfort;
    }
    solver_->set_terminal_slack_weights(terminal_slack);

    // 行の上下限。段を落とす操作は**行の削除ではなく ±FREE_BOUND への開き**で表す
    for (int stage = 1; stage < Solver::N; ++stage) {
      const auto index = static_cast<std::size_t>(stage);
      std::array<double, Solver::NH> lower{};
      std::array<double, Solver::NH> upper{};
      lower.fill(-FREE_BOUND);
      upper.fill(FREE_BOUND);

      lower[ROW_STEER_RATE_HARD] = -limits.steer_rate_hard;
      upper[ROW_STEER_RATE_HARD] = limits.steer_rate_hard;

      const double s = index < seed.size() ? seed[index].s : seed.back().s;
      if (safety_active) {
        const double v_legal = section_velocity_upper(input.compiled_constraints, s);
        upper[ROW_VELOCITY_SECTION] = std::min(v_legal, FREE_BOUND);
        for (int row = 0; row < NUM_CORRIDOR_ROWS; ++row) {
          upper[static_cast<std::size_t>(FIRST_CORRIDOR_ROW_SAFETY + row)] = 0.0;
        }
      }
      if (comfort_active) {
        if (index >= exempt_steer) {
          lower[ROW_STEER_RATE_NOMINAL] = -limits.steer_rate_nom;
          upper[ROW_STEER_RATE_NOMINAL] = limits.steer_rate_nom;
        }
        if (index >= exempt_lat) {
          lower[ROW_LATERAL_ACCEL] = -limits.base.a_lat_nom;
          upper[ROW_LATERAL_ACCEL] = limits.base.a_lat_nom;
        }
        if (index >= exempt_a) {
          lower[ROW_ACCEL_COMFORT] = limits.base.a_nom_min;
          upper[ROW_ACCEL_COMFORT] = limits.base.a_nom_max;
        }
        if (index >= exempt_j) {
          lower[ROW_JERK_COMFORT] = -limits.j_nom;
          upper[ROW_JERK_COMFORT] = limits.j_nom;
        }
        if (index >= exempt_v) {
          upper[ROW_VELOCITY_NOMINAL] = limits.base.v_nom;
        }
      }
      solver_->set_h_bounds(stage, lower, upper);
    }
    {
      std::array<double, Solver::NHN> lower{};
      std::array<double, Solver::NHN> upper{};
      lower.fill(-FREE_BOUND);
      upper.fill(FREE_BOUND);
      const double s = seed.back().s;
      if (safety_active) {
        upper[ROW_E_VELOCITY_SECTION] =
          std::min(section_velocity_upper(input.compiled_constraints, s), FREE_BOUND);
        for (int row = 0; row < NUM_CORRIDOR_ROWS; ++row) {
          upper[static_cast<std::size_t>(FIRST_CORRIDOR_ROW_E_SAFETY + row)] = 0.0;
        }
      }
      if (comfort_active) {
        lower[ROW_E_LATERAL_ACCEL] = -limits.base.a_lat_nom;
        upper[ROW_E_LATERAL_ACCEL] = limits.base.a_lat_nom;
        lower[ROW_E_ACCEL_COMFORT] = limits.base.a_nom_min;
        upper[ROW_E_ACCEL_COMFORT] = limits.base.a_nom_max;
        upper[ROW_E_VELOCITY_NOMINAL] = limits.base.v_nom;
      }
      solver_->set_terminal_h_bounds(lower, upper);
    }

    // 初期値: Level 1 は前周期解 (無ければ rough_plan)、Level 2 は **Level 1 の iterate**
    // (ソルバーが保持しているものをそのまま使うので再注入しない。S5 §8.1)、
    // Level 3 は certificate 固定 — 目的が停止に差し替わり前段の「進む」解は別盆地にある
    // (S5 §8.1 の proposal からの差分)。ただし前段の iterate が力学残差 / A で落ちていたら
    // seed に使わず入力の seed へ戻す (S5 §8.5 I-6)
    const OptimizedTrajectory * guess = nullptr;
    bool reseed = true;
    if (stop_mode) {
      guess = &certificate;
    } else if (level == SolveLevel::LEVEL_2 && previous_iterate_usable) {
      reseed = false;
    } else if (warm_start && warm_start->points.size() == NUM_POINTS) {
      guess = &*warm_start;
    }
    for (int stage = 0; reseed && stage <= Solver::N; ++stage) {
      const auto index = static_cast<std::size_t>(stage);
      std::array<double, Solver::NX> x{};
      std::array<double, Solver::NU> u{};
      if (guess != nullptr) {
        const auto & point = guess->points[index];
        x = to_array(point);
        u = {point.w, point.j};
      } else {
        const auto & point = plan.points[index];
        x = {
          point.pose.position.x(),
          point.pose.position.y(),
          yaw_reference[index],
          point.kappa,
          point.v,
          point.a};
      }
      solver_->set_guess(stage, x, u);
    }

    const int max_iterations = level == SolveLevel::LEVEL_1   ? params.max_iter_level1
                               : level == SolveLevel::LEVEL_2 ? params.max_iter_level2
                                                              : params.max_iter_level3;
    solver_->set_max_iterations(max_iterations);
    const auto solution = solver_->solve();
    result.debug.elapsed_ms += solution.solve_time_ms;

    // iterate を軌道へ。ソルバーのステータスは採否に使わない (solved ≠ satisfied)
    OptimizedTrajectory trajectory;
    trajectory.points.reserve(NUM_POINTS);
    for (std::size_t k = 0; k < NUM_POINTS; ++k) {
      auto point = from_array(solution.x[k], dt * static_cast<double>(k));
      point.pose.yaw = autoware_utils_math::normalize_radian(point.pose.yaw);
      if (k + 1 < NUM_POINTS) {
        point.w = solution.u[k][0];
        point.j = solution.u[k][1];
      }
      trajectory.points.push_back(point);
    }

    const auto verification =
      verify_trajectory(input.context, input.compiled_constraints, trajectory, limits);
    previous_iterate_usable = verification.dynamics_ok && verification.vehicle_ok;
    if (!verification.passes(level)) {
      // 落ちた段の診断を残す (どの tier で落ちたか / ソルバーが何をして落ちたか)
      result.debug.message += "level " + std::to_string(static_cast<int>(level) + 1) + ": " +
                              verification.message + " (acados status " +
                              std::to_string(solution.status) + ", " +
                              std::to_string(solution.sqp_iterations) + " it, residual " +
                              std::to_string(verification.max_dynamics_residual) + "); ";
      continue;
    }
    result.trajectory = std::move(trajectory);
    result.status = TrajectoryOptimizerStatus::SUCCESS;
    result.debug.which_level = static_cast<int>(level) + 1;
    result.debug.message += verification.message;  // 落ちた段の診断を消さない
    return result;
  }

  // 全段落ち: certificate (最大快適減速の停止軌道) を最終結果として返す (S5 §8.2)。
  // 構成により力学と A を必ず満たすので、下流は追従できる
  result.trajectory = certificate;
  result.status = TrajectoryOptimizerStatus::SUCCESS;
  result.debug.which_level = 3;
  result.debug.used_certificate = true;
  result.debug.message += "all three levels failed; falling back to the certificate";
  return result;
}

}  // namespace autoware::safety_planner

PLUGINLIB_EXPORT_CLASS(
  autoware::safety_planner::NlpTrajectoryOptimizer,
  autoware::safety_planner::TrajectoryOptimizerInterface)
