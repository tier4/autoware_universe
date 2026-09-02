# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""NlpTrajectoryOptimizer が駆動する acados OCP の codegen (docs/spec/nlp_and_fallback.md S5).

**数値 (上下限・重み・参照・dt) は 1 つも codegen 時に決めない**。ここで固定するのは構造だけ:

  N               ステージ数 (rough_planner.num_points - 1 と一致させる)
  どの状態・入力に box を張るか
  どの行にスラックを付けるか
  コストの型 (LINEAR_LS)

三段フォールバック (S5 §8.1) は**同一の生成物のまま境界値の書き換えだけ**で実現する
(行を消すのではなく ±1e6 へ開いて無効化する。S6 §4.1 原則 2)。ソルバーを作り直さないので、
段を落とすたびのコストは再解 1 回分だけ。
"""

import os
import sys

from acados_template import AcadosModel
from acados_template import AcadosOcp
from acados_template import AcadosOcpSolver
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from generators.time_bicycle_model import ROW_STEER_RATE_HARD  # noqa: E402
from generators.time_bicycle_model import time_bicycle_model  # noqa: E402

# ステージ数。codegen 時に固定。C++ 側は生成された RBP_NLP_TIME_N を読み直して合わせる。
# 既定は rough_planner の既定格子 (num_points = 101, time_step_s = 0.1 → T = 10 s)。
DEFAULT_N = 100

# 無効化した行の開き幅。「常に満たされる無効行」の実体 (S6 §4.4)。
FREE_BOUND = 1.0e6


def build_ocp(n_horizon=DEFAULT_N, code_export_directory="c_generated_code_nlp_time"):
    ocp = AcadosOcp()

    model = time_bicycle_model()

    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.p = model.p
    model_ac.con_h_expr = model.con_h_expr
    model_ac.con_h_expr_e = model.con_h_expr_e
    model_ac.name = model.name
    ocp.model = model_ac

    ocp.code_export_directory = code_export_directory

    nx = model.x.rows()
    nu = model.u.rows()
    nh = model.con_h_expr.rows()
    nh_e = model.con_h_expr_e.rows()

    ocp.solver_options.N_horizon = n_horizon

    # --- コスト: 状態と入力の LINEAR_LS。重み・参照は C++ が毎周期上書きする -------------------
    # y   = [px, py, theta, kappa, v, a, w, j]   (S5 §4 の追従項 + 平滑項)
    # y_e = [px, py, theta, kappa, v, a]         (終端停止項は v_N / a_N の重みで表す)
    ny = nx + nu
    ny_e = nx

    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"

    vx = np.zeros((ny, nx))
    vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = vx
    vu = np.zeros((ny, nu))
    vu[nx:, :] = np.eye(nu)
    ocp.cost.Vu = vu
    ocp.cost.Vx_e = np.eye(ny_e, nx)

    ocp.cost.W = np.eye(ny)
    ocp.cost.W_e = np.eye(ny_e)
    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # --- box 制約 (すべて tier A = ハード、スラックなし) ------------------------------------
    # 入力 box |w| <= w_max, |j| <= j_hard。状態 box は kappa, v, a。
    # A をハードにできる根拠は S5 §7.4: コンディショニング済み初期状態からの前進シミュレーション
    # (certificate) がこれらを必ず満たすので、実行可能集合が空にならない。
    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.lbu = -np.ones(nu)
    ocp.constraints.ubu = np.ones(nu)

    ocp.constraints.idxbx = np.array([model.idx_kappa, model.idx_v, model.idx_a])
    ocp.constraints.lbx = np.array([-1.0, 0.0, -1.0])
    ocp.constraints.ubx = np.array([1.0, 1.0, 1.0])

    # 初期状態はコンディショニング済み ego に pin する (S5 §2 / §6)。
    ocp.constraints.x0 = np.zeros(nx)

    # 終端も kappa, v, a の同じ box。位置・姿勢は box を張らない (終端は自由で、
    # 参照追従と終端停止項がコストで引く)。
    ocp.constraints.idxbx_e = np.array([model.idx_kappa, model.idx_v, model.idx_a])
    ocp.constraints.lbx_e = np.array([-1.0, 0.0, -1.0])
    ocp.constraints.ubx_e = np.array([1.0, 1.0, 1.0])

    # --- 非線形行 --------------------------------------------------------------------------
    # 行 0 (ステアレートのハード上限) 以外は**すべてスラック付き**。理由は行の性質で決まる:
    # B (コリドー・区間速度) と C (快適) は地図・知覚・ego の入り方に依存するので、
    # ハードにすると実行不能で iterate が返らない周期が出る。スラックにしておけば必ず
    # iterate が返り、成否は独立検証 (S5 §9 / C++ 側の verify) が決める (solved != satisfied)。
    #
    # ペナルティは**二次のみ** (zl = zu = 0 のまま。L1 は S5 §3.4 で禁止 — 実験記録で収束を壊した)。
    # 係数 Zl / Zu は C++ が rho_B = 1e6 / rho_C = 1e4 で書く。
    slacked = np.array([row for row in range(nh) if row != ROW_STEER_RATE_HARD])
    ocp.constraints.lh = np.full(nh, -FREE_BOUND)
    ocp.constraints.uh = np.full(nh, FREE_BOUND)
    ocp.constraints.idxsh = slacked
    ocp.cost.Zl = np.ones(len(slacked))
    ocp.cost.Zu = np.ones(len(slacked))
    ocp.cost.zl = np.zeros(len(slacked))
    ocp.cost.zu = np.zeros(len(slacked))

    ocp.constraints.lh_e = np.full(nh_e, -FREE_BOUND)
    ocp.constraints.uh_e = np.full(nh_e, FREE_BOUND)
    ocp.constraints.idxsh_e = np.arange(nh_e)
    ocp.cost.Zl_e = np.ones(nh_e)
    ocp.cost.Zu_e = np.ones(nh_e)
    ocp.cost.zl_e = np.zeros(nh_e)
    ocp.cost.zu_e = np.zeros(nh_e)

    # 置き場所だけ。C++ が全ステージのパラメータを毎周期書く。
    ocp.parameter_values = np.zeros(model.p.rows())

    # --- ソルバー ---------------------------------------------------------------------------
    # 1 ステージ = 正規化 1 単位。物理時間は dt パラメータが持つ (time_bicycle_model の docstring)。
    ocp.solver_options.tf = float(n_horizon)
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    # RK4 1 ステップ/ステージ (S5 §1.2)。certificate・検証の離散写像もこれと同一にする。
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_max_iter = 50
    # 最悪周期を縛るのは SQP 反復数ではなく QP 反復数 (空間版の実測: 単一 QP に 100 ms 超)。
    ocp.solver_options.qp_solver_iter_max = 20
    ocp.solver_options.hpipm_mode = "SPEED"
    # 実行可能性 (eq/ineq/comp) は 1e-4、最適性 (stat) は 1e-1 と分離する (S5 §8.4)。
    ocp.solver_options.tol = 1.0e-4
    ocp.solver_options.tol_stat = 1.0e-1
    ocp.solver_options.levenberg_marquardt = 1.0e-4
    ocp.solver_options.qp_solver_warm_start = 2

    return ocp


def main():
    n_horizon = int(os.environ.get("RBP_NLP_N_HORIZON", DEFAULT_N))
    ocp = build_ocp(n_horizon=n_horizon)
    AcadosOcpSolver.generate(ocp, json_file="acados_ocp_{}.json".format(ocp.model.name))


if __name__ == "__main__":
    main()
