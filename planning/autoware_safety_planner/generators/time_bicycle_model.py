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

"""時間パラメタライズ運動学モデル (docs/spec/nlp_and_fallback.md S5 §2).

    状態   x = [px, py, theta, kappa, v, a]
    入力   u = [w, j]                        w = dkappa/dt, j = da/dt
    力学   px' = v cos(theta), py' = v sin(theta), theta' = v kappa,
           kappa' = w, v' = a, a' = j

kappa を状態に持つのがこの定式化の要。ステア角 delta = atan(L kappa) とステアレート
ddelta/dt = L w / (1 + (L kappa)^2) が状態と入力の関数になるので、事後チェックではなく
**制約として書ける** (SSC 側が構造的に書けないもの)。1/v はどこにも現れず v = 0 は特異点でない。

時間刻み dt は**モデルパラメータ**でありソルバーの時間刻みではない: 積分器は 1 ステージで
正規化 1 単位だけ進み (tf == N)、C++ 側が毎周期 rough_plan の刻みを dt に書く。
これが「生成物を作り直さずに計画格子を変えられる」ための仕掛け
(空間版 acados_mpt_optimizer が踏んだ「codegen 時と違う dt が解けない」痛点の回避)。
"""

import types

from casadi import SX
from casadi import cos
from casadi import sin
from casadi import sqrt
from casadi import vertcat

MODEL_NAME = "rbp_nlp_time"

# 1 ステージあたりのコリドー半空間の数: 横 (左右) + 前方カット = 3。
# 横 2 面はステージが属する semantic cube の面、前方カットはそのステージの時刻に前方へ効く
# 占有・停止線から直接引く。codegen 時に固定で、C++ 側の NUM_PLANES と一致していること。
#
# **後方カット (s >= s0) は載せない**。v >= 0 の box があるので後退はせず、後方に守る対象も
# 無い (停止線・先行車は前方カットが持つ)。加えて先頭 cube の s0 は経路始端で切れるため、
# footprint 後端が構造的に外へ出て rho_B のスラックが解を前方へ押してしまう。
NUM_PLANES = 3
# 支持関数の |n . w| の平滑化 (S6 §4.2 の注記)。sqrt(y^2 + eps) >= |y| なので必ず**厳しい側**へ
# ずれ、平滑化した行で実行可能な点は素の絶対値でも実行可能。
LATERAL_SUPPORT_EPSILON = 1.0e-6

# --- con_h_expr の行レイアウト (C++ 側と同じ順序を保つこと) ------------------------------------
# 行 1 以降はすべてスラック付き。行 0 だけがハードな非線形行で、w = 0 が常に満たすので
# 実行可能集合を空にできない (S5 §3.1: A tier は構成的に実行可能でなければならない)。
ROW_STEER_RATE_HARD = 0
ROW_STEER_RATE_NOMINAL = 1
ROW_LATERAL_ACCEL = 2
ROW_ACCEL_COMFORT = 3
ROW_JERK_COMFORT = 4
ROW_VELOCITY_SECTION = 5
ROW_VELOCITY_NOMINAL = 6
# コリドー行は B (侵入禁止, uh = 0, rho_B) の 1 ブロックだけ。C の追加マージン行は持たない
# (面からの余裕は cube の margin_m が既に持っており、二重に取ると車線幅に対して構造的に破れる)。
NUM_CORRIDOR_ROWS = 2 * NUM_PLANES
FIRST_CORRIDOR_ROW_SAFETY = 7
NUM_ROWS = FIRST_CORRIDOR_ROW_SAFETY + NUM_CORRIDOR_ROWS

# 終端ステージには入力が無いので、w / j を読む行は構造的に不在 (S5 §3.1)。残るのは状態だけの行。
ROW_E_LATERAL_ACCEL = 0
ROW_E_ACCEL_COMFORT = 1
ROW_E_VELOCITY_SECTION = 2
ROW_E_VELOCITY_NOMINAL = 3
FIRST_CORRIDOR_ROW_E_SAFETY = 4
NUM_ROWS_E = FIRST_CORRIDOR_ROW_E_SAFETY + NUM_CORRIDOR_ROWS


def _corridor_rows(
    x_pos, y_pos, theta, plane_nx, plane_ny, plane_d, foot_min, foot_max, half_width
):
    """支持関数の行: footprint 長方形が半空間 n . p <= d に収まること.

    後軸を基準点とした長方形の方向 n への支持関数は

        n . p_base + alpha (n . forward) + (W/2) |n . left|

    で、縦方向の max は**縦端ごとに 1 行出す**ことで消してある (各行が状態について滑らかになり、
    2 行の組は長方形について厳密)。
    """
    forward_x = cos(theta)
    forward_y = sin(theta)
    rows = []
    for j in range(len(plane_d)):
        normal_dot_position = plane_nx[j] * x_pos + plane_ny[j] * y_pos
        normal_dot_forward = plane_nx[j] * forward_x + plane_ny[j] * forward_y
        normal_dot_left = -plane_nx[j] * forward_y + plane_ny[j] * forward_x
        lateral_support = half_width * sqrt(
            normal_dot_left * normal_dot_left + LATERAL_SUPPORT_EPSILON
        )
        for longitudinal in (foot_min, foot_max):
            rows.append(
                normal_dot_position
                + longitudinal * normal_dot_forward
                + lateral_support
                - plane_d[j]
            )
    return rows


def time_bicycle_model():
    """モデル本体・ステージ行・終端行を作る.

    パラメータ p = [dt, wheel_base,
                    footprint_longitudinal_min, footprint_longitudinal_max, footprint_half_width,
                    (nx, ny, d) x NUM_PLANES]
    """
    model = types.SimpleNamespace()

    x_pos = SX.sym("px")
    y_pos = SX.sym("py")
    theta = SX.sym("theta")
    kappa = SX.sym("kappa")
    velocity = SX.sym("v")
    accel = SX.sym("a")
    x = vertcat(x_pos, y_pos, theta, kappa, velocity, accel)

    curvature_rate = SX.sym("w")
    jerk = SX.sym("j")
    u = vertcat(curvature_rate, jerk)

    time_step = SX.sym("dt")
    wheel_base = SX.sym("wheel_base")
    foot_min = SX.sym("footprint_longitudinal_min")
    foot_max = SX.sym("footprint_longitudinal_max")
    half_width = SX.sym("footprint_half_width")
    plane_nx = [SX.sym("plane_nx_{}".format(j)) for j in range(NUM_PLANES)]
    plane_ny = [SX.sym("plane_ny_{}".format(j)) for j in range(NUM_PLANES)]
    plane_d = [SX.sym("plane_d_{}".format(j)) for j in range(NUM_PLANES)]
    p = vertcat(
        time_step,
        wheel_base,
        foot_min,
        foot_max,
        half_width,
        *[c for j in range(NUM_PLANES) for c in (plane_nx[j], plane_ny[j], plane_d[j])],
    )

    xdot = vertcat(
        SX.sym("pxdot"),
        SX.sym("pydot"),
        SX.sym("thetadot"),
        SX.sym("kappadot"),
        SX.sym("vdot"),
        SX.sym("adot"),
    )

    # 1 ステージ = 正規化 1 単位なので d/dtau = dt * d/dt。
    f_expl = time_step * vertcat(
        velocity * cos(theta),
        velocity * sin(theta),
        velocity * kappa,
        curvature_rate,
        accel,
        jerk,
    )

    # ステアレート ddelta/dt = L w / (1 + (L kappa)^2)。分母が常に正で行が滑らかなので、
    # ± 2 行に割らず**両側 1 行**として書く。
    steer_rate = wheel_base * curvature_rate / (1.0 + (kappa * wheel_base) ** 2)
    lateral_accel = velocity * velocity * kappa

    corridor = _corridor_rows(
        x_pos, y_pos, theta, plane_nx, plane_ny, plane_d, foot_min, foot_max, half_width
    )

    model.con_h_expr = vertcat(
        steer_rate,  # A: ハード、±steer_rate_hard
        steer_rate,  # C: スラック、±steer_rate_nominal
        lateral_accel,  # C: スラック、±lateral_accel_nominal
        accel,  # C: スラック、[a_nom_min, a_nom_max]
        jerk,  # C: スラック、±jerk_nominal
        velocity,  # B: スラック、<= 区間速度上限
        velocity,  # C: スラック、<= ノミナル速度
        *corridor,  # B: スラック (rho_B)、uh = 0
    )
    # 終端は状態だけの部分集合。S5 §3.2 が終端にも B 行を課すのは、この定式化の終端が
    # 参照点に pin されず自由で、課さないとホライゾン末尾がコリドー外へ逃げるため。
    model.con_h_expr_e = vertcat(
        lateral_accel,
        accel,
        velocity,
        velocity,
        *corridor,
    )

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.name = MODEL_NAME

    model.idx_px = 0
    model.idx_py = 1
    model.idx_theta = 2
    model.idx_kappa = 3
    model.idx_v = 4
    model.idx_a = 5
    model.idx_w = 0
    model.idx_j = 1
    return model
