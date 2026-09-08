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

"""The time-parameterized kinematic model.

    state   x = [px, py, theta, kappa, v, a]
    input   u = [w, j]                        w = dkappa/dt, j = da/dt
    f       px' = v cos(theta), py' = v sin(theta), theta' = v kappa,
            kappa' = w, v' = a, a' = j

Carrying kappa in the state is what this formulation is about: the steer angle
delta = atan(L kappa) and the steer rate ddelta/dt = L w / (1 + (L kappa)^2) become functions of
the state and the input, so they can be written **as constraints** rather than checked afterwards,
which the SSC formulation structurally cannot. Nothing divides by v, so v = 0 is not a singularity.

The step dt is a **model parameter**, not the step of the solver: the integrator advances one
normalized unit per stage (tf == N) and the C++ side writes the step of the rough plan into dt
every cycle. That is what lets the planning grid change without regenerating the solver, which the
space-parameterized acados_mpt_optimizer could not do.
"""

import types

from casadi import SX
from casadi import cos
from casadi import sin
from casadi import sqrt
from casadi import vertcat

MODEL_NAME = "rbp_nlp_time"

# Corridor half spaces per stage: the two lateral ones plus the forward cut. The lateral ones are
# faces of the semantic cube the stage belongs to, and the forward cut comes straight from the
# occupancies and stop lines in effect ahead at that time. It is fixed at generation time and must
# match NUM_PLANES on the C++ side.
#
# There is **no rear cut**: v >= 0 keeps the ego from reversing and there is nothing behind to
# protect, stop lines and lead vehicles being held by the forward cut. Besides, the s0 of the first
# cube is clipped at the start of the path, so a rear cut would structurally push the rear of the
# footprint outside and let its slack drive the solution forward.
NUM_PLANES = 3
# Smoothing of the |n . w| in the support function. sqrt(y^2 + eps) >= |y|, so it always errs on
# the **strict** side and a point feasible for the smoothed row is feasible for the exact one.
LATERAL_SUPPORT_EPSILON = 1.0e-6

# --- row layout of con_h_expr; keep it in the same order as the C++ side ---
# Every row from 1 on carries a slack. Row 0 is the only hard nonlinear row, and w = 0 always
# satisfies it, so it cannot empty the feasible set.
ROW_STEER_RATE_HARD = 0
ROW_STEER_RATE_NOMINAL = 1
ROW_LATERAL_ACCEL = 2
ROW_ACCEL_COMFORT = 3
ROW_JERK_COMFORT = 4
ROW_VELOCITY_SECTION = 5
ROW_VELOCITY_NOMINAL = 6
# The corridor contributes one block of safety rows (no intrusion, uh = 0) and no comfort margin
# row: the clearance from a face is already in the margin_m of the cube, and taking it twice is
# structurally violated at the width of a lane.
NUM_CORRIDOR_ROWS = 2 * NUM_PLANES
FIRST_CORRIDOR_ROW_SAFETY = 7
NUM_ROWS = FIRST_CORRIDOR_ROW_SAFETY + NUM_CORRIDOR_ROWS

# The last stage has no input, so every row reading w or j is structurally absent there; what
# remains are the rows over the states alone.
ROW_E_LATERAL_ACCEL = 0
ROW_E_ACCEL_COMFORT = 1
ROW_E_VELOCITY_SECTION = 2
ROW_E_VELOCITY_NOMINAL = 3
FIRST_CORRIDOR_ROW_E_SAFETY = 4
NUM_ROWS_E = FIRST_CORRIDOR_ROW_E_SAFETY + NUM_CORRIDOR_ROWS


def _corridor_rows(
    x_pos, y_pos, theta, plane_nx, plane_ny, plane_d, foot_min, foot_max, half_width
):
    """Support function rows: the footprint rectangle stays inside the half space n . p <= d.

    The support function of the rectangle, anchored at the rear axle, in the direction n is

        n . p_base + alpha (n . forward) + (W/2) |n . left|

    The maximum over the longitudinal extent is removed by emitting **one row per longitudinal
    end**, which keeps each row smooth in the state while the pair is exact for the rectangle.
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
    """Builds the model, the stage rows and the terminal rows.

    The parameters are p = [dt, wheel_base,
                            footprint_longitudinal_min, footprint_longitudinal_max,
                            footprint_half_width, (nx, ny, d) x NUM_PLANES]
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

    # One stage is one normalized unit, hence d/dtau = dt * d/dt.
    f_expl = time_step * vertcat(
        velocity * cos(theta),
        velocity * sin(theta),
        velocity * kappa,
        curvature_rate,
        accel,
        jerk,
    )

    # The steer rate ddelta/dt = L w / (1 + (L kappa)^2). Its denominator is always positive and
    # the row is smooth, so it is written as a single two-sided row rather than split in two.
    steer_rate = wheel_base * curvature_rate / (1.0 + (kappa * wheel_base) ** 2)
    lateral_accel = velocity * velocity * kappa

    corridor = _corridor_rows(
        x_pos, y_pos, theta, plane_nx, plane_ny, plane_d, foot_min, foot_max, half_width
    )

    model.con_h_expr = vertcat(
        steer_rate,  # hard, +-steer_rate_hard
        steer_rate,  # comfort, slacked, +-steer_rate_nominal
        lateral_accel,  # comfort, slacked, +-lateral_accel_nominal
        accel,  # comfort, slacked, [a_nom_min, a_nom_max]
        jerk,  # comfort, slacked, +-jerk_nominal
        velocity,  # safety, slacked, <= the interval speed limit
        velocity,  # comfort, slacked, <= the nominal speed
        *corridor,  # safety, slacked, uh = 0
    )
    # The terminal rows are the subset over the states alone. The safety rows are imposed there
    # too because the last stage is free rather than pinned to a reference point, and without them
    # the end of the horizon escapes the corridor.
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
