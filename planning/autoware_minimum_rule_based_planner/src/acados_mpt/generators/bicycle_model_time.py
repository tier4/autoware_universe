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

import types

from casadi import SX
from casadi import cos
from casadi import sin
from casadi import tan
from casadi import vertcat


def bicycle_model_time():
    """Kinematic bicycle in time, with the per-stage step `dt` as a parameter.

    States: X, Y, psi, delta (world pose + front steering angle as a state).
    Controls: u_delta = d delta / d t (steering rate)
              v       = longitudinal velocity (free input)
    Parameters: L = wheelbase, dt = physical time covered per stage.

    Internally acados integrates with a unit step (Tf = N), and we scale the
    physical dynamics by dt so that one acados step corresponds to dt seconds
    of physical time:

        dX/dtau     = dt * v * cos(psi)
        dY/dtau     = dt * v * sin(psi)
        dpsi/dtau   = dt * v * tan(delta) / L
        ddelta/dtau = dt * u_delta

    Treating v as a free input (instead of fixing the per-stage arc length)
    lets the solver choose the actual arc-length traversed, which is what
    enables the optimizer to land on the target pose when the input path
    needs significant reshaping.
    """
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "kinematic_bicycle_time"

    X = SX.sym("X")
    Y = SX.sym("Y")
    psi = SX.sym("psi")
    delta = SX.sym("delta")
    x = vertcat(X, Y, psi, delta)

    L = SX.sym("L")
    dt = SX.sym("dt")
    p = vertcat(L, dt)

    u_delta = SX.sym("u_delta")  # d delta / d t (rad/s)
    v = SX.sym("v")  # longitudinal velocity (m/s)
    u_vec = vertcat(u_delta, v)

    Xdot = SX.sym("Xdot")
    Ydot = SX.sym("Ydot")
    psidot = SX.sym("psidot")
    deltadot = SX.sym("deltadot")
    xdot = vertcat(Xdot, Ydot, psidot, deltadot)

    f_expl = vertcat(
        dt * v * cos(psi),
        dt * v * sin(psi),
        dt * v * tan(delta) / L,
        dt * u_delta,
    )

    # Box-bound defaults; the C++ side overrides these per solve via constraints_set.
    model.delta_max = 0.7  # rad
    model.u_delta_max = 1.0  # rad/s
    model.v_min = 0.0  # m/s (forward-only by default; allow zero)
    model.v_max = 20.0  # m/s

    params = types.SimpleNamespace()
    params.L = L
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u_vec
    model.p = p
    model.name = model_name
    model.params = params
    return model, constraint
