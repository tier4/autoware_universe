# Copyright 2025 TIER IV, Inc.
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

"""
Combined longitudinal (FOPDT) + lateral (curvature-based bicycle) model in time.
State x = [s, v, a, eY, ePsi], input u = [u_cmd, delta].
Longitudinal: ṡ=v, v̇=a, τ_equiv·ȧ = u_cmd − a.
Lateral: deY/dt = v·(deY/ds), dePsi/dt = v·(dePsi/ds) using curvature (eY, ePsi, delta).
"""

import types

from casadi import SX
from casadi import atan
from casadi import cos
from casadi import tan
from casadi import vertcat


def combined_longitudinal_lateral_model(tau_equiv: float = 1.5):
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "combined_longitudinal_lateral"

    # states: longitudinal (s, v, a) + lateral (eY, ePsi)
    s = SX.sym("s")
    v = SX.sym("v")
    a = SX.sym("a")
    eY = SX.sym("eY")
    ePsi = SX.sym("ePsi")

    x = vertcat(s, v, a, eY, ePsi)

    # control: acceleration command + steering
    u_cmd = SX.sym("u_cmd")
    delta = SX.sym("delta")
    u = vertcat(u_cmd, delta)

    # parameters: tau_equiv, kappa_ref, lf, lr (v is state)
    p_tau = SX.sym("tau_equiv")
    kappa_ref_s = SX.sym("kappa_ref")
    lf = SX.sym("lf")
    lr = SX.sym("lr")

    p = vertcat(p_tau, kappa_ref_s, lf, lr)

    # xdot
    s_dot = SX.sym("s_dot")
    v_dot = SX.sym("v_dot")
    a_dot = SX.sym("a_dot")
    eYdot = SX.sym("eYdot")
    ePsidot = SX.sym("ePsidot")

    xdot = vertcat(s_dot, v_dot, a_dot, eYdot, ePsidot)

    # longitudinal: ṡ = v, v̇ = a, τ_equiv·ȧ = u_cmd − a
    f_lon = vertcat(v, a, (u_cmd - a) / p_tau)

    # lateral: curvature-based, d/dt = v * (d/ds)
    beta = atan(lr * tan(delta) / (lf + lr))
    kappa = cos(beta) * tan(delta) / (lf + lr)
    deY_ds = tan(ePsi + beta) * (1 - kappa_ref_s * eY)
    dePsi_ds = kappa * (1 - kappa_ref_s * eY) / cos(ePsi) - kappa_ref_s
    f_lat = vertcat(v * deY_ds, v * dePsi_ds)

    f_expl = vertcat(f_lon, f_lat)

    # bounds
    model.u_cmd_min = -3.0
    model.u_cmd_max = 2.0
    model.delta_min = -0.7
    model.delta_max = 0.7
    model.eY_min = -1.5
    model.eY_max = 1.5

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.name = model_name
    model.params = types.SimpleNamespace(lf=lf, lr=lr)
    return model, constraint
