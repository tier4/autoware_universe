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
State x = [s, v, a, eY, ePsi, steer], input u = [u_cmd, delta_cmd].
Longitudinal: ṡ=v, v̇=a, τ_equiv·ȧ = u_cmd - a.
Steering: τ_steer·steeṙ = delta_cmd - steer.
Lateral: deY/dt = v·(deY/ds), dePsi/dt = v·(dePsi/ds) using curvature (eY, ePsi, steer).

Delay (dead time τ) is not included in this model. The dynamics use u_cmd at current
time; in a full FOPDT formulation one would have u_cmd(t-τ). Delay can be handled in
the MPC description file (OCP/solver wrapper) instead, e.g. by delay compensation:
predict state forward by delay_compensation_time and pass as x0, and shift the applied
input so the first portion of the horizon covers the delay. See longitudinal_inferred_model.md
and the path_tracking_mpc_spatial_with_body_points-style OCP setup (e.g. in
autoware_path_optimizer) for reference.
"""

import types

from casadi import SX
from casadi import atan
from casadi import cos
from casadi import tan
from casadi import vertcat


def combined_longitudinal_lateral_model(tau_equiv: float = 1.5, steer_tau: float = 0.27):
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "combined_longitudinal_lateral"

    # states: longitudinal (s, v, a) + lateral (eY, ePsi) + steering actuator state
    s = SX.sym("s")
    v = SX.sym("v")
    a = SX.sym("a")
    eY = SX.sym("eY")
    ePsi = SX.sym("ePsi")
    steer = SX.sym("steer")

    x = vertcat(s, v, a, eY, ePsi, steer)

    # control: acceleration command + steering command
    u_cmd = SX.sym("u_cmd")
    delta_cmd = SX.sym("delta_cmd")
    u = vertcat(u_cmd, delta_cmd)

    # parameters: tau_equiv, kappa_ref, lf, lr, steer_tau (v is state)
    p_tau = SX.sym("tau_equiv")
    kappa_ref_s = SX.sym("kappa_ref")
    lf = SX.sym("lf")
    lr = SX.sym("lr")
    p_steer_tau = SX.sym("steer_tau")

    p = vertcat(p_tau, kappa_ref_s, lf, lr, p_steer_tau)

    # xdot
    s_dot = SX.sym("s_dot")
    v_dot = SX.sym("v_dot")
    a_dot = SX.sym("a_dot")
    eYdot = SX.sym("eYdot")
    ePsidot = SX.sym("ePsidot")
    steerdot = SX.sym("steer_dot")

    xdot = vertcat(s_dot, v_dot, a_dot, eYdot, ePsidot, steerdot)

    # longitudinal: ṡ = v, v̇ = a, τ_equiv·ȧ = u_cmd - a (no dead time τ here; handle in MPC/OCP)
    f_lon = vertcat(v, a, (u_cmd - a) / p_tau)

    # lateral: curvature-based, d/dt = v * (d/ds)
    beta = atan(lr * tan(steer) / (lf + lr))
    kappa = cos(beta) * tan(steer) / (lf + lr)
    deY_ds = tan(ePsi + beta) * (1 - kappa_ref_s * eY)
    dePsi_ds = kappa * (1 - kappa_ref_s * eY) / cos(ePsi) - kappa_ref_s
    f_lat = vertcat(v * deY_ds, v * dePsi_ds, (delta_cmd - steer) / p_steer_tau)

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
