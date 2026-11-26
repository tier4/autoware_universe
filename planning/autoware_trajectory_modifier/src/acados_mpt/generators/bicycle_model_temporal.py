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


def bicycle_model_temporal():
    """First-order Dubins bicycle matching MPPI (without input-delay FIFOs).

    Same continuous plant as ``FirstOrderDubinsBicycle``:
      a_dot     = (a_cmd - a) / tau_a
      delta_dot = (delta_cmd - delta) / tau_d   (|delta_dot| <= max_steer_rate via h)
      v_dot     = a
      psi_dot   = (v / L) * tan(delta)
      x_dot,y_dot = v * [cos(psi), sin(psi)]

    States: x, y, psi, v, a, delta
    Controls: a_cmd, delta_cmd
    Parameters: lf, lr, tau_a, tau_d  with L = lf + lr (= wheel_base).
    """
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "kinematic_bicycle_temporal"

    x_pos = SX.sym("x")
    y_pos = SX.sym("y")
    psi = SX.sym("psi")
    v = SX.sym("v")
    a = SX.sym("a")
    delta = SX.sym("delta")
    x = vertcat(x_pos, y_pos, psi, v, a, delta)

    lf = SX.sym("lf")
    lr = SX.sym("lr")
    tau_a = SX.sym("tau_a")
    tau_d = SX.sym("tau_d")
    p = vertcat(lf, lr, tau_a, tau_d)

    a_cmd = SX.sym("a_cmd")
    delta_cmd = SX.sym("delta_cmd")
    u = vertcat(a_cmd, delta_cmd)

    xdot_sym = SX.sym("xdot")
    ydot_sym = SX.sym("ydot")
    psidot_sym = SX.sym("psidot")
    vdot_sym = SX.sym("vdot")
    adot_sym = SX.sym("adot")
    deltadot_sym = SX.sym("deltadot")
    xdot = vertcat(xdot_sym, ydot_sym, psidot_sym, vdot_sym, adot_sym, deltadot_sym)

    L = lf + lr
    x_dot = v * cos(psi)
    y_dot = v * sin(psi)
    psi_dot = (v / L) * tan(delta)
    v_dot = a
    a_dot = (a_cmd - a) / tau_a
    delta_dot = (delta_cmd - delta) / tau_d

    f_expl = vertcat(x_dot, y_dot, psi_dot, v_dot, a_dot, delta_dot)

    # Steer-rate residual: |(delta_cmd - delta)/tau_d| <= max_steer_rate
    # Intermediate stages use con_h_expr; stage 0 needs con_h_expr_0 (acados nh vs nh_0).
    steer_rate = (delta_cmd - delta) / tau_d
    model.con_h_expr = vertcat(steer_rate)
    model.con_h_expr_0 = vertcat(steer_rate)

    model.a_min = -3.5
    model.a_max = 3.5
    model.delta_min = -0.7
    model.delta_max = 0.7
    model.delta_cmd_min = -0.7
    model.delta_cmd_max = 0.7
    model.max_steer_rate = 3.0

    params = types.SimpleNamespace()
    params.lf = lf
    params.lr = lr
    params.tau_a = tau_a
    params.tau_d = tau_d
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.name = model_name
    model.params = params
    return model, constraint
