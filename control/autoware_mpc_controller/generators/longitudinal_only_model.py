# Copyright 2025 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use it except in compliance with the License.
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
Longitudinal-only FOPDT model: state x = [s, v, a], input u = [u_cmd].
Dynamics: ṡ = v, v̇ = a, τ·ȧ = u_cmd - a.
Parameter: p = [tau_equiv].
Same longitudinal dynamics as the combined model, no lateral or steering.
"""

import types

from casadi import SX
from casadi import vertcat


def longitudinal_only_model(tau_equiv: float = 0.1):
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "longitudinal_only"

    s = SX.sym("s")
    v = SX.sym("v")
    a = SX.sym("a")
    x = vertcat(s, v, a)

    u_cmd = SX.sym("u_cmd")
    u = vertcat(u_cmd)

    p_tau = SX.sym("tau_equiv")
    p = vertcat(p_tau)

    s_dot = SX.sym("s_dot")
    v_dot = SX.sym("v_dot")
    a_dot = SX.sym("a_dot")
    xdot = vertcat(s_dot, v_dot, a_dot)

    # ṡ = v, v̇ = a, τ·ȧ = u_cmd - a
    f_expl = vertcat(v, a, (u_cmd - a) / p_tau)
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.name = model_name

    model.u_cmd_min = -3.0
    model.u_cmd_max = 2.0
    return model, constraint
