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

"""Kinematic bicycle model with steering-angle state for trajectory optimization.

States  (nx=5): x [m], y [m], psi [rad], v [m/s], delta [rad]
Inputs  (nu=2): a [m/s^2] (longitudinal acceleration), delta_rate [rad/s] (steering angle rate)
Params  (np=1): wheelbase [m]

Dynamics:
    x_dot     = v * cos(psi)
    y_dot     = v * sin(psi)
    psi_dot   = v * tan(delta) / wheelbase
    v_dot     = a
    delta_dot = delta_rate

Nonlinear constraint expression (soft-bounded in the OCP):
    a_lat = v^2 * tan(delta) / wheelbase
"""

from types import SimpleNamespace

from casadi import SX
from casadi import cos
from casadi import sin
from casadi import tan
from casadi import vertcat

MODEL_NAME = "ml_planner_optimizer"

# Default input bounds baked into the generated solver.
# The C++ wrapper overrides them at runtime from ROS parameters.
DEFAULT_A_MIN_MPS2 = -4.0
DEFAULT_A_MAX_MPS2 = 3.0
DEFAULT_DELTA_RATE_MAX_RPS = 1.0

# Default state bounds (stages 1..N). Overridden at runtime as well.
DEFAULT_V_MIN_MPS = 0.0
DEFAULT_V_MAX_MPS = 30.0
DEFAULT_DELTA_MAX_RAD = 0.7


def kinematic_bicycle_model():
    """Build the symbolic model consumed by generate_solver.py."""
    # States
    x = SX.sym("x")
    y = SX.sym("y")
    psi = SX.sym("psi")
    v = SX.sym("v")
    delta = SX.sym("delta")
    states = vertcat(x, y, psi, v, delta)

    # Inputs
    a = SX.sym("a")
    delta_rate = SX.sym("delta_rate")
    inputs = vertcat(a, delta_rate)

    # Online parameters
    wheelbase = SX.sym("wheelbase")
    params = vertcat(wheelbase)

    # Explicit dynamics
    f_expl = vertcat(
        v * cos(psi),
        v * sin(psi),
        v * tan(delta) / wheelbase,
        a,
        delta_rate,
    )

    xdot = vertcat(
        SX.sym("x_dot"),
        SX.sym("y_dot"),
        SX.sym("psi_dot"),
        SX.sym("v_dot"),
        SX.sym("delta_dot"),
    )

    model = SimpleNamespace()
    model.name = MODEL_NAME
    model.x = states
    model.xdot = xdot
    model.u = inputs
    model.p = params
    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl

    # Lateral acceleration for the soft nonlinear constraint
    model.con_h_expr = v * v * tan(delta) / wheelbase

    model.a_min = DEFAULT_A_MIN_MPS2
    model.a_max = DEFAULT_A_MAX_MPS2
    model.delta_rate_max = DEFAULT_DELTA_RATE_MAX_RPS
    model.v_min = DEFAULT_V_MIN_MPS
    model.v_max = DEFAULT_V_MAX_MPS
    model.delta_max = DEFAULT_DELTA_MAX_RAD

    return model
