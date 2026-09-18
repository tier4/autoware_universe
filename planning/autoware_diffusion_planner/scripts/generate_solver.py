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

"""Generate the acados OCP solver used by the diffusion planner trajectory optimization.

Invoked by CMake at build time with CWD set to the build directory; the generated C code
is exported to ./c_generated_code/ and never touches the source tree.

The horizon must stay aligned with the model output: N = 80 steps of dt = 0.1 s (OUTPUT_T
and PREDICTION_TIME_STEP_S on the C++ side). Cost weights and constraint bounds baked here
are only defaults; the C++ wrapper overrides them at runtime from ROS parameters.
"""

from acados_template import AcadosModel
from acados_template import AcadosOcp
from acados_template import AcadosOcpSolver
import numpy as np
from vehicle_model import kinematic_bicycle_model

HORIZON_N = 80
HORIZON_TF_S = 8.0

# Default cost weights (see config/diffusion_planner.param.yaml for the runtime values).
# Velocity and steering angle carry no tracking weight: the model outputs positions and
# headings only, so no reference exists for them. The velocity profile emerges from the
# time-indexed position tracking and the input regularization/bounds.
# Note: the C++ wrapper overwrites W every solve with a position block rotated to the
# per-stage reference heading (separate longitudinal/lateral weights); the isotropic
# position weight baked here is only a placeholder.
DEFAULT_WEIGHT_POSITION = 0.5
DEFAULT_WEIGHT_YAW = 0.05
DEFAULT_WEIGHT_VELOCITY = 0.0
DEFAULT_WEIGHT_STEERING = 0.0
DEFAULT_WEIGHT_ACCELERATION = 0.1
DEFAULT_WEIGHT_STEERING_RATE = 10.0
DEFAULT_TERMINAL_WEIGHT_SCALE = 2.5

DEFAULT_A_LAT_MAX_MPS2 = 3.0
DEFAULT_WHEELBASE_M = 2.75

SQP_MAX_ITER = 50
SQP_TOL = 1e-4


def build_ocp(N=HORIZON_N, Tf=HORIZON_TF_S):
    ocp = AcadosOcp()

    model = kinematic_bicycle_model()

    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x = model.x
    model_ac.xdot = model.xdot
    model_ac.u = model.u
    model_ac.p = model.p
    model_ac.con_h_expr = model.con_h_expr
    model_ac.name = model.name
    ocp.model = model_ac

    ocp.code_export_directory = "c_generated_code"

    nx = model.x.rows()  # 5: x, y, psi, v, delta
    nu = model.u.rows()  # 2: a, delta_rate
    ny = nx + nu
    ny_e = nx

    ocp.solver_options.N_horizon = N

    q_diag = np.array(
        [
            DEFAULT_WEIGHT_POSITION,
            DEFAULT_WEIGHT_POSITION,
            DEFAULT_WEIGHT_YAW,
            DEFAULT_WEIGHT_VELOCITY,
            DEFAULT_WEIGHT_STEERING,
        ]
    )
    r_diag = np.array([DEFAULT_WEIGHT_ACCELERATION, DEFAULT_WEIGHT_STEERING_RATE])

    # acados scales stage costs by the step length dt = Tf / N; multiply by N / Tf so the
    # configured weights keep their intuitive per-sample magnitude (same convention on the
    # C++ side when overriding W at runtime).
    unscale = N / Tf
    ocp.cost.cost_type = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W = unscale * np.diag(np.concatenate([q_diag, r_diag]))
    ocp.cost.W_e = (DEFAULT_TERMINAL_WEIGHT_SCALE / unscale) * np.diag(q_diag)

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[nx:, :] = np.eye(nu)
    ocp.cost.Vu = Vu

    ocp.cost.Vx_e = np.eye(ny_e, nx)

    ocp.cost.yref = np.zeros(ny)
    ocp.cost.yref_e = np.zeros(ny_e)

    # Input box constraints
    ocp.constraints.lbu = np.array([model.a_min, -model.delta_rate_max])
    ocp.constraints.ubu = np.array([model.a_max, model.delta_rate_max])
    ocp.constraints.idxbu = np.array([0, 1])

    # State box constraints on v and delta (stages 1..N-1 and terminal stage)
    ocp.constraints.idxbx = np.array([3, 4])
    ocp.constraints.lbx = np.array([model.v_min, -model.delta_max])
    ocp.constraints.ubx = np.array([model.v_max, model.delta_max])
    ocp.constraints.idxbx_e = np.array([3, 4])
    ocp.constraints.lbx_e = np.array([model.v_min, -model.delta_max])
    ocp.constraints.ubx_e = np.array([model.v_max, model.delta_max])

    # Initial state equality constraint (stage-0 lbx == ubx, set per solve)
    ocp.constraints.x0 = np.zeros(nx)

    # Soft lateral acceleration constraint: |v^2 * tan(delta) / wheelbase| <= a_lat_max.
    # Softened with slacks so a violating initial state cannot make the OCP infeasible.
    ocp.constraints.lh = np.array([-DEFAULT_A_LAT_MAX_MPS2])
    ocp.constraints.uh = np.array([DEFAULT_A_LAT_MAX_MPS2])
    ocp.constraints.idxsh = np.array([0])
    ocp.cost.zl = np.array([1.0e2])
    ocp.cost.zu = np.array([1.0e2])
    ocp.cost.Zl = np.array([1.0e3])
    ocp.cost.Zu = np.array([1.0e3])

    ocp.parameter_values = np.array([DEFAULT_WHEELBASE_M])

    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_max_iter = SQP_MAX_ITER
    ocp.solver_options.tol = SQP_TOL

    return ocp


def create_solver(N=HORIZON_N, Tf=HORIZON_TF_S):
    """Build a solver usable from Python (offline tuning / visualization)."""
    ocp = build_ocp(N, Tf)
    return AcadosOcpSolver(ocp, json_file="acados_ocp.json")


def main():
    ocp = build_ocp()
    AcadosOcpSolver.generate(ocp, json_file="acados_ocp.json")


if __name__ == "__main__":
    main()
