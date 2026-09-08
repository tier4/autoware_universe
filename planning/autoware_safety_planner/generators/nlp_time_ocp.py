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

"""Code generation of the acados OCP that NlpTrajectoryOptimizer drives.

**No number is decided here**: no bound, weight, reference or dt. Only the structure is fixed:

  N               the number of stages (kept equal to rough_planner.num_points - 1)
  which states and inputs carry a box
  which rows carry a slack
  the type of the cost (LINEAR_LS)

The three-level fallback works on this one generated solver by rewriting bounds: a row is disabled
by opening it to +-1e6 rather than by removing it. The solver is never rebuilt, so dropping a level
costs one more solve and nothing else.
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

# Number of stages, fixed at generation time; the C++ side reads the generated RBP_NLP_TIME_N back.
# The default matches the default grid of the rough planner (101 points, 0.1 s, i.e. T = 10 s).
DEFAULT_N = 100

# How far a disabled row is opened, which is what makes it always satisfied.
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

    # --- cost: LINEAR_LS over the states and the inputs; C++ overwrites the weights and the
    # reference every cycle ---
    # y   = [px, py, theta, kappa, v, a, w, j]   (the tracking and the smoothing terms)
    # y_e = [px, py, theta, kappa, v, a]         (stopping at the end is the weight on v_N and a_N)
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

    # --- box constraints, all of them hard and without a slack ---
    # The inputs are bounded by |w| <= w_max and |j| <= j_hard, the states in kappa, v and a.
    # They can be hard because the certificate, simulated forward from the conditioned initial
    # state, satisfies every one of them, so the feasible set is never empty.
    ocp.constraints.idxbu = np.arange(nu)
    ocp.constraints.lbu = -np.ones(nu)
    ocp.constraints.ubu = np.ones(nu)

    ocp.constraints.idxbx = np.array([model.idx_kappa, model.idx_v, model.idx_a])
    ocp.constraints.lbx = np.array([-1.0, 0.0, -1.0])
    ocp.constraints.ubx = np.array([1.0, 1.0, 1.0])

    # The initial state is pinned to the conditioned ego state.
    ocp.constraints.x0 = np.zeros(nx)

    # The last stage carries the same box on kappa, v and a. The position and the heading are left
    # free there, and the tracking and the terminal stop term pull them through the cost.
    ocp.constraints.idxbx_e = np.array([model.idx_kappa, model.idx_v, model.idx_a])
    ocp.constraints.lbx_e = np.array([-1.0, 0.0, -1.0])
    ocp.constraints.ubx_e = np.array([1.0, 1.0, 1.0])

    # --- nonlinear rows ---
    # Every row but row 0, the hard steer rate, carries a slack. The safety rows (the corridor and
    # the interval speed limits) and the comfort rows depend on the map, the perception and the
    # state the ego enters in, so making them hard would leave cycles infeasible with no iterate at
    # all. With a slack an iterate always comes back, and the independent verification on the C++
    # side decides whether it is acceptable: solved is not satisfied.
    #
    # The penalty is **quadratic only** (zl and zu stay 0); an L1 penalty wrecked the convergence.
    # C++ writes the coefficients Zl and Zu.
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

    # Only a placeholder; C++ writes the parameters of every stage each cycle.
    ocp.parameter_values = np.zeros(model.p.rows())

    # --- solver ---
    # One stage is one normalized unit; the physical time lives in the dt parameter, see the
    # docstring of time_bicycle_model.
    ocp.solver_options.tf = float(n_horizon)
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    # RK4, one step per stage. The certificate and the verification use the same discrete map.
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_max_iter = 50
    # What bounds the worst cycle is the number of QP iterations, not of SQP ones: a single QP was
    # measured at over 100 ms in the space-parameterized version.
    ocp.solver_options.qp_solver_iter_max = 20
    ocp.solver_options.hpipm_mode = "SPEED"
    # Feasibility is required to 1e-4 while optimality is only required to 1e-1.
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
