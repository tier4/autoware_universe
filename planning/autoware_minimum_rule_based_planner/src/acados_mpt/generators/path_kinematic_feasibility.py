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

from acados_template import AcadosModel
from acados_template import AcadosOcp
from acados_template import AcadosOcpSolver
from generators.bicycle_model_time import bicycle_model_time
import numpy as np


class PathKinematicFeasibilityOCP:
    """Time-axis OCP for path-shape kinematic feasibility (acados, full SQP).

    States x = [X, Y, psi, delta]; inputs u = [u_delta = d delta / d t, v].
    LINEAR_LS cost on y_stage = [X, Y, psi, delta, u_delta, v] tracking the
    input reference path, where v is also tracked toward an input-derived
    nominal speed v_ref.

    Uses full SQP (multi-iteration) instead of SQP_RTI: SQP_RTI's
    single-iteration policy occasionally produces wavy outputs when the warm
    start is far from the optimum, so we trade a higher per-cycle cost for
    convergence.
    """

    def __init__(self, N, build=True, generate=True):
        # The time step dt is a runtime CasADi parameter, so the acados
        # integrator works in unit steps (Tf == N). The physical horizon is
        # dt(runtime) * N seconds.
        self.N = N
        self.Tf = float(N)

        self.constraint, self.model, self.acados_solver = self.acados_settings(build, generate)

    def acados_settings(self, build=True, generate=True):
        ocp = AcadosOcp()

        model, constraint = bicycle_model_time()

        model_ac = AcadosModel()
        model_ac.f_impl_expr = model.f_impl_expr
        model_ac.f_expl_expr = model.f_expl_expr
        model_ac.x = model.x
        model_ac.xdot = model.xdot
        model_ac.u = model.u
        model_ac.p = model.p
        model_ac.name = model.name
        ocp.model = model_ac

        ocp.code_export_directory = "c_generated_code"

        nx = model.x.rows()
        nu = model.u.rows()
        ny_st = nx + nu  # 6 = (X, Y, psi, delta, u_delta, v)
        ny_e = nx  # 4 = (X, Y, psi, delta)

        ocp.solver_options.N_horizon = self.N

        # ----- Cost -----
        # Endpoint-heavy weighting: stage cost is mostly regularisation on
        # (delta, u_delta, v_dev) so that the solver freely chooses a smooth
        # path. The endpoint weight anchors the final (X, Y, psi).
        # Q acts on (X, Y, psi, delta); R acts on (u_delta, v).
        # The v entry of yref is set per-stage from the input speed profile,
        # so a non-zero R[v,v] becomes a velocity-tracking weight.
        Q = np.diag([0.05, 0.05, 0.01, 0.5])
        R = np.diag([5.0, 0.1])
        Qe = np.diag([10.0, 10.0, 2.0, 0.1])

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        ocp.cost.W = np.block(
            [[Q, np.zeros((Q.shape[0], R.shape[1]))], [np.zeros((R.shape[0], Q.shape[1])), R]]
        )
        ocp.cost.W_e = Qe

        Vx = np.zeros((ny_st, nx))
        Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx = Vx

        Vu = np.zeros((ny_st, nu))
        Vu[nx : nx + nu, :] = np.eye(nu)
        ocp.cost.Vu = Vu

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e

        ocp.cost.yref = np.zeros(ny_st)
        ocp.cost.yref_e = np.zeros(ny_e)

        # ----- Constraints -----
        # Input box: [u_delta, v] each independently bounded.
        ocp.constraints.lbu = np.array([-model.u_delta_max, model.v_min])
        ocp.constraints.ubu = np.array([model.u_delta_max, model.v_max])
        ocp.constraints.idxbu = np.array([0, 1])

        # State box on delta only (index 3 in x = [X, Y, psi, delta]).
        # |delta| <= delta_max -> |kappa| = |tan(delta)/L| is bounded.
        ocp.constraints.lbx = np.array([-model.delta_max])
        ocp.constraints.ubx = np.array([model.delta_max])
        ocp.constraints.idxbx = np.array([3])

        ocp.constraints.lbx_e = np.array([-model.delta_max])
        ocp.constraints.ubx_e = np.array([model.delta_max])
        ocp.constraints.idxbx_e = np.array([3])

        # Initial state box (full state, fixed per solve via constraints_set(0, "lbx"/"ubx", ...))
        ocp.constraints.x0 = np.zeros(nx)

        # ----- Parameters -----
        # p = [L, dt]; defaults are placeholders. C++ side updates each solve.
        n_p = model.p.shape[0]
        ocp.parameter_values = np.zeros(n_p)
        ocp.parameter_values[0] = 2.0  # L wheelbase
        ocp.parameter_values[1] = 0.1  # dt (seconds per stage)

        # ----- Solver options -----
        ocp.solver_options.tf = self.Tf
        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP"
        # Cap SQP iterations tightly to bound per-cycle latency. The C++ side
        # accepts approximate (status == 2) solutions when the KKT residual is
        # small enough, so a hard cap of 5 trades full convergence for a
        # predictable solve time.
        ocp.solver_options.nlp_solver_max_iter = 5
        # 1e-4 is overkill for path output and was unreachable on tight curves
        # (the bilinear v·cos/sin terms make the QP step ill-conditioned, so
        # the iterator stalled around KKT ≈ 1e-2). 1e-3 still gives good
        # geometric quality.
        ocp.solver_options.tol = 1e-3
        # Levenberg-Marquardt regularisation stabilises the Gauss-Newton step
        # on the bilinear v·cos(ψ) / v·sin(ψ) / v·tan(δ) terms — without it,
        # full SQP repeatedly stalls on tight curves where SQP_RTI's
        # single-step policy used to "muddle through".
        ocp.solver_options.levenberg_marquardt = 1e-3
        # Merit-function backtracking globalises the SQP step so the iterator
        # is robust to warm starts that are far from the optimum.
        ocp.solver_options.globalization = "MERIT_BACKTRACKING"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.num_steps = 1

        if not build:
            AcadosOcpSolver.generate(ocp, json_file="acados_ocp_time.json")
            return constraint, model, None

        acados_solver = AcadosOcpSolver(
            ocp, json_file="acados_ocp_time.json", build=build, generate=generate
        )
        return constraint, model, acados_solver


def main():
    # CMake invokes this script with CWD = acados_mpt binary dir; exports to ./c_generated_code/
    # N = 40 stages. The physical step dt is a runtime parameter, so the
    # horizon is dt_runtime * N seconds (default dt = 0.1 s -> 4 s).
    N = 40
    _ = PathKinematicFeasibilityOCP(N, build=False, generate=True)


if __name__ == "__main__":
    main()
