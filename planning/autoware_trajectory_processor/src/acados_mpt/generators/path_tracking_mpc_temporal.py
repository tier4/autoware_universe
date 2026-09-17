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
from generators.bicycle_model_temporal import bicycle_model_temporal
import numpy as np


class PathTrackingMPCTemporal:
    """Temporal MPC: first-order Dubins (MPPI plant without delay FIFOs).

    States x,y,psi,v,a,delta; inputs a_cmd, delta_cmd. Set yref / yref_e per solve.
    """

    def __init__(self, Tf, N, build=True, generate=True):
        self.Tf = Tf
        self.N = N

        self.constraint, self.model, self.acados_solver = self.acados_settings(build, generate)

    def acados_settings(self, build=True, generate=True):
        ocp = AcadosOcp()

        model, constraint = bicycle_model_temporal()

        model_ac = AcadosModel()
        model_ac.f_impl_expr = model.f_impl_expr
        model_ac.f_expl_expr = model.f_expl_expr
        model_ac.x = model.x
        model_ac.xdot = model.xdot
        model_ac.u = model.u
        model_ac.p = model.p
        if hasattr(model, "con_h_expr"):
            model_ac.con_h_expr = model.con_h_expr
        if hasattr(model, "con_h_expr_0"):
            model_ac.con_h_expr_0 = model.con_h_expr_0
        model_ac.name = model.name
        ocp.model = model_ac

        ocp.code_export_directory = "c_generated_code"

        nx = model.x.rows()
        nu = model.u.rows()
        ny_st = nx + nu
        ny_e = nx

        ocp.solver_options.N_horizon = self.N

        # Q: path-frame lon/lat, psi, v, a, delta (qa=qdelta=0 by default)
        Q = np.diag([0.0, 3.0e-1, 2.0e-2, 1.40e-1, 0.0, 0.0])
        R = np.diag([2.5e-2, 1.4e-1])  # a_cmd, delta_cmd
        Qe = 2.5 * Q

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"
        unscale = self.N / self.Tf

        # Stage residual [e_lon, e_lat, psi, v, a, delta, a_cmd, delta_cmd] once runtime sets Vx.
        ocp.cost.W = unscale * np.block(
            [[Q, np.zeros((Q.shape[0], R.shape[1]))], [np.zeros((R.shape[0], Q.shape[1])), R]]
        )
        ocp.cost.W_e = Qe / unscale

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

        # Command bounds
        ocp.constraints.lbu = np.array([model.a_min, model.delta_cmd_min])
        ocp.constraints.ubu = np.array([model.a_max, model.delta_cmd_max])
        ocp.constraints.idxbu = np.array([0, 1])

        # Applied actuator state bounds (indices 4=a, 5=delta)
        ocp.constraints.lbx = np.array([model.a_min, model.delta_min])
        ocp.constraints.ubx = np.array([model.a_max, model.delta_max])
        ocp.constraints.idxbx = np.array([4, 5])
        ocp.constraints.lbx_e = np.array([model.a_min, model.delta_min])
        ocp.constraints.ubx_e = np.array([model.a_max, model.delta_max])
        ocp.constraints.idxbx_e = np.array([4, 5])

        # Steer-rate path constraint (stage 0 + intermediate stages)
        ocp.constraints.lh = np.array([-model.max_steer_rate])
        ocp.constraints.uh = np.array([model.max_steer_rate])
        ocp.constraints.lh_0 = np.array([-model.max_steer_rate])
        ocp.constraints.uh_0 = np.array([model.max_steer_rate])

        ocp.constraints.x0 = np.zeros(nx)

        n_p = model.p.shape[0]
        ocp.parameter_values = np.zeros(n_p)
        ocp.parameter_values[0] = 1.0  # lf
        ocp.parameter_values[1] = 1.0  # lr
        ocp.parameter_values[2] = 0.15  # tau_a
        ocp.parameter_values[3] = 0.08  # tau_d

        ocp.solver_options.tf = self.Tf
        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"  # Forward Euler when num_stages=1
        ocp.solver_options.sim_method_num_stages = 1
        ocp.solver_options.num_steps = 1
        # Online-friendly early exit: prefer a short SQP budget over grinding to 1e-4.
        # Runtime can still override via AcadosInterface::setNlpSolverLimits.
        ocp.solver_options.nlp_solver_max_iter = 10
        ocp.solver_options.tol = 1e-6
        ocp.solver_options.nlp_solver_ext_qp_res = 0
        ocp.solver_options.print_level = 0

        if not build:
            AcadosOcpSolver.generate(ocp, json_file="acados_ocp.json")
            return constraint, model, None

        acados_solver = AcadosOcpSolver(
            ocp, json_file="acados_ocp.json", build=build, generate=generate
        )

        return constraint, model, acados_solver


def main():
    # CMake invokes this script with CWD = acados_mpt binary dir; exports to ./c_generated_code/
    # dt = Tf / N = 0.1 s (aligned with example_track_xyv default --dt)
    N = 80
    Tf = 8.0
    _ = PathTrackingMPCTemporal(Tf, N, build=False, generate=True)


if __name__ == "__main__":
    main()
