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
Lateral MPC using curvature-based bicycle dynamics in time.
Model: d/dt (eY, ePsi) = v * (d/ds terms), with v = ds/dt from longitudinal_fopdt_model.
Horizon is time Tf [s]; parameters p = [v, kappa_ref, lf, lr].
"""

from __future__ import annotations

try:
    from .bicycle_model_curvilinear import bicycle_model_curvilinear
except ImportError:
    from bicycle_model_curvilinear import bicycle_model_curvilinear

try:
    from acados_template import AcadosModel
    from acados_template import AcadosOcp
    from acados_template import AcadosOcpSolver
except ImportError as e:
    raise ImportError(
        "lateral_mpc_curvilinear requires acados_template and casadi."
    ) from e

import numpy as np


# Default wheelbase [m]
DEFAULT_LF = 1.0
DEFAULT_LR = 1.0
# Default speed [m/s] for parameter_values; v = ds/dt from longitudinal model
DEFAULT_V = 5.0


class LateralMPCCurvilinear:
    """
    Lateral path-tracking MPC with curvature-based dynamics in time.
    deY/dt = v*(deY/ds), dePsi/dt = v*(dePsi/ds); v = ds/dt from longitudinal model.
    Horizon is time Tf [s]. Parameters p = [v, kappa_ref, lf, lr].
    """

    def __init__(
        self,
        Tf: float,
        N: int,
        lf: float = DEFAULT_LF,
        lr: float = DEFAULT_LR,
        v_default: float = DEFAULT_V,
        build: bool = True,
        generate: bool = True,
        code_export_directory: str = "c_generated_code_lateral",
        json_file: str = "acados_ocp_lateral.json",
    ):
        self.Tf = Tf
        self.N = N
        self.lf = lf
        self.lr = lr
        self.v_default = v_default
        self.code_export_directory = code_export_directory
        self.json_file = json_file
        self.constraint, self.model, self.acados_solver = self._acados_settings(
            build, generate
        )

    def _acados_settings(self, build: bool = True, generate: bool = True):
        ocp = AcadosOcp()

        model, constraint = bicycle_model_curvilinear()

        model_ac = AcadosModel()
        model_ac.f_impl_expr = model.f_impl_expr
        model_ac.f_expl_expr = model.f_expl_expr
        model_ac.x = model.x
        model_ac.xdot = model.xdot
        model_ac.u = model.u
        model_ac.p = model.p
        model_ac.name = model.name
        ocp.model = model_ac

        ocp.code_export_directory = self.code_export_directory

        nx = model.x.rows()
        nu = model.u.rows()
        ny = nx + nu
        ny_e = nx

        ocp.solver_options.N_horizon = self.N

        # cost: track (eY, ePsi) to 0, penalize steering
        Q = np.diag([1e-2, 1e-1])
        R = np.eye(nu) * 2e-1
        Qe = 5.0 * Q

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"
        unscale = self.N / self.Tf
        ocp.cost.W = unscale * np.block(
            [
                [Q, np.zeros((Q.shape[0], R.shape[1]))],
                [np.zeros((R.shape[0], Q.shape[1])), R],
            ]
        )
        ocp.cost.W_e = Qe / unscale

        Vx = np.zeros((ny, nx))
        Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx = Vx

        Vu = np.zeros((ny, nu))
        Vu[ny - 1, 0] = 1.0
        ocp.cost.Vu = Vu

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e

        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)

        ocp.constraints.lbu = np.array([model.delta_min])
        ocp.constraints.ubu = np.array([model.delta_max])
        ocp.constraints.idxbu = np.array([0])

        ocp.constraints.x0 = np.zeros(nx)

        # p = [v, kappa_ref, lf, lr]; v = ds/dt from longitudinal model
        ocp.parameter_values = np.array([self.v_default, 0.0, self.lf, self.lr])

        # time horizon [s]
        ocp.solver_options.tf = self.Tf
        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        ocp.solver_options.nlp_solver_type = "SQP"
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.num_steps = 1
        ocp.solver_options.nlp_solver_max_iter = 20
        ocp.solver_options.tol = 1e-4

        if not build:
            AcadosOcpSolver.generate(ocp, json_file=self.json_file)
            return constraint, model, None

        acados_solver = AcadosOcpSolver(
            ocp,
            json_file=self.json_file,
            build=build,
            generate=generate,
        )
        return constraint, model, acados_solver


def main():
    Tf = 5.0
    N = 50
    LateralMPCCurvilinear(
        Tf, N,
        lf=1.0, lr=1.0,
        v_default=5.0,
        build=False,
        generate=True,
    )
    print("Lateral curvilinear MPC code generated.")


if __name__ == "__main__":
    main()
