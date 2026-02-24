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
Longitudinal MPC using the FOPDT model (path_tracking_mpc_spatial_with_body_points-style).

Builds an acados OCP for longitudinal control: state (s, v, a), input (u_cmd),
cost on tracking (s, v, a) and input, and acceleration bounds.
"""

from __future__ import annotations

try:
    from .longitudinal_fopdt_model import longitudinal_fopdt_model
except ImportError:
    from longitudinal_fopdt_model import longitudinal_fopdt_model

try:
    from acados_template import AcadosModel
    from acados_template import AcadosOcp
    from acados_template import AcadosOcpSolver
except ImportError as e:
    raise ImportError(
        "longitudinal_mpc requires acados_template (and casadi). Install with pip or use the package venv."
    ) from e

import numpy as np


class LongitudinalMPC:
    """
    Longitudinal MPC with FOPDT dynamics (s, v, a) and commanded acceleration input.

    Mirrors the structure of PathTrackingMPCSpatialWithBodyPoints: builds an
    Acados OCP from the longitudinal model, sets LINEAR_LS cost and input
    constraints, and optionally generates/builds the solver.
    """

    def __init__(
        self,
        Tf: float,
        N: int,
        tau_equiv: float = 1.5,
        u_min: float = -3.0,
        u_max: float = 2.0,
        build: bool = True,
        generate: bool = True,
        code_export_directory: str = "c_generated_code",
        json_file: str = "acados_ocp.json",
    ):
        self.Tf = Tf
        self.N = N
        self.tau_equiv = tau_equiv
        self.u_min = u_min
        self.u_max = u_max
        self.code_export_directory = code_export_directory
        self.json_file = json_file
        self.constraint, self.model, self.acados_solver = self._acados_settings(
            build, generate
        )

    def _acados_settings(self, build: bool = True, generate: bool = True):
        ocp = AcadosOcp()

        # export model (same style as bicycle_model_spatial_with_body_points)
        model, constraint = longitudinal_fopdt_model(tau_equiv=self.tau_equiv)
        model.u_min = self.u_min
        model.u_max = self.u_max

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

        # Cost: track state (s, v, a) and penalize input; LINEAR_LS
        Q = np.diag([1e0, 1e0, 1e-1])   # s, v, a
        R = np.eye(nu) * 2e-1            # u_cmd
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

        # Input constraints: acceleration bounds
        ocp.constraints.lbu = np.array([model.u_min])
        ocp.constraints.ubu = np.array([model.u_max])
        ocp.constraints.idxbu = np.array([0])

        ocp.constraints.x0 = np.zeros(nx)

        ocp.parameter_values = np.array([self.tau_equiv])

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
    Tf = 10.0
    N = 50
    mpc = LongitudinalMPC(
        Tf, N,
        tau_equiv=1.5,
        u_min=-3.0,
        u_max=2.0,
        build=False,
        generate=True,
    )
    print("Longitudinal MPC code generated.")


if __name__ == "__main__":
    main()
