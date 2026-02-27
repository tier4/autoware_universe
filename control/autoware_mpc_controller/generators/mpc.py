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
Combined longitudinal + lateral MPC (path_tracking_mpc_spatial_with_body_points-style).

Builds a single acados OCP from the combined model: state x = [s, v, a, eY, ePsi],
input u = [u_cmd, delta], cost on tracking all states and inputs, bounds on
acceleration and steering. One solver for both axes.

Delay compensation is implemented in C++ (acados_interface.hpp/cpp); this script
only generates the OCP and C code.
"""

from __future__ import annotations

try:
    from .combined_longitudinal_lateral_model import combined_longitudinal_lateral_model
except ImportError:
    from combined_longitudinal_lateral_model import combined_longitudinal_lateral_model

try:
    from acados_template import AcadosModel
    from acados_template import AcadosOcp
    from acados_template import AcadosOcpSolver
except ImportError as e:
    raise ImportError(
        "mpc requires acados_template (and casadi). Install with pip or use the package venv."
    ) from e

import numpy as np

# Default wheelbase [m]
DEFAULT_LF = 1.0
DEFAULT_LR = 1.0


class CombinedMPC:
    """
    Combined longitudinal + lateral MPC with FOPDT longitudinal and curvature-based
    lateral dynamics in time. Single OCP: state [s, v, a, eY, ePsi], input [u_cmd, delta].
    Delay compensation is done in C++ (see acados_interface).
    """

    def __init__(
        self,
        Tf: float,
        N: int,
        tau_equiv: float = 1.5,
        u_cmd_min: float = -3.0,
        u_cmd_max: float = 2.0,
        delta_min: float = -0.7,
        delta_max: float = 0.7,
        lf: float = DEFAULT_LF,
        lr: float = DEFAULT_LR,
        build: bool = True,
        generate: bool = True,
        code_export_directory: str = "c_generated_code",
        json_file: str = "acados_ocp.json",
    ):
        self.Tf = Tf
        self.N = N
        self.tau_equiv = tau_equiv
        self.u_cmd_min = u_cmd_min
        self.u_cmd_max = u_cmd_max
        self.delta_min = delta_min
        self.delta_max = delta_max
        self.lf = lf
        self.lr = lr
        self.code_export_directory = code_export_directory
        self.json_file = json_file
        self.constraint, self.model, self.acados_solver = self._acados_settings(
            build, generate
        )

    def _acados_settings(self, build: bool = True, generate: bool = True):
        ocp = AcadosOcp()

        model, constraint = combined_longitudinal_lateral_model(tau_equiv=self.tau_equiv)
        model.u_cmd_min = self.u_cmd_min
        model.u_cmd_max = self.u_cmd_max
        model.delta_min = self.delta_min
        model.delta_max = self.delta_max

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

        # Cost: track (s, v, a, eY, ePsi) and penalize (u_cmd, delta)
        Q = np.diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1])  # s, v, a, eY, ePsi
        R = np.diag([2e-2, 2e-1])  # u_cmd, delta
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
        Vu[nx, 0] = 1.0
        Vu[nx + 1, 1] = 1.0
        ocp.cost.Vu = Vu

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e

        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)

        # Input constraints: acceleration and steering bounds
        ocp.constraints.lbu = np.array([model.u_cmd_min, model.delta_min])
        ocp.constraints.ubu = np.array([model.u_cmd_max, model.delta_max])
        ocp.constraints.idxbu = np.array([0, 1])

        ocp.constraints.x0 = np.zeros(nx)

        # p = [tau_equiv, kappa_ref, lf, lr]; kappa_ref set per-stage at runtime
        ocp.parameter_values = np.array([self.tau_equiv, 0.0, self.lf, self.lr])

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
    mpc = CombinedMPC(
        Tf, N,
        tau_equiv=1.5,
        u_cmd_min=-3.0,
        u_cmd_max=2.0,
        delta_min=-0.7,
        delta_max=0.7,
        lf=1.0,
        lr=1.0,
        build=False,
        generate=True,
    )
    print("Combined MPC code generated.")


if __name__ == "__main__":
    main()
