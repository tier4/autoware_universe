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

"""Offline runner for the ML planner trajectory optimization OCP.

Solves the OCP against a pose-only reference (positions and headings, matching what the
model outputs) and plots reference vs. solution (position, yaw, velocity, acceleration,
steering angle, steering rate).

Reference file format (whitespace-separated, one line per point):
    line 1        : x0  ->  x y psi v delta          (initial state, base_link)
    lines 2..N+1  : ref ->  x y psi                  (one line per 0.1 s step, t=0.1..8.0)

Without --input, a synthetic noisy curved reference is generated.

Usage (inside the acados venv):
    /opt/acados/.venv/bin/python3 run_optimizer_offline.py [--input ref.txt] [--output out.png]
"""

import argparse
from pathlib import Path

from generate_solver import DEFAULT_TERMINAL_WEIGHT_SCALE
from generate_solver import DEFAULT_WEIGHT_ACCELERATION
from generate_solver import DEFAULT_WEIGHT_STEERING_RATE
from generate_solver import DEFAULT_WEIGHT_YAW
from generate_solver import HORIZON_N
from generate_solver import HORIZON_TF_S
from generate_solver import create_solver
import numpy as np

DT = HORIZON_TF_S / HORIZON_N


def rotated_position_block(yaw, w_lon, w_lat):
    """W_pos = R(yaw) diag(w_lon, w_lat) R(yaw)^T, mirroring the C++ wrapper."""
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [w_lon * c * c + w_lat * s * s, (w_lon - w_lat) * c * s],
            [(w_lon - w_lat) * c * s, w_lon * s * s + w_lat * c * c],
        ]
    )


def set_weights(solver, x0, refs, w_lon, w_lat):
    unscale = HORIZON_N / HORIZON_TF_S
    W = np.diag(
        [
            0.0,
            0.0,
            DEFAULT_WEIGHT_YAW,
            0.0,
            0.0,
            DEFAULT_WEIGHT_ACCELERATION,
            DEFAULT_WEIGHT_STEERING_RATE,
        ]
    )
    for k in range(HORIZON_N):
        yaw = x0[2] if k == 0 else refs[k - 1, 2]
        W[:2, :2] = rotated_position_block(yaw, w_lon, w_lat)
        solver.cost_set(k, "W", unscale * W)
    W_e = np.diag([0.0, 0.0, DEFAULT_WEIGHT_YAW, 0.0, 0.0])
    W_e[:2, :2] = rotated_position_block(refs[-1, 2], w_lon, w_lat)
    solver.cost_set(HORIZON_N, "W", (DEFAULT_TERMINAL_WEIGHT_SCALE / unscale) * W_e)


def make_synthetic_reference(rng):
    """Constant-speed arc with position noise, mimicking raw model output (poses only)."""
    v = 8.0
    kappa = 0.02
    t = np.arange(1, HORIZON_N + 1) * DT
    psi = kappa * v * t
    x = np.cumsum(v * DT * np.cos(psi))
    y = np.cumsum(v * DT * np.sin(psi))
    x += rng.normal(0.0, 0.15, HORIZON_N)
    y += rng.normal(0.0, 0.15, HORIZON_N)
    x0 = np.array([0.0, 0.0, 0.0, v, 0.0])
    return x0, np.stack([x, y, psi], axis=1)


def load_reference(path):
    data = np.loadtxt(path)
    x0 = data[0, :5]
    refs = data[1:, :3]
    if refs.shape[0] < HORIZON_N:
        raise ValueError(f"need {HORIZON_N} reference lines, got {refs.shape[0]}")
    return x0, refs[:HORIZON_N]


def solve(solver, x0, refs, wheelbase):
    ny = 7
    for stage in range(HORIZON_N + 1):
        solver.set(stage, "p", np.array([wheelbase]))

    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)
    solver.set(0, "yref", np.concatenate([x0, np.zeros(2)]))
    for k in range(1, HORIZON_N):
        yref = np.zeros(ny)
        yref[:3] = refs[k - 1]
        solver.set(k, "yref", yref)
    yref_e = np.zeros(5)
    yref_e[:3] = refs[-1]
    solver.set(HORIZON_N, "yref", yref_e)

    for k in range(HORIZON_N + 1):
        solver.set(k, "x", x0.copy())

    status = solver.solve()
    xs = np.array([solver.get(k, "x") for k in range(HORIZON_N + 1)])
    us = np.array([solver.get(k, "u") for k in range(HORIZON_N)])
    solve_time = solver.get_stats("time_tot")
    return status, xs, us, solve_time


def plot(x0, refs, xs, us, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t_ref = np.arange(1, HORIZON_N + 1) * DT
    t_x = np.arange(HORIZON_N + 1) * DT
    t_u = np.arange(HORIZON_N) * DT

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    ax = axes[0][0]
    ax.plot(refs[:, 0], refs[:, 1], ".", ms=3, label="reference")
    ax.plot(xs[:, 0], xs[:, 1], "-", lw=1.5, label="optimized")
    ax.plot(x0[0], x0[1], "k*", ms=10, label="base_link")
    ax.set_title("path (x-y)")
    ax.axis("equal")
    ax.legend()

    for a, (title, ref, sol, ts) in zip(
        axes.flat[1:],
        [
            ("yaw [rad]", refs[:, 2], xs[:, 2], t_x),
            ("velocity [m/s]", None, xs[:, 3], t_x),
            ("acceleration [m/s^2]", None, us[:, 0], t_u),
            ("steering angle [rad]", None, xs[:, 4], t_x),
            ("steering rate [rad/s]", None, us[:, 1], t_u),
        ],
    ):
        if ref is not None:
            a.plot(t_ref, ref, ".", ms=3, label="reference")
        a.plot(ts, sol, "-", lw=1.5, label="optimized")
        a.set_title(title)
        a.set_xlabel("t [s]")
        a.legend()

    fig.tight_layout()
    fig.savefig(output, dpi=120)
    print(f"saved plot to {output}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=None, help="reference trajectory file")
    parser.add_argument("--output", type=Path, default=Path("optimizer_result.png"))
    parser.add_argument("--wheelbase", type=float, default=2.75)
    parser.add_argument("--w-lon", type=float, default=0.5, help="longitudinal position weight")
    parser.add_argument("--w-lat", type=float, default=0.5, help="lateral position weight")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    if args.input is not None:
        x0, refs = load_reference(args.input)
    else:
        x0, refs = make_synthetic_reference(np.random.default_rng(args.seed))

    solver = create_solver()
    set_weights(solver, x0, refs, args.w_lon, args.w_lat)
    status, xs, us, solve_time = solve(solver, x0, refs, args.wheelbase)
    print(f"status={status} (0 = success), solve_time={solve_time * 1e3:.2f} ms")
    plot(x0, refs, xs, us, args.output)


if __name__ == "__main__":
    main()
