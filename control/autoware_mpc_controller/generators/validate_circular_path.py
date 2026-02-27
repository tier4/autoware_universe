#!/usr/bin/env python3
"""
Minimal validation: use the same MPC as the generator (acados Python), track a circular path.

Run from generators/ with acados venv and libs:
  cd generators
  export LD_LIBRARY_PATH=/opt/acados/lib ACADOS_SOURCE_DIR=/opt/acados
  PYTHONPATH=/opt/acados/interfaces/acados_template python3 validate_circular_path.py
(Or use the acados .venv: /opt/acados/.venv/bin/python3)
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from mpc import CombinedMPC


def step_dynamics(x, u_cmd, delta_cmd, dt, tau, steer_tau, kappa_ref, lf, lr):
    s, v, a, e_y, e_psi, steer = x

    beta = math.atan(lr * math.tan(steer) / (lf + lr))
    kappa = math.cos(beta) * math.tan(steer) / (lf + lr)
    cos_epsi = math.cos(e_psi)
    if abs(cos_epsi) < 1e-3:
        cos_epsi = 1e-3 if cos_epsi >= 0.0 else -1e-3

    de_y_ds = math.tan(e_psi + beta) * (1.0 - kappa_ref * e_y)
    de_psi_ds = kappa * (1.0 - kappa_ref * e_y) / cos_epsi - kappa_ref

    xdot = np.array(
        [
            v,
            a,
            (u_cmd - a) / tau,
            v * de_y_ds,
            v * de_psi_ds,
            (delta_cmd - steer) / steer_tau,
        ]
    )
    return x + dt * xdot


def main():
    tf = 10.0
    n_horizon = 50
    dt = tf / n_horizon

    # Circular path with fixed curvature.
    radius_m = 35.0
    kappa_ref = 1.0 / radius_m
    v_ref = 0.5

    mpc = CombinedMPC(tf, n_horizon, build=True, generate=True)
    solver = mpc.acados_solver
    if solver is None:
        print("Solver is None (build=False in CombinedMPC?).")
        return 1

    tau = mpc.tau_equiv
    steer_tau = mpc.steer_tau
    lf = mpc.lf
    lr = mpc.lr
    p = np.array([tau, kappa_ref, lf, lr, steer_tau], dtype=float)

    # Start with lateral/heading error to verify convergence on curved path.
    x0 = np.array([0.0, 0.0, 0.0, 0.8, 0.12, 0.0], dtype=float)

    n_steps = 160
    history = {
        "t": [],
        "s": [],
        "s_ref": [],
        "v": [],
        "v_ref": [],
        "a": [],
        "u_cmd": [],
        "delta": [],
        "e_y": [],
        "e_psi": [],
        "delta_ff": [],
        "x_ref": [],
        "y_ref": [],
        "x": [],
        "y": [],
    }

    for step in range(n_steps):
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)

        # Keep parameters fixed across the horizon for constant-radius circle.
        for k in range(n_horizon):
            solver.set(k, "p", p)
        solver.set(n_horizon, "p", p)

        s0 = float(x0[0])
        for k in range(n_horizon):
            s_ref = s0 + v_ref * k * dt
            yref = np.array([s_ref, v_ref, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            solver.cost_set(k, "yref", yref)
        s_ref_n = s0 + v_ref * n_horizon * dt
        yref_e = np.array([s_ref_n, v_ref, 0.0, 0.0, 0.0, 0.0])
        solver.cost_set(n_horizon, "yref", yref_e)

        status = solver.solve()
        if status != 0:
            print(f"step {step}: solve status {status}")
            break

        u0 = solver.get(0, "u")
        u_cmd, delta_cmd = float(u0[0]), float(u0[1])
        x0 = step_dynamics(x0, u_cmd, delta_cmd, dt, tau, steer_tau, kappa_ref, lf, lr)

        t = (step + 1) * dt
        theta = x0[0] * kappa_ref
        x_ref = radius_m * math.sin(theta)
        y_ref = radius_m * (1.0 - math.cos(theta))
        # Left normal to reference tangent (yaw_ref = theta)
        nx = -math.sin(theta)
        ny = math.cos(theta)
        x_world = x_ref + x0[3] * nx
        y_world = y_ref + x0[3] * ny

        history["t"].append(t)
        history["s"].append(x0[0])
        history["s_ref"].append(v_ref * t)
        history["v"].append(x0[1])
        history["v_ref"].append(v_ref)
        history["a"].append(x0[2])
        history["u_cmd"].append(u_cmd)
        history["delta"].append(x0[5])
        history["e_y"].append(x0[3])
        history["e_psi"].append(x0[4])
        history["delta_ff"].append(math.atan((lf + lr) * kappa_ref))
        history["x_ref"].append(x_ref)
        history["y_ref"].append(y_ref)
        history["x"].append(x_world)
        history["y"].append(y_world)

        if step % 20 == 0:
            print(
                f"step {step:3d}  v={x0[1]:.2f}  a={x0[2]:.2f}  "
                f"eY={x0[3]:.3f}  ePsi={x0[4]:.3f}  u={u_cmd:.2f}  delta={x0[5]:.3f}"
            )

    print(
        f"Done. Final v={x0[1]:.2f} (v_ref={v_ref}) "
        f"eY={x0[3]:.3f} ePsi={x0[4]:.3f}"
    )

    try:
        import matplotlib.pyplot as plt

        t = np.array(history["t"])
        fig, axes = plt.subplots(2, 4, figsize=(16, 8))
        ax_s, ax_v, ax_a, ax_xy, ax_ey, ax_eps, ax_d, ax_blank = axes.flat

        ax_s.plot(t, history["s_ref"], "k--", label="s_ref")
        ax_s.plot(t, history["s"], "b-", label="s")
        ax_s.set_title("Path position")
        ax_s.set_xlabel("t (s)")
        ax_s.set_ylabel("s (m)")
        ax_s.grid(True)
        ax_s.legend()

        ax_v.plot(t, history["v_ref"], "k--", label="v_ref")
        ax_v.plot(t, history["v"], "b-", label="v")
        ax_v.set_title("Velocity")
        ax_v.set_xlabel("t (s)")
        ax_v.set_ylabel("v (m/s)")
        ax_v.grid(True)
        ax_v.legend()

        ax_a.plot(t, history["a"], "b-", label="a (state)")
        ax_a.plot(t, history["u_cmd"], "r-", label="u_cmd")
        ax_a.set_title("Acceleration")
        ax_a.set_xlabel("t (s)")
        ax_a.set_ylabel("(m/s^2)")
        ax_a.grid(True)
        ax_a.legend()

        ax_xy.plot(history["x_ref"], history["y_ref"], "k--", label="reference circle")
        ax_xy.plot(history["x"], history["y"], "b-", label="simulated path")
        ax_xy.set_title("XY trajectory")
        ax_xy.set_xlabel("x (m)")
        ax_xy.set_ylabel("y (m)")
        ax_xy.axis("equal")
        ax_xy.grid(True)
        ax_xy.legend()

        ax_ey.plot(t, history["e_y"], "b-", label="eY")
        ax_ey.axhline(0.0, color="k", linestyle="--", linewidth=1)
        ax_ey.set_title("Lateral error")
        ax_ey.set_xlabel("t (s)")
        ax_ey.set_ylabel("eY (m)")
        ax_ey.grid(True)
        ax_ey.legend()

        ax_eps.plot(t, history["e_psi"], "b-", label="ePsi")
        ax_eps.axhline(0.0, color="k", linestyle="--", linewidth=1)
        ax_eps.set_title("Heading error")
        ax_eps.set_xlabel("t (s)")
        ax_eps.set_ylabel("ePsi (rad)")
        ax_eps.grid(True)
        ax_eps.legend()

        ax_d.plot(t, history["delta"], "b-", label="delta")
        ax_d.plot(t, history["delta_ff"], "k--", label="delta_ff")
        ax_d.set_title("Steering")
        ax_d.set_xlabel("t (s)")
        ax_d.set_ylabel("delta (rad)")
        ax_d.grid(True)
        ax_d.legend()

        ax_blank.axis("off")

        plt.tight_layout()
        plt.show()
    except ImportError:
        print("matplotlib not found; skipping plots")
    return 0


if __name__ == "__main__":
    sys.exit(main())
