#!/usr/bin/env python3
"""
Step response of the combined longitudinal + lateral dynamics (same as in combined_longitudinal_lateral_model.py).
No MPC, no acados — just integrate the ODE and plot. Use to sanity-check the dynamics.

Run from generators/:
  python3 step_response_dynamics.py

Requires: numpy, matplotlib. Opens a window with 4 plots; close the window to exit.
To save figure instead of showing: add --save step_response.png
"""
import math
import numpy as np


def dynamics(x, u_cmd, delta, tau, kappa_ref, lf, lr):
    """x = [s, v, a, eY, ePsi]. Returns x_dot."""
    s, v, a, eY, ePsi = x
    # Longitudinal: ṡ = v, v̇ = a, τ·ȧ = u_cmd - a
    s_dot = v
    v_dot = a
    a_dot = (u_cmd - a) / tau
    # Lateral: beta = atan(lr*tan(delta)/(lf+lr)), kappa = cos(beta)*tan(delta)/(lf+lr)
    # deY_ds = tan(ePsi+beta)*(1 - kappa_ref*eY), dePsi_ds = kappa*(1 - kappa_ref*eY)/cos(ePsi) - kappa_ref
    # eY_dot = v*deY_ds, ePsi_dot = v*dePsi_ds
    if lf + lr < 1e-9:
        beta, kappa = 0.0, 0.0
    else:
        beta = math.atan(lr * math.tan(delta) / (lf + lr))
        kappa = math.cos(beta) * math.tan(delta) / (lf + lr)
    cePsi = math.cos(ePsi)
    if abs(cePsi) < 1e-9:
        cePsi = 1e-9 if cePsi >= 0 else -1e-9
    deY_ds = math.tan(ePsi + beta) * (1 - kappa_ref * eY)
    dePsi_ds = kappa * (1 - kappa_ref * eY) / cePsi - kappa_ref
    eY_dot = v * deY_ds
    ePsi_dot = v * dePsi_ds
    return np.array([s_dot, v_dot, a_dot, eY_dot, ePsi_dot])


def run_step_response(dt=0.02, t_end=15.0, u_cmd_step=1.0, delta_step=0.0, tau=1.5, kappa_ref=0.0, lf=1.0, lr=1.0, step_time=2.0):
    """Integrate with Euler. Step u_cmd at step_time (and optionally delta)."""
    n = int(t_end / dt) + 1
    t = np.linspace(0, t_end, n)
    x = np.zeros((n, 5))
    u_cmd_arr = np.zeros(n)
    delta_arr = np.zeros(n)
    x[0] = [0.0, 0.0, 0.0, 0.0, 0.0]
    for i in range(n - 1):
        if t[i] >= step_time:
            u_cmd_arr[i] = u_cmd_step
            delta_arr[i] = delta_step
        else:
            u_cmd_arr[i] = 0.0
            delta_arr[i] = 0.0
        xdot = dynamics(x[i], u_cmd_arr[i], delta_arr[i], tau, kappa_ref, lf, lr)
        x[i + 1] = x[i] + dt * xdot
    u_cmd_arr[-1] = u_cmd_arr[-2]
    delta_arr[-1] = delta_arr[-2]
    return t, x, u_cmd_arr, delta_arr


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Step response of combined longitudinal + lateral dynamics")
    parser.add_argument("--save", type=str, default="", help="Save figure to path and exit (no GUI)")
    args = parser.parse_args()

    import matplotlib
    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tau = 1.5
    lf, lr = 1.0, 1.0
    kappa_ref = 0.0

    # 1) Longitudinal step: u_cmd 0 -> 1 at t=2s
    t, x, u_cmd, delta = run_step_response(
        dt=0.02, t_end=15.0, u_cmd_step=1.0, delta_step=0.0,
        tau=tau, kappa_ref=kappa_ref, lf=lf, lr=lr, step_time=2.0
    )
    s, v, a, eY, ePsi = x[:, 0], x[:, 1], x[:, 2], x[:, 3], x[:, 4]

    # 2) Lateral step: hold v=5, step delta 0 -> 0.1 at t=2s (lateral dynamics only)
    dt2 = 0.02
    t2 = np.arange(0, 10.0 + dt2, dt2)
    n2 = len(t2)
    x2_alt = np.zeros((n2, 5))
    x2_alt[0] = [0.0, 5.0, 0.0, 0.0, 0.0]
    delta2 = np.where(t2 >= 2.0, 0.1, 0.0)
    for i in range(n2 - 1):
        # Hold v=5: set u_cmd = a so a_dot = 0, and don't let v drift (use a = 0, u_cmd = 0)
        u_cmd_hold = 0.0
        d = delta2[i]
        xdot = dynamics(x2_alt[i], u_cmd_hold, d, tau, kappa_ref, lf, lr)
        x2_alt[i + 1] = x2_alt[i] + dt2 * xdot
        x2_alt[i + 1, 1] = 5.0  # hold v constant for clear lateral step
    s2, v2, a2, eY2, ePsi2 = x2_alt[:, 0], x2_alt[:, 1], x2_alt[:, 2], x2_alt[:, 3], x2_alt[:, 4]

    fig, axes = plt.subplots(2, 2, figsize=(10, 8))

    # Longitudinal step response
    ax = axes[0, 0]
    ax.plot(t, u_cmd, "k--", label="u_cmd")
    ax.plot(t, a, "b-", label="a")
    ax.plot(t, v, "g-", label="v")
    ax.plot(t, s, "c-", label="s")
    ax.set_xlabel("t (s)")
    ax.set_title("Longitudinal step: u_cmd 0→1 at t=2s")
    ax.legend()
    ax.grid(True)

    ax = axes[0, 1]
    ax.plot(t, v, "g-", label="v")
    ax.axhline(1.0 * tau, color="gray", linestyle=":", alpha=0.8)  # steady-state v ≈ u_cmd*tau for constant a
    ax.set_xlabel("t (s)")
    ax.set_ylabel("v (m/s)")
    ax.set_title("Velocity (FOPDT: a → u_cmd, v → ∫a)")
    ax.legend()
    ax.grid(True)

    # Lateral step response (v=5, delta 0→0.1 at t=2s)
    ax = axes[1, 0]
    ax.plot(t2, delta2, "k--", label="delta")
    ax.plot(t2, eY2, "b-", label="eY")
    ax.plot(t2, ePsi2, "g-", label="ePsi")
    ax.set_xlabel("t (s)")
    ax.set_title("Lateral step: delta 0→0.1 at t=2s (v=5 m/s)")
    ax.legend()
    ax.grid(True)

    ax = axes[1, 1]
    ax.plot(t2, v2, "b-", label="v")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("v (m/s)")
    ax.set_title("Velocity (lateral test)")
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    if args.save:
        plt.savefig(args.save)
        print(f"Saved to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
