#!/usr/bin/env python3
"""
Minimal validation: use the same MPC as the generator (acados Python), track a straight line.

Run from generators/ with acados venv and libs:
  cd generators
  export LD_LIBRARY_PATH=/opt/acados/lib ACADOS_SOURCE_DIR=/opt/acados
  PYTHONPATH=/opt/acados/interfaces/acados_template python3 validate_straight_line.py
(Or use the acados .venv: /opt/acados/.venv/bin/python3)
"""
import sys
import os

# so we can import mpc and combined_longitudinal_lateral_model
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from mpc import CombinedMPC

def main():
    Tf = 10.0
    N = 50
    dt = Tf / N
    v_ref = 5.0

    # Same OCP as C++ node; build=True gives us the Python solver (generate=True if no code yet)
    mpc = CombinedMPC(Tf, N, build=True, generate=True)
    solver = mpc.acados_solver
    if solver is None:
        print("Solver is None (build=False in CombinedMPC?).")
        return 1

    # Straight line: kappa_ref = 0 (already in default params)
    s0 = 0.0
    x0 = np.array([s0, 0.0, 0.0, 0.0, 0.0])  # s, v, a, eY, ePsi

    num_steps = 100
    history = {
        "t": [],
        "s": [],
        "s_ref": [],
        "v": [],
        "v_ref": [],
        "a": [],
        "u_cmd": [],
        "delta": [],
    }
    for step in range(num_steps):
        # Fix initial state
        solver.set(0, "lbx", x0)
        solver.set(0, "ubx", x0)

        # Cost reference: advance s along horizon, constant v_ref (same as C++ setCostReference)
        for k in range(N):
            s_ref = s0 + v_ref * k * dt
            yref = np.array([s_ref, v_ref, 0.0, 0.0, 0.0, 0.0, 0.0])
            solver.cost_set(k, "yref", yref)
        s_ref_N = s0 + v_ref * N * dt
        yref_e = np.array([s_ref_N, v_ref, 0.0, 0.0, 0.0])
        solver.cost_set(N, "yref", yref_e)

        status = solver.solve()
        if status != 0:
            print(f"step {step}: solve status {status}")
            break

        u0 = solver.get(0, "u")
        u_cmd, delta = float(u0[0]), float(u0[1])

        # Simple Euler step (longitudinal only for this test)
        tau = mpc.tau_equiv
        s0 = x0[0] + x0[1] * dt
        x0[1] = x0[1] + u_cmd * dt  # v
        x0[2] = x0[2] + (u_cmd - x0[2]) / tau * dt  # a (FOPDT)
        x0[0] = s0
        # keep eY, ePsi 0

        t = (step + 1) * dt
        history["t"].append(t)
        history["s"].append(x0[0])
        history["s_ref"].append(v_ref * t)
        history["v"].append(x0[1])
        history["v_ref"].append(v_ref)
        history["a"].append(x0[2])
        history["u_cmd"].append(u_cmd)
        history["delta"].append(delta)

        if step % 20 == 0:
            print(f"step {step:3d}  s={x0[0]:.1f}  v={x0[1]:.2f}  a={x0[2]:.2f}  u_cmd={u_cmd:.2f}  delta={delta:.3f}")

    print(f"Done. Final v={x0[1]:.2f} (v_ref={v_ref})")

    try:
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 2, figsize=(10, 8))
        ax_s, ax_v, ax_a, ax_d = axes.flat
        t = np.array(history["t"])
        ax_s.plot(t, history["s_ref"], "k--", label="s_ref")
        ax_s.plot(t, history["s"], "b-", label="s")
        ax_s.set_ylabel("s (m)")
        ax_s.set_xlabel("t (s)")
        ax_s.set_title("Path position")
        ax_s.legend()
        ax_s.grid(True)
        ax_v.plot(t, history["v_ref"], "k--", label="v_ref")
        ax_v.plot(t, history["v"], "b-", label="v")
        ax_v.set_ylabel("v (m/s)")
        ax_v.set_xlabel("t (s)")
        ax_v.set_title("Velocity")
        ax_v.legend()
        ax_v.grid(True)
        ax_a.plot(t, history["a"], "b-", label="a (state)")
        ax_a.plot(t, history["u_cmd"], "r-", label="u_cmd")
        ax_a.set_ylabel("(m/s²)")
        ax_a.set_xlabel("t (s)")
        ax_a.set_title("Acceleration")
        ax_a.legend()
        ax_a.grid(True)
        ax_d.plot(t, history["delta"], "b-", label="delta")
        ax_d.set_ylabel("delta (rad)")
        ax_d.set_xlabel("t (s)")
        ax_d.set_title("Steering")
        ax_d.legend()
        ax_d.grid(True)
        plt.tight_layout()
        plt.show()
    except ImportError:
        print("matplotlib not found; skipping plots")
    return 0

if __name__ == "__main__":
    sys.exit(main())
