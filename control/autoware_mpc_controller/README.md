# autoware_mpc_controller

This package provides a **longitudinal-only MPC** (Acados, FOPDT model) used by `autoware_trajectory_follower_node` when `longitudinal_controller_mode: longitudinal_mpc`. It does not provide a standalone node; the controller runs inside the trajectory follower together with the lateral controller (e.g. MPC or pure_pursuit).

## Contents

- **Longitudinal MPC controller** — `LongitudinalMpcController` (implements `LongitudinalControllerBase`), built from:
  - **Generators:** `generators/longitudinal_only_model.py` (FOPDT: s, v, a; u_cmd; τ_equiv), `generators/mpc_longitudinal_only.py` (Acados OCP, codegen into `c_generated_code/`).
  - **C++:** `acados_mpc/acados_longitudinal_interface.hpp/.cpp`, `include/autoware/longitudinal_mpc_controller/`, `src/longitudinal_mpc_controller.cpp`.
- **Visualization** — `scripts/visualize_mpc_control.py` for steering/velocity/acceleration and ref/pred trajectory; supports `--longitudinal-mpc` and `--compare-pid` to compare with PID output.

Build requires **acados** (set `ACADOS_SOURCE_DIR` or use `/opt/acados`). Then `colcon build` runs the longitudinal-only codegen and builds `acados_longitudinal_interface` and `longitudinal_mpc_controller`.

## Config

- **Trajectory follower:** Set `longitudinal_controller_mode: longitudinal_mpc` in `trajectory_follower_node.param.yaml` (or launch arg).
- **Longitudinal MPC params:** `autoware_launch/config/control/trajectory_follower/default/longitudinal/longitudinal_mpc.param.yaml`.

## Docs

- **`docs/longitudinal_inferred_model.md`** — Longitudinal FOPDT dynamics and relation to PID/lateral MPC (reference for the longitudinal model).
- **`docs/replace_controllers.md`** — Note on removal of combined MPC; longitudinal-only usage.
