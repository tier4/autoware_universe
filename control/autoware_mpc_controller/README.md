# autoware_mpc_controller

This package provides a **combined lateral–longitudinal MPC** in the same style as `autoware_path_optimizer/src/acados_mpc`: generated C code, a C++ interface, and a node that can replace the existing longitudinal and lateral MPC controllers.

**Roadmap (mirroring path_optimizer):**
1. **Generated C code** — Python generators (`generators/mpc.py`) build an acados OCP and generate C into `generators/c_generated_code/`.
2. **C++ interface** — `acados_mpc/` (at package root, next to `generators/` and `src/`) builds acados, runs the generator, and builds the `acados_interface` library (`acados_interface.hpp`, `acados_interface.cpp`). Same API pattern as path_optimizer: `setInitialState`, `setParameters`, `getControl`, etc.
3. **Node** — `src/node.cpp` → `mpc_controller_node` links the interface and will be extended to subscribe to trajectory/state, run delay compensation, solve, and publish acceleration + steering (replacing longitudinal + lateral MPC nodes).

Build requires **acados** (set `ACADOS_SOURCE_DIR` or `ACADOS_WORKSPACE_DIR`, or place acados in `external/acados`). Then `colcon build` builds acados, generates the MPC C code, and builds the interface and node.

## Document: longitudinal and lateral dynamics

**`docs/longitudinal_inferred_model.md`** — Describes (I) longitudinal dynamics inferred from the PID (FOPDT, τ_equiv, LPF relation), (II) lateral dynamics from the MPC lateral controller (time-based kinematics bicycle), and (III) curvilinear (spatial) lateral dynamics used in this package’s lateral MPC. Use it as the reference when building a combined MPC.

## Combined longitudinal + lateral model

- **`generators/combined_longitudinal_lateral_model.py`**  
  Single combined model: **x = [s, v, a, eY, ePsi]**, **u = [u_cmd, δ]**, **p = [tau_equiv, kappa_ref, lf, lr]**. Longitudinal: ṡ=v, v̇=a, ȧ=(u_cmd−a)/τ_equiv. Lateral: deY/dt = v·(deY/ds), dePsi/dt = v·(dePsi/ds) (curvature-based, v from state). Use for a single acados OCP with both axes.

## MPC (acados) — single combined OCP

- **`generators/mpc.py`**  
  Builds one acados OCP from the combined model: state **x = [s, v, a, eY, ePsi]**, input **u = [u_cmd, δ]**, LINEAR_LS cost on all states and inputs, bounds on acceleration and steering. Same style as `path_tracking_mpc_spatial_with_body_points.py`.

  ```python
  from generators.mpc import CombinedMPC
  mpc = CombinedMPC(Tf=10.0, N=50, build=False, generate=True)
  ```

  Requires **acados_template** and **casadi**. Run from `generators/` or with the package on `PYTHONPATH`:

  ```bash
  cd src/autoware/universe/control/autoware_mpc_controller/generators
  python3 mpc.py
  ```
  This generates `c_generated_code/` and `acados_ocp.json`. When building the package with CMake (and acados available), the same generator is run from `acados_mpc/CMakeLists.txt` and the resulting C is compiled into the `acados_interface` library.

  **Delay compensation:** Implemented in C++ using the **acados sim integrator** (same dynamics as the OCP). Use `getControlWithDelayCompensation(current_state, delay_time, p)` or the convenience overload with `(current_state, delay_time, kappa_ref, tau_equiv, lf, lr)`; apply `sol.utraj[0]`. Prediction uses `predictStateAfterDelay(x, delay_time, p)` which runs the acados sim over the delay interval. See `docs/longitudinal_inferred_model.md` for background.

## Model building blocks (for reference)

Longitudinal (FOPDT) and lateral (curvilinear) are combined in `combined_longitudinal_lateral_model.py`. The doc summarizes:

- **Longitudinal:** States (s, v, a), input u_cmd, ṡ=v, v̇=a, τ_equiv·ȧ = u−a. Dead time τ is handled by delay compensation in the MPC/OCP layer (see doc).
- **Lateral:** Same curvature structure in **time**: d/dt = v·(d/ds). **`generators/bicycle_model_curvilinear.py`** has state [eY, ePsi], control δ, **`generators/longitudinal_fopdt_model.py`** has state (s, v, a), control u_cmd. Both are used inside the combined model.
