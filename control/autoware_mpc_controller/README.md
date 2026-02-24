# autoware_mpc_controller

This package provides model-description files for a combined lateral–longitudinal MPC, consistent with the longitudinal dynamics inferred from the PID longitudinal controller and the lateral dynamics from `autoware_mpc_lateral_controller`.

## Document: longitudinal and lateral dynamics

**`docs/longitudinal_inferred_model.md`** — Describes (I) longitudinal dynamics inferred from the PID (FOPDT, τ_equiv, LPF relation), (II) lateral dynamics from the MPC lateral controller (time-based kinematics bicycle), and (III) curvilinear (spatial) lateral dynamics used in this package’s lateral MPC. Use it as the reference when building a combined MPC.

## Combined longitudinal + lateral model

- **`generators/combined_longitudinal_lateral_model.py`**  
  Single combined model: **x = [s, v, a, eY, ePsi]**, **u = [u_cmd, δ]**, **p = [tau_equiv, kappa_ref, lf, lr]**. Longitudinal: ṡ=v, v̇=a, ȧ=(u_cmd−a)/τ_equiv. Lateral: deY/dt = v·(deY/ds), dePsi/dt = v·(dePsi/ds) (curvature-based, v from state). Use for a single acados OCP with both axes.

## Longitudinal model (FOPDT)

Summary from that doc:

- **States:** along-path position `s`, longitudinal speed `v`, longitudinal acceleration `a`.
- **Input:** commanded acceleration `u` (sent to the vehicle interface).
- **Kinematics:** ṡ = v, v̇ = a.
- **Actuator (first-order + dead time):** equivalent first-order lag τ_equiv with  
  **τ_equiv · (da/dt) = u − a**  
  (Dead time τ is typically handled by delay compensation elsewhere; the model file focuses on the continuous lag.)

## Model

- **`generators/longitudinal_fopdt_model.py`**  
  Same style as `bicycle_model_spatial_with_body_points.py`: one function `longitudinal_fopdt_model(tau_equiv=1.5)` returning `(model, constraint)`. CasADi SX for state `(s, v, a)`, control `u_cmd`, parameter `tau_equiv`, dynamics ṡ=v, v̇=a, ȧ=(u−a)/τ_equiv. Requires **casadi**.

## Longitudinal MPC (acados)

- **`generators/longitudinal_mpc.py`**  
  Builds an acados OCP for longitudinal control in the same style as `path_tracking_mpc_spatial_with_body_points.py`: uses the FOPDT model, LINEAR_LS cost (track s, v, a; penalize u), and acceleration bounds. Generates C code and optional solver.

  ```python
  from generators.longitudinal_mpc import LongitudinalMPC
  mpc = LongitudinalMPC(Tf=10.0, N=50, build=False, generate=True)
  ```

  Requires **acados_template** and **casadi**. Run from `generators/` or with the package on `PYTHONPATH`:

  ```bash
  cd src/autoware/universe/control/autoware_mpc_controller/generators
  python3 longitudinal_mpc.py
  ```
  This generates `c_generated_code/` and `acados_ocp.json` in that directory.

## Lateral model: curvature-based dynamics in time

Lateral dynamics use the **same curvature structure** as the spatial model but in **time**: **d/dt = v · (d/ds)** with **v = ds/dt** from the longitudinal FOPDT model (ṡ = v).

- **`generators/bicycle_model_curvilinear.py`**  
  State **x = [eY, ePsi]**, control **δ**, parameters **p = [v, kappa_ref, lf, lr]** (v from longitudinal).  
  **deY/dt = v · tan(ePsi+β)(1 − κ_ref·eY)**, **dePsi/dt = v · (κ(1 − κ_ref·eY)/cos(ePsi) − κ_ref)**  
  with β = atan(lr·tan(δ)/(lf+lr)), κ = cos(β)·tan(δ)/(lf+lr).

- **`generators/lateral_mpc_curvilinear.py`**  
  Lateral MPC: **time** horizon **Tf** [s], same cost and constraints. Run from `generators/`:
  ```bash
  python3 lateral_mpc_curvilinear.py
  ```
  Generates `c_generated_code_lateral/` and `acados_ocp_lateral.json`.
