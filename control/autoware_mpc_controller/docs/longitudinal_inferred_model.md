# Longitudinal and lateral dynamics (reference)

This document summarizes (1) the longitudinal dynamics inferred from the PID longitudinal controller (used by this package’s longitudinal-only MPC), and (2) the lateral (steering) dynamics used by `autoware_mpc_lateral_controller`. Part I is the reference for the longitudinal FOPDT model in `generators/longitudinal_only_model.py`.

---

# Part I. Longitudinal dynamics (inferred from PID)

## 1. What the PID assumes (implicit longitudinal model)

From the PID controller and its README:

### 1.1 Kinematics (explicit in delay prediction)

In `predictedStateAfterDelay` and `calcPoseAfterTimeDelay`:

- `pred_vel = current_vel + current_acc * dt`
- `running_distance = |v*dt + 0.5*acc*dt²|`

So the assumed kinematics are the double integrator:

- **ṡ = v**
- **v̇ = a**

(s = along-path position, v = longitudinal speed, a = longitudinal acceleration.)

### 1.2 Actuator delay (explicit)

The controller uses `delay_compensation_time` (e.g. 0.17 s) and predicts state *after* that delay. The effective assumption is:

**a_actual(t) = a_cmd(t − τ)**

i.e. a **pure delay** τ between command and acceleration.

### 1.3 Jerk limit (explicit)

`applyDiffLimitFilter(raw_ctrl_cmd.acc, m_prev_raw_ctrl_cmd.acc, dt, m_max_jerk, m_min_jerk)` limits the rate of change of the acceleration command. So the controller does not assume instant step changes in acceleration. With **j = da/dt** (jerk), this can be interpreted as a constraint on jerk or as reflecting actuator/plant lag.

### 1.4 Acceleration feedback

The final command is:

```cpp
acc_err = control_data.current_motion.acc * vel_sign - raw_ctrl_cmd.acc;
m_lpf_acc_error->filter(acc_err);
acc_cmd = raw_ctrl_cmd.acc - m_lpf_acc_error->getValue() * m_acc_feedback_gain;
```

So **m_lpf_acc_error->getValue()** is the **filtered acceleration tracking error** (measured − commanded). It is multiplied by `m_acc_feedback_gain` and subtracted from the raw command, forming an inner feedback loop that compensates for the unknown relationship between commanded and measured acceleration (delay, lag, slope, etc.).

---

## 2. Inferred model: first-order plus dead time (FOPDT)

A minimal longitudinal model consistent with the PID structure is:

### 2.1 States and control

- **States:** s (along path), v (longitudinal speed), a (longitudinal acceleration).
- **Input:** u = commanded acceleration (what is sent to the vehicle interface).
- **Jerk:** j = da/dt.

### 2.2 Dynamics

- **Kinematics:**
  ṡ = v,  v̇ = a.

- **Actuator (FOPDT):**
  Dead time τ and first-order lag with time constant τ_a:

  **τ_a · j = τ_a · (da/dt) = u(t − τ) − a**

### 2.3 Optional disturbance

The acceleration feedback can be interpreted as compensating an additive disturbance d (e.g. slope): a_meas = a + d. The PID does not model d explicitly; it corrects via the filtered error.

---

## 3. Control input: acceleration vs jerk

- **In the FOPDT formulation above:** The **control** is **u** (commanded acceleration). Jerk j is the *result* of the dynamics: j = (u(t−τ) − a) / τ_a. So **control = acceleration**.

- **Alternative formulation (jerk as control):** If we take state (s, v, a) and **control j** with ȧ = j and bounds on j, then the **control** is **jerk**; acceleration is the integral of j. Both formulations are used in the literature; the PID interface is acceleration command.

---

## 4. LPF on acceleration error: relation to τ_lpf

### 4.1 Recurrence in code

From `lowpass_filter.hpp`, at each step (period `dt`):

```cpp
double filter(const double u) {
  const double ret = m_gain * m_x + (1.0 - m_gain) * u;
  m_x = ret;
  return ret;
}
```

So:

**x_f(k) = gain · x_f(k−1) + (1 − gain) · u(k)**

with u(k) = acceleration error (measured_acc − raw_acc). The parameter is **lpf_acc_error_gain** (default 0.98).

### 4.2 LPF gain ↔ time constant τ_lpf

This is the standard discrete-time first-order low-pass filter. Its equivalent continuous time constant is:

**τ_lpf = − dt / ln(gain)**

So:

- **gain → 1** (e.g. 0.98) ⇒ **τ_lpf large** ⇒ only low-frequency error gets through.
- **Smaller gain** ⇒ **τ_lpf smaller** ⇒ faster response to error, less filtering.

Example: dt = 0.03 s, gain = 0.98 ⇒ τ_lpf ≈ 0.03 / 0.0202 ≈ **1.48 s**.

### 4.3 How τ_lpf relates to the FOPDT model (τ_a, τ)

| Quantity | Role | Relation to original param |
|----------|------|----------------------------|
| **τ** | Plant: dead time (delay) | **delay_compensation_time** in the PID. |
| **τ_a** | Plant: time constant from (delayed) command to acceleration | Not a direct PID param; to be identified or tuned per platform. |
| **τ_lpf** | Controller: time constant of LPF on acceleration error | **τ_lpf = − dt / ln(lpf_acc_error_gain)** with dt = control period. |

So:

- **τ_a** = actuator/plant lag (FOPDT first-order part).
- **τ_lpf** = controller filter on the error (set by **lpf_acc_error_gain** and **dt**).

They are not the same: one is in the plant, one in the controller. In the loop, the LPF bandwidth (1/τ_lpf) is often chosen on the order of or slower than the dominant plant dynamics (e.g. τ_a) so that τ_lpf ≥ τ_a roughly, avoiding amplification of noise or delay artifacts. The existing **lpf_acc_error_gain** (e.g. 0.98) gives the τ_lpf that was tuned for the (unknown) τ_a.

---

## 5. MPC design: no LPF, continuous dynamics, equivalent lag

For the longitudinal MPC (and for a hypothetical combined lateral+longitudinal MPC):

1. **No LPF after the MPC.** The MPC output (e.g. acceleration command) is sent directly to the vehicle interface. The PID's LPF was on the *error* to shape feedback; the MPC encodes dynamics in the model, so adding an LPF on the output would duplicate or distort the intended response.

2. **Assume continuous dynamics.** The longitudinal model is the continuous FOPDT (delay τ + first-order lag). Discretization is only for the solver; the model itself is continuous.

3. **Merge τ_a and τ_lpf into one equivalent time constant.** In the PID, the plant has lag τ_a and the controller filters the error with τ_lpf. For the MPC we do not have a separate LPF, so we fold both effects into a single equivalent first-order lag **τ_equiv**:

   **τ_equiv = τ_a + τ_lpf**

   - **τ_a**: actuator/plant lag (platform-specific).
   - **τ_lpf**: from the PID's LPF on acceleration error, τ_lpf = −dt/ln(lpf_acc_error_gain).

   So the MPC uses **one** lag: from (delayed) command to acceleration we have

   **τ_equiv · (da/dt) = u(t − τ) − a**

   with **τ** = `delay_compensation_time` unchanged. This keeps the same overall "slowness" (plant + former LPF) without a separate post-MPC filter.

---

## 6. Summary for longitudinal (MPC)

1. **Longitudinal kinematics (continuous):** ṡ = v, v̇ = a.
2. **Longitudinal actuator (continuous FOPDT):** Delay τ (from `delay_compensation_time`) and **single** first-order lag τ_equiv = τ_a + τ_lpf:
   - **τ_equiv · (da/dt) = u(t − τ) − a**
3. **No LPF** on the MPC output; dynamics are fully in the continuous model.

---

# Part II. Lateral dynamics (from autoware_mpc_lateral_controller)

The lateral controller uses a **bicycle model** (kinematics or dynamics). The following matches the implementation in `vehicle_model_bicycle_kinematics.hpp/cpp` and the algorithm doc `model_predictive_control_algorithm.md`.

## 7. Lateral model: kinematics bicycle with steering lag

### 7.1 State and input

- **State (Frenet / path frame):**  
  **x = [e, θ, δ]^T**  
  - **e** — lateral error (distance from reference path) [m]  
  - **θ** — heading (yaw) error relative to reference tangent [rad]  
  - **δ** — steering angle [rad]  

- **Input:**  
  **u = δ_d** — desired (commanded) steering angle [rad]

- **Parameters:**  
  - **W** — wheelbase [m]  
  - **κ** (or κ_r) — reference path curvature at current point [1/m]  
  - **v** — vehicle speed (from trajectory or measurement) [m/s]  
  - **τ** — steering first-order time constant [s] (e.g. `vehicle_model_steer_tau`, default 0.1)

### 7.2 Nonlinear continuous-time dynamics

The kinematics bicycle model in the lateral controller assumes:

- **de/dt = v · sin(θ)**  
  (lateral error rate in path-normal direction.)

- **dθ/dt = v · tan(δ) / W − κ·v**  
  (yaw rate relative to path: curvature from steering minus reference curvature.)

- **dδ/dt = −(δ − u) / τ**  
  (first-order steering actuator: δ tracks desired δ_d = u with time constant τ.)

So:

**dx/dt = f(x, u)** with

- f₁ = v·sin(θ)  
- f₂ = v·tan(δ)/W − κ·v  
- f₃ = −(δ − u)/τ  

### 7.3 Reference steering (Ackermann)

For a given curvature κ, the reference steering angle is:

**δ_r = atan(W · κ)**  

(optionally clamped to steering limits ±δ_lim.)

### 7.4 Discretization and parameters in the controller

- The lateral MPC discretizes the continuous nonlinear dynamics with step **dt** (e.g. `mpc_prediction_dt`: 0.1 s).
- **Wheelbase W** comes from vehicle info.
- **Steering limit** δ_lim and **steer time constant** τ are set in config: e.g. `vehicle_model_steer_tau` (0.1 s). **Input delay** for steering can be handled separately (e.g. `input_delay` 0.3 s) via delay compensation, analogous to longitudinal delay.

### 7.5 Optional: dynamics bicycle model

The package also provides a **dynamics** bicycle model (four states: lateral error, lateral velocity, yaw error, yaw rate) with tire stiffness parameters. The **kinematics** model above is the one with the simple (e, θ, δ) state and first-order steering lag; the dynamics model is used when `vehicle_model_type: "dynamics"` and adds lateral/yaw dynamics.

---

## 8. Lateral + longitudinal (reference)

For reference, the separate axes used in the stack:

| Axis        | State           | Input        | Main dynamics |
|------------|------------------|--------------|-------------------------------|
| Longitudinal | s, v, a        | u (acc cmd)  | ṡ=v, v̇=a, τ_equiv·ȧ = u(t−τ)−a |
| Lateral    | e, θ, δ         | δ_d (steer)  | ė=v·sin(θ), θ̇=v·tan(δ)/W−κ·v, δ̇=−(δ−δ_d)/τ_steer |

- **Longitudinal:** delay τ and equivalent lag τ_equiv as in Part I; used by this package’s longitudinal-only MPC.  
- **Lateral:** curvature κ (and v) from reference path; steering first-order lag τ_steer (e.g. `vehicle_model_steer_tau`); handled by `autoware_mpc_lateral_controller`.  
- **Coupling:** shared speed **v** (from longitudinal state or reference).

This document and `autoware_mpc_lateral_controller` provide the background; this package implements only the longitudinal FOPDT MPC.

---

## 9. Curvature-based lateral dynamics in time (this package)

This package uses the **same curvature structure** as the spatial model but in **time**, with **v = ds/dt** from the longitudinal model (ṡ = v):

- **State (path frame):** **x = [eY, ePsi]** — lateral offset and heading error.
- **Control:** **u = δ** (steering angle).
- **Parameters:** **v** (speed from longitudinal), **κ_ref**, **lf**, **lr** (wheelbase).
- **Time dynamics:** **d/dt = v · (d/ds)**:

  - **deY/dt = v · tan(ePsi + β) · (1 − κ_ref · eY)**
  - **dePsi/dt = v · (κ · (1 − κ_ref · eY) / cos(ePsi) − κ_ref)**

with **β = atan(lr · tan(δ) / (lf + lr))** and **κ = cos(β) · tan(δ) / (lf + lr)**.

Horizon is **time Tf** [s]. The longitudinal-only MPC in this package uses only Part I (FOPDT); see **`generators/longitudinal_only_model.py`** and **`generators/mpc_longitudinal_only.py`**.
