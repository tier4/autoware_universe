# Replacing Longitudinal + Lateral Controllers with Combined MPC

This document describes how to use the **combined longitudinal + lateral MPC** (`autoware_mpc_controller`) as a **drop-in replacement** for the existing separate longitudinal (e.g. PID) and lateral (e.g. MPC) controllers in the Autoware control stack.

## Overview

- **Existing stack**: `autoware_trajectory_follower_node::Controller` runs a longitudinal controller (e.g. PID) and a lateral controller (e.g. MPC), merges their outputs into `autoware_control_msgs/Control`, and publishes to `~/output/control_cmd` (often remapped to `/control/trajectory_follower/control_cmd` or `/trajectory_follower/control_cmd`).
- **Replacement**: The `mpc_controller_node` from this package subscribes to the **same inputs**, computes **both** acceleration and steering with a single combined MPC, and publishes the **same** `Control` message to the same topic name so downstream nodes (e.g. vehicle_cmd_gate, shift_decider) do not need changes.

## Interface alignment

### Subscriptions (same as trajectory_follower Controller)

| Topic (relative to node)      | Message type                                      | Description                |
|------------------------------|----------------------------------------------------|----------------------------|
| `~/input/reference_trajectory` | `autoware_planning_msgs/msg/Trajectory`           | Reference path to follow    |
| `~/input/current_odometry`   | `nav_msgs/msg/Odometry`                           | Ego pose and twist         |
| `~/input/current_steering`    | `autoware_vehicle_msgs/msg/SteeringReport`        | Current steering angle     |
| `~/input/current_accel`      | `geometry_msgs/msg/AccelWithCovarianceStamped`   | Current acceleration       |
| `~/input/current_operation_mode` | `autoware_adapi_v1_msgs/msg/OperationModeState` (optional) | Operation mode        |

### Publisher

| Topic (relative to node) | Message type                    | Description                    |
|--------------------------|----------------------------------|--------------------------------|
| `~/output/control_cmd`   | `autoware_control_msgs/msg/Control` | Lateral + longitudinal command |

- **Control.lateral**: `steering_tire_angle` (rad) from MPC `delta`; `steering_tire_rotation_rate` can be left 0 or estimated.
- **Control.longitudinal**: `acceleration` (m/s²) from MPC `u_cmd`; `velocity` from reference trajectory at current arc length; `is_defined_acceleration` = true.

### Remapping for drop-in

To replace the trajectory_follower Controller in a launch file, run `mpc_controller_node` with the **same remaps** as the Controller, for example:

- `~/input/reference_trajectory` → `/planning/trajectory` (or your planning output)
- `~/input/current_odometry` → `/localization/kinematic_state` (if that topic publishes `nav_msgs/Odometry`)
- `~/input/current_accel` → `/localization/acceleration`
- `~/input/current_steering` → `/vehicle/status/steering_status`
- `~/input/current_operation_mode` → `/system/operation_mode/state` (optional)
- `~/output/control_cmd` → `/control/trajectory_follower/control_cmd` (or `/trajectory_follower/control_cmd` so vehicle_cmd_gate still receives control_cmd)

If your stack uses `autoware_localization_msgs/KinematicState` for kinematic_state, add a relay or converter to Odometry, or extend this node to subscribe to KinematicState.

## Parameters

- **`enable_controller`** (bool, default: `true`): If `false`, the node does not compute or publish control commands (MPC is effectively off). Set to `true` to enable the combined MPC.
- **`ctrl_period`** (double): Control period in seconds (e.g. 0.03).
- **`delay_compensation_time`** (double): Actuator + pipeline delay in seconds (e.g. 0.17).
- **`tau_equiv`** (double): Longitudinal FOPDT equivalent time constant (e.g. 1.5).
- **`wheelbase_lf`** (double): Distance from CG to front axle (m).
- **`wheelbase_lr`** (double): Distance from CG to rear axle (m).
- **`max_steer_rad`** (double): Maximum steering tire angle (rad), for clamping MPC output.
- **`timeout_trajectory_sec`** (double): Trajectory timeout; no control if trajectory is older (optional).

## Turning the old controller on/off

The **trajectory_follower Controller** (PID + lateral MPC) supports an **`enable_controller`** parameter (default: `true`). Set it to **`false`** to disable that node’s control output so you can use the combined MPC instead:

- **Trajectory follower off**: `enable_controller: false` (in the trajectory_follower node’s params, or e.g. `-p enable_controller:=false`). The node still runs but does not compute or publish control commands.
- **Combined MPC off**: On `mpc_controller_node`, set `enable_controller: false` to disable the combined MPC.

Use one controller at a time: when the combined MPC is active, set the trajectory_follower’s `enable_controller` to `false`; when using the old PID+MPC stack, set the combined MPC node’s `enable_controller` to `false` (or do not run it).

### How the new (combined MPC) controller is turned on

1. **Start the node**  
   Run `mpc_controller_node` (e.g. from your launch or `ros2 run autoware_mpc_controller mpc_controller_node`).

2. **Params**  
   - **`enable_controller`** defaults to **`true`**, so the node publishes control as soon as it has trajectory/odom/accel. Set to `false` to disable.
   - Optional: load the package param file so all defaults are in one place:
     - **Param file:** `config/mpc_controller_node.param.yaml` (installed to `share/autoware_mpc_controller/config/`).
     - Load it when starting the node, e.g.  
       `--params-file $(ros2 pkg prefix autoware_mpc_controller)/share/autoware_mpc_controller/config/mpc_controller_node.param.yaml`  
       or from a launch file:  
       `<param from="$(find-pkg-share autoware_mpc_controller)/config/mpc_controller_node.param.yaml"/>`.

3. **Remaps**  
   Remap its topics so it receives trajectory/odometry/accel/steering and publishes to the same `control_cmd` topic the rest of the stack uses (e.g. `/control/trajectory_follower/control_cmd`).

## Launch / config

1. **Option A – Standalone replacement**  
   In your control launch (e.g. where you currently start the trajectory_follower Controller):
   - Either leave the trajectory_follower Controller running and set **`enable_controller: false`** on it so it stops publishing, or do not start it (or start it in a disabled group).
   - Start `mpc_controller_node` with the remaps above and a param file that sets `enable_controller: true`, `ctrl_period`, `delay_compensation_time`, `tau_equiv`, `wheelbase_lf`, `wheelbase_lr`, and optionally `max_steer_rad` and `timeout_trajectory_sec`.

2. **Option B – Combined mode inside trajectory_follower (future)**  
   The trajectory_follower_node could be extended with a "combined_mpc" mode that instantiates this combined controller and returns both lateral and longitudinal outputs so the existing merge/publish flow is reused. That would require implementing the trajectory_follower_base interfaces (or a combined interface) in this package and registering the mode in trajectory_follower_node.

## State and reference

The combined MPC expects state **x = [s, v, a, eY, ePsi]** and parameters **p = [tau_equiv, kappa_ref, lf, lr]**:

- **s**: Arc length along the reference trajectory to the projection of the ego pose.
- **v**: Longitudinal speed (e.g. from odometry `twist.twist.linear.x`).
- **a**: Current longitudinal acceleration (e.g. from `current_accel`).
- **eY**: Lateral error (signed distance from reference path), e.g. from `motion_utils::calcLateralOffset`.
- **ePsi**: Heading error (ego yaw − reference yaw), normalized to [-π, π].
- **kappa_ref**: Reference curvature at the current reference point (e.g. from trajectory point `front_wheel_angle_rad` as `tan(angle)/wheelbase`, or from `motion_utils::calcCurvature`).

The node builds **x** and **p** from the subscribed topics and the current trajectory, then calls the acados interface with delay compensation and publishes the resulting `Control` message.

## Build and run

- Build the package (including acados codegen) as usual; ensure acados is installed and `ACADOS_SOURCE_DIR` (or `/opt/acados`) is set.
- Run the node, e.g.:
  ```bash
  ros2 run autoware_mpc_controller mpc_controller_node --ros-args \
    -r __ns:=/control/trajectory_follower \
    -r input/reference_trajectory:=/planning/trajectory \
    -r input/current_odometry:=/localization/kinematic_state \
    -r input/current_accel:=/localization/acceleration \
    -r input/current_steering:=/vehicle/status/steering_status \
    -r output/control_cmd:=/control/trajectory_follower/control_cmd \
    -p ctrl_period:=0.03 -p delay_compensation_time:=0.17 \
    -p tau_equiv:=1.5 -p wheelbase_lf:=1.0 -p wheelbase_lr:=1.0
  ```

Adjust namespaces and topic names to match your launch and remaps.
