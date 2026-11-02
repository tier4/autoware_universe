# Autoware MPPI Optimizer

The `autoware_mppi_optimizer` package optimizes trajectories with Model Predictive Path Integral (MPPI) control. Its trajectory modifier plugin applies MPPI to the first candidate in an ordered processing pipeline.

## Overview

This package depends on the external [mppi_generic_vendor](https://github.com/autowarefoundation/mppi_generic_vendor) package (`MPPI-Generic`) and ships first-order Dubins path-tracking extensions under `include/mppi/`. A CUDA library interface (`FirstOrderDubinsMppiInterface`) provides the optimizer for C++ / ROS callers. The active controller uses colored acceleration and steering noise, 16,384 rollouts per iteration, and 80 control steps at 0.1 s. The shipped configuration runs 10 iterations.

### Layout

```text
autoware_mppi_optimizer/
├── include/mppi/         # First-order Dubins dynamics, cost, path utilities
├── include/autoware/mppi_optimizer/
│   └── first_order_dubins_mppi_interface.hpp
└── src/first_order_dubins/
    └── first_order_dubins_mppi_interface.cu
```

### Requirements

- CUDA Toolkit (curand, cufft)
- Eigen3

TensorRT is not required by this package. CUDA remains required for the optimizer library, plugin, and CUDA-linked tests.

## Trajectory modifier plugin

Configure `autoware::mppi_optimizer::plugin::TrajectoryMppiOptimizer` in the `plugin_names` list of `autoware_trajectory_modifier`. Place it first to optimize the primary candidate before other modifiers and optimizers.

Only candidate index zero is processed; secondary candidates return before parameter updates or execution-history changes.

The plugin requires odometry, tracked objects, route, and raw lanelet map data from the modifier. Acceleration, steering, and supplied predicted objects are used when available. Disabled, shadow-mode, conditional-gate skip, and error cycles preserve the upstream candidate. A rejected result preserves input geometry and may apply its deterministic velocity-limit fallback. A reference-length skip can still apply the existing short-reference steering hold.

Plugin parameters are below `mppi_optimizer`. The `enabled` and `shadow_mode` parameters control result application. Debug topics are below `~/debug/mppi` in the trajectory modifier node. The transient-local `enabled` debug topic is true only for a cycle where an optimized MPPI trajectory replaced the primary candidate; it is false when MPPI is disabled, skipped, rejected, or running in shadow mode.

`output_acceleration_state` and `output_steering_state` independently select the predicted physical actuator states for optimized trajectory points. Each defaults to `false`, which keeps the existing acceleration and steering command output. MPPI has no velocity command: optimized points use predicted velocity, except where the existing engage adjustment applies. The engage adjustment applies in both acceleration output modes: it can overwrite early published velocity and acceleration with fixed 0.25 values, subject to the first point's effective velocity limit. It runs after model-state validation; consistency with every point's limit and the validated state remains an unresolved design decision. Selecting the steering state keeps the predicted tire angle during a short-reference steering command hold.

The selections apply to the optimized prefix only, up to 80 points. Remaining points and skipped or rejected outputs retain their existing behavior, including velocity-limit and steering-hold fallbacks. The debug `optimized_trajectory` uses the selected values; `nominal_control_trajectory` continues to show commands.

### Conditional optimization

Set `optimize_only_on_input_collision: true` to run MPPI only when an enabled input collision condition is met. `trigger_on_road_border_collision` and `trigger_on_static_object_collision` are independent; either match starts optimization. Both triggers default to true, while conditional mode defaults to false to retain the existing behavior. When neither enabled condition matches, the plugin leaves all input points unchanged, including their velocity and steering fields, and publishes a skip diagnostic. With both triggers disabled, every cycle passes through unchanged.

By default, activation uses `obstacle_collision_margin` and `road_border_collision_margin`. Set `use_soft_margins_for_input_collision: true` to also use the soft-cost zones: the object trigger uses `obstacle_collision_margin + obstacle_safe_margin`, and the road-border trigger uses `road_border_collision_margin + road_border_safe_margin`. This changes when MPPI starts; it does not change the MPPI costs or output collision validation.

Set `use_vegetation_pointcloud: true` to include points with `class_id == VEGETATION` from the trajectory modifier's obstacle pointcloud. Points are transformed into the trajectory frame, cropped to the ego vehicle height and a corridor around the input trajectory, and projected into the third channel of MPPI's static 2D distance texture. The GPU uses that texture to screen possible contact, then checks the original points against the inflated ego footprint before excluding a rollout. `vegetation_collision_margin` sets the hard clearance from the ego footprint; `vegetation_safe_margin` adds the soft barrier outside it. `vegetation_min_height_m` and `vegetation_height_buffer_m` set the vertical crop relative to ego height. `ignore_obstacles` also disables vegetation. Missing or invalid clouds clear the vegetation scene for that cycle.

`min_input_collision_activation_duration_s` keeps the conditional gate open for at least that many seconds after the last matching collision check. Every matching cycle refreshes the hold. The default `0.0` disables the hold. The timer uses the node's ROS clock, so paused simulation time pauses the hold; a backward clock jump clears it. Disabling MPPI, missing required inputs, short trajectories, parameter updates, route or map changes, and processing errors also clear it. The hold requests an MPPI optimization, but output validation can still reject the result. The diagnostic `input_collision/activation_source` reports `condition`, `hold`, or `none`; `~/debug/mppi/enabled` continues to report whether an optimized output was applied.

The check covers the first 80 input points, which are the points MPPI can replace. It samples between points and uses the same inflated vehicle footprint, object boxes, vegetation points, and road-border segment intersection routines as MPPI's output validation. The static-object condition uses the stationary avoidance targets selected by `autoware_avoidance_target_detector` and, when enabled, nearby vegetation points; it does not include moving vehicles or stationary objects filtered out by that selector. A matched trigger or activation hold does not override `min_optimization_length` or the two-point minimum. Set `min_optimization_length: 0.0` to disable the length check; a reference with no planar extent still cannot be optimized. A reference that fails these prerequisites clears the activation hold. A trigger cannot be enabled alongside its corresponding `ignore_obstacles` or `ignore_road_borders` option.

### Object prediction

The route-aware selector still owns which tracked objects reach MPPI. Moving vehicles on routably connected lanes are intentionally left to other modules. A conservative swept-path bounding-box check admits tracked objects whose supplied predictions cross the candidate corridor, and those objects remain admitted through the selector's distance check. The selector's classification and lane policy still apply.

Selected tracked objects are associated with supplied predicted objects by UUID. For each associated object, MPPI uses every path whose confidence equals the highest confidence, including all exact ties. Each tied path occupies a separate obstacle slot; confidence is used for selection, not as a probability weight in the collision cost. Missing associations or objects without predicted paths use the constant longitudinal-velocity, fixed-yaw fallback. Invalid confidence, malformed selected paths, duplicate UUIDs, or incompatible frames raise an error instead of silently dropping hypotheses.

Object poses are interpolated in XY and shortest-arc yaw at `(i + 1) × 0.1 s`, matching the ego post-step states. Beyond a supplied path's duration, its endpoint is held. World coordinates retain double precision until subtracting the ego-local origin. There is no message-age compensation or stale-object policy yet.

Capacity is still fixed at 64 obstacle hypotheses, including tied paths. Exceeding it rejects optimization; paths are never silently truncated. `dynamic_obstacle_horizon_s` limits moving hypotheses, while hypotheses classified as static remain active for the full horizon; zero means the full horizon.

### Nominal control and execution history

The nominal comes from the fitted reference curvature, a continuity-checked previous accepted control sequence, or the optional fresh MPC steering prefix. Temporal MPT and the plugin's steering EMA have been removed, along with their parameters. The generic interface control-postprocessor hook remains; states are replayed after it changes controls.

Both the first acceleration and first steering command receive colored-noise exploration. Previously issued controls are represented by actuator-delay queues rather than a frozen prefix of newly sampled controls. Physical constraints and the accepted standstill or short-reference steering hold can still constrain the sampled commands.

The plugin commits pending warm-start and execution history only when it selects an optimized result for application. Shadow, rejected, and fallback results do not commit the attempted optimization. This selection is not an acknowledgement from the downstream actuator controller.

Input point spacing is required to be 0.1 s. The plugin warns when adjacent `time_from_start` values differ by more than 1 microsecond from that spacing, or duration fields are invalid. It preserves the supplied timestamps and does not resample the reference; the warning does not reject the input.

### Rollout eligibility and output validation

Finite, collision-free rollouts are eligible for weighting. Eligible costs are normalized using the configured upper percentile and receive exponential weights; ESS adapts lambda for the next cycle. This is the current intentional MPPI variant. Zeroing a soft obstacle, road-border, or lateral weight does not disable its hard safety check. Ablation flags and object selection remain separate controls over scene coverage.

A weighted control sequence can be invalid even when contributing rollouts are valid, so the final reconstructed post-step states are validated separately. `validation_interpolation_substeps` accepts integers from 1 through 100 (default 1). With 1, only `x[1]` through `x[H]` are checked; larger values add evenly spaced geometric checks between those states, with matching fractional obstacle times. For example, 4 adds three checks per interval. This does not cover the initial ego-to-`x[1]` interval or guarantee continuous swept-volume clearance.

`skip_if_invalid: true` rejects an invalid optimized result and preserves input geometry, with the deterministic velocity-limit fallback when active. A cycle with no eligible rollouts always rejects, regardless of that option. The meaning of disabling final rejection and any future option to retain unsafe rollouts in weighting remain unresolved requirements; neither policy was changed. Returning the upstream input when MPPI fails is accepted behavior, not a guarantee that the fallback is safe.

## Live distance-texture visualization

Set `enable_distance_map_texture_debug: true` under `mppi_optimizer` to open the CUDA-OpenGL
distance-map viewer. It displays road-border, drivable-boundary, and dynamic-obstacle ESDF panels.
Drag the slider below the obstacle panel, or use the arrow/Home/End keys, to select its horizon
timestep. Red is at or inside the boundary, green is at or beyond the configured safe margin, and
gray means that texture channel is not valid yet. The viewer requires a graphical display; it is
disabled by default and closing its window stops its updates.

## Offline debug logging + retune

Enable CSV logging from the MPPI plugin parameters:

```yaml
mppi_optimizer:
  enable_debug_trajectory_log: true
  # Empty -> current working directory of the trajectory modifier node
  debug_trajectory_log_directory: ""
```

Each cycle writes:

```text
<debug_trajectory_log_directory>/
  index.csv
  cost_params.csv
  vehicle_params.csv
  runtime_options.csv
  000000_reference.csv
  000000_optimized.csv
  000000_ego.csv
  000000_nominal.csv
  000000_nominal_traj.csv
  000000_objects.csv
  000000_road_borders.csv
  000000_drivable.csv
  000000_rollout_diagnostics.csv
  ...
```

`*_ego.csv` stores the odometry / accel / steer initial condition used online.
Offline retune loads ego, cost/vehicle/runtime parameters, nominal controls, tracked objects, road/drivable geometry, preferred centerline, limits, and control/delay history where recorded. Additional per-frame CSV files carry this context. Supplied predicted-path hypotheses and vegetation are not serialized/replayed by this workflow, so a no-op retune is not guaranteed to reproduce the online scene. Rollout diagnostics include decoded collision reason, time, geometry index, and object UUID when available. The separate cost-test report parser now accepts encoded safety-status integers as well as legacy boolean values.

Various features can be disabled by changing the following parameters set in `mppi_optimizer.param.yaml`:

```yaml
ignore_obstacles: true
ignore_road_borders: true
ignore_drivable_area: true
force_cold_start_each_step: true
min_optimization_length: 0.0
steering_hold_reference_length_threshold_m: 0.5
use_last_control_as_nominal: true
max_lateral_jerk_mps3: 2.5
standstill_steer_rate_lim: 0.15
restart_steer_command_rate_lim: 0.15
restart_steer_command_acceleration_lim: 0.5
restart_velocity_threshold_mps: 0.5
use_mpc_predicted_trajectory_as_nominal_steering: false
mpc_predicted_trajectory_max_age_s: 0.5
nominal_initial_steering_max_deviation_rad: 0.1
```

Then restart the trajectory modifier and compare live MPPI to offline retune.

Notes:

- `ignore_obstacles` removes tracked/predicted obstacle hypotheses and vegetation from MPPI.
- `ignore_road_borders` drops static road-border segments before MPPI.
- `ignore_drivable_area` is retained as an ablation flag; on this stack boundary crash is already
  disabled in the cost (`isEgoOutsideDrivableArea` always false).
- `force_cold_start_each_step` invalidates only the reusable nominal horizon. It preserves actuator
  delay FIFOs and execution-history diagnostics.
- `min_optimization_length` skips MPPI for a stopping reference shorter than the configured arc
  length in meters; `0.0` disables the length-based skip.
- Rollout steering propagation smoothly releases `standstill_steer_rate_lim` up to
  `restart_velocity_threshold_mps`. Issued steering commands also obey the restart command-rate
  and command-acceleration limits. Above the release speed, physical steering rate is the smaller
  of hardware `steer_rate_lim` and `max_lateral_jerk_mps3 * wheel_base / velocity^2`.
- The diffusion-reference nominal derives curvature from a local least-squares path fit over
  `nominal_curvature_fit_window_m`. It advances through the path by predicted travel distance, so
  a stopped vehicle does not consume one noisy reference point per controller tick.
- Restart command continuity uses the last accepted command and command rate independently of the
  reusable trajectory warm start, which is still invalidated while stopped.
- On entry to the stopped state, MPPI latches the last accepted steering command (or the measured
  steering fallback) and holds it exactly. Rollouts release the hold once predicted absolute
  velocity reaches `last_control_warm_start_stop_exit_velocity_mps`, after which the restart
  command-rate and command-acceleration limits provide the transition back to optimized steering.
- The same command is held across the entire output when the current reference arc length is at or
  below `steering_hold_reference_length_threshold_m`. This also applies when short stopping
  references skip MPPI optimization. A reference with no planar extent also skips optimization
  and follows this steering-hold path. Set the threshold to `0.0` to disable the length trigger.
- `use_last_control_as_nominal` warm-starts `u_nom` from the previous applied MPPI result when its
  timestamp, plant replay, and shifted reference remain continuous. The elapsed timestamp selects
  the shift count, and the current reference seed fills the newly exposed tail. Up to three final
  reused samples blend toward the fresh seed for continuity.
- `use_mpc_predicted_trajectory_as_nominal_steering` replaces the nominal steering prefix with
  steering inferred from the fresh MPC-predicted path only when MPPI was not applied on the
  preceding primary-candidate cycle. The MPC trajectory is read from
  `~/input/mpc_predicted_trajectory`; `mpc_predicted_trajectory_max_age_s` rejects stale input.
  Acceleration and the nominal suffix continue to come from the reference seed or accepted
  warm-start sequence.
- `nominal_initial_steering_max_deviation_rad` limits `u_nom[0]` around the steering predicted when
  that command reaches the actuator after the existing steering-delay queue. A discontinuous
  reused horizon is discarded before the current-reference seed is clamped; `0.0` disables the
  guard.

### Replay only

```bash
ros2 run autoware_mppi_optimizer mppi_debug_visualizer.py -- \
  --log-dir /path/to/debug_trajectory_log_directory
```

### Batch retune (CLI)

```bash
ros2 run autoware_mppi_optimizer mppi_offline_retune -- \
  --log-dir /path/to/debug_trajectory_log_directory \
  --out-dir "$HOME/.cache/autoware/mppi_retune" \
  --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml \
  --set track_coeff=2000 --set steer_rate_coeff=5000 \
  --copy-reference
```

### Interactive compare + retune

Same plots as `mppi_debug_visualizer.py` (XY, heading, velocity, accel, steer, steer-rate,
rollout cost/weight distributions, and a stacked selected-output cost breakdown), with
diffusion reference (cyan), logged MPPI (red), and retuned MPPI (green):

```bash
# Option A — visualizer with retune panel
ros2 run autoware_mppi_optimizer mppi_debug_visualizer.py -- \
  --log-dir /path/to/debug_trajectory_log_directory \
  --enable-retune \
  --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml

# Option B — wrapper alias
ros2 run autoware_mppi_optimizer mppi_offline_tuner.py -- \
  --log-dir /path/to/debug_trajectory_log_directory \
  --params-yaml $(ros2 pkg prefix autoware_mppi_optimizer)/share/autoware_mppi_optimizer/config/mppi_optimizer.param.yaml
```

Adjust sliders, press **Retune** (or `r`). Overlay updates in place; metrics show max position/velocity error vs the reference.

## Review and test coverage

[Feature requirements and implementation status](FEATURE_REQUIREMENTS_REVIEW.md) preserves the user's feedback, subsequent decisions, and remaining TBD items. [GPU performance review](GPU_PERFORMANCE_REVIEW.md) traces the current execution path and distinguishes static findings from measurements still needed.

The `test/` directory covers prediction selection and confidence ties, capacity and malformed input, post-step obstacle timing, first-command noise, zero-weight safety checks, activation, output/engage modes, interpolated validation, warm-start/history behavior, and report parsing. CPU-capable and GPU-dependent cases are registered separately where applicable. GPU initialization errors fail the affected tests; absence of a CUDA device can skip GPU cases. Display/OpenGL-dependent cases can still skip when their environment is unavailable. The latency benchmark uses 20 optimizer iterations and an interface-level synthetic workload, so its timings are not production plugin latency guarantees.
