# Autoware MPPI Optimizer

The `autoware_mppi_optimizer` package optimizes trajectories with Model Predictive Path Integral (MPPI) control. Its trajectory processor plugin applies MPPI to the first candidate in an ordered processing pipeline.

## Overview

This package depends on the external [mppi_generic_vendor](https://github.com/autowarefoundation/mppi_generic_vendor) package (`MPPI-Generic`) and ships first-order Dubins path-tracking extensions under `include/mppi/`. A CUDA library interface (`FirstOrderDubinsMppiInterface`) wraps the two-lane double-park MPPI example for use from C++ / ROS nodes.

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

## Trajectory processor plugin

Configure `autoware::mppi_optimizer::plugin::TrajectoryMppiOptimizer` in the `plugin_names` list of `autoware_trajectory_processor`. Place it first to optimize the primary candidate before other modifiers and optimizers.

The plugin uses odometry, acceleration, steering, tracked objects, route, and raw lanelet map data from the processor. It returns the input points when MPPI is disabled, skipped, running in shadow mode, or reports an error. A rejected result preserves the input geometry and may still apply its deterministic velocity-limit fallback.

Plugin parameters are below `mppi_optimizer`. The `enabled` and `shadow_mode` parameters control result application. Debug topics are below `~/debug/mppi` in the trajectory processor node. The transient-local `enabled` debug topic is true only for a cycle where an optimized MPPI trajectory replaced the primary candidate; it is false when MPPI is disabled, skipped, rejected, or running in shadow mode.

### Steering output filtering

The curvature-adaptive EMA uses the magnitude of the preceding **filtered** steering as its turn
estimate. A new target cannot increase its own smoothing factor. `steering_filter_alpha_turn`
defaults to `0.5`, retaining smoothing through turns, turn exits, and reversals; setting it to `1`
explicitly allows unfiltered transitions once the preceding steering reaches the turn threshold.
The filter is not a hard steering-rate limiter.

When the nominal sequence is shifted from the previous accepted MPPI output, its first seed command
has already been filtered. The host preserves it only when GPU optimization leaves it unchanged;
otherwise the new first command is filtered with the rest of the horizon. The MPPI interface passes
the seed source, shift count, and preservation decision through
`FirstOrderDubinsMppiPostprocessingContext`, then recomputes predicted states from the final
controls.

Filter history is committed only for filtered output selected for publication. A limited fallback
is filtered from the pre-candidate history; an unfiltered fallback, skipped optimization, or error
invalidates history so resumed filtering starts from measured steering. Turn smoothing introduces
response lag, so assess tracking and clearance together with steering continuity when tuning alpha.

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
  # Empty -> current working directory of the trajectory processor node
  debug_trajectory_log_directory: ""
```

Each cycle writes:

```text
<debug_trajectory_log_directory>/
  index.csv
  cost_params.csv
  vehicle_params.csv
  000000_reference.csv
  000000_optimized.csv
  000000_ego.csv
  ...
```

`*_ego.csv` stores the odometry / accel / steer initial condition used online.
Offline retune loads ego + cost/vehicle params from the log so a no-op retune can match
the logged MPPI (obstacles are still not replayed).

Various features can be disabled by changing the following parameters set in `mppi_optimizer.param.yaml`:

```yaml
ignore_obstacles: true
ignore_road_borders: true
ignore_drivable_area: true
force_cold_start_each_step: true
min_optimization_length: 0.0
use_last_control_as_nominal: true
nominal_initial_steering_max_deviation_rad: 0.1
```

Then restart the trajectory processor and compare live MPPI to offline retune.

Notes:

- `ignore_obstacles` drops tracked objects before MPPI (matches offline's empty objects).
- `ignore_road_borders` drops static road-border segments before MPPI.
- `ignore_drivable_area` is retained as an ablation flag; on this stack boundary crash is already
  disabled in the cost (`isEgoOutsideDrivableArea` always false).
- `force_cold_start_each_step` invalidates only the reusable nominal horizon. It preserves actuator
  delay FIFOs and execution-history diagnostics.
- `min_optimization_length` skips MPPI for a stopping reference shorter than the configured arc
  length in meters; `0.0` disables the length-based skip.
- `use_last_control_as_nominal` warm-starts `u_nom` from the previous applied MPPI result when its
  timestamp, plant replay, and shifted reference remain continuous. The elapsed timestamp selects
  the shift count, and the current diffusion seed fills the newly exposed tail.
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
