# MPPI cost-function review

This document describes the cost functions currently implemented by
`autoware_mppi_optimizer`. It is based on the first-order Dubins bicycle cost in
`include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.{cuh,cu}`, the host-side
integration in `src/first_order_dubins/first_order_dubins_mppi_interface.cu`, and the rollout and
weighting code supplied by `mppi_generic_vendor`.

The package contains one concrete rollout cost class,
`FirstOrderDubinsBicycleCost<NUM_TIMESTEPS>`. The files `moving_car_obstacles.hpp`,
`parked_car_obstacles.hpp`, `path_tracking_geometry.cuh`, `sat.cuh`, and the distance-map files are
geometry, scenario, or acceleration helpers; they do not define additional additive objectives.

## 1. Objective assembly

For the configured horizon `H = 80`, each rollout is integrated with `dt = 0.1 s`. A stage cost is
evaluated on every **post-step** state, including the final post-step state. A terminal cost is then
evaluated on that same final state:

```text
            H-1
J_rollout = ( sum [L_state(k) + L_control(k) + L_comfort(k) + L_sampling(k)]
             k=0
              + Phi(x_H) ) / H
```

Consequences:

- All running and terminal components are horizon-averaged; they are not multiplied by `dt`.
- The final state contributes once to the final running stage and once to the terminal cost.
- `L_sampling` is the MPPI-Generic likelihood-ratio term. The active colored-noise sampler inherits
  this term, but its `control_cost_coeff` array is left at zero by this package, so it currently
  contributes zero. It is not part of `FirstOrderDubinsMppiCostBreakdown`.
- A non-positive coefficient disables its corresponding ordinary quadratic term. Environment
  barriers have the activation rules described below.

The reported `FirstOrderDubinsMppiCostBreakdown` is reconstructed on the CPU using the same model
and cost equations, then divided by `H`. It describes the returned control sequence after optional
steering filtering and velocity-profile enforcement. It therefore need not equal the GPU baseline,
which is the minimum sampled rollout cost before those post-processing steps.

## 2. Notation and reference data

The formulas below use:

- `p = (x, y)`: rear-axle position of a post-step rollout state.
- `psi`: rollout yaw.
- `v`, `a`, and `delta`: longitudinal velocity, acceleration state, and steering state.
- `u_a` and `u_delta`: constrained acceleration and steering commands.
- `p_ref[k]` and `psi_ref[k]`: index-aligned reference position and yaw at stage `k`.
- `p_end` and `psi_end`: final pose of the full input reference trajectory, not merely the end of
  the 80-sample reference horizon.
- `wrap(q)`: shortest signed angular distance in `[-pi, pi]`.
- `C`: the full spatial reference corridor, capped/downsampled to 256 vertices.
- `s`: clamped cumulative chord length of the closest projection onto `C`.
- `s_total`: total corridor chord length.
- `e_lat`: signed closest-segment cross-track error; positive is left of the segment tangent.
- `[z]_+ = max(0, z)`.

The spatial corridor is preferred for all spatial costs. If it contains fewer than two points,
the index-aligned 80-sample reference polyline is used instead. GPU projection is warm-started from
the previous stage's closest segment; the first projection performs a full scan.

## 3. Cost summary

| Breakdown field                    | Running | Terminal | Main parameter                       | Raw quantity                                      |
| ---------------------------------- | :-----: | :------: | ------------------------------------ | ------------------------------------------------- |
| `spatial_overspeed`                |   yes   |    no    | `spatial_overspeed_coeff`            | progress-weighted positive velocity error squared |
| `track`                            |   yes   |   yes    | `track_coeff`                        | index-aligned rear-axle position error squared    |
| `heading`                          |   yes   |   yes    | `heading_coeff`                      | index-aligned wrapped heading error squared       |
| `terminal_error`                   |   no    |   yes    | `terminal_error_coeff`               | final XY error squared                            |
| `terminal_heading`                 |   no    |   yes    | `terminal_heading_coeff`             | final wrapped heading error squared               |
| `lateral_distance`                 |   yes   |   yes    | `lateral_distance_coeff`             | spatial cross-track error squared                 |
| `lateral_boundary`                 |   yes   |   yes    | derived from `crash_contact_penalty` | soft barrier near `boundary_threshold`            |
| `lateral_yaw_error`                |   yes   |   yes    | `lateral_yaw_error_coeff`            | heading error to closest segment squared          |
| `remaining_distance`               |   yes   |   yes    | `remaining_distance_coeff`           | remaining corridor length squared                 |
| `path_overshoot`                   |   yes   |   yes    | `path_overshoot_coeff`               | extension past corridor tip squared               |
| `track_center`                     |   yes   |   yes    | `track_center_coeff`                 | footprint-center position error squared           |
| `corner_buffer`                    |   yes   |   yes    | `corner_buffer_coeff`                | four-corner boundary-clearance violations         |
| `drivable_area`                    |   yes   |   yes    | `drivable_area_barrier_weight`       | footprint-to-boundary soft barrier                |
| `obstacle`                         |   yes   |   yes    | derived from `crash_contact_penalty` | footprint-to-OBB soft barrier                     |
| `road_border`                      |   yes   |   yes    | derived from `crash_contact_penalty` | footprint-to-segment soft barrier                 |
| `acceleration_command`             |   yes   |    no    | `accel_cmd_coeff`                    | acceleration command squared                      |
| `steering_command`                 |   yes   |    no    | `steer_cmd_coeff`                    | steering command squared                          |
| `lateral_acceleration`             |   yes   |    no    | `lateral_acceleration_coeff`         | modeled lateral acceleration squared              |
| `lateral_jerk`                     |   yes   |    no    | `lateral_jerk_coeff`                 | modeled lateral jerk squared                      |
| `longitudinal_jerk`                |   yes   |    no    | `longitudinal_jerk_coeff`            | modeled longitudinal jerk squared                 |
| `steering_rate`                    |   yes   |    no    | `steer_rate_coeff`                   | modeled, clamped steering rate squared            |
| `kinematic_velocity_overlimit`     |   yes   |    no    | `overlimit_coeff`                    | velocity interval violation squared               |
| `kinematic_acceleration_overlimit` |   yes   |    no    | `overlimit_coeff`                    | scaled acceleration interval violation squared    |
| `kinematic_jerk_overlimit`         |   yes   |    no    | `overlimit_coeff`                    | scaled jerk interval violation squared            |

`running_total`, `terminal_total`, and `total` are aggregates, not independent costs.

## 4. Reference-tracking costs

### 4.1 Index-aligned position tracking

```text
L_track(k) = track_coeff * ||p - p_ref[k]||^2
```

This tracks the rear axle. Despite being commonly called time-indexed tracking, the reference
builder currently selects input point `k + start_idx`; it does not interpolate the source
trajectory by `k * dt`. Its temporal meaning therefore depends on the sampling of the incoming
trajectory.

At the terminal state, the same term is evaluated against `p_ref[H-1]` and multiplied by
`track_terminal_scale`.

### 4.2 Index-aligned heading tracking

```text
L_heading(k) = heading_coeff * wrap(psi - psi_ref[k])^2
```

The terminal copy is also multiplied by `track_terminal_scale`.

### 4.3 Vehicle-center tracking

The rear-axle pose is shifted forward by `ego_axle_to_box_center`:

```text
p_center = p + ego_axle_to_box_center * (cos(psi), sin(psi))
L_center(k) = track_center_coeff * ||p_center - p_ref[k]||^2
```

This can be used instead of, or in addition to, rear-axle `track`. Its terminal copy is multiplied
by `track_terminal_scale`.

## 5. Terminal-goal costs

These independent terms compare the final rollout state to the end of the **full** input
trajectory:

```text
Phi_terminal_position = terminal_error_coeff * ||p_H - p_end||^2
Phi_terminal_heading  = terminal_heading_coeff * wrap(psi_H - psi_end)^2
```

They are not multiplied by `track_terminal_scale`. This permits the index-aligned tracking terms
to be disabled while retaining a strong full-trajectory terminal objective.

## 6. Spatial corridor costs

The rollout point is projected onto the closest segment of the full diffusion-reference polyline.
The projection provides `e_lat`, segment tangent yaw, `s`, remaining length, tip overshoot, and an
interpolated reference velocity.

### 6.1 Lateral distance

```text
L_lateral = lateral_distance_coeff * e_lat^2
```

At the terminal state this is multiplied by `track_terminal_scale`.

### 6.2 Lateral yaw error

```text
L_lateral_yaw = lateral_yaw_error_coeff * wrap(psi - psi_segment)^2
```

At the terminal state this is multiplied by `track_terminal_scale`.

### 6.3 Remaining distance

The projection arc length is clamped to `[0, s_total]`:

```text
s_remaining = s_total - s
L_remaining = remaining_distance_coeff * s_remaining^2
```

This term is applied at every running stage, so it rewards spatial progress throughout the
rollout. At the terminal state it is multiplied by `track_terminal_scale`.

### 6.4 Path overshoot

Overshoot is nonzero only when the closest projection lies past the final endpoint along the last
segment's forward extension:

```text
L_overshoot = path_overshoot_coeff * s_overshoot^2
```

At the terminal state this is multiplied by `track_terminal_scale`.

### 6.5 Spatial reference-velocity overspeed

Reference velocity is linearly interpolated between the vertices of the closest corridor segment.
The corridor is built after an active external/map velocity-limit profile has been applied, so the
profile may already include those limits.

```text
progress  = clamp(s / s_total, 0, 1)
overspeed = v - v_ref(s)
L_spatial_overspeed = spatial_overspeed_coeff * progress * [overspeed]_+^2
```

The term is disabled when `s_total <= 1e-6 m` and is deliberately omitted from the terminal cost.
Its progress multiplier makes reference overspeed nearly free at the path start and strongest at
the end.

### 6.6 Lateral-boundary barrier

The hard corridor threshold is symmetric about the reference:

```text
d_boundary = boundary_threshold - |e_lat|
L_lateral_boundary = W_lat * [lateral_boundary_soft_margin - d_boundary]_+^2
```

The interface derives:

```text
W_lat = crash_contact_penalty / max(lateral_boundary_soft_margin, 1e-3)^2
```

Thus the barrier begins at
`|e_lat| = boundary_threshold - lateral_boundary_soft_margin`, equals
`crash_contact_penalty` at `|e_lat| = boundary_threshold`, and grows quadratically beyond it. It is
also evaluated at the terminal state, but is **not** multiplied by `track_terminal_scale`.

## 7. Command and comfort costs

The rollout kernel constrains controls before dynamics integration and cost evaluation.

### 7.1 Command magnitude

```text
L_accel_command = accel_cmd_coeff * u_a^2
L_steer_command = steer_cmd_coeff * u_delta^2
```

These terms bias commands toward zero; they do not penalize changes relative to the nominal
sequence or the previous command.

### 7.2 Modeled longitudinal jerk and steering rate

Using the configured first-order actuator time constants:

```text
j_long = (u_a - a) / max(accel_time_constant, 1e-4)
delta_rate_raw = (u_delta - delta) / max(steer_time_constant, 1e-4)
delta_rate = clamp(delta_rate_raw, -max_steer_rate, max_steer_rate)

L_longitudinal_jerk = longitudinal_jerk_coeff * j_long^2
L_steering_rate     = steer_rate_coeff * delta_rate^2
```

Here `a` and `delta` come from the post-step output. See the actuator-delay caveat in the review
findings.

### 7.3 Lateral acceleration and jerk

```text
kappa       = tan(delta) / wheel_base
kappa_rate  = sec(delta)^2 * delta_rate / wheel_base
a_lateral   = v^2 * kappa
j_lateral   = v^2 * kappa_rate + 3 * v * a * kappa

L_lateral_acceleration = lateral_acceleration_coeff * a_lateral^2
L_lateral_jerk         = lateral_jerk_coeff * j_lateral^2
```

The steering rate used here is the same rate-limited value used by the steering-rate cost and the
bicycle dynamics.

## 8. Kinematic-limit costs

Optional intervals can be supplied for velocity, longitudinal acceleration, and longitudinal
jerk. The live plugin obtains velocity/acceleration/jerk constraints from `VelocityLimit`; an
optional lanelet velocity profile can add pointwise velocity bounds.

For an interval `[lower, upper]`, define:

```text
interval_violation(z) = [lower - z]_+^2 + [z - upper]_+^2
```

The component quantities are:

```text
q_velocity     = interval_violation(v)
q_acceleration = 0.5^2 * interval_violation(a)
q_jerk         = 0.2^2 * interval_violation(j_long)
```

Equivalently, acceleration violations are multiplied by `0.5` before squaring and jerk violations
by `0.2` before squaring. The shared uncapped cost is
`overlimit_coeff * (q_velocity + q_acceleration + q_jerk)`.

The complete per-stage kinematic-limit cost is capped at `crash_contact_penalty`. When the sum is
capped, the three diagnostic components are proportionally scaled so their sum remains equal to
the cap. Inactive intervals contribute zero. A pointwise reference velocity limit at stage `k`
sets the velocity interval to `[0, ref_max_velocity[k]]`.

These are soft costs. The dynamics separately clamp acceleration commands/states and, by default,
prevent reverse velocity.

## 9. Environment-clearance costs

All three environment terms use the generic quadratic barrier:

```text
B(d, margin, weight) = weight * [margin - d]_+^2
```

Distances are computed from a four-circle conservative approximation of the ego footprint. The
circle centers lie along the vehicle centerline and each circle encloses its corresponding
longitudinal box slice.

On the GPU, road/drivable distances use a 1024 x 1024, 0.15 m 2D texture and obstacle distances use
a 512 x 512 x H, 0.30 m 3D texture. Hardware interpolation smooths the sampled field. Samples
outside a texture grid fall back to exact segment/OBB geometry, rather than using clamped texture
coordinates. The CPU breakdown always uses exact geometry, so small texture-resolution differences
between optimized GPU costs and the reconstructed breakdown are expected.

### 9.1 Obstacle barrier

`d_obstacle` is the minimum signed clearance from the four-circle ego approximation to the closest
oriented obstacle box at stage `k`:

```text
L_obstacle = W_obstacle *
             [obstacle_collision_margin + obstacle_safe_margin - d_obstacle]_+^2

W_obstacle = crash_contact_penalty / max(obstacle_safe_margin, 1e-3)^2
```

The cost begins outside the collision envelope, equals `crash_contact_penalty` when physical
clearance reaches `obstacle_collision_margin`, and grows beyond that value during overlap. Static
and time-varying obstacle trajectories both participate. The implementation stores at most 64
obstacles.

### 9.2 Road-border barrier

`d_road` is the nonnegative minimum clearance from the four-circle ego approximation to a road
border segment:

```text
L_road = W_road *
         [road_border_collision_margin + road_border_safe_margin - d_road]_+^2

W_road = crash_contact_penalty / max(road_border_safe_margin, 1e-3)^2
```

It equals `crash_contact_penalty` at the collision-margin boundary. Up to 256 road-border segments
are stored.

### 9.3 Drivable-area boundary barrier

```text
L_drivable = drivable_area_barrier_weight *
             [drivable_area_safe_margin - d_drivable]_+^2
```

`d_drivable` is the minimum segment clearance minus the ego-circle radius, so it becomes negative
when the circle approximation overlaps a supplied boundary segment. Unlike the lateral, obstacle,
and road-border barriers, this weight is supplied directly and is not derived from
`crash_contact_penalty`. Up to 256 drivable-area boundary segments are stored.

This is a distance-to-segments field, not a polygon signed-distance field: it does not distinguish
the inside from the outside of the drivable area.

### 9.4 Corner buffer

For the four exact corners of the ego oriented box:

```text
L_corner = corner_buffer_coeff *
           sum_over_corners [corner_safe_margin - d_corner]_+^2
```

Each `d_corner` is the unsigned distance to the closest drivable-area boundary segment. The term is
disabled when no drivable segments are present or `corner_buffer_coeff <= 0`.

## 10. Terminal composition

The unscaled terminal cost is:

```text
Phi = track_terminal_scale *
        (track + heading + lateral_distance + lateral_yaw_error
         + remaining_distance + path_overshoot + track_center)
      + terminal_error + terminal_heading
      + lateral_boundary + corner_buffer
      + drivable_area + obstacle + road_border
```

The names in this expression denote their coefficient-weighted forms. Spatial overspeed, command,
comfort, and kinematic-limit costs do not have terminal copies. Setting `track_terminal_scale = 0`
does not disable the independent terminal pose terms or the terminal safety barriers.

## 11. Hard validation is not a rollout cost

After optimization and post-processing, `validateOptimizedTrajectory()` rejects the returned
trajectory at the first state that has any of these conditions:

- `|e_lat| >= boundary_threshold`;
- ego OBB (inflated by `obstacle_collision_margin`) overlaps an obstacle OBB;
- ego OBB (inflated by `road_border_collision_margin`) intersects a road-border segment;
- velocity is negative.

This validation does not add `crash_contact_penalty` to rollouts. The rollout cost now sets its
`crash_status` safety flag when it detects lateral-boundary contact, obstacle contact, or road-border
contact. The flag is reduced to `unsafe_rollout_fraction` for temperature adaptation; it does not
itself add another cost. `crash_contact_penalty` serves three cost-calibration roles:

1. calibration target for the lateral-boundary barrier;
2. calibration target for obstacle and road-border barriers;
3. numerical cap for the combined kinematic-limit cost at one running stage.

There is no hard drivable-area polygon validation. The runtime `skip_if_invalid` option determines
whether a failed optimized output is rejected.

## 12. Cost normalization and MPPI weights

After raw rollout costs are produced, a three-level GPU histogram estimates the configured upper
percentile (95% by default). Costs above it are clamped before normalization:

```text
J_upper        = percentile(J, cost_normalization_percentile)
S_i_normalized = clamp((J_i - min(J)) / (J_upper - min(J)), 0, 1)
w_i_raw        = exp(-S_i_normalized / lambda)
w_i            = w_i_raw / sum(w_raw)
ESS            = 1 / sum(w_i^2)
```

If the retained finite cost range is below `1e-6`, finite rollouts at the retained percentile receive
normalized cost zero while any upper-tail rollouts still receive one. A non-finite rollout receives
normalized cost one when any finite rollout exists; if all rollouts are non-finite, weights are
uniform.

Robust normalization has an important tuning implication: multiplying **all** cost coefficients
by a common positive scalar normally leaves the weights unchanged. Coefficients matter primarily
through the relative balance and shape of terms.

Lambda is fixed across all optimization iterations in one control step. After the final iteration,
ESS adapts the lambda prepared for the next control step:

```text
lambda_next = clamp(
  lambda_used * exp(lambda_adaptation_gain * (target_ess_ratio - ESS / N)),
  lambda_min,
  lambda_max)
```

If the final iteration's `unsafe_rollout_fraction` reaches
`unsafe_rollout_fraction_threshold`, `lambda_next` is forced to `lambda_max`. Tracking, terminal,
comfort, and velocity-limit costs cannot activate this override.

## 13. Parameter sources and shipped configuration

There are three parameter representations:

- `param/trajectory_mppi_optimizer_parameters.yaml`: generated ROS parameter schema and defaults;
- `config/mppi_optimizer.param.yaml`: shipped runtime override values;
- `FirstOrderDubinsMppiCostParams`: direct C++ API and offline-tool defaults.

They are not identical, so a value should always be interpreted with its source. With the current
shipped `config/mppi_optimizer.param.yaml`, the main cost settings are:

| Family                       | Current shipped settings                                                               |
| ---------------------------- | -------------------------------------------------------------------------------------- |
| Weighting                    | `lambda=0.1`, bounds `[0.01, 1.0]`, target ESS ratio `0.2`, robust percentile `0.95`   |
| Index tracking               | `track_coeff=0`, `heading_coeff=0`, `track_terminal_scale=1`                           |
| Independent terminal         | `terminal_error_coeff=2500`, `terminal_heading_coeff=2000`                             |
| Spatial tracking             | `lateral_distance_coeff=200`, `lateral_yaw_error_coeff=50`                             |
| Spatial progress             | `remaining_distance_coeff=0.02`, `path_overshoot_coeff=50`                             |
| Spatial velocity             | `spatial_overspeed_coeff=50`                                                           |
| Alternate footprint tracking | `track_center_coeff=0`, `corner_buffer_coeff=200`, corner margin `0.3 m`               |
| Command                      | `accel_cmd_coeff=0.5`, `steer_cmd_coeff=20`                                            |
| Comfort                      | lateral acceleration `10`, lateral jerk `1`, longitudinal jerk `2`, steering rate `20` |
| Kinematic interval           | effective `overlimit_coeff=10000`                                                      |
| Lateral barrier              | threshold `1.2 m`, soft margin `0.5 m`, target penalty `50000`                         |
| Obstacle barrier             | collision margin `0.2 m`, safe margin `0.5 m`, target penalty `50000`                  |
| Road-border barrier          | collision margin `0.2 m`, safe margin `0.5 m`, target penalty `50000`                  |
| Drivable-area barrier        | safe margin `0`, weight `0` (disabled)                                                 |

The active cost mix is therefore primarily terminal pose, spatial corridor/progress/overspeed,
command effort, comfort, kinematic limits when supplied, and obstacle/road/lateral barriers.

## 14. Review findings

### 14.1 Drivable-area cost is not an inside/outside cost

The 2D drivable texture stores unsigned point-to-segment distance. Subtracting the ego-circle
radius makes contact negative, but a vehicle far outside the boundary again has a large positive
distance and no barrier cost. `pointInPolygon()` exists as a geometry helper but is not used, and
`setDrivableAreaPolygon()`/`clearDrivableArea()` are declared without an implementation. The hard
validator also does not check drivable-area containment. If drivable containment is required, this
should become a true signed distance or an explicit polygon/occupancy check.

### 14.2 Actuator-delay handling is inconsistent in comfort costs

The dynamics applies delayed acceleration and steering commands from its FIFO, but `comfortTerms()`
uses the newly issued `u_a` and `u_delta` directly. It also uses the post-step acceleration and
steering states. With input delay enabled, the reported `j_long`, `delta_rate`, and derived lateral
jerk are therefore not the actuator derivatives that produced that step. Even without delay, they
represent the derivative at the post-step state under a repeated current command rather than the
explicit-Euler derivative used during the step. This can make comfort tuning disagree with plant
motion, especially during command transitions.

### 14.3 `overlimit_coeff` is not connected to the generated plugin parameters

`overlimit_coeff` exists in `FirstOrderDubinsMppiCostParams`, the direct ROS helper, offline tools,
and the shipped config. It is absent from `trajectory_mppi_optimizer_parameters.yaml` and is not
copied in `TrajectoryMppiOptimizer::make_cost_params()`. The live trajectory-processor plugin thus
uses the C++ struct default (`10000`) rather than a generated parameter value. The shipped value
happens to match that default, masking the issue.

### 14.4 Several cost components are omitted from live diagnostics

`FirstOrderDubinsMppiCostBreakdown::componentTotal()` includes `lateral_boundary` and all three
kinematic overlimit components, but `publish_cost_diagnostics()` does not publish those four keys.
The Python visualizer expects `state/lateral_boundary` and `kinematic/*_overlimit`, so these terms
can be active and included in totals while appearing absent from the live stacked breakdown.

### 14.5 `crash_contact_penalty` is named and documented like a direct hard-crash cost

No direct contact penalty or persistent crash status is added by this cost class. The parameter is
a barrier calibration target and kinematic-cost cap, while hard collisions are handled only after
optimization. Renaming it or clarifying the generated parameter description would reduce tuning
errors.

### 14.6 Spatial projection assumes locally continuous progress

The GPU uses a closest-segment warm start and local hill-climb after the first stage. This is fast
for an ordinary, forward-moving corridor, but a self-intersecting path or a rollout that jumps to a
distant branch can converge to a local rather than global closest segment. That affects every
spatial cost and the hard lateral validation.

### 14.7 Reference velocity is assumed finite

Corridor reference velocities are copied and interpolated without an `isfinite` check. A NaN makes
the `overspeed > 0` condition false and silently disables spatial overspeed at that projection.
Input trajectory validation should guarantee finite velocity values or sanitize this array before
upload.

### 14.8 Raw-cost variance telemetry counts non-finite rollouts inconsistently

The fused CUDA statistics kernel excludes non-finite raw costs from its moment sums, but the host
divides those sums by the total rollout count rather than the finite rollout count. This does not
change the rollout weights or any cost term, but it biases free-energy variance telemetry whenever
some rollouts are non-finite.

### 14.9 The distance-map viewer is not a direct cost visualization

The OpenGL viewer displays point-to-segment/OBB texture values. The rollout cost subsequently
accounts for the ego circle radius and, for obstacle and road-border costs, adds the collision
margin to the barrier activation distance. The viewer is passed only `obstacle_safe_margin` and
`road_border_safe_margin`. Its red/green transition is therefore useful for inspecting the raw
fields, but it does not mark the exact clearance at which the footprint-level rollout barriers
activate.

## 15. Practical tuning notes

- Tune relative families, not a common global scale, because current-rollout robust normalization
  removes most global scaling effects.
- Keep `terminal_error_coeff` separate from `track_terminal_scale`: the former targets the full
  input endpoint, while the latter scales several terminal copies of running spatial/index costs.
- `remaining_distance_coeff` is accumulated at every stage; even a small coefficient can dominate
  a long corridor because the early-horizon remaining distance is large and squared.
- `lateral_boundary_soft_margin` controls both where the barrier begins and its derived curvature.
  Increasing it starts the barrier earlier but lowers its derived weight so the cost still reaches
  `crash_contact_penalty` at the hard threshold.
- Obstacle and road safe margins similarly alter both activation range and derived weight.
- Setting `drivable_area_barrier_weight` to zero disables only the drivable boundary field. The
  lateral, obstacle, and road barriers remain calibrated from `crash_contact_penalty`.
- A post-filtered or velocity-profile-overwritten output should be assessed with `output_total_cost`;
  the sampled `baseline_cost` describes a different sequence and is not expected to match exactly.
