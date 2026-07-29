# autoware_minimum_rule_based_planner

## Overview

A minimum rule-based trajectory planner that generates safe and feasible trajectories for autonomous driving. It follows the planned route by constructing trajectories from lanelet centerline and applies multi-stage optimization for geometric smoothness, velocity profiles, and obstacle avoidance.

## Features

- **Centerline-based path planning**: Generates paths along lanelet centerline from the HD map, extending backward and forward from the ego vehicle's position
- **Smooth goal connection**: Refines the path near the goal pose for smooth stopping
- **Path shifting**: Shifts the centerline path to start from the ego vehicle's current pose, using curvature-aware shift distance calculation based on ego velocity and lateral acceleration limits
- **Trajectory smoothing**: Applies an Elastic Band smoother for geometric smoothing via a plugin interface
- **Trajectory modification**: Applies modifier plugins (e.g., obstacle stop) for safety modifications
- **Map-based stop planning**: Embeds stop points at map-defined stop targets (stop lines, walkways, crosswalks, traffic lights, intersections, private areas) and publishes "Go" / "Stop" candidate trajectories
- **Velocity optimization**: Computes a jerk-filtered velocity profile respecting constraints on acceleration, jerk, and lateral acceleration
- **Joint shape / velocity optimization (optional)**: `acados_mpt.enable` replaces the two stages above with a single arc-length parameterised NLP (`acados_mpt_optimizer`) that decides the path shape and its speed profile together, with curvature, speed, acceleration, jerk, lateral acceleration and steering angle / rate as constraints instead of post-hoc filters
- **Test mode**: Supports bypassing path planning by directly receiving a `PathWithLaneId` topic

## Inputs / Outputs

### Inputs

| Topic                            | Type                         | Description                                                                         |
| -------------------------------- | ---------------------------- | ----------------------------------------------------------------------------------- |
| `~/input/route`                  | `LaneletRoute`               | Planned route                                                                       |
| `~/input/vector_map`             | `LaneletMapBin`              | HD map                                                                              |
| `~/input/odometry`               | `Odometry`                   | Ego pose and velocity                                                               |
| `~/input/acceleration`           | `AccelWithCovarianceStamped` | Ego acceleration                                                                    |
| `~/input/objects`                | `PredictedObjects`           | Surrounding obstacles                                                               |
| `~/input/steering`               | `SteeringReport`             | Ego steering angle; only used by the acados MPT optimizer, as its initial curvature |
| `~/input/test/path_with_lane_id` | `PathWithLaneId`             | Test mode: bypasses path planning                                                   |

### Outputs

| Topic                                     | Type                    | Description                                                             |
| ----------------------------------------- | ----------------------- | ----------------------------------------------------------------------- |
| `~/output/candidate_trajectories`         | `CandidateTrajectories` | Planned trajectory                                                      |
| `~/debug/path_with_lane_id`               | `PathWithLaneId`        | Debug: planned path                                                     |
| `~/debug/trajectory`                      | `Trajectory`            | Debug: final output trajectory                                          |
| `~/debug/shifted_trajectory`              | `Trajectory`            | Debug: trajectory after path shifting                                   |
| `~/debug/optimizer/{name}/trajectory`     | `Trajectory`            | Debug: trajectory after each optimizer plugin                           |
| `~/debug/optimizer/acados_mpt/trajectory` | `Trajectory`            | Debug: go trajectory after the acados MPT optimizer (only when enabled) |
| `~/debug/modifier/{name}/trajectory`      | `Trajectory`            | Debug: trajectory after each modifier plugin                            |
| `~/debug/processing_time_detail_ms`       | `ProcessingTimeDetail`  | Debug: processing time breakdown                                        |

## Parameters

{{ json_to_markdown("planning/autoware_minimum_rule_based_planner/schema/minimum_rule_based_planner.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.

Jerk-filtered smoother parameters are defined in `config/velocity_smoother/jerk_filtered_smoother.param.yaml`.
EB smoother parameters are defined in `config/trajectory_optimizer_plugins/elastic_band_smoother.param.yaml`.

## acados MPT optimizer

`acados_mpt.enable` (default `false`, read-only) runs the optimiser; `acados_mpt.shadow` (default
`true`) decides whether anything downstream sees it.

|                                           | `enable: false`                             | `enable: true`, `shadow: true`                         | `enable: true`, `shadow: false`                        |
| ----------------------------------------- | ------------------------------------------- | ------------------------------------------------------ | ------------------------------------------------------ |
| 5. smooth path                            | EB smoother                                 | EB smoother                                            | skipped (the NLP decides the shape)                    |
| 8. velocity optimization                  | `VelocitySmoother` + `JerkFilteredSmoother` | unchanged; the NLP is solved for the go candidate only | `acados_mpt_optimizer`, once per candidate (go / stop) |
| published trajectory                      | as today                                    | **as today**                                           | from the NLP                                           |
| `~/debug/optimizer/acados_mpt/trajectory` | -                                           | the NLP result                                         | the NLP result                                         |

Shadow mode is the way to compare the two without putting the NLP in the control loop; it also
shows the solve cost in `~/debug/processing_time_detail_ms` (`acados_mpt`), which is worth checking
before switching `shadow` off - the go candidate is the expensive one, and a curved route has been
measured at 134 ms against the 100 ms planning period.

The NLP is fed the trajectory as it leaves the modifiers and the map-based stop planner: its point
velocities become per-point speed caps, and the first zero becomes the stop station. Deciding shape
first and speed second means the shape can be made infeasible and the speed planner pays for it in
deceleration - curvature itself can never be fixed by a velocity filter. Here both are decided
together, so the optimiser can choose between slowing down and reshaping.

A solved problem is not a satisfied one: the nonlinear limits are slacked, so every solve is
re-verified against the limits and the dynamics residual. When that verification fails - or the
solver does - the cycle falls back to the EB smoother plus the velocity smoother, which stay loaded
for exactly that reason, and logs why. In shadow mode the debug topic simply stops updating.

Two things the NLP needs from the node, both learned the hard way in lsim:

- **the measured curvature** (`tan(steer) / wheel_base`, from `~/input/steering`). The initial
  curvature is a hard constraint, so a wrong one cannot be smoothed away - the trajectory has to
  drive the arc it was given and come back. Estimated from the input path around the ego it spans
  only ~1.5 m, which turns the few centimetres of lateral offset an ego always has into a curvature
  of the wrong sign; that produced a 1.5 m bulge on a straight lane.
- **an engage speed** when pulling out. The NLP pins its initial speed to the ego's, so from a
  standstill the profile starts at exactly zero and the longitudinal controller never leaves its
  stopped state. The same `velocity_smoother.set_engage_speed` parameters apply, except that the
  floor stops at the next stop point instead of painting over it.

Not wired up yet: the drivable-area / object corridor (the path shape is optimised against the
limits only), and dynamic objects.
