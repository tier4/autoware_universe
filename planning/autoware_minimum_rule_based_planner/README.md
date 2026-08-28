# autoware_minimum_rule_based_planner

## Overview

A minimum rule-based trajectory planner that generates safe and feasible trajectories for autonomous driving. It follows the planned route by constructing trajectories from lanelet centerline and applies multi-stage optimization for geometric smoothness, velocity profiles, and obstacle avoidance.

## Features

- **Centerline-based path planning**: Generates paths along lanelet centerline from the HD map, extending backward and forward from the ego vehicle's position
- **Goal handling**: Selected by `path_planning.early_stop.enable`. When disabled (default), the path is refined near the goal pose so that ego reaches the goal itself (smooth goal connection). When enabled, the path is instead cut short before the goal so that ego stops in front of it without pulling over to the side of the road
- **Path shifting**: Shifts the centerline path to start from the ego vehicle's current pose, using curvature-aware shift distance calculation based on ego velocity and lateral acceleration limits
- **Trajectory smoothing**: Applies an Elastic Band smoother for geometric smoothing via a plugin interface
- **MPPI optimization**: Optionally refines the smoothed trajectory before deterministic collision and stop-rule processing. The planner uses the `TrajectoryMppiOptimizer` plugin and falls back to the unmodified trajectory when MPPI is unavailable, disabled, running in shadow mode, or rejects its result
- **Trajectory modification**: Applies modifier plugins (e.g., obstacle stop) for safety modifications
- **Map-based stop planning**: Embeds stop points at map-defined stop targets (stop lines, walkways, crosswalks, traffic lights, intersections, private areas) and publishes "Go" / "Stop" candidate trajectories
- **Velocity optimization**: Computes a jerk-filtered velocity profile respecting constraints on acceleration, jerk, and lateral acceleration
- **Test mode**: Supports bypassing path planning by directly receiving a `PathWithLaneId` topic

## Inputs / Outputs

### Inputs

| Topic                                 | Type                         | Description                                                          |
| ------------------------------------- | ---------------------------- | -------------------------------------------------------------------- |
| `~/input/route`                       | `LaneletRoute`               | Planned route                                                        |
| `~/input/vector_map`                  | `LaneletMapBin`              | HD map                                                               |
| `~/input/odometry`                    | `Odometry`                   | Ego pose and velocity                                                |
| `~/input/acceleration`                | `AccelWithCovarianceStamped` | Ego acceleration                                                     |
| `~/input/steering_status`             | `SteeringReport`             | Current steering angle for MPPI                                      |
| `~/input/objects`                     | `PredictedObjects`           | Surrounding obstacles                                                |
| `~/input/tracked_objects`             | `TrackedObjects`             | Native MPPI obstacle input; predicted objects are used as a fallback |
| `~/input/pointcloud`                  | `PointCloud2`                | Obstacle point cloud for trajectory modifiers                        |
| `~/input/external_velocity_limit_mps` | `VelocityLimit`              | Optional external velocity limit consumed by MPPI                    |
| `~/input/test/path_with_lane_id`      | `PathWithLaneId`             | Test mode: bypasses path planning                                    |

### Outputs

| Topic                                 | Type                    | Description                                                        |
| ------------------------------------- | ----------------------- | ------------------------------------------------------------------ |
| `~/output/candidate_trajectories`     | `CandidateTrajectories` | Planned trajectory                                                 |
| `~/debug/path_with_lane_id`           | `PathWithLaneId`        | Debug: planned path                                                |
| `~/debug/trajectory`                  | `Trajectory`            | Debug: final output trajectory                                     |
| `~/debug/shifted_trajectory`          | `Trajectory`            | Debug: trajectory after path shifting                              |
| `~/debug/optimizer/{name}/trajectory` | `Trajectory`            | Debug: trajectory after generic optimizers such as the EB smoother |
| `~/debug/mppi/*`                      | Multiple                | MPPI references, result, markers, and enable state                 |
| `~/debug/modifier/{name}/trajectory`  | `Trajectory`            | Debug: trajectory after each modifier plugin                       |
| `~/debug/processing_time_detail_ms`   | `ProcessingTimeDetail`  | Debug: processing time breakdown                                   |

## Parameters

{{ json_to_markdown("planning/autoware_minimum_rule_based_planner/schema/minimum_rule_based_planner.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.

Jerk-filtered smoother parameters are defined in `config/velocity_smoother/jerk_filtered_smoother.param.yaml`.
EB smoother parameters are defined in `config/trajectory_optimizer_plugins/elastic_band_smoother.param.yaml`.
The common MPPI defaults are loaded from `autoware_mppi_optimizer`, then overridden by
`config/mppi_optimizer.param.yaml` for this planner.
