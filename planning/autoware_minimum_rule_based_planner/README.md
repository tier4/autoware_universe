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
- **Test mode**: Supports bypassing path planning by directly receiving a `PathWithLaneId` topic

## Inputs / Outputs

### Inputs

| Topic                            | Type                         | Description                       |
| -------------------------------- | ---------------------------- | --------------------------------- |
| `~/input/route`                  | `LaneletRoute`               | Planned route                     |
| `~/input/vector_map`             | `LaneletMapBin`              | HD map                            |
| `~/input/odometry`               | `Odometry`                   | Ego pose and velocity             |
| `~/input/acceleration`           | `AccelWithCovarianceStamped` | Ego acceleration                  |
| `~/input/objects`                | `PredictedObjects`           | Surrounding obstacles             |
| `~/input/test/path_with_lane_id` | `PathWithLaneId`             | Test mode: bypasses path planning |

### Outputs

| Topic                                 | Type                    | Description                                   |
| ------------------------------------- | ----------------------- | --------------------------------------------- |
| `~/output/candidate_trajectories`     | `CandidateTrajectories` | Planned trajectory (with turn signal)         |
| `~/debug/path_with_lane_id`           | `PathWithLaneId`        | Debug: planned path                           |
| `~/debug/trajectory`                  | `Trajectory`            | Debug: final output trajectory                |
| `~/debug/shifted_trajectory`          | `Trajectory`            | Debug: trajectory after path shifting         |
| `~/debug/optimizer/{name}/trajectory` | `Trajectory`            | Debug: trajectory after each optimizer plugin |
| `~/debug/modifier/{name}/trajectory`  | `Trajectory`            | Debug: trajectory after each modifier plugin  |
| `~/debug/processing_time_detail_ms`   | `ProcessingTimeDetail`  | Debug: processing time breakdown              |
| `~/debug/turn_indicator`              | `MarkerArray`           | Debug: turn signal state and its maneuver     |

## Turn signal

The `turn_indicators_command` field of every published candidate trajectory is filled in by
`TurnIndicatorDecider`. Both candidates get the same command: they share the path shape and differ
only in stop position.

Three maneuvers raise a signal, in this priority order:

| Maneuver          | Lit when                                                                                             | Cleared when                                                   |
| ----------------- | ---------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| Intersection turn | a `turn_direction=left/right` lanelet is within `max(v * search_time, intersection_search_distance)` | ego's heading matches the heading the turn exits on            |
| Private-area exit | a `location=private` run rejoining a public lane is within the same distance                         | as above                                                       |
| Pull-out          | ego is STOPPED more than `departure_lateral_threshold` off the lane centerline (bus stop, shoulder)  | ego is back within `lateral_shift_threshold` of the centerline |
| Pull-over         | an off-centerline goal is within `pull_over_search_distance`                                         | ego has stopped within `goal_arrival_distance` of the goal     |

Out of scope: lane changes and avoidance. While the upstream planner runs one, ego is laterally
offset from its lane and this module plans a path back to the centre - but the pull-out latch only
arms for a **stopped** vehicle, so no signal is raised. `departure_lateral_threshold` must therefore
stay above the lateral deviation a lane change or avoidance can produce. Pull-out is additionally
suppressed inside the pull-over range, so the two cannot fight over the direction at an
off-centerline goal.

## Parameters

{{ json_to_markdown("planning/autoware_minimum_rule_based_planner/schema/minimum_rule_based_planner.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.

Jerk-filtered smoother parameters are defined in `config/velocity_smoother/jerk_filtered_smoother.param.yaml`.
EB smoother parameters are defined in `config/trajectory_optimizer_plugins/elastic_band_smoother.param.yaml`.
