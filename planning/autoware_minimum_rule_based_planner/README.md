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

## Turn signal

The `turn_indicators_command` field of every published candidate trajectory is filled in by
`TurnIndicatorDecider`. Both candidates get the same command: they share the path shape and differ
only in stop position.

Four maneuvers raise a signal. All distances below are arc lengths along the path, and the
activation distance is `max(v * 3.0 s, search_distance)`.

| Maneuver          | Lit when                                                                          | Cleared when                                        |
| ----------------- | --------------------------------------------------------------------------------- | --------------------------------------------------- |
| Intersection turn | a `turn_direction=left/right` lanelet starts within the activation distance       | ego's heading matches the heading the turn exits on |
| Private-area exit | the boundary where a `location=private` lanelet rejoins a public one is within it | as above                                            |
| Pull-out          | ego is STOPPED more than 1.5 m off the lane centerline (bus stop, shoulder)       | ego is back within 0.5 m of the centerline          |
| Pull-over         | an off-centerline goal (> 0.5 m) is within `search_distance`                      | ego has stopped within 1.0 m of the goal            |

The exit heading is the path heading 15 m past the maneuver, so the end condition is decided by
ego's pose rather than by where the lanelet happens to end. A private lanelet that carries a
`turn_direction` tag is handled by the intersection rule; the private case exists because such a
lanelet may be tagged `straight` or not tagged at all, and it then takes its side from the yaw
change across the merge (a merge that is geometrically straight raises no signal).

The first two rows differ only in how they are detected and where they start from; once found they
are signalled by the same rule and neither outranks the other - the one ego reaches first along the
path wins, so an exit ego is still completing keeps the light while the turn beyond it is already
in range. Pull-out and pull-over are only consulted when no such maneuver is lit, in that order.

Out of scope: lane changes and avoidance. While the upstream planner runs one, ego is laterally
offset from its lane and this module plans a path back to the centre - but the pull-out latch only
arms for a **stopped** vehicle, so no signal is raised. The 1.5 m departure threshold must therefore
stay above the lateral deviation a lane change or avoidance can produce. Pull-out is additionally
suppressed inside the pull-over range, so the two cannot fight over the direction at an
off-centerline goal.

Everything lives in `src/turn_indicator_decider.{hpp,cpp}`. The decision rules sit in the
`turn_indicator` namespace as pure functions (unit-tested on plain numbers); `TurnIndicatorDecider`
feeds them the path and lanelet geometry. The thresholds that are not exposed as parameters are
`constexpr` in the header.

## Parameters

{{ json_to_markdown("planning/autoware_minimum_rule_based_planner/schema/minimum_rule_based_planner.schema.json") }}

Parameters can be set via YAML configuration files in the `config/` directory.

Jerk-filtered smoother parameters are defined in `config/velocity_smoother/jerk_filtered_smoother.param.yaml`.
EB smoother parameters are defined in `config/trajectory_optimizer_plugins/elastic_band_smoother.param.yaml`.
