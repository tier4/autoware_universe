# autoware_minimum_rule_based_planner

## Overview

A lightweight rule-based trajectory planner that generates safe and feasible trajectories for autonomous driving. It follows the planned route by constructing trajectories from lanelet centerlines and applies multi-stage optimization for geometric smoothness, velocity profiles, and obstacle avoidance.

## Features

- **Centerline-based path planning**: Generates paths along lanelet centerlines from the HD map, extending backward and forward from the ego vehicle's position
- **Path shifting**: Smoothly shifts the centerline path to start from the ego vehicle's current pose using spline or quintic polynomial interpolation
- **Trajectory optimization**: Applies an Elastic Band (EB) smoother for geometric optimization via a plugin interface
- **Velocity optimization**: Computes a jerk-filtered velocity profile respecting constraints on acceleration, jerk, and lateral acceleration
- **Obstacle stop**: Stops the vehicle before detected obstacles using the predicted objects input

## Processing Pipeline

The planner runs at a configurable frequency (default: 10 Hz) and executes the following steps in order:

1. **Path planning** — Build a path from lanelet centerlines along the route
2. **Path shifting** — Align trajectory start with ego pose
3. **Trajectory smoothing** — Apply optimizer plugins (e.g., EB smoother)
4. **Trajectory modification** — Apply modifier plugins (e.g., obstacle stop)
5. **Velocity optimization** — Apply jerk-filtered velocity smoothing

## Inputs / Outputs

### Inputs

| Topic                  | Type                         | Description           |
| ---------------------- | ---------------------------- | --------------------- |
| `~/input/route`        | `LaneletRoute`               | Planned route         |
| `~/input/vector_map`   | `LaneletMapBin`              | HD map                |
| `~/input/odometry`     | `Odometry`                   | Ego pose and velocity |
| `~/input/acceleration` | `AccelWithCovarianceStamped` | Ego acceleration      |
| `~/input/objects`      | `PredictedObjects`           | Surrounding obstacles |

### Outputs

| Topic                             | Type                    | Description        |
| --------------------------------- | ----------------------- | ------------------ |
| `~/output/candidate_trajectories` | `CandidateTrajectories` | Planned trajectory |

## Architecture

The planner uses a plugin-based architecture for trajectory optimizers and modifiers:

- **Optimizer plugins** (e.g., `eb_smoother`): Applied for geometric trajectory smoothing
- **Modifier plugins** (e.g., `obstacle_stop_modifier`): Applied for safety modifications such as stopping before obstacles

## Parameters

Key parameters are defined in `config/minimum_rule_based_planner_parameters.yaml`:

| Parameter                             | Default   | Description                      |
| ------------------------------------- | --------- | -------------------------------- |
| `planning_frequency_hz`               | `10.0`    | Planning loop frequency          |
| `path_length.backward`                | `5.0 m`   | Path length behind ego           |
| `path_length.forward`                 | `100.0 m` | Path length ahead of ego         |
| `path_shift.enable`                   | `true`    | Enable path shifting to ego pose |
| `post_resample.max_trajectory_length` | `300.0 m` | Maximum output trajectory length |
| `post_resample.min_trajectory_length` | `30.0 m`  | Minimum output trajectory length |

Velocity smoother and optimizer parameters are defined in their respective config files under `config/`.
