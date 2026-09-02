# Autoware ML Planner

## Overview

The **Autoware ML Planner** is a trajectory generation module for autonomous vehicles, designed to work within the [Autoware](https://autoware.org/) ecosystem. It leverages the [ML Planner](https://github.com/ZhengYinan-AIR/Diffusion-Planner) model, as described in the paper ["Diffusion-Based Planning for Autonomous Driving with Flexible Guidance"](https://arxiv.org/abs/2501.15564) by Zheng et al. <!-- cSpell:ignore Zheng -->

This planner generates smooth, feasible, and safe trajectories by considering:

- Dynamic and static obstacles
- Vehicle kinematics
- User-defined constraints
- Lanelet2 map context
- Traffic signals and speed limits

It is implemented as a ROS 2 component node, making it easy to integrate into Autoware-based stacks. The node is aimed at working within the proposed [Autoware new planning framework](https://github.com/tier4/new_planning_framework).

---

## How to use

### (1) Prerequisites

Make sure that the directory specified in `planning/autoware_ml_planner/config/ml_planner.param.yaml` points to the correct model version and contains the required model weight and parameter files.

```bash
$ ls ~/autoware_data/ml_models/ml_planner/v4.0/
ml_planner.onnx ml_planner.param.json
```

This can be downloaded by following [Download artifacts](https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/README.md#download-artifacts).

### (2) Launch the planning simulator

Pass `planning_setting:=ml_planner` to switch the planning stack from the rule-based scenario planner to the ML planner. This argument automatically swaps the trajectory generator, the planning validator input topic, and the diagnostics graph, so no additional launch-file edits are required.

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/path/to/your/map \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  planning_setting:=ml_planner
```

## Features

- **Diffusion-based trajectory generation** for flexible and robust planning

  [![Diffusion-Based trajectory generation](media/ml_planner.gif)](media/ml_planner.gif)

- **Integration with Lanelet2 maps** for lane-level context

  [![Lanelet Map Integration](media/lanelet_map_integration.png)](media/lanelet_map_integration.png)

- **Dynamic and static obstacle handling** using perception inputs

  [![Static Agent Reaction](media/ml_planner_reacts_to_bus.gif)](media/ml_planner_reacts_to_bus.gif)

  [![ML Planner](media/reaction_to_other_agents.gif)](media/reaction_to_other_agents.gif)

- **Traffic signal and speed limit awareness**

  [![Traffic Light Support](media/traffic_light_support.gif)](media/traffic_light_support.gif)

- **ONNX Runtime** inference for fast neural network execution
- **ROS 2 publishers** for planned trajectories, predicted objects, and debug markers

---

## Trajectory optimization

The raw model output is a noisy, position-only 80-point sequence (x, y, cos(yaw), sin(yaw) at t = 0.1 .. 8.0 s) that does not necessarily start at base_link. When `trajectory_optimization.enable` is true, an [acados](https://docs.acados.org/)-based OCP refines it before publishing:

- **Model**: kinematic bicycle with steering-angle state — states (x, y, yaw, v, delta), inputs (acceleration, steering rate)
- **Horizon**: N = 80, dt = 0.1 s (aligned 1:1 with the model output)
- **Cost**: tracks the raw positions (and weakly heading), regularizes acceleration and steering rate for smoothness. The position error is split along the reference heading — longitudinal (schedule/velocity-profile freedom) and lateral (path deviation) errors are weighted separately via a per-stage rotated 2x2 weight block. No velocity/acceleration/steering references exist (the model outputs poses only, and none are fabricated by finite differences); the velocity profile emerges from the time-indexed position tracking, the comfort regularization, and the bounds
- **Constraints**: initial state fixed to the current ego state (base_link pose, odometry velocity, measured steering angle), velocity/steering/input box bounds, soft lateral acceleration bound
- **Output**: 80 points (t = 0.1 .. 8.0 s, same timing convention as the raw output) that are dynamically consistent with the current ego state, with velocity, acceleration, steering angle, and heading rate profiles

The OCP is defined in `scripts/generate_solver.py` (vehicle model in `scripts/vehicle_model.py`); the C code is generated at build time into the build tree. Cost weights and constraint bounds are injected at node startup from ROS parameters, so tuning does not require rebuilding. Building requires acados at `ACADOS_SOURCE_DIR` (default `/opt/acados`) with its Python venv; without it the package builds with optimization support disabled.

`scripts/run_optimizer_offline.py` solves the same OCP offline against a recorded or synthetic reference and plots the result for tuning:

```bash
cd scripts
/opt/acados/.venv/bin/python3 run_optimizer_offline.py --output result.png
```

Debug topics (published while the optimizer runs): `~/debug/optimization/raw_trajectory` (raw model output), `~/debug/optimization/solver_status`, `~/debug/optimization/solve_time_ms`. On solver failure the raw trajectory is published unchanged.

### Road border avoidance

When `road_border_avoidance.enable` is true, the raw model output is checked against the road borders of the lanelet map before being handed to the trajectory optimization. For every trajectory point the ego footprint, inflated by `footprint_margin_m`, is placed at the point's pose and tested for overlap with the road border line strings (boost::geometry). Overlapping points are shifted perpendicular to their heading, away from the nearest border, in `shift_step_m` increments until the footprint clears all borders (the total offset is capped at `max_lateral_shift_m`; if the cap is reached the shift is kept as best effort and a warning is logged). With `propagate_shift` enabled (default) the offset is carried over to all subsequent points along each point's own lateral direction, so the path stays shifted after passing the border instead of snapping back to the raw output. Yaw and all other fields stay untouched. Debug topics: `~/debug/road_border_avoidance/adjusted_trajectory` (the shifted reference) and `~/debug/road_border_avoidance/shifted_point_count`.

### Stop point fixing

When `stop_point_fixing.enable` is true, the optimized trajectory is post-processed so that a stopping maneuver ends in a clean stop. The first point at or below `velocity_threshold_mps` after at least `min_deceleration_duration_sec` of continuous deceleration becomes the stop point. A non-decelerating point resets the measured duration. The stop point and all subsequent points are fixed to its pose with zero velocity, acceleration, and heading rate. The trajectory before the fixing is published on `~/debug/stop_point_fixing/unfixed_trajectory`. This prevents the small residual velocities of the optimized solution from making the vehicle creep past the intended stop point.

---

## Parameters

{{ json_to_markdown("planning/autoware_ml_planner/schema/ml_planner.schema.json") }}

Parameters can be set via YAML (see `config/ml_planner.param.yaml`).

---

## Inputs

| Topic                     | Message Type                                        | Description                                                   |
| ------------------------- | --------------------------------------------------- | ------------------------------------------------------------- |
| `~/input/odometry`        | nav_msgs/msg/Odometry                               | Ego vehicle odometry                                          |
| `~/input/tracked_objects` | autoware_perception_msgs/msg/TrackedObjects         | Detected dynamic objects                                      |
| `~/input/traffic_signals` | autoware_perception_msgs/msg/TrafficLightGroupArray | Traffic light states                                          |
| `~/input/vector_map`      | autoware_map_msgs/msg/LaneletMapBin                 | Lanelet2 map                                                  |
| `~/input/route`           | autoware_planning_msgs/msg/LaneletRoute             | Route information                                             |
| `~/input/turn_indicators` | autoware_vehicle_msgs/msg/TurnIndicatorsReport      | Turn indicator information                                    |
| `~/input/steering_status` | autoware_vehicle_msgs/msg/SteeringReport            | Measured steering angle (used by the trajectory optimization) |

## Services

| Service           | Type                 | Description                                                                                                                                                                                                                                                  |
| ----------------- | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `~/service/start` | std_srvs/srv/SetBool | While enabled (`data: true`), every ego velocity entry of the model input (`ego_agent_past`) is overwritten with 1 m/s, making the model plan as if the vehicle were already moving (useful to trigger a start from standstill). Disable with `data: false`. |

## Outputs

| Topic                           | Message Type                                              | Description                                                |
| ------------------------------- | --------------------------------------------------------- | ---------------------------------------------------------- |
| `~/output/trajectory`           | autoware_planning_msgs/msg/Trajectory                     | Planned trajectory for the ego vehicle                     |
| `~/output/trajectories`         | autoware_internal_planning_msgs/msg/CandidateTrajectories | Multiple candidate trajectories                            |
| `~/output/predicted_objects`    | autoware_perception_msgs/msg/PredictedObjects             | Predicted future states of dynamic objects                 |
| `~/output/turn_indicators`      | autoware_vehicle_msgs/msg/TurnIndicatorsCommand           | Planned turn indicator command                             |
| `~/output/debug/traffic_signal` | autoware_perception_msgs/msg/TrafficLightGroup            | First traffic light on route (ego forward) for RViz/ad_api |
| `~/debug/lane_marker`           | visualization_msgs/msg/MarkerArray                        | Lane debug markers                                         |
| `~/debug/route_marker`          | visualization_msgs/msg/MarkerArray                        | Route debug markers                                        |

---

## Testing

Unit tests are provided and can be run with:

```bash
colcon test --packages-select autoware_ml_planner
colcon test-result --all
```

---

## ONNX Model and Versioning

The ML Planner relies on an ONNX model for inference.

The sampler model consumes the `turn_indicators` history tensor and returns both `trajectory`
and `turn_indicator_logits`. The three logit classes are DISABLE, ENABLE_LEFT, and ENABLE_RIGHT.
To ensure compatibility between models and the ROS 2 node implementation, the model versioning scheme follows **major** and **minor** numbers:
The model version is defined either by the directory name provided to the node or within the `ml_planner.param.json` configuration file.

- **Major version**
  Incremented when there are changes in the model **inputs/outputs or architecture**.

  > :warning: Models with different major versions are **not compatible** with the current ROS node.

- **Minor version**
  Incremented when **only the weight files are updated**.
  As long as the major version matches, the node remains compatible, and the new model can be used directly.

To download the latest model, follow [Download artifacts](https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/README.md#download-artifacts).

### Model Version History

| Version | Release Date | Notes                                                                                                                                                                                                                                                                                                                                                                                                                                    | ROS Node Compatibility |
| ------- | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------- |
| **0.1** | 2025/07/05   | - First public release<br>- Route planning based on TIER IV real data                                                                                                                                                                                                                                                                                                                                                                    | NG                     |
| **1.0** | 2025/09/12   | - Route Termination learning<br>- Output turn-signal (indicator) <br>- Lane type integration in HD map for improved accuracy<br>- Added datasets:<br>&nbsp;&nbsp;- Synthetic Data: **4.0M points**<br>&nbsp;&nbsp;- Real Data: **1.5M points**                                                                                                                                                                                           | NG                     |
| **2.0** | 2025/11/26   | - Increased the number of acceptable lane types ("crosswalk", "pedestrian_lane" and "walkway") for left and right boundaries. <br>- Added `Polygon` and `LineString` as acceptable input types. <br>- Increased the maximum length of each history record to 3 seconds. <br>- Added support for turn_indicator as an input (this is just an interface, not used in v2.0 weights). <br>- Increased `NUM_SEGMENTS_IN_LANE` from 70 to 140. | NG                     |
| **3.0** | 2026/01/09   | - Added `TURN_INDICATOR_OUTPUT_KEEP` to allow the model to focus on the timing of status change. <br>- Conducted Supervised Fine-Tuning (SFT) with carefully filtered data. <br>- Increased the encoder layers from 3 to 6.                                                                                                                                                                                                              | OK                     |
| **3.1** | 2026/03/05   | - ONNX simplified model for faster TRT engine build and reduced GPU memory. <br>- Same weights as v3.0 (no retraining).                                                                                                                                                                                                                                                                                                                  | OK                     |
| **4.0** | 2026/03/23   | - Added `delay` input for Real-Time Chunking (RTC): reuses first N timesteps from the previous prediction for trajectory continuity. <br>- Added one-hot type encoding for polygons (`intersection_area`) and line strings (`stop_line`, `road_border`). <br>- Increased `NUM_LINE_STRINGS` from 10 to 60. <br>- Added line string resampling (`line_string_max_step_m`). <br>- Added debug visualization for line strings.              | OK                     |

---

## Development & Contribution

- Follow the [Autoware coding guidelines](https://autowarefoundation.github.io/autoware-documentation/main/contributing/).
- Contributions, bug reports, and feature requests are welcome via GitHub issues and pull requests.

---

## References

- [ML Planner (original repo)](https://github.com/ZhengYinan-AIR/Diffusion-Planner)
- [Diffusion planner (our fork of the previous repo, used to train the model)](https://github.com/tier4/Diffusion-Planner)
- ["Diffusion-Based Planning for Autonomous Driving with Flexible Guidance"](https://arxiv.org/abs/2309.00615)

---

## License

This package is released under the Apache 2.0 License.
