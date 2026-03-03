# Dynamic Obstacle Avoidance Module

## Purpose / Role

This module constrains the drivable area around obstacles near the ego path so that downstream path generation stays inside a safer corridor.

- This module: clips drivable area by adding obstacle polygons.
- Obstacle Avoidance module: generates a path inside the clipped drivable area.

Although the package name contains `dynamic`, the current implementation is intentionally simplified and focused on **stopped or very low-speed objects**.

## Current Scope

### What this module handles

- Objects close to the ego path and enabled by class filter (`car`, `truck`, `bus`, `trailer`, `unknown`, `bicycle`, `motorcycle`, `pedestrian`).
- Objects whose speed norm is below `target_object.stopped_object.max_object_vel`.
- In-lane drivable-area clipping (it does not expand drivable lanes by itself).

### What this module does not handle

- Dynamic-object reasoning based on predicted paths.
- Cut-in / cut-out / crossing classification.
- Time-to-collision based longitudinal extension.
- Lateral feasibility checks based on ego lateral acceleration/jerk.

In other words, moving-object avoidance behavior that relied on predictive motion logic is not part of the current module behavior.

## Algorithm Overview

The processing flow has two stages.

1. Select target objects.
2. Generate obstacle polygons and subtract them from drivable area.

### 1) Target object selection

For each perceived object:

- Check object class against target class flags.
- Check speed threshold with `target_object.stopped_object.max_object_vel`.
- Check lateral proximity to the ego path using `max_obj_lat_offset_to_ego_path`.

Objects that pass these checks are considered avoidance candidates.

### 2) Polygon generation and drivable-area clipping

#### Regulated objects (vehicle-like classes)

- Build a polygon along the ego reference path around the object longitudinal interval.
- Lateral width is determined from object lateral coverage and:
  - `drivable_area_generation.lat_offset_from_obstacle`
  - `drivable_area_generation.max_lat_offset_to_avoid`
  - `drivable_area_generation.lpf_gain_for_lat_avoid_to_offset`
  - `target_object.front_object.max_ego_path_lat_cover_ratio`

The result is a path-aligned clipped region.

#### Unregulated objects (pedestrian-like classes)

- Build polygon from the **current object footprint only** (no predicted-path hull).
- Expand it by `drivable_area_generation.margin_distance_around_pedestrian`.
- Subtract ego reserve polygon to keep minimum ego corridor.

## Relation to Other Avoidance Modules

Compared with [Static Obstacle Avoidance module](https://autowarefoundation.github.io/autoware_universe/main/planning/autoware_behavior_path_static_obstacle_avoidance_module/):

- Static Obstacle Avoidance is responsible for larger path-shift behavior (including outside-lane maneuvering depending on scenario).
- This module is a drivable-area clipping module around nearby low-speed obstacles.

Both may contribute to overall avoidance behavior depending on planner configuration.

## Limitations

- Moving objects above `stopped_object.max_object_vel` are filtered out by design.
- The module keeps existing drivable lanes from upstream and does not perform internal lane expansion.
- `max_lat_offset_to_avoid` limits how much lateral clearance can be enforced in clipping.

## Parameters

All parameters are under `dynamic_avoidance`.

### common

| Name                               | Unit  | Type   | Description                                                     | Default |
| :--------------------------------- | :---- | :----- | :-------------------------------------------------------------- | :------ |
| `common.enable_debug_info`         | `[-]` | `bool` | Enable debug logging for this module                            | `false` |
| `common.use_hatched_road_markings` | `[-]` | `bool` | Enable hatched road marking handling in drivable area utilities | `true`  |

### target_object

| Name                                                                | Unit    | Type     | Description                                                     | Default |
| :------------------------------------------------------------------ | :------ | :------- | :-------------------------------------------------------------- | :------ |
| `target_object.car`                                                 | `[-]`   | `bool`   | Avoid cars                                                      | `true`  |
| `target_object.truck`                                               | `[-]`   | `bool`   | Avoid trucks                                                    | `true`  |
| `target_object.bus`                                                 | `[-]`   | `bool`   | Avoid buses                                                     | `true`  |
| `target_object.trailer`                                             | `[-]`   | `bool`   | Avoid trailers                                                  | `true`  |
| `target_object.unknown`                                             | `[-]`   | `bool`   | Avoid unknown objects                                           | `false` |
| `target_object.bicycle`                                             | `[-]`   | `bool`   | Avoid bicycles                                                  | `true`  |
| `target_object.motorcycle`                                          | `[-]`   | `bool`   | Avoid motorcycles                                               | `true`  |
| `target_object.pedestrian`                                          | `[-]`   | `bool`   | Avoid pedestrians                                               | `true`  |
| `target_object.successive_num_to_entry_dynamic_avoidance_condition` | `[-]`   | `int`    | Consecutive count to enter valid target state                   | `5`     |
| `target_object.successive_num_to_exit_dynamic_avoidance_condition`  | `[-]`   | `int`    | Consecutive count threshold to remove valid target state        | `1`     |
| `target_object.max_obj_lat_offset_to_ego_path`                      | `[m]`   | `double` | Maximum lateral distance from ego path to consider object       | `1.0`   |
| `target_object.front_object.max_ego_path_lat_cover_ratio`           | `[-]`   | `double` | Ignore object if it laterally covers too much of ego path width | `0.3`   |
| `target_object.stopped_object.max_object_vel`                       | `[m/s]` | `double` | Maximum speed treated as stopped/low-speed target               | `0.5`   |

### drivable_area_generation

| Name                                                         | Unit  | Type     | Description                                            | Default |
| :----------------------------------------------------------- | :---- | :------- | :----------------------------------------------------- | :------ |
| `drivable_area_generation.lat_offset_from_obstacle`          | `[m]` | `double` | Base lateral clearance margin from obstacle            | `1.0`   |
| `drivable_area_generation.margin_distance_around_pedestrian` | `[m]` | `double` | Buffer margin for unregulated-object footprint polygon | `2.0`   |
| `drivable_area_generation.max_lat_offset_to_avoid`           | `[m]` | `double` | Maximum lateral offset reserved for clipping policy    | `0.5`   |
| `drivable_area_generation.lpf_gain_for_lat_avoid_to_offset`  | `[-]` | `double` | LPF gain for lateral avoidance boundary stabilization  | `0.9`   |
