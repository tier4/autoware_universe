# Freespace Area Module

## Overview

This behavior_path_planner scene module plans through lanelet2 freespace `Area` primitives with a
freespace planner (Hybrid A\* or RRT\*) so that the vehicle can drive Lane → Area → Lane without
stopping, or stop at a goal pose located inside an Area.

The module relies on the route_handler Area API (`getRouteAreas()`, `getNextAreaTransit()`,
`getRouteAreaAtPose()`, `isGoalInRouteArea()`), which is only populated when the
behavior_path_planner node parameter `allow_area_route` is `true` (it calls
`RouteHandler::setAllowArea(true)`).

## Activation

The module requests execution when all of the following hold:

- The route contains at least one Area (`getRouteAreas()` non-empty; zero overhead otherwise).
- Either
  - the next `AreaTransit` from ego's closest route lanelet exists and the arc-length distance
    from ego to the end of the entry lanelets is within `activation.lookahead_distance`, or
  - ego is already inside a route Area (mid-area operation / recovery).

## Modes

- **Transit mode** (`exit_lanelets` non-empty): the A\* goal is the first centerline pose of the
  exit lane. After the area segment, the exit-lane centerline is appended so the vehicle continues
  lane driving without stopping. The module transits to SUCCESS once ego is on an exit lanelet and
  has passed the area-to-lane junction.
- **Terminal mode** (`exit_lanelets` empty, goal inside the Area): the A\* goal is the route goal
  pose and the composed path ends with zero velocity. SUCCESS requires position/yaw tolerance and
  standstill.

## Planning

Planning runs asynchronously in a dedicated timer (own callback group) at `planner.update_rate`,
mirroring the goal_planner freespace-parking worker: a mutex-guarded request/response pair and an
atomic running flag. `plan()` never blocks on A\*. Requests are rejected while the costmap is
missing or older than 1 s.

The A\* waypoints are converted with `utils::convertWayPointsToPathWithLaneId` (lane ids of the
entry/exit lanelets are stamped onto area points) and then re-encoded so that reverse segments get
a 180-degree yaw flip and positive velocities. This makes cusps detectable by the downstream
`direction_change` module (`detectCuspPoints()` compares consecutive yaws), which handles the
stop-and-reverse gear switching. The path is intentionally NOT divided at reversing indices.

## Latch & replan

Once a plan succeeds it is latched and returned on every cycle. The latch is invalidated (and a
replan requested) when:

- lateral deviation from the latched path exceeds `latch.replan_lateral_deviation`,
- the worker reports an obstacle on the latched trajectory
  (`latch.replan_when_obstacle_found`, via the algorithm's `hasObstacleOnTrajectory`),
- ego is stopped away from the goal longer than `latch.stuck_time_threshold`,
- the route UUID changes.

## Drivable area

The module sets `DrivableAreaInfo::drivable_margin` (half vehicle width plus
`planner.vehicle_shape_margin`), which makes the planner manager generate a corridor around the
composed path (the same mechanism used by freespace pull-over). The
`enable_expanding_freespace_areas` path was evaluated and rejected: `getBoundWithFreeSpaceAreas`
only searches parking-lot polygons nearest to ego and stitches them into existing lane bounds, so
it does not fit arbitrary route Areas. A polygon-splitting helper
(`freespace_area_utils::generateBoundsFromAreaPolygon`) is implemented and unit-tested for future
use, but not wired into the output yet.

### Limitations

- The corridor drivable area does not expose the full Area polygon; downstream drivable-area
  checks see only a tube around the path.
- On replan while moving, the new start pose is ego's current pose rather than a point ahead on
  the latched path.
- Obstacle checks are performed against the whole latched path, not only the remaining segment.

## Parameters

See `config/freespace_area.param.yaml`. Namespaces:

| Namespace                                   | Content                                                                                                   |
| ------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `freespace_area.activation`                 | `lookahead_distance`                                                                                      |
| `freespace_area.planner`                    | `algorithm` (astar/rrtstar), `velocity`, `vehicle_shape_margin`, `update_rate`                            |
| `freespace_area.planner.common`             | freespace planner common search/costmap configs                                                           |
| `freespace_area.planner.astar` / `.rrtstar` | algorithm-specific configs                                                                                |
| `freespace_area.latch`                      | `replan_lateral_deviation`, `replan_when_obstacle_found`, `obstacle_check_margin`, `stuck_time_threshold` |
| `freespace_area.path`                       | `junction_blend_distance`, `goal_position_tolerance`, `goal_yaw_tolerance_deg`                            |
