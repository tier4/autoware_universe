# Testing

## Overview

`test/closed_loop/` is a lightweight closed-loop simulation that calls `SafetyPlanner::plan` periodically without launching a ROS node.
Each cycle (0.1 s) the ego is advanced along the output trajectory (perfect tracking; no controller or vehicle model),
and the loop runs until one of: goal reached (within 1.0 m of the goal and |v| < 0.1 m/s) / stalled / plan failure /
invalid trajectory / step limit.
Every output trajectory is validated (number of points, non-finite values, monotonic `time_from_start`, point interval,
yaw jump, negative velocity, max steer angle, distance between the first point and ego).

Scenarios live in `test_data/<name>.yaml`, one scenario per file. Besides the map / vehicle / route, a scenario may carry
`predicted_objects` (a `PredictedObjects` message in the `topic_snapshot_saver` yaml format, held fixed over the run) and
`expectation` (`goal_reached`, the default, or `stop`: the ego must stall with |v| < 0.1 m/s before the goal, e.g. behind an obstacle). The list of scenarios to run is
`test_data/scenarios.yaml`; it is read when the test executable starts, so adding or disabling a case
needs no C++ change (only a `colcon build` to re-install `test_data/`, unless `--symlink-install` is used).
The planner parameters come from the production config: `config/safety_planner.param.yaml` plus the per-plugin files under `config/constraint_generator/` and `config/trajectory_planner/`.

## Build

```bash
# without plots (default)
colcon build --packages-select autoware_safety_planner

# with plots
colcon build --packages-select autoware_safety_planner --cmake-args -DEXPORT_TEST_PLOT_FIGURE=ON
```

`EXPORT_TEST_PLOT_FIGURE` is stored in the CMake cache, so pass `-DEXPORT_TEST_PLOT_FIGURE=OFF` explicitly to turn it back off.
Plotting uses `autoware_pyplot` (pybind11 + matplotlib). The test pins the backend to `Agg`
(with Qt5Agg the process segfaults at exit).

## Run

```bash
# via colcon
colcon test --packages-select autoware_safety_planner --event-handlers console_direct+
colcon test-result --verbose

# run the executable directly (handy for gtest filters and repeated runs)
source install/setup.bash
./build/autoware_safety_planner/test_autoware_safety_planner
./build/autoware_safety_planner/test_autoware_safety_planner --gtest_filter='*behavior_normal_route*'
./build/autoware_safety_planner/test_autoware_safety_planner --gtest_list_tests
```

Plugins are resolved by pluginlib from `plugins.xml` under `install/`, so `source install/setup.bash` is required even when
running the executable directly.

Test names have the form `Scenarios/ClosedLoopTest.ReachesGoalWithValidTrajectories/<yaml basename>`.
On failure the gtest message contains the termination reason (e.g. `stalled at step N (goal distance = X m)`) and the list of
violations (`step N: <reason>`).

## Outputs

All artifacts go to `build/autoware_safety_planner/test_results/closed_loop/` (the build tree; nothing is written into the
source tree). They are written regardless of pass/fail. `<name>` below is the snake_case test name, e.g.
`reaches_goal_with_valid_trajectories_behavior_normal_route`.

| File                      | Content                                                                                                                                                                                                                                                |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `<name>_ego.csv`          | one row per cycle: `step, t, x, y, z, yaw, v, a, steer, num_trajectory_points` (ego state at the start of the cycle). The last row is the ego state at termination, i.e. the state the goal check was evaluated on; it has `num_trajectory_points = 0` |
| `<name>_trajectories.csv` | one row per trajectory point of every cycle: `step, t, idx, time_from_start, x, y, z, yaw, v, a, steer` (raw output of `SafetyPlanner::plan`, normal side)                                                                                             |
| `<name>.png`              | only with `EXPORT_TEST_PLOT_FIGURE=ON`: route lanelets, reference path (green) and output trajectory (blue) of every cycle, ego trace, vehicle footprint every 1 s, ego velocity over time                                                             |

Units are SI (m, s, m/s, m/s^2, rad). `t` is the simulation time of the cycle (`step * 0.1`), and `time_from_start` is relative
to that cycle. A typical analysis is `pandas.read_csv(...)` and grouping `_trajectories.csv` by `step`.

## Writing test cases

A scenario file only needs the map, the vehicle info and the route:

```yaml
format_version: 1
map_path_uri: package://autoware_test_utils/test_map/lanelet2_map.osm
vehicle_info_param_uri: package://autoware_test_utils/config/test_vehicle_info.param.yaml
route:
  start_pose: { position: ..., orientation: ... }
  goal_pose: { position: ..., orientation: ... }
  segments: # one entry per lanelet, in order
    - preferred_primitive: { id: 9102, primitive_type: "" }
      primitives:
        - { id: 9102, primitive_type: lane }
```

The `route` block is the same format as the `LaneletRoute` output of
`autoware_test_utils::topic_snapshot_saver`, so a snapshot taken from a bag or from planning simulation
can be dropped in as is. It is parsed by `autoware::test_utils::parse<LaneletRoute>`, which reads only
`start_pose`, `goal_pose` and `segments`.

### Generating a scenario with the GUI

`scripts/generate_test_data.py` draws the lanelet2 map with matplotlib and writes a scenario file, so
neither RViz nor a planning simulator launch is needed.

```bash
source install/setup.bash
ros2 run autoware_safety_planner generate_test_data.py

# another map / vehicle / output directory
ros2 run autoware_safety_planner generate_test_data.py \
  --map-uri package://autoware_test_utils/test_map/2km_test.osm \
  --vehicle-info-uri package://autoware_test_utils/config/test_vehicle_info.param.yaml \
  --out-dir /tmp
```

`--out-dir` defaults to `test_data/` in the source tree (the script is symlink-installed, so `__file__`
points at the source); with a copying install, pass `--out-dir` explicitly.

1. Hover over a lanelet to see its id. Lanelets are colored by subtype and the arrow on the centerline
   shows the driving direction.
2. Press `Set start`, then drag on the map: the press point becomes the position and the drag direction
   becomes the yaw. The button disarms itself after one drag.
3. Do the same with `Set goal`.
4. Type the scenario name in the text box (default `new_route`) and press `Generate`. The file is written
   to `test_data/<name>.yaml` (or `--out-dir`) and the resulting lanelet ids are shown next to the buttons.

The route is the shortest path of `lanelet2::routing::RoutingGraph` between the lanelet under the start
pose and the one under the goal pose, taking only lanelets whose direction is within 45 deg of the given
yaw. If no such lanelet exists, or the two are not connected, the reason is shown in the panel and no
file is written. Each segment gets a single primitive; neighboring lanes are not added because the
closed-loop test does not evaluate lane changes.

Map drawing and the lanelet id tooltip are reused from `autoware_lanelet2_utils`
(`scripts/test_case_generator.py`), which is loaded from its install directory, so that package has to be
built. That script can also be used on its own to just look at a map:

```bash
ros2 run autoware_lanelet2_utils test_case_generator.py --view-map <path to lanelet2_map.osm>
```

### Registering the scenario

Add the file name to `test_data/scenarios.yaml`; the test executable reads the list at startup, so no C++
change is needed:

```yaml
scenarios:
  - behavior_normal_route.yaml
  - <name>.yaml
```

Comment a line out to skip a case without deleting its file. A scenario file that is not listed is never
run, and a name listed here without a matching file makes the test fail while loading the scenario.

`test_data` is installed to share, so run `colcon build --packages-select autoware_safety_planner` after
adding a scenario. With `--symlink-install` each installed file is a symlink to the source, so editing
`scenarios.yaml` or a scenario itself takes effect without a rebuild, but a newly added file still needs
one to get its symlink.

The gtest case name is the file name without its extension, so `<name>.yaml` runs as
`Scenarios/ClosedLoopTest.ReachesGoalWithValidTrajectories/<name>`.
