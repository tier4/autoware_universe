# autoware_virtual_traffic_light_rviz_plugin

## Purpose

An RViz panel that plays the infrastructure side of virtual traffic lights in simulation. It lists
every virtual traffic light that planning sends a command for and publishes an approval and a
finalization state for each of them, so the stop at the stop line and at the end line can be
checked without a real infrastructure or a dedicated node.

## Inputs / Outputs

| Direction | Default topic                                                | Type                                               |
| --------- | ------------------------------------------------------------ | -------------------------------------------------- |
| Input     | `/planning/scenario_planning/status/infrastructure_commands` | `tier4_v2x_msgs/msg/InfrastructureCommandArray`    |
| Output    | `/system/v2x/virtual_traffic_light_states`                   | `tier4_v2x_msgs/msg/VirtualTrafficLightStateArray` |

Both topics can be changed in the panel and are saved in the RViz config.

## Usage

1. In RViz, select `Panels` → `Add New Panel` → `VirtualTrafficLightPanel`.
2. Set a route that passes a lanelet with a virtual traffic light. One row appears per command
   type and id (`ID`, `State`, `Approval`, `Finalized`, `Lane`, `Type`), in the order of the
   received command array (route order for the trajectory modifier); a row disappears when no
   command for it is received for 3 seconds.
3. Click `PUBLISH ANSWERS: OFF` (grey) to switch it to `PUBLISH ANSWERS: ON` (green), then click
   the `Approval` and `Finalized` buttons of a row to switch them between `FALSE` (red) and `TRUE`
   (green). The states are published at 10 Hz while publishing is on. A row is highlighted while
   planning waits for the infrastructure (`REQUESTING`, or `FINALIZING` without finalization).

Publishing is off whenever the panel starts, so opening RViz never approves a virtual
traffic light.

An invalid topic name typed into `Commands` or `States` is rejected: the panel keeps the previous
topic and shows the error in the status line.

## Known limitations

- Rows are keyed by command type and id. When two publishers send a command with the same type and
  id (e.g. the behavior velocity planner and the trajectory modifier both enabled, or one virtual
  traffic light on two route lanelets), the row shows the state and lane of the last command
  received; the published answer is the same for both.
- The states are stamped with the RViz node clock. When planning runs with `use_sim_time:=true`
  and RViz does not, the stamps do not match and planning treats the states as outdated.
