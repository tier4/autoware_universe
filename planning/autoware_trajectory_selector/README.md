# Trajectory Selector

## Purpose/Role

`trajectory_selector_node` is the ROS 2 node that orchestrates the full candidate trajectory pipeline. It buffers trajectory candidates from multiple upstream planners via `autoware_trajectory_concatenator`, validates each candidate against the safety and traffic-rule plugins of `autoware_trajectory_validator`, and publishes the surviving trajectories for downstream processing.

## Algorithm Overview

1. Candidate trajectories arrive on two input topics: `~/input/trajectories_generative` and `~/input/trajectories_backup`. The node buffers each message and uses a trigger mode to select the input that starts a concatenation and validation cycle.
2. In generative trigger mode, a generative message starts a cycle immediately. The node buffers backup messages for the next cycle. If no generative message arrives within `fallback_period_ms`, the fallback timer starts a cycle. Each generative message resets this timer.
3. When the fallback timer fires in generative trigger mode, the node changes to backup trigger mode. In backup trigger mode, a backup message starts a cycle immediately and resets the timer. If no backup message arrives within `fallback_period_ms`, the fallback timer starts a cycle.
4. When a generative message arrives in backup trigger mode, the node changes back to generative trigger mode.
5. During each cycle, the concatenator removes entries older than `duration_time`. It then returns the merged trajectory set from all registered generators.
6. If any mandatory sensor input (odometry, predicted objects, acceleration, or HD map) is unavailable, the cycle returns early without publishing.
7. The validator runs all loaded plugins against each trajectory. Trajectories rejected by any enforced plugin (listed in `filter_names`) are removed from the output set.
8. The surviving trajectories are published to `~/output/trajectories`.

## Interface

### Topics

| Direction  | Topic name                                              | Message Type                                                | Description                                                            |
| ---------- | ------------------------------------------------------- | ----------------------------------------------------------- | ---------------------------------------------------------------------- |
| Subscriber | `~/input/trajectories_generative`                       | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories from generative planners                                  |
| Subscriber | `~/input/trajectories_backup`                           | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories from backup planners                                      |
| Subscriber | `~/input/lanelet2_map`                                  | `autoware_map_msgs/msg/LaneletMapBin`                       | HD map (transient local QoS; loaded once at startup)                   |
| Subscriber | `~/input/odometry`                                      | `nav_msgs/msg/Odometry`                                     | Current ego pose and velocity                                          |
| Subscriber | `~/input/objects`                                       | `autoware_perception_msgs/msg/PredictedObjects`             | Surrounding dynamic obstacles                                          |
| Subscriber | `~/input/acceleration`                                  | `geometry_msgs/msg/AccelWithCovarianceStamped`              | Current ego acceleration                                               |
| Subscriber | `~/input/traffic_signals`                               | `autoware_perception_msgs/msg/TrafficLightGroupArray`       | Traffic light states (optional; missing data does not block the timer) |
| Publisher  | `~/output/trajectories`                                 | `autoware_internal_planning_msgs/msg/CandidateTrajectories` | Trajectories that passed all enforced validator plugins                |
| Publisher  | `~/debug/processing_time_detail_ms/trajectory_selector` | `autoware_internal_debug_msgs/msg/ProcessingTimeTree`       | Per-function processing time breakdown                                 |

### Parameters

{{ json_to_markdown("planning/autoware_trajectory_selector/schema/trajectory_selector.schema.json") }}

Other configuration is provided through the sub-package parameter files loaded at launch:

- Concatenation parameters: see [`autoware_trajectory_concatenator`](../autoware_trajectory_concatenator/README.md#parameters)
- Validation parameters: see [`autoware_trajectory_validator`](../autoware_trajectory_validator/README.md#parameters)
