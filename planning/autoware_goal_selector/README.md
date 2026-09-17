# Goal Selector

## Purpose

`Goal Selector` sets the next goal once the ego arrives at a predefined trigger position.

Each trigger has a list of goal candidates in priority order. The node picks the first candidate
that is not overlapped by any perceived object and publishes it as the next goal.

## Behavior

1. The mission planner reports `ARRIVED` on the route state when the ego has stopped at the route
   goal.
2. If the route goal lies inside a trigger area, the goal candidates of that trigger are evaluated
   in priority order and the first vacant one is taken.
   If every candidate is blocked, the evaluation is repeated on every objects update while the
   route state stays `ARRIVED`.
3. The selected goal is published once per arrival.

The published goal reaches the mission planner through `autoware_routing_adaptor`, which clears the
current route and sets the new one. Since the ego is stopped, the route can be cleared safely.

## Interfaces

### Subscriptions

| Name                  | Type                                            | Description       |
| --------------------- | ----------------------------------------------- | ----------------- |
| `~/input/route_state` | `autoware_planning_msgs/msg/RouteState`         | route state       |
| `~/input/route`       | `autoware_planning_msgs/msg/LaneletRoute`       | route             |
| `~/input/objects`     | `autoware_perception_msgs/msg/PredictedObjects` | perceived objects |

### Publications

| Name             | Type                                 | Description   |
| ---------------- | ------------------------------------ | ------------- |
| `~/output/goal`  | `geometry_msgs/msg/PoseStamped`      | the next goal |
| `~/debug/marker` | `visualization_msgs/msg/MarkerArray` | debug marker  |

### Parameters

{{ json_to_markdown("planning/autoware_goal_selector/schema/goal_selector.schema.json") }}

The triggers and the goal candidates are declared by name, so they are not covered by the schema
above.

| Name                             | Type         | Description                                 |
| -------------------------------- | ------------ | ------------------------------------------- |
| `goals`                          | string array | Names of the goal candidates defined below  |
| `<goal name>.x/y/z/yaw`          | double       | Pose of the goal candidate in the map frame |
| `triggers`                       | string array | Names of the triggers defined below         |
| `<trigger name>.x/y`             | double       | Position of the trigger in the map frame    |
| `<trigger name>.radius`          | double       | Radius of the trigger area [m]              |
| `<trigger name>.goal_candidates` | string array | Goal candidate names in priority order      |

## Debug marker

`~/debug/marker` is published only while it has subscribers. It shows every trigger area as a
yellow circle and every goal candidate as the vehicle footprint used by the vacancy check, green
while free and red while blocked. The candidate labels are placed ahead of each footprint and
staggered, so that candidates standing side by side stay readable.

The trigger has no elevation in the parameter file, so its circle is drawn at the elevation of its
first goal candidate.

## Assumptions / Known limits

- A trigger fires only when the ego arrives at a route goal inside its area. Stopping there for any
  other reason, such as in the middle of a route or in manual driving, does not fire it.
- Every perceived object blocks a goal. No filtering by class, velocity or existence probability is applied.
- The node does not engage the vehicle. The operator has to engage it after the new route is set,
  because arriving at a goal switches the operation mode to STOP.
- The node does not know whether the mission planner accepted the goal. Every goal candidate has to be
  a pose the mission planner can plan a route to, otherwise the goal is silently rejected.
