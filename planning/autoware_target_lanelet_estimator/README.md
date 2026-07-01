# Target Lanelet Estimator

## Purpose

`Target Lanelet Estimator` estimates which lanelet(s) the ego vehicle intends to drive on.
It takes the planned route, the vector map and the planner trajectory (e.g. from the Diffusion Planner) and assigns an **independent probability to each route lanelet** that expresses how likely the vehicle is going to drive on it.

The estimation follows a recursive Bayesian filter. Because a lane change makes the vehicle straddle two lanelets and the route topic carries no lateral connectivity, the probability is kept per lanelet and the probabilities inside a segment are **not** normalized to sum to one. When the trajectory footprint lies outside every lanelet in the map, the node reports an out-of-lanelet state.

## Inner-workings / Algorithms

The posterior is updated every time a trajectory is received, for the lanelets in the update scope only. The rest of the route keeps its previous probability.

### Update scope

- The segment that contains the ego (the earliest trajectory point on a lanelet) and the next segment.
- The next segment is included only if the trajectory actually reaches it.
- The scope is decided from trajectory **points**, not from the footprint.

### Likelihood

For each trajectory point the vehicle footprint is built and intersected with the lanelet polygon; the likelihood is the maximum overlap-area ratio over all points.

$$L_k(\mathbf{t}_k, l_i) = \max_{\text{points in } \mathbf{t}_k} \frac{\text{intersection area of footprint and lanelet}}{\text{footprint area}}$$

Taking the maximum lets both lanes reach a high likelihood during a lane change.

### Prior

The prior is the previous posterior propagated through a transition model. Within the same segment a small leak probability is used; across segments the transition weight depends on whether the lanelets are connected in the routing graph (`routingRelation`).

### Posterior

$$P_k(l_i \in D \mid \mathbf{t}_k) = \frac{P_k(l_i \in D)\, L_k(\mathbf{t}_k, l_i)}{P_k(l_i \in D)\, L_k(\mathbf{t}_k, l_i) + \left(1 - P_k(l_i \in D)\right)\left(1 - L_k(\mathbf{t}_k, l_i)\right)}$$

## Interfaces

### Parameters

Defined in `config/target_lanelet_estimator.param.yaml`.

| Name                                    | Type   | Default | Description                                                                        |
| --------------------------------------- | ------ | ------- | ---------------------------------------------------------------------------------- |
| `preferred_lanelet_initial_probability` | double | 0.8     | Initial probability of a preferred lanelet                                         |
| `other_lanelet_initial_probability`     | double | 0.2     | Initial probability of a non-preferred lanelet                                     |
| `same_segment_lane_change_probability`  | double | 0.05    | Per-lane leak probability of the transition model within the same segment          |
| `following_transition_weight`           | double | 0.8     | Transition weight to a connected successor lanelet                                 |
| `non_following_transition_weight`       | double | 0.2     | Transition weight to an unconnected lanelet                                        |
| `selection_likelihood_threshold`        | double | 0.001   | A route lanelet is reported as a target when its likelihood exceeds this threshold |
| `out_of_lanelet_search_margin`          | double | 2.0     | [m] Padding of the search box used for the out-of-lanelet check                    |

Vehicle dimensions are loaded from `vehicle_info.param.yaml`.

### Subscriptions

| Name                 | Type                                    | Description            |
| -------------------- | --------------------------------------- | ---------------------- |
| `~/input/vector_map` | autoware_map_msgs/msg/LaneletMapBin     | vector map of Lanelet2 |
| `~/input/route`      | autoware_planning_msgs/msg/LaneletRoute | planned route          |
| `~/input/trajectory` | autoware_planning_msgs/msg/Trajectory   | planner trajectory     |

### Publications

| Name                                    | Type                                                      | Description                                               |
| --------------------------------------- | --------------------------------------------------------- | --------------------------------------------------------- |
| `~/output/target_lanelet_ids`           | autoware_internal_debug_msgs/msg/Int64MultiArrayStamped   | ids of the lanelets overlapped by the trajectory          |
| `~/output/target_lanelet_probabilities` | autoware_internal_debug_msgs/msg/Float64MultiArrayStamped | posterior probability of each target lanelet (same order) |
| `~/output/out_of_lanelet`               | autoware_internal_debug_msgs/msg/BoolStamped              | true when the footprint overlaps no lanelet in the map    |
| `~/debug/route_marker`                  | visualization_msgs/msg/MarkerArray                        | per-lanelet probability heatmap (blue → red)              |

## Visualization

The `~/debug/route_marker` heatmap colors each route lanelet the trajectory has overlapped by its posterior probability: the higher the probability, the redder and more opaque the lanelet. Route lanelets the trajectory has not reached stay transparent.

## Assumptions / Known limits

- The route topic carries no lateral (left/right) connectivity and does not guarantee that primitives in a segment are adjacent, which is why each lanelet is estimated independently and the probabilities within a segment are not normalized to sum to one.
- The probability is updated on every trajectory message.
- This is a temporary package; the deployment target is not yet decided. ROS I/O lives in `node.cpp` and the core logic (`get_target_lanelets`) in `impl.cpp`.
