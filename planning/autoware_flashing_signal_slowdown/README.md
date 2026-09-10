# autoware_flashing_signal_slowdown

Detects a flashing amber traffic light in the traffic signal recognition output and requests a
velocity limit while it flashes. The previous limit is restored once it stops, so the vehicle
simply accelerates back to normal.

The target lights flash only while a facility-side sensor detects a person nearby, and are
completely dark otherwise, so the existing traffic light module does not react to them.

## Interface

| Direction | Name                                    | Type                                                            |
| --------- | --------------------------------------- | --------------------------------------------------------------- |
| subscribe | `~/input/traffic_signals`               | `autoware_perception_msgs/msg/TrafficLightGroupArray`           |
| publish   | `~/output/max_velocity`                 | `autoware_internal_planning_msgs/msg/VelocityLimit`             |
| publish   | `~/output/velocity_limit_clear_command` | `autoware_internal_planning_msgs/msg/VelocityLimitClearCommand` |

## Algorithm

A dark lamp and the dark half of a flashing cycle are identical in a single sample, so the
detector never tries to tell them apart. Each frame is reduced to `On` (amber and `SOLID_ON`),
`Off` (`SOLID_OFF`) or `Unknown`, and flashing is declared only once `detect_min_transitions`
transitions appear within `history_window_sec`, spread over at least `detect_min_span_sec`.

| Situation              | Transitions in the window | Result       |
| ---------------------- | ------------------------- | ------------ |
| Dark, the normal state | 0                         | not flashing |
| Steadily lit           | 0                         | not flashing |
| Flashing at 1Hz        | about 6 in a 3s window    | flashing     |

`Unknown` frames are dropped rather than stored, so a missing frame cannot change the state, and
the release depends only on the time since the last transition. Detection therefore takes about
1.3s while the release deliberately takes about 3.1s.

State is kept per traffic light group id and the slowdown holds while any monitored light is
still flashing.

## Velocity limit

The limit goes to `autoware_external_velocity_limit_selector` on the internal path, which keeps
one entry per sender and publishes the lowest of them. A release only has to clear this node's
own entry: the selector falls back to its `max_vel` parameter by itself, so no normal velocity is
duplicated here, and a limit set from the API is left untouched.

Both topics are latched (`transient_local`), so the selector picks up the current state even if
it starts later. The entry survives this node dying while a light flashes, as it does for every
other sender on that path.

## Parameters

See `config/flashing_signal_slowdown.param.yaml`. `target_traffic_light_group_ids` is site
specific and is set on the launcher side, in
`autoware_launch/config/planning/flashing_signal_slowdown/`. The node monitors nothing and logs
the count as zero when it is empty.

## Testing

`FlashingDetector` takes time as a plain `double`, so the state machine is covered by
`test/test_flashing_detector.cpp` without starting a node.

Behaviour on a running system is judged from `/planning/scenario_planning/max_velocity`, whose
`sender` field reads `flashing_signal_slowdown` while this node holds the limit, rather than from
the vehicle speed: the traffic light module inserts stop points without going through the
velocity limit selector, so a speed change alone does not identify its cause. Feed the input from
the RViz `TrafficLightPublishPanel` with the status set to `FLASHING`, and always check that a
steady lamp does **not** trigger a slowdown.
