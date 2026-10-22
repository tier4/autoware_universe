# Time-sequence raw optimizer

Pose-only time-sequence tracking plugin for `autoware_trajectory_optimizer`, ported from X2 `autoware_trajectory_processor` ([tier4/autoware_universe#3359](https://github.com/tier4/autoware_universe/pull/3359), [tier4/autoware_universe#3407](https://github.com/tier4/autoware_universe/pull/3407)).

The plugin is listed first in `plugin_names` and stays **off** until `use_time_sequence_raw_optimizer` is true.

## What it does

- Rewrites the first horizon of a candidate with an independent acados kinematic-bicycle OCP (model `kinematic_bicycle_time_seq`, not Temporal MPT).
- Optional road-border lateral shift before the OCP.
- Stopped-regime steering policy. The OCP still runs; only `front_wheel_angle_rad` is overwritten:
  - **TRACK** — moving: publish the solved trajectory
  - **HOLD** — ego stopped and the reference is shorter than 1.5 m with every speed within 0.15 m/s: latch the measured tire angle on the first hold cycle
  - **ZERO** — within `goal_steer_zero_distance_m` of the route goal, latched once ego has stopped: overwrite the tire angle with 0.002 rad
- A failed acados solve publishes the previous successful map-frame trajectory. The first failure, before any success is stored, leaves the input unchanged.
- Seeds OCP \(x_0\) speed from stamp-aligned ego twist ([#3590](https://github.com/tier4/autoware_universe/pull/3590)). Pose, steering, and acceleration still come from the stamped ego buffer when `use_stamped_ego_state` is true ([#3407](https://github.com/tier4/autoware_universe/pull/3407)).
- Once the predicted endpoint is within `goal.snap_distance_m` of `route.goal_pose`, the terminal reference is the route goal and extra terminal weights stay latched ([#3515](https://github.com/tier4/autoware_universe/pull/3515)). Far-away unlatch drops the latch only. A route-goal position change clears the previous plan ([#3527](https://github.com/tier4/autoware_universe/pull/3527)).
- Temporal consistency resamples the previous plan by timestamp (`k ← (k+1) + age/dt`) and stays on while goal snap is latched. It is skipped on a road-border shift, a stale warm start, or a one-cycle latch mismatch ([#3527](https://github.com/tier4/autoware_universe/pull/3527), [#3590](https://github.com/tier4/autoware_universe/pull/3590)).

## Enabling (companion set)

Partial enable caused stop/engage regressions on X2. Turn these together:

1. `use_time_sequence_raw_optimizer: true`
2. Planner `velocity_smoothing_window: 1` (package default is 8)
3. Velocity optimizer `set_engage_speed: true` and lower `target_pull_out_acc_mps2` (X2 used 0.2)
4. MPC: `keep_steer_control_until_converged: false`, `stop_state_steer_hold_duration: 9999.0`, `enable_confidence_steer_slew_limit: false`

Do **not** set `stop_state_steer_hold_duration` to `0.0`.

Related X2 launch-only follow-ups (pull-out 0.5 m/s / PID clip, accel limit) are **not** part of this plugin port.

## Parameters

Loaded from `config/plugins/trajectory_time_sequence_raw_optimizer.param.yaml` (vehicle overlay in `autoware_launch`).

Debug topics under `~/debug/time_sequence_raw_optimizer/` when `publish_debug_topics` is true.
