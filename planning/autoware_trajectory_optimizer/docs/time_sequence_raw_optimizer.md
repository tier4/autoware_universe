# Time-sequence raw optimizer

Pose-only time-sequence tracking plugin for `autoware_trajectory_optimizer`, ported from X2 `autoware_trajectory_processor` ([tier4/autoware_universe#3359](https://github.com/tier4/autoware_universe/pull/3359), [tier4/autoware_universe#3407](https://github.com/tier4/autoware_universe/pull/3407)).

The plugin is listed first in `plugin_names` and stays **off** until `use_time_sequence_raw_optimizer` is true.

## What it does

- Rewrites the first horizon of a candidate with an independent acados kinematic-bicycle OCP (model `kinematic_bicycle_time_seq`, not Temporal MPT).
- Optional road-border lateral shift before the OCP.
- Stopped-regime steering policy:
  - **TRACK** — moving: full acados
  - **HOLD** (`solver_status: -1`) — ego stopped + short stopped reference: latch steering, skip acados
  - **ZERO** (`solver_status: -2`) — near route goal: sticky zero steer, skip acados
- Seeds OCP \(x_0\) from localization interpolated at `candidate_header.stamp` when `use_stamped_ego_state` is true ([#3407](https://github.com/tier4/autoware_universe/pull/3407)).

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
