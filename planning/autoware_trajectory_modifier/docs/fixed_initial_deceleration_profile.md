# Fixed initial deceleration profiles for velocity limit plugins

## Status and scope

Design and implementation plan for an opt-in behavior in `ExternalVelocityLimit` and
`MapVelocityLimits`. The implementation is present in the package; build and test validation are
pending.

Add a shared boolean ROS parameter, `fix_initial_deceleration_profile`, with default `false`, and
shared reuse tolerances under `fixed_velocity_limit_profile`.
When false, both plugins continue to call `detail::apply_velocity_limits()` on every candidate in
every callback. When true, each plugin computes a braking profile once for a given candidate and
set of effective limits, then advances that same profile as time passes. It computes a new profile
when the effective limits change or the observed ego state departs from the state predicted by the
profile. If the inputs needed to establish continuity are unavailable, it uses the existing
per-callback calculation for that callback and does not reuse an unverified profile.

“Fixed” refers to the **longitudinal speed and acceleration schedule**, not to copying an old
trajectory message. Every callback still maps that schedule onto the current input trajectory,
preserving its point count, timestamps, first pose, and current path geometry.

## Existing behavior and constraints

- Both plugins call `detail::apply_velocity_limits()` with the latest odometry speed and measured
  acceleration. Its forward/backward/forward passes therefore start from a different ego state on
  each callback, which can move the braking onset and reshape the deceleration curve.
- `ExternalVelocityLimit` receives a retained latest `VelocityLimit` message. Its effective inputs
  are `max_velocity`, the selected absolute deceleration, and the selected absolute jerk. A new
  message with the same effective values must not reset a fixed profile.
- `MapVelocityLimits` queries the route handler at trajectory poses. Map limits are spatial; the
  same speed zone moves to different point indices as the ego advances. Point-index comparison
  would incorrectly treat normal motion as a limit change.
- The host reuses a plugin instance for every candidate. `TrajectoryModifierData` carries a
  candidate index, but `CandidateTrajectory` also has a `generator_id`. Diffusion and ML planner
  batches assign the **same** generator UUID to several candidates, so UUID alone cannot identify
  a profile. State must not leak from one candidate into another.
- `apply_velocity_limits()` may return `Unchanged`, may return a direct cap via its 15% deadband,
  or may build a retimed braking profile. Only the last case creates a fixed profile. The current
  `VelocityLimitResult` does not distinguish those outcomes, so it must be extended.
- The helper currently owns both profile calculation and spatial interpolation. Reuse requires
  separating those operations. Its comment that output never exceeds incoming velocity should be
  checked during this refactor: the final forward pass can set a point from ego reachability even
  after the input point was capped.

## Behavioral contract

1. A profile is anchored at the odometry measurement timestamp and the ego's measured speed,
   acceleration, and map-frame pose at first computation. Cache a piecewise velocity/acceleration
   function `v(t), a(t)` and integrated progress `s(t)` for elapsed time `t >= 0`. Keep the initial
   reference path for state and map-limit comparison. A profile belongs to exactly one plugin
   instance and candidate identity.
2. On callback `k`, let `T_k` be the current odometry timestamp and `T_0` the anchor timestamp.
   At trajectory point time `tau_i = time_from_start[i]`, use the original profile at
   `t_i = (T_k - T_0) + tau_i`. Do **not** re-anchor the cached profile to the latest ego speed or
   acceleration. This is the key property that prevents the onset from moving on every iteration.
   Represent each segment with bounded constant jerk `j`: over segment-local time `u`, evaluate
   `a(u) = a0 + j*u`, `v(u) = v0 + a0*u + j*u²/2`, and
   `s(u) = s0 + v0*u + a0*u²/2 + j*u³/6`. This avoids an acceleration jump solely because a
   callback samples between old trajectory points. Carry enough state to continue braking to the
   applicable terminal limit, ramp acceleration back to zero, then hold the settled speed. New
   map-limit transitions detected beyond the original horizon trigger a new profile.
3. Before reuse, compare the actual ego speed, acceleration, projected longitudinal progress,
   lateral displacement, and heading against `v(t)`, `a(t)`, and the pose at `s(t)`. Use a symmetric
   deviation test so both faster and slower tracking errors cause a replan. Suggested initial
   internal thresholds are 0.3 m/s, 0.3 m/s², 0.5 m longitudinally, 0.5 m laterally, and 0.17 rad
   in heading. These are configurable under `fixed_velocity_limit_profile`, retain the original
   values as defaults, and are provisional calibration values; replay tests should establish
   final values. Updating a tolerance clears cached profiles.
4. Compare **effective constraints**, not message stamps or trajectory indices. For the external
   plugin, compare finite non-negative `max_velocity`, the selected deceleration, and selected
   jerk. For the map plugin, compare route UUID, map identity/content, overrides, deceleration,
   jerk, and the spatial sequence of applicable speed limits along the overlapping path and new
   lookahead. Match limits by map-frame path position or route station; a speed-zone boundary
   shifting to a new point index is not a change. A newly visible restriction, removed
   restriction, changed value, or path switch is a change. Normalize optional/invalid map limits
   using the same rules as `apply_velocity_limits()` before comparing. Do not use an unquantified
   hash of floating-point positions as the only check.
5. The current input trajectory is an upper speed envelope wherever that is physically
   feasible. Apply the cached schedule only where it limits that input, and retime the **current**
   polyline using the resulting velocities. If an upstream speed reduction intersects the cached
   braking segment or produces an acceleration/jerk-incompatible join, treat the incoming
   envelope as an effective constraint change and recompute. In particular, the map plugin must
   account for changes made by the preceding external plugin. Reuse must never itself raise an
   incoming point's velocity. The fresh calculation may still exceed an incoming speed or a new
   cap where the measured ego state and jerk/deceleration bounds make that target physically
   unreachable; this is the existing helper's feasible transition policy.
6. A direct cap with no generated braking schedule is applied normally on every callback. An
   `Unchanged` result creates no cache. A cached profile can be released once the ego has reached
   its settled limit and no braking segment remains in the lookahead; future callbacks still
   check current limits and start a new profile when needed.

## Cache ownership and invalidation

Use a bounded cache per plugin instance. Carry `CandidateTrajectory::generator_id` into
`TrajectoryModifierData`, along with the existing candidate index and count. Treat the UUID as a
generator group, not as a unique candidate key. Within a group, match each new candidate to at
most one prior cache entry by path overlap, starting pose/heading, and expected ego progress; use
the prior within-group ordinal only as a tie-breaker. Reject ambiguous matches and compute a new
profile for those candidates. Track entries already matched in the current batch and prune entries
not seen in it. This supports multiple candidates from one generator without sharing a profile or
assuming that ranking preserves their order. If the UUID is absent, permit reuse only for a
single-candidate input; otherwise calculate normally. Verify frame and path continuity even after
matching, since a generator can switch routes or produce a new branch.

| Event                                                                                                                      | Action                                                                        |
| -------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| Relevant limit value, route, map, override, or upstream effective envelope changes                                         | Discard this candidate's profile and compute from current ego state.          |
| Ego deviation exceeds any threshold                                                                                        | Discard and compute from current ego state.                                   |
| Plugin disabled, `fix_initial_deceleration_profile` switched, or relevant tuning parameter changed                         | Clear affected cache entries. Unrelated parameter updates do not clear them.  |
| Empty trajectory, missing required input, invalid/non-monotone point times, nonfinite state, or unusable timestamps/frame  | Do not reuse; follow the existing valid-input behavior and clear stale cache. |
| Odometry timestamp does not advance or jumps backward, acceleration stamp is incompatible, or path projection is ambiguous | Recalculate without caching until continuity is demonstrable.                 |
| Same limits but a normal shift of the trajectory window along the same path                                                | Reuse and sample the original schedule at the new elapsed time.               |

For map changes, also rebuild `ExtendedRouteHandler` when the map message changes, even if the
route UUID stays the same; the current code rebuilds it only on route UUID change. Cache entries
must be invalidated when this happens. Map constraints should be evaluated on both the incoming
and retimed path, since retiming can move points across a speed-zone boundary. A path branch or
large geometric departure is treated as a changed spatial constraint context.

For a stable map and route, retain the initial reference polyline and its terminal effective
limit. Project each current input and retimed point to a unique station on that polyline. On the
overlapping part, compare the current map lookup with a lookup at the reference pose at the same
station. Beyond the original path horizon, compare new lookups with the cached terminal limit;
the first new transition changes the effective constraint set and starts a fresh profile. Reject
projection if two path segments are equally plausible or if lateral/heading separation exceeds
the continuity thresholds. This spatial comparison permits a zone boundary to cross trajectory
point indices without triggering a rebuild, while detecting a lane change into another zone.

Use odometry timestamps in the same ROS clock domain for elapsed time. Check candidate header
and acceleration stamp freshness against odometry before caching; a zero, stale, or inconsistent
stamp must not cause extrapolation of the old schedule. The default freshness window of 0.2 s is
configurable as `fixed_velocity_limit_profile.stamp_difference_threshold` and should be calibrated
with the ego-deviation thresholds. Keep the legacy
calculation available as the fallback for inputs that cannot establish a reliable elapsed time.
Log or count cache misses by reason so replay can distinguish tracking errors from map changes
and missing timestamps.

## Shared implementation shape

Refactor `velocity_limits.hpp/.cpp` around three explicit operations:

1. **Resolve constraints:** evaluate the external or map limit callback on the current path and
   record the effective limit field and first active violation. Return whether the outcome is
   unchanged, a direct cap, or a generated braking profile.
2. **Build profile:** use the existing feasibility/smoothing passes as target knots, then fit a
   continuous, bounded-jerk schedule from the measured ego state. Check speed, acceleration, and
   jerk bounds throughout each segment, not only at knots; adjust infeasible target knots under
   the same feasible-transition policy. Store the segment coefficients and a terminal extension
   that reaches a settled legal speed without an artificial jump at the original horizon. Keep
   the existing `apply_velocity_limits()` entry point as the default-mode wrapper to preserve
   current callers and tests. The fixed mode may differ slightly between knots because it needs
   a continuous-time profile.
3. **Validate and apply profile:** verify identity, clocks, ego tracking, path continuity,
   spatial constraints, and input-envelope compatibility. Sample `v(t)` at the current
   timestamps, combine it with the input envelope, update acceleration consistently, and retime
   the current polyline. Re-evaluate map constraints on the result. If any check fails, build a
   new profile from the current state and replace the candidate's cache.

Treat nonfinite values and malformed timestamps as explicit errors instead of allowing them into
a profile. Keep tolerance for comparing floating-point limits small (for example, 1e-3 m/s for
speed) and separate from the much larger ego-tracking thresholds. Cache writes occur only after
a successful profile build and validation. The host currently processes candidates serially;
keep cache mutation within that callback and do not share a profile between plugin instances.

## Implementation plan

1. Add `fix_initial_deceleration_profile: false` and the shared
   `fixed_velocity_limit_profile.*` tolerances to the generated parameter schema and package
   default YAML, and document the behavior and fallback in the package README. Read them in both
   plugins' `update_params()`; clear cached profiles when the flag or a reuse tolerance changes.
   Verify that generated `TrajectoryModifierParams` exposes the new fields.
2. Add candidate generator ID to `TrajectoryModifierData` and populate it in the host loop. Add
   bounded per-candidate storage and one-to-one matching within each generator group, using the
   identity and continuity rules above. Keep separate caches in `ExternalVelocityLimit` and
   `MapVelocityLimits`.
3. Split the shared limiter into constraint resolution, profile generation, profile sampling,
   and current-polyline retiming. Preserve the default-mode outputs. Make direct-cap versus
   generated-profile explicit and add the terminal continuation needed for long-lived reuse.
4. Implement the common ego-state/time/path checks and per-candidate reuse state machine. The
   state transitions are `NoProfile -> FixedProfile -> Reuse`, or `FixedProfile -> Recompute`
   when a stated invalidation condition occurs; `Unchanged` and direct-cap callbacks do not
   establish `FixedProfile`.
5. Integrate external effective-limit comparison and map spatial-limit comparison. Fix the map
   handler's map-replacement detection. Ensure the map plugin handles upstream envelope changes
   after the external plugin has run.
6. Add focused tests, then run package build and test gates. Use replay with repeated callbacks
   and changing vehicle states to tune the provisional thresholds before enabling the flag in a
   deployment configuration. The shipped default remains `false`.

## Verification and acceptance criteria

- With the flag false, existing external/map unit and integration tests retain their expected
  trajectories. Parameter updates and a pipeline reload do not crash.
- With the flag true and constant limits, a perfectly tracking ego sees the **same anchored
  braking curve shifted by elapsed time** over at least three callbacks, including nonuniform
  point timestamps and a receding trajectory window. Count profile builds to prove no rebuild
  occurred. Poses are resampled on the current path, not copied from the first callback.
- Changing external cap, deceleration, or jerk rebuilds exactly the affected candidate's
  profile. A republished message with equal effective values does not. Clearing/invalidating
  the external limit follows the existing `Unchanged` behavior and removes stale cache.
- A map speed-zone boundary moving between point indices while the ego progresses does not
  rebuild. Route/map/override changes, a newly visible stricter or looser zone, and a path branch
  do rebuild. A retimed point entering a lower zone cannot silently retain an invalid profile.
- Speed, acceleration, progress, lateral, and heading deviations each trigger a rebuild beyond
  threshold and allow reuse within threshold. Zero/backward/stale timestamps and malformed
  trajectories fall back without applying stale samples.
- Two candidates with the same or different generator IDs keep independent profiles. Reordering
  preserves reuse only when matching is unique; removal and missing or duplicate IDs cannot
  transfer a profile to a different path. Toggling the parameter and changing relevant tuning
  values clear the appropriate caches.
- For every valid output, point count, timestamps, and first pose match the input; the retimed
  path stays on the current input polyline. Reuse does not raise incoming velocity. Check
  acceleration/deceleration and jerk bounds throughout cached segments, at arbitrary cache
  sampling times, and at joins to an upstream speed envelope. Test the existing physically
  unreachable transition policy separately for fresh calculations.

## Design risks to resolve during implementation

The existing limiter's three kinematic passes and spatial retiming are coupled, and its final
forward pass can overwrite a capped point. The refactor must characterize this behavior with a
regression test before promising strict input-envelope preservation. In addition, a future map
restriction outside the previous horizon cannot be anticipated from an old profile; detecting a
newly visible transition is part of limit-change detection. Both issues are acceptance gates for
the opt-in mode, rather than reasons to silently reuse an unsafe schedule.
