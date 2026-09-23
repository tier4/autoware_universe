# Deployment identity, continuity and latency

ResWorld launch now requires `deployment_manifest.json` beside the planner ONNX.
Generate it with OnePlanner's `projects/resworld/scripts/make_ml_package_param.py`.
Older packages must be regenerated; do not disable the check to use a mixed package.
The manifest hashes the planner, extractor and their contract/provenance JSON files,
and lists effective ROS parameters for comparison before either engine is created.
This checks accidental package/config mismatch; it is not a signed authenticity mechanism.
Training normalization is embedded in the planner; `args_path` must remain empty.

Both TensorRT caches are tied to graph bytes, engine bytes and build precision by
an `.identity` sidecar. Missing or mismatched identities cause a rebuild. An old
installation will therefore rebuild its engines once on upgrade, even if their
mtimes appear current. Rebuild failures prevent startup. Keep engine directories
writable. TensorRT remains responsible for version/device compatibility checks.

## Localization continuity

A time reversal or an implausible inter-pose displacement/yaw change clears ego
history and advances a generation counter. The BEV provider consumes the generation
and clears all cached maps, including its assembled history pointer. BEV poses also
have an independent continuity check. This handles forward-time localization jumps,
not only bag loops. Detection bounds are slack plus maximum rate times elapsed time:

- `localization_reset.translation_slack_m`: 2.0
- `localization_reset.max_speed_mps`: 60.0
- `localization_reset.yaw_slack_rad`: 0.35
- `localization_reset.max_yaw_rate_rps`: 2.0

These are conservative configurable bounds, not a localization-quality estimator.
Tune them against the vehicle/localizer replay. Smaller jumps below the bounds are
not detected. Cache warmup policy applies after reset.

## Freshness and timing

The inspected upstream cuda_blackboard subscriber has `KeepLast(1)`. No second queue
or planner timer was added. Verify this property in the version installed on target.
Duplicate cloud stamps are rejected before expensive feature extraction; old clouds
continue to be rejected by `bev_feature.max_delay_ms`. Clouds more than `bev_feature.max_future_skew_ms` (default 20 ms) ahead
of the node clock are rejected too. Backward timestamps still reach the cache reset
path. Missing history after overload waits for real features; it is not fabricated.

Debug topics report:

- `debug/input_age_ms`: LiDAR age at the start of the inference tick.
- `debug/pipeline_latency_ms`: LiDAR capture to completion of publication calls
  (previously it incorrectly used tick start). This does not measure DDS delivery
  or controller consumption.
- `debug/pipeline_latency_p95_ms`, `debug/pipeline_latency_p99_ms`: last 512 successful
  outputs, nearest-rank quantiles; reset on localization discontinuity.
- `debug/processing_time/p95_ms`, `p99_ms`: same window for tick execution time.
- Existing collect/inference/postprocess timing topics remain available.

Diagnostics also report stale/duplicate cloud counts, BEV cache device bytes and
`bev_feature.extraction_gpu_ms`. CUDA events surround feature extraction on its
existing stream; results are queried after inference's existing completion wait.
They include queued GPU work between the events, not upstream blackboard wait time.
No new stream/device synchronization is introduced. Host inference timing can include
waiting for earlier extraction on the shared stream; do not sum it with GPU extraction
as if they were disjoint stages.

Expired BEV buffers are now recycled before acquiring the next map. A host regression
shows only three source-buffer allocations for a three-frame exact-stride history,
instead of one unnecessary extra buffer. Real cache memory also depends on sensor
rate: intermediate source timestamps remain retained to serve the target history.

## Verification and profiling limits

Host tests cover artifact tampering, graph/engine/precision identity, path rejection,
pose discontinuities/yaw wrap, percentile calculations, and timestamp interpolation.
OnePlanner's `check_autoware_cache.py` checks the actual native cache using CPU memory
shims, including reset isolation and allocation counts. These do not execute CUDA.

On the target, record the above debug topics and diagnostics during the same bag,
then compare input age, p95/p99, dropped-cloud counts and cached bytes. Profile the
actual component-container process with Nsight Systems (CUDA and OS runtime tracing),
not merely the launch client. Check allocation calls in steady state, memcpy direction
and size, and the reasons for stream waits. No FP16 conversion, CUDA Graph capture or
additional zero-copy change is claimed here: they require target measurements and
numerical parity. Full ROS/CUDA build and replay were explicitly deferred by the user.
