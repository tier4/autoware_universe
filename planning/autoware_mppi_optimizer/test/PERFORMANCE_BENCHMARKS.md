# Standalone MPPI performance benchmarks

This GTest executable measures the CPU and GPU work of `FirstOrderDubinsMppiInterface` without a
ROS graph, simulator, rosbag, map server, or display. It uses the real optimizer library and still
requires its normal runtime libraries and a compatible CUDA GPU. It does not provide a CPU MPPI
backend or measure plugin activation, route queries, publication, or whole-system scheduling.

## Explicit build and execution

`MPPI_BUILD_PERFORMANCE_BENCHMARKS` defaults to `OFF`. Even with the option enabled, the executable
is **not registered with CTest**, so ordinary `colcon test` and `ctest` do not execute it. There is
no benchmark target in the default build. The opt-in target also requires `BUILD_TESTING=ON`.

In an already configured Autoware workspace, rebuild both the modified vendor headers and optimizer.
These commands are instructions for the developer; they were not run during implementation:

```bash
colcon build --packages-select MPPI-Generic autoware_mppi_optimizer \
  --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DMPPI_BUILD_PERFORMANCE_BENCHMARKS=ON
source install/setup.bash

build/autoware_mppi_optimizer/test_performance_benchmarks \
  --gtest_filter=MppiPerformanceBenchmark.FreeSpaceCached \
  --gtest_output=xml:mppi_benchmark.xml
```

For the prepared path-projection change, run the ordinary geometry and cost parity tests first,
then compare complete-call latency with separate cost-kernel phase timing. Use the same build type,
profiles, seed, warmup, samples, GPU conditions, and instrumentation mode as the saved baseline;
the projection delta arrays add runtime-upload bytes even on cycles with unchanged geometry.
The initial runs have completed; their results and comparison limits are recorded in
[GPU_PERFORMANCE_REVIEW.md](../GPU_PERFORMANCE_REVIEW.md#validation-and-timing-follow-up-2026-11-04).
The subsequent review added direct original/prepared device comparisons with exact numerical
equality for segment selection, raw projection parameter, and lateral distance, plus production
safety-status checks. These additions and the corrected representable near-tie inputs passed in
the user's 2026-11-05 pruning-validation run. The translated-vertex cost test does not exercise the interface's frame lifecycle;
the interface test below crosses the actual recentering threshold.

```bash
build/autoware_mppi_optimizer/test_path_tracking_geometry
build/autoware_mppi_optimizer/test_distance_map \
  --gtest_filter='DistanceMapGpuTest.AdversarialProjectionMatchesHostWithAndWithoutPersistentHints:DistanceMapGpuTest.ReferenceOnlyProjectionUsesTheSameGlobalSearch:DistanceMapGpuTest.PreparedProjectionMatchesOriginalDeviceArithmeticAndSafety:DistanceMapGpuTest.PreparedProjectionTracksReferenceCorridorAndTranslatedVertices'
build/autoware_mppi_optimizer/test_costs \
  --gtest_filter='GpuCostEvaluation.AdversarialProjectionCostsAndTerminalMatchHost:GpuCostEvaluation.CombinedAndSplitRolloutsUseGlobalProjectionAndSafetyFlags:GpuCostEvaluation.ZeroSoftWeightsStillReportExactSafetyViolations:GpuCostEvaluation.PreparedProjectionPreservesLateralSafetyBoundary'
build/autoware_mppi_optimizer/test_first_order_dubins_mppi_interface \
  --gtest_filter='FirstOrderDubinsMppiInterfaceGpuTest.PublishesPreciseMapPositionsAcrossOriginChange'

MPPI_BENCHMARK_WARMUP=10 MPPI_BENCHMARK_SAMPLES=100 MPPI_BENCHMARK_PHASE_TIMING=0 \
  MPPI_BENCHMARK_OUTPUT_DIR=mppi_benchmark_results/projection_after_latency \
  build/autoware_mppi_optimizer/test_performance_benchmarks \
  --gtest_filter='MppiPerformanceBenchmark.FreeSpaceCached:MppiPerformanceBenchmark.Hairpin256Cached:MppiPerformanceBenchmark.StaticObstacles64Cached:MppiPerformanceBenchmark.StaticObstacleGeometryUpdates:MppiPerformanceBenchmark.GridBoundaryCrossings'

MPPI_BENCHMARK_WARMUP=10 MPPI_BENCHMARK_SAMPLES=100 MPPI_BENCHMARK_PHASE_TIMING=1 \
  MPPI_BENCHMARK_OUTPUT_DIR=mppi_benchmark_results/projection_after_phases \
  build/autoware_mppi_optimizer/test_performance_benchmarks \
  --gtest_filter='MppiPerformanceBenchmark.FreeSpaceCached:MppiPerformanceBenchmark.Hairpin256Cached:MppiPerformanceBenchmark.StaticObstacles64Cached:MppiPerformanceBenchmark.StaticObstacleGeometryUpdates:MppiPerformanceBenchmark.GridBoundaryCrossings'
```

Use your workspace's existing build wrapper/environment if required. CUDA architecture flags must
support the target GPU. The optional benchmark additionally uses `yaml-cpp` and `nlohmann_json`,
declared as test dependencies. Reconfigure after changing revisions so the recorded revision and
compiler metadata describe the current build. Revision strings include Git's tracked-dirty marker;
they are recorded at configure time, not obtained from the running shared library. A missing Git
checkout or vendor checkout outside the usual workspace layout is reported as `unavailable`.

Run all scenarios by omitting the filter. GTest filtering, repetition, and XML output work normally.
Each scenario/repetition writes a separate JSON file, including raw measurements. Run one benchmark
process at a time on an otherwise idle GPU. The benchmark neither changes GPU clocks/power settings
nor excludes other processes; record those conditions with `MPPI_BENCHMARK_ENVIRONMENT`.

For GPU phase timing and cache assertions, explicitly enable instrumentation:

```bash
MPPI_BENCHMARK_PHASE_TIMING=1 MPPI_BENCHMARK_SAMPLES=200 \
  MPPI_BENCHMARK_OUTPUT_DIR=results/mppi_instrumented \
  build/autoware_mppi_optimizer/test_performance_benchmarks \
  --gtest_filter='MppiPerformanceBenchmark.*Updates:MppiPerformanceBenchmark.*Crossings'
```

A missing CUDA device skips the case and produces no performance report. CUDA initialization or
execution errors fail the case. No universal latency threshold is asserted. Inspect the GTest result
and JSON `measurements_valid` before using measurements as a baseline.

## Configuration

| Environment variable                | Default                                            | Meaning                                                                                   |
| ----------------------------------- | -------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| `MPPI_BENCHMARK_WARMUP`             | `10`                                               | Unmeasured stabilization calls, including the separately recorded first call; minimum 1   |
| `MPPI_BENCHMARK_SAMPLES`            | `100`                                              | Warm measured calls; minimum 1                                                            |
| `MPPI_BENCHMARK_PHASE_TIMING`       | `0`                                                | `1` enables optional host ranges, CUDA events, and rebuild assertions                     |
| `MPPI_BENCHMARK_PROJECTION_PRUNING` | `1`                                                | `0` selects the prepared exhaustive GPU projector; `1` enables conservative group pruning |
| `MPPI_BENCHMARK_ITERATIONS`         | `0`                                                | `0` uses the YAML value; otherwise 1–100 optimizer iterations                             |
| `MPPI_BENCHMARK_DEVICE`             | `0`                                                | Device index after `CUDA_VISIBLE_DEVICES` remapping                                       |
| `MPPI_BENCHMARK_COST_PROFILE`       | Build copy of `config/mppi_optimizer.param.yaml`   | Cost/runtime YAML with `/**.ros__parameters.mppi_optimizer` structure                     |
| `MPPI_BENCHMARK_VEHICLE_PROFILE`    | Build copy of `test/data/performance_vehicle.yaml` | Explicit vehicle parameters under a `vehicle` map                                         |
| `MPPI_BENCHMARK_OUTPUT_DIR`         | `mppi_benchmark_results`                           | JSON output directory                                                                     |
| `MPPI_BENCHMARK_ENVIRONMENT`        | Empty                                              | Free-text run conditions: clocks, power mode, competing workloads, run label, etc.        |

The default **optimizer cost profile** is the shipped YAML, currently 10 iterations. The default
**vehicle profile** is an explicitly versioned synthetic small-vehicle fixture, not a claim about a
deployed product. Replace its values with the target vehicle's interface units to compare that
configuration. Supply every vehicle field and the cost fields shown in the shipped profile; internal
derived barrier-weight inputs may be omitted. Runtime fields absent from a custom profile use the
interface defaults and are recorded. Malformed, missing required, or nonfinite parameters fail.

The benchmark deliberately disables debug logging, visualization, iteration capture, forced cold
starts, and previous-control nominal reuse. Output validation/rejection stays enabled. It calls
`optimizeTrajectory(..., defer_commit=true)` and never commits the candidate. Every cycle advances
timestamps by 0.1 s but replays a prepared snapshot rather than advancing a simulated vehicle with
the proposed command. This keeps delay/history assumptions consistent and avoids contaminating
subsequent samples with unapplied commands. The RNG sequence and ESS temperature adaptation persist
across warmup and measurement. Each scenario starts with a fresh interface and fixed seed 1.

All resolved parameter inputs, overrides, input profile text, and selected rollout kernel appear in
the report. Internal barrier weights are derived by the optimizer; `resolved_input_profile` records
the supplied inputs rather than replacing that derivation. Use the same profiles, warmup/sample
counts, scene, and instrumentation mode for comparisons.
The optional iteration override can measure 20 iterations, but it does not reproduce the old harness's
different default cost parameters and state lifecycle. Fixed rollout/horizon dimensions remain
16,384 × 80, with a 0.1 s model timestep; changing those dimensions is outside this harness.

## Exact spatial pruning validation

The user ran the selected validation and first off/on benchmark comparison on 2026-11-05: all
18 selected tests/diagnostics and all 32 benchmark runs passed. Results and limits are recorded in
the [review](../GPU_PERFORMANCE_REVIEW.md#grouped-pruning-validation-and-timing-results-2026-11-05).
The assistant did not run builds, tests, benchmarks, or profiling. For another environment, reconfigure
and rebuild the optimizer with the opt-in benchmark enabled using the build command above; this
change does not require a new vendor modification. Run these checks before collecting timings:

```bash
build/autoware_mppi_optimizer/test_path_tracking_geometry
build/autoware_mppi_optimizer/test_distance_map \
  --gtest_filter='DistanceMapGpuTest.*Projection*'
build/autoware_mppi_optimizer/test_costs \
  --gtest_filter='GpuCostEvaluation.*Projection*:GpuCostEvaluation.ZeroSoftWeightsStillReportExactSafetyViolations'
build/autoware_mppi_optimizer/test_first_order_dubins_mppi_interface \
  --gtest_filter='FirstOrderDubinsMppiInterfaceGpuTest.PublishesPreciseMapPositionsAcrossOriginChange'
```

The projection tests compare original/prepared/pruned device arithmetic with exact numerical
equality, verify conservative bounds against independent segment distances, and cover geometry
lifecycle and safety decisions. The disabled work-count diagnostic is a separate, explicit run:

```bash
build/autoware_mppi_optimizer/test_distance_map --gtest_also_run_disabled_tests \
  --gtest_filter='DistanceMapGpuTest.DISABLED_ProjectionPruningWorkCounts'
```

It prints groups examined/pruned, segment evaluations, and exhaustive fallbacks for group sizes
4, 8, and 16 on synthetic separated/overlapping paths, across all hints. It also checks parity and
bounds. It does not measure optimizer latency or record the actual optimizer query distribution.
The production group size is eight, and production instantiations contain no counter updates.

Use separate processes to compare exhaustive (`0`) and grouped (`1`) searches in the same binary:

```bash
mppi_filter='MppiPerformanceBenchmark.FreeSpaceCached:MppiPerformanceBenchmark.Hairpin256Cached:MppiPerformanceBenchmark.StaticObstacles64Cached:MppiPerformanceBenchmark.StaticObstacleGeometryUpdates:MppiPerformanceBenchmark.GridBoundaryCrossings:MppiPerformanceBenchmark.ReferenceGeometryUpdates:MppiPerformanceBenchmark.ShortProjectionPath:MppiPerformanceBenchmark.OverlappingProjectionGroups'
for mppi_phase in 0 1; do
  for mppi_pruning in 0 1; do
    MPPI_BENCHMARK_PHASE_TIMING="$mppi_phase" \
      MPPI_BENCHMARK_PROJECTION_PRUNING="$mppi_pruning" \
      MPPI_BENCHMARK_OUTPUT_DIR="mppi_benchmark_results/pruning_${mppi_pruning}_phase_${mppi_phase}" \
      build/autoware_mppi_optimizer/test_performance_benchmarks \
      --gtest_filter="$mppi_filter"
  done
done
```

Repeat with the pruning order reversed, under the same GPU conditions. Require valid measurements,
zero unexpected outcomes, matching workload/profile/kernel, and `projection_pruning_math_supported`
and `projection_bounds_valid` both true before attributing a result to pruning. `projection_points`
and `projection_group_size` identify the active path/cutoff; the short scene intentionally remains
exhaustive in both modes. The bounds are cached in both modes and included in both runtime uploads,
so this switch isolates search savings. Compare matching original scenes against revision
`afab67af33` as well to measure net preprocessing/upload overhead and compiler effects.

Compare complete-call median/p95 and process CPU time in latency mode, then cost GPU time and
`MPPI/projection_reference_bounds`, `MPPI/projection_corridor_bounds`, and
`MPPI/scene_runtime_upload` in phase mode. The two bounds phases are host-only and nested inside
data preparation; do not add them to inclusive parent timings. `scene_runtime_bytes` records the
uploaded snapshot size. CMake disables pruning when configured math flags violate its precision
contract; reconfigure after changing flags, including NVCC environment flags. The
[review](../GPU_PERFORMANCE_REVIEW.md#floating-point-bound-contract) documents that contract and
the conservative rounding argument. The first supplied comparison shows substantial gains on the
hairpin and reference-update scenes; reverse-order repetitions and a comparison against the earlier
revision are still needed to quantify variability and net preparation/upload overhead.

## Scenarios and workload checks

| Scenario                                                             | Workload                                                                                     |
| -------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| `FreeSpaceCached`                                                    | Straight reference, no obstacles or borders                                                  |
| `StaticObstacles16Cached`, `StaticObstacles64Cached`                 | Parked objects outside the clear reference corridor                                          |
| `StaticObstacleReferenceCollision`                                   | Shallow hard-margin overlap with one stationary object, leaving room for avoidance           |
| `RoadBorderReferenceNearCollision`                                   | Reference within the soft border margin but outside the hard margin                          |
| `TightCorridor256Cached`                                             | Maximum-capacity road-border geometry                                                        |
| `StaticObstacleGeometryUpdates`                                      | Alternate one stationary object's reported pose; rebuild obstacle maps                       |
| `GridBoundaryCrossings`                                              | Translate reference, ego, objects, and borders together by alternating 0/0.6 m offsets       |
| `Hairpin256Cached`                                                   | 256-point corridor with a nearby returning branch; 80 optimized steps                        |
| `ReferenceGeometryUpdates`                                           | Alternate a 0/0.03125 m lateral shift of the hairpin; rebuild bounds and nearest-segment map |
| `ShortProjectionPath`                                                | First nine straight samples; padded 80-step reference, eight-segment projection cutoff       |
| `OverlappingProjectionGroups`                                        | Straight initial horizon with repeatedly overlapping tail segments; weak pruning             |
| `NoEligibleRollouts`                                                 | An obstacle encloses the initial state; require explicit no-eligible-rollout rejection       |
| `PreferredCenterlineTextureCached`, `PreferredCenterlineExactCached` | One centerline segment, texture/exact comparison                                             |
| `PreferredCenterline256GridCrossings`                                | 256 centerline segments and alternating grid positions                                       |

The bounded grid offsets cross snapped grid boundaries without triggering local-frame recentering.
They preserve relative scene geometry. Object-update cases change geometry rather than merely
timestamps; all objects remain stationary within each prediction horizon. One tracked object
produces one hypothesis here. Tied predicted paths, moving objects, vegetation, and production
closed-loop history are not covered by this initial matrix.

Before timing, the reference-collision cases check their stated hard/soft input condition. Every
optimization must process the full horizon and requested iteration count. Successful cases must
return a validated, unrejected candidate; the infeasible case must report the expected rejection.
The short-path scene publishes nine optimized points while retaining 80 rollout steps; the
interface pads its time-aligned reference with the last sample. Other scenes publish 80 points.
Unexpected results remain in the JSON and fail GTest, marking the report invalid for comparison.
Custom profiles may legitimately fail these quality gates; such a failure is not a timing result.

With phase timing enabled, sampling/rollout/weight/reduction range and CUDA event-pair counts must
equal the requested iterations. When `rollout_kernel` is `split`, the dynamics and cost subphases
must each have the same counts, including on the first optimizer call. Startup kernel-selection
trials are excluded from these subphase counts. The combined kernel must have neither subphase.
After initialization, cached scenes must have no map or corridor-bound rebuilds. Obstacle updates must
rebuild only the obstacle map; reference updates rebuild corridor bounds and the nearest-segment map.
Grid crossings must rebuild corridor bounds, static and nearest-segment maps, plus
nonempty obstacle/active centerline maps. Counters are the observed instrumented rebuild-range
invocations, not inferred from timestamps. Instrumentation-off runs omit these counters/checks.

## Measurement boundaries and report interpretation

`wall_ms` uses `std::chrono::steady_clock` around `optimizeTrajectory` and a final
`cudaDeviceSynchronize`. Prior GPU work is drained before timing. `process_cpu_ms` uses Linux
`CLOCK_PROCESS_CPUTIME_ID` over the same region, including all process threads and driver CPU work.
It is not elapsed waiting time and may exceed wall time with multiple active threads. Input creation,
assertions, event collection, serialization, and result destruction are outside the timed region.

The first optimizer call is recorded separately and excluded from warm statistics. It includes
optimizer setup, allocations, kernel selection, and initial map builds. Device discovery, interface
construction, parameter setting, and optional event-pool allocation happen earlier. Consequently,
this is **not a process/CUDA cold-start measurement**. Use a fresh process and external startup timing
if startup latency is the target.

Optional instrumentation reuses preallocated CUDA event pairs and range storage. Events are recorded
on the stream executing each phase; no per-phase synchronization is introduced. Results are queried
after the benchmark's final completion wait. Errors, incomplete events, and recorder overflow fail
instead of silently returning zero. An inactive recorder performs no clock reads or CUDA event work.

Named ranges cover reference preparation, scene staging/upload, static/obstacle/nearest/centerline
map generation, sampling, rollout, weighting, reduction, final transfers and waits, host state replay,
validation, and debug/result preparation. `host_ms` is inclusive wall duration, not active CPU time.
`gpu_ms` is elapsed stream time and can contain scheduling/launch gaps. Host-only ranges have
`gpu_ms: null`. Nested ranges are not additive, and these ranges do not account for every operation.
GPU elapsed time is not SM utilization, occupancy, memory bandwidth, or energy consumption.

For the split rollout path, two nested NVTX/event ranges identify its individual kernel launches:

| JSON phase name         | Timed work                                                              |
| ----------------------- | ----------------------------------------------------------------------- |
| `MPPI/rollout`          | Inclusive rollout total, including both subphases and work between them |
| `MPPI/rollout_dynamics` | Dynamics launch preparation and trajectory propagation kernel           |
| `MPPI/rollout_cost`     | Cost launch preparation and trajectory cost/safety evaluation kernel    |

The subphases use the same CUDA stream as their parent and introduce no synchronization. CUDA
events are recorded only with phase timing enabled; the NVTX ranges are also available to Nsight
Systems with phase timing off. Subphase ranges are enabled only for optimizer iterations, so the
startup kernel-selection trials retain their original launch path. The combined kernel continues
to report only `MPPI/rollout` because dynamics and cost execute inside a single kernel.

Each sample's `phases` and the aggregate `phase_summary` include these names. Times are totals
across all iterations in one optimizer call; for example, at 10 iterations the dynamics phase
contains 10 launches, not the timing of one launch. Instrumented console output also shows the
rollout/dynamics/cost GPU means per call. Do not add the parent time to its subphase times. The two
subphases need not sum exactly to the parent because the parent also spans the gap between ranges
and their event overhead. Rebuild both the vendor headers and optimizer to obtain the new ranges;
older reports do not contain the subphases.

The JSON includes first-call data, raw warm samples, per-phase totals per cycle, and mean/median/p95/
minimum/maximum/population-standard-deviation summaries. p95 uses nearest rank. Missing phase
invocations contribute zero to the per-cycle summary. There is no p99 estimate from these small
default sample sets. Use enough samples/repeated processes to assess variation before setting a
hardware-specific regression threshold. CPU/GPU identity, CUDA/driver versions, compiler flags,
configured revisions, profiles, and user-supplied environment notes accompany every report.

Retain a latency-only baseline because event instrumentation changes execution cost. For a deeper
timeline investigation, profile the same executable using its NVTX ranges, for example:

```bash
MPPI_BENCHMARK_PHASE_TIMING=0 MPPI_BENCHMARK_SAMPLES=20 \
  nsys profile --trace=cuda,nvtx --output=mppi_profile \
  build/autoware_mppi_optimizer/test_performance_benchmarks \
  --gtest_filter=MppiPerformanceBenchmark.GridBoundaryCrossings
```

Profiler runs are separate from baseline measurements. Reported milliseconds cannot establish a
deployment deadline or GPU utilization reduction on their own.
