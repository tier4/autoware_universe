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

| Environment variable                       | Default                                            | Meaning                                                                                                                 |
| ------------------------------------------ | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| `MPPI_BENCHMARK_WARMUP`                    | `10`                                               | Unmeasured stabilization calls, including the separately recorded first call; minimum 1                                 |
| `MPPI_BENCHMARK_SAMPLES`                   | `100`                                              | Warm measured calls; minimum 1                                                                                          |
| `MPPI_BENCHMARK_PHASE_TIMING`              | `0`                                                | `1` enables optional host ranges, CUDA events, and rebuild assertions                                                   |
| `MPPI_BENCHMARK_PROJECTION_PRUNING`        | `1`                                                | `0` selects the prepared exhaustive GPU projector; `1` enables conservative group pruning                               |
| `MPPI_BENCHMARK_COOPERATIVE_OUTPUT_STORES` | `1`                                                | `0` selects legacy dynamics output stores; `1` selects cooperative stores; both benchmark variants force split rollouts |
| `MPPI_BENCHMARK_ITERATIONS`                | `0`                                                | `0` uses the YAML value; otherwise 1–100 optimizer iterations                                                           |
| `MPPI_BENCHMARK_DEVICE`                    | `0`                                                | Device index after `CUDA_VISIBLE_DEVICES` remapping                                                                     |
| `MPPI_BENCHMARK_COST_PROFILE`              | Build copy of `config/mppi_optimizer.param.yaml`   | Cost/runtime YAML with `/**.ros__parameters.mppi_optimizer` structure                                                   |
| `MPPI_BENCHMARK_VEHICLE_PROFILE`           | Build copy of `test/data/performance_vehicle.yaml` | Explicit vehicle parameters under a `vehicle` map                                                                       |
| `MPPI_BENCHMARK_OUTPUT_DIR`                | `mppi_benchmark_results`                           | JSON output directory                                                                                                   |
| `MPPI_BENCHMARK_ENVIRONMENT`               | Empty                                              | Free-text run conditions: clocks, power mode, competing workloads, run label, etc.                                      |

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

All resolved parameter inputs, overrides, input profile text, selected rollout kernel and dynamics output-store mode appear in
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
hairpin and reference-update scenes. The subsequent balanced repetitions and fresh profiles are
recorded in the [2026-11-06 review](../GPU_PERFORMANCE_REVIEW.md#balanced-repetitions-and-pruned-kernel-profiles-2026-11-06):
the main gains persist, the short-path result is near-neutral, and numerical reproducibility remains
open. A comparison against the earlier revision is still needed to quantify net preparation/upload
overhead.

### Repeated comparison and fresh profiles

The manual [follow-up runner](run_projection_followup.sh) collects the next set of evidence from
an existing build. It is not registered with CTest and requires an explicit mode. Run it from the
workspace root in your sourced Autoware environment, with other GPU workloads idle. For example,
in zsh:

```bash
source install/setup.zsh
mppi_followup=src/autoware/universe/planning/autoware_mppi_optimizer/test/run_projection_followup.sh
export MPPI_BENCHMARK_ENVIRONMENT='idle GPU; normal power mode; clocks not locked'

bash "$mppi_followup" repeat
```

Replace the environment note with the actual conditions. The runner fixes 10 warmup calls,
100 measured calls, and 10 optimizer iterations to match the supplied comparison. It runs four
independent pruning off/on pairs in each timing mode across all eight scenes: **128 JSON reports**.
Pair 1 starts with pruning on, reversing the original comparison; subsequent pairs alternate
the order, so each setting runs first twice. Each setting/timing combination starts a new process;
each scenario constructs a fresh optimizer. Cost/vehicle profile and device overrides remain
available through the existing environment variables.

Each invocation creates a unique directory under `mppi_benchmark_results/projection_<mode>_*`.
It saves console output, JSON reports, the executable and resolved optimizer-library hashes, linked library paths, current repository
revisions/status, and `nvidia-smi -q` snapshots before/after each benchmark process. Snapshots record
conditions at those boundaries; they do not measure clock/temperature variation during a run. The
manifest's checkout state supplements the JSON's configure-time revisions and cannot prove that
all loaded libraries were rebuilt from that state. Reconfigure/rebuild if the binary is stale.
The runner stops on execution errors, missing reports (including CUDA skips), invalid measurements,
unexpected outcomes, incompatible projection math/bounds, or a kernel other than `split`.

Compare paired latency-only medians/p95 and CPU time across processes, keeping phase-timing runs
separate. For `ShortProjectionPath`, check whether the earlier 1% slowdown persists and whether
it follows pruning order. For `GridBoundaryCrossings`, compare baseline cost, eligible rollouts,
and both temperature fields at matching cycle indices, including repeated runs with the same
pruning setting. Timing variation and numerical variation need separate conclusions. The runner
validates report eligibility; matching profiles/build metadata and numerical comparisons still
require analysis. This remains a same-binary comparison, so it does not quantify net overhead
against the earlier revision.

Then collect the pruned timeline and kernel profiles in separate invocations:

```bash
bash "$mppi_followup" nsys
bash "$mppi_followup" ncu
```

`nsys` profiles free space, the 256-point hairpin, and 64 static obstacles with CUDA/NVTX tracing
and CPU sampling/context-switch tracing disabled. Each process uses 10 warmups and 20 measured
calls. Keep the `.nsys-rep` timeline and `timeline.txt` summaries: the summaries include startup
and warmup work, so inspect the steady part of the timeline when assessing launch gaps, sampling,
dynamics, cost, and synchronization. These options follow the
[Nsight Systems CLI documentation](https://docs.nvidia.com/nsight-systems/UserGuide/).

`ncu` collects **six reports**: dynamics and cost separately for each of those three scenes.
It uses the full metric set, kernel replay, cache flushing, source import, and no clock control,
matching the earlier cost profile's collection policy. The escaped slash in each NVTX range name
and trailing slash select the push/pop range; kernel-name filtering and `--launch-skip 100`
skip its 10 warmup calls × 10 iterations. Startup kernel-selection trials are outside these ranges.
`--launch-count 3` collects the first three iterations of the first measured call; these are
three kernel launches, not three complete optimizer calls. The process completes three measured
calls and saves its benchmark JSON. See the
[Nsight Compute CLI documentation](https://docs.nvidia.com/nsight-compute/NsightComputeCli/).
Inspect each `.txt` for three matching kernel entries and any collection warnings before using it;
retain the `.ncu-rep` for CUDA/SASS source correlation and metrics beyond the text summary.

The runner finds tools on `PATH`, with an additional lookup under NVIDIA's installation directories
for `ncu`. Use `MPPI_NCU=/absolute/path/to/ncu` or `MPPI_NSYS=/absolute/path/to/nsys` if needed.
Only Nsight Compute uses `sudo` for GPU performance-counter access; `LD_LIBRARY_PATH`, GPU selection,
and benchmark settings are explicitly passed after `sudo` so ROS libraries remain discoverable.
Do not launch the entire runner with `sudo`. Profiler reports/JSON written by the elevated process
may be root-owned.

Profiler JSON is labeled in `environment_note` and stored separately. Even though phase events
are disabled, its wall/CPU times are perturbed by tracing or replay and are **not latency baselines**.
Use the fresh profile to choose between dynamics, sampling, remaining projection work, and other
cost terms. No new optimization should be justified solely by the old exhaustive-projection profile.

### Dynamics-to-cost cache handoff

Use the explicit `handoff` mode to measure cost with and without the cache state created by its
preceding dynamics kernel:

```bash
source install/setup.zsh
mppi_followup=src/autoware/universe/planning/autoware_mppi_optimizer/test/run_projection_followup.sh
export MPPI_BENCHMARK_ENVIRONMENT='scalar derivatives; describe actual clock policy and background load'
bash "$mppi_followup" handoff
```

First rebuild the intended implementation yourself, and retain the matching source during capture.
At the initial 2026-11-06 investigation, the installed library still contained the reverted array
code despite a restored scalar checkout; the subsequent `projection_handoff_20261106_060707_WcpvOW`
collection uses the rebuilt scalar library. The environment note labels your intended build; the executable
and resolved optimizer-library hashes identify what actually ran. For an array/scalar comparison,
collect this mode once from each verified build under similar conditions, changing the note.

This mode uses **application replay for both cache policies**, with strict kernel matching,
no profiler clock control, and nine explicit hardware metrics instead of the full metric set.
For each of free space, hairpin and static-64, it runs two pairs in order `all, none` then
`none, all`: **12 `.ncu-rep` files** under `projection_handoff_*`, with raw-metric `.txt` exports.
Each report profiles just one cost launch: `--launch-skip 100 --launch-count 1`, the first
iteration after 10 warmup calls. The producer is not profiled. Each application execution completes
three measured calls, with phase events disabled; its latency figures are profiler-perturbed.

Application replay re-executes preceding GPU work for each pass. The `all` policy flushes caches
before the selected kernel; `none` preserves the inherited state subject to profiler/background
interference. Kernel replay with `none` is unsuitable for this comparison because successive
passes can reuse cache contents from the selected kernel itself. See NVIDIA's
[application replay](https://docs.nvidia.com/nsight-compute/ProfilingGuide/index.html#application-replay)
and [cache control](https://docs.nvidia.com/nsight-compute/ProfilingGuide/index.html#cache-control)
documentation.

Keep **every** companion JSON: there can be several per capture, one for each application run.
The runner validates all reports and checks identical workload metadata and numerical diagnostics
for the first optimizer call and the first measured call, both within each capture and across its
scene's cache policies. It stops on discrepancies. These checks and strict kernel matching do not
prove identical input bytes. If they fail, share the retained results before interpreting merged
counters; a fixed-input fixture may be needed to separate cache effects from optimizer variability.
Later-cycle diagnostic differences are outside the selected launch and are not a matching gate.

Inspect each raw export for exactly one cost kernel and valid duration, SM frequency, L1/L2 hit
rates, global-load sectors, L2 read sectors/misses, and DRAM read/write bytes. Check the console for
replay/collection warnings. Compare `none` versus `all` within each build before comparing those
effects across builds; the old kernel-replay/full-set captures are not the control for this mode.
Aggregate hit rates do not isolate the output allocation. Retain unprofiled and Systems results
for latency conclusions. Findings and the proposed producer-only store experiment are recorded in
the [performance review](../GPU_PERFORMANCE_REVIEW.md#dynamics-output-handoff-and-cost-cache-investigation-2026-11-06).

The first [cache-policy results](../GPU_PERFORMANCE_REVIEW.md#first-cache-policy-results-2026-11-06-0607-utc)
used one hardware pass per capture, so **one companion JSON per capture is correct**. Disabling
cache flushing did not improve cost time or materially reduce L2 read misses in that scalar-build
collection. This does not establish identical cache behavior in unprofiled execution.

### Cooperative dynamics output-store validation

The split dynamics kernel now distributes each shared output tile across X/Y copy workers for
odd output dimensions, including the bicycle's 13 components. The global layout stays
`[distribution][rollout][time][component]`, and Z distributions remain independent. The barriers
before the copy and before shared storage reuse are unchanged. Even-sized outputs retain the
previous vectorized path; no additional buffer, kernel launch or synchronization is introduced.
The documented correctness checks and the first performance collection of the constraint-fixed
build have completed. The subsequent same-build comparison support expands the tests to both
production store specializations and requires a fresh validation after rebuilding.

The initial November 6 user-run validation exposed 12 racecheck errors and 18 warnings in the
bicycle's control-constraint handling. After the ownership fix, the user reran all ten selected
regression tests successfully: three output/constraint tests, three derivative tests and four
cost/rollout tests. All three output/constraint tests also passed memcheck, racecheck and
synccheck, with zero errors and zero racecheck hazards/warnings. The `112*` November 6 captures
then showed small dynamics improvements, essentially unchanged cost time and no consistent
cache-handoff latency benefit. See the
[constraint-fixed performance analysis](../GPU_PERFORMANCE_REVIEW.md#constraint-fixed-performance-and-next-experiment-2026-11-07),
[constraint fix and validation](../GPU_PERFORMANCE_REVIEW.md#constraint-ownership-fix-and-validation-2026-11-06)
and the earlier
[results and constraint-race analysis](../GPU_PERFORMANCE_REVIEW.md#cooperative-output-store-results-and-constraint-race-2026-11-06)
for capture identities, comparison limits and the next validation steps.

The new `test_dynamics_output` executable contains GPU correctness tests, registered with the
`dynamics;gpu;gtest` CTest labels. It is separate from other CUDA test translation units to avoid
the vendor RiskMeasure multiple-definition issue. Performance benchmarks remain opt-in and are
never registered with CTest.

Run these commands yourself from the workspace root after sourcing your usual build environment:

```bash
colcon build --packages-select MPPI-Generic autoware_mppi_optimizer --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DMPPI_BUILD_PERFORMANCE_BENCHMARKS=ON
source install/setup.zsh

build/autoware_mppi_optimizer/test_dynamics_output
build/autoware_mppi_optimizer/test_dynamics_derivative
build/autoware_mppi_optimizer/test_costs \
  --gtest_filter='CostEvaluation.ObstacleYawInvalidationUsesNumericComparisonAndSign:GpuCostEvaluation.CachedObstacleAxes*:GpuCostEvaluation.CombinedAndSplitRollouts*:GpuCostEvaluation.VelocityDependentSteeringRatePropagationMatchesHost:GpuCostEvaluation.DelayedSteadyTurnAndCommandStepReplayMatchesHost'
```

`MPPI-Generic` is the package name for `src/vendor/mppi_generic_vendor`. Include it in the rebuild
so the installed headers and configure-time metadata agree with the source, including environments
that copy headers rather than symlinking them.

`GpuDynamicsOutput.CooperativeStorePreservesBitsAndLayout` checks eight output dimensions
(1, 2, 3, 4, 7, 13, 16, 33), seven block shapes, three horizons (1, 3, 80), and both consecutive and
alternate timestep writes. It compares every destination bit against a host reference, including
signed zero, subnormals, infinities and NaN payloads. It also checks guards, unchanged inputs and
untouched timesteps, using deliberately unaligned base pointers. Shapes cover `(32,2,1)`, partial
warps, non-power-of-two Y, multiple X blocks/Z distributions, and workers with no copy work.
Rollout counts cover full grids, partial final X blocks and a single rollout.

`GpuDynamicsOutput.ProductionRolloutMatchesLegacyStoresBits` compares both production split dynamics
specializations (legacy and cooperative stores) against a test-only frozen loop from vendor
`56e4b48`, retaining the old output stores. All use the production scalar bicycle model and colored-noise sampler with identical uploaded
controls; random generation is not invoked. All 13 outputs at every one of 80 timesteps and all
constrained controls must match bit for bit. The test checks seven block shapes, three delay
configurations, forward/reverse constraints, distinct distributions, guards and immutable initial
states, with full and partial grids and a single rollout. The frozen reference uses X=1 so every
reference block is complete for every rollout count. It retains the existing model arithmetic;
the separate derivative tests retain their independent arithmetic reference. The combined/split
cached-axis test also exercises the host launcher with 33 rollouts and a dynamics block width of 32.

`GpuDynamicsOutput.ControlConstraintsMatchHostAcrossWorkers` checks the production device
constraint method against serial host evaluation for 22 command/state cases, eight parameter
configurations and the same seven block shapes. It covers positive/negative range clipping,
deadbands and their boundaries, nonzero zero-control values, hold/release and short-reference
behavior, restart rate/acceleration limits, disabled restart blending and reverse prevention
enabled/disabled. A parent steering range narrower than the held command makes constraint ordering
observable. Known range/hold/reverse outcomes also anchor the host fixture independently.
Host comparisons allow 2e-6 absolute error for floating-point contraction; all GPU shapes must
match the Y=1 GPU result bit for bit. Per-record guards, shared state and global inputs must remain
unchanged. The kernel repeatedly reuses shared storage with the rollout's caller barriers and
adds no synchronization inside the constraint method.

There are three tests in `test_dynamics_output`. The legacy-store reference and both production
variants use the corrected model: their parity check isolates the copy change, while the
serial-host comparison separately checks the constraint behavior. Both combined/split tests in
`test_costs` also exercise the production launcher's two store selections and retain their
independent cost/safety expectations.

Then check memory and synchronization explicitly:

```bash
(
  set -e
  mppi_sanitizer=/usr/local/cuda-12.8/bin/compute-sanitizer
  for mppi_check in memcheck racecheck synccheck; do
    "$mppi_sanitizer" --tool "$mppi_check" --error-exitcode 1 \
      build/autoware_mppi_optimizer/test_dynamics_output
  done
)
```

The subshell stops with a nonzero status on the first failure. Require all three sanitizer
summaries to be clean; a GTest PASS line alone does not establish sanitizer success.

### Matched store comparison from one build

The first user-run matrix completed successfully on 2026-11-07 in
`projection_stores_20261106_212505_4n792O`: all 128 reports are valid and all stored call diagnostics
match, including across timing modes. Cooperative stores reduce complete-call medians by
7.69–12.87% and dynamics GPU means by 34.79–39.94%, with essentially unchanged cost time.
All 32 latency pairs improve in both median and p95. See the
[controlled comparison analysis](../GPU_PERFORMANCE_REVIEW.md#controlled-store-comparison-results-2026-11-07)
for build hashes, per-scene results and limits. This establishes the performance benefit on the
measured RTX 4090 workload; the collection does not include fresh output from the expanded
correctness/sanitizer checks. Retain cooperative stores as the default. Repeat the matrix after
relevant implementation changes or on another target environment using these instructions.

After rebuilding both packages and passing the correctness/sanitizer commands above, collect the
two store variants from that same build:

```bash
source install/setup.zsh
mppi_followup=src/autoware/universe/planning/autoware_mppi_optimizer/test/run_projection_followup.sh
export MPPI_BENCHMARK_ENVIRONMENT='constraint-fixed stores comparison; describe actual clocks and background load'
bash "$mppi_followup" stores
```

The `stores` mode keeps projection pruning enabled and alternates legacy/cooperative output
stores over four balanced pairs in each timing mode, across all eight scenes: **128 JSON reports**.
Each process uses 10 warmups, 100 measured calls and 10 optimizer iterations. It creates
`projection_stores_<timestamp>_*` with per-process reports, GPU snapshots and the executable and
resolved-library hashes. The new benchmark field `dynamics_output_store` must match the requested
variant. The ordinary `repeat` mode still toggles pruning only and keeps the selected store mode.

Both store choices explicitly use the split pipeline. Startup kernel-selection trials remain
identical for both variants; the store choice and split override are applied after controller
construction, before any optimize call. Initialization is recorded separately and excluded from
the steady-state comparison. The production interface keeps its normal kernel selection unless
its benchmark-only setter is explicitly called. There is no new ROS parameter, device runtime
branch, buffer or barrier; selection dispatches two compile-time dynamics specializations on
the host. Cooperative stores remain the default.

The runner invokes [compare_dynamics_output_stores.py](compare_dynamics_output_stores.py) after
collection. This offline script requires all 128 eligible reports, matching metadata and all
stored first/measured-call diagnostics across the eight processes per scene/timing mode. It
exits with an error on missing, invalid or divergent data, retaining the raw reports. A successful
comparison writes `store_comparison.json` and prints complete-call median results. The JSON
includes each process's statistics and paired differences for wall median/p95, CPU mean, dynamics
GPU, cost GPU and their sum. Values summarize four independent processes; GPU phase results come
from separate instrumented runs. Diagnostics agreement is not proof of full-trajectory bit parity;
the correctness tests supply independent evidence for the covered inputs.

For focused profiling after that comparison, the existing modes accept the same store selection.
For example, collect both full-set profiles from the unchanged build:

```bash
for mppi_stores in 0 1; do
  MPPI_BENCHMARK_COOPERATIVE_OUTPUT_STORES="$mppi_stores" bash "$mppi_followup" ncu
done
```

Use `nsys` or `handoff` in place of `ncu` when the timing comparison indicates a launch-gap or
cache question. Keep source matching the build during source import and keep the same clock
policy/background load throughout. The historical `012*`, `060707*` and `08*` collections predate
the constraint fix; the `112*` collections use cooperative stores after the fix. None substitutes
for the new same-build legacy/cooperative comparison.

Check that dynamics output-store excessive sectors fall from the previous **14,909,440 per launch**,
without reintroducing local loads/stores. Inspect registers, shared memory and instruction count
alongside duration. Preserve cost-output correctness and assess dynamics **plus cost**, followed
by complete-call medians/p95 and CPU time. Fewer store sectors alone do not establish a latency
benefit, and the preceding handoff experiment did not demonstrate a cost-cache speedup.

### Scalar dynamics derivative validation

The derivative-storage change computes six named scalar values and writes each Y worker's owned
entries directly. The integration and rollout barriers are unchanged. The November 6 user-run
results confirm passing isolated exact-parity tests and elimination of the sampled dynamics kernel's
local loads/stores. A later reverted-build comparison under similar measured clocks/background
clients confirms a 12.8–18.9% reduction in dynamics phase time. Pruning-enabled complete-call means
improve in all eight scenes, but medians and pruning-disabled results remain mixed; the overall
benefit is modest and scene-dependent. See the
[reverted-build comparison](../GPU_PERFORMANCE_REVIEW.md#reverted-build-comparison-under-similar-conditions-2026-11-06).
The original array-based calculation is retained only in `test/dynamics_derivative_reference.cuh`
as an independent numerical reference; it is not linked into the production optimizer or benchmark.

Reconfigure and build the optimizer, using your normal workspace environment:

```bash
colcon build --packages-select autoware_mppi_optimizer --cmake-force-configure \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DMPPI_BUILD_PERFORMANCE_BENCHMARKS=ON
source install/setup.zsh

build/autoware_mppi_optimizer/test_dynamics_derivative

build/autoware_mppi_optimizer/test_costs \
  --gtest_filter='GpuCostEvaluation.VelocityDependentSteeringRatePropagationMatchesHost:GpuCostEvaluation.DelayedSteadyTurnAndCommandStepReplayMatchesHost:GpuCostEvaluation.CombinedAndSplitRollouts*:GpuCostEvaluation.ZeroSoftWeightsStillReportExactSafetyViolations'
```

`DynamicsDerivative.ScalarHostMatchesOriginalBits` checks the shared arithmetic on the host.
`GpuDynamicsDerivative.ScalarDeviceMatchesOriginalBitsAndWriteOwnership` checks exact device bits,
signed zero, sentinel guards, immutable inputs and each worker's exclusive write set.
`GpuDynamicsDerivative.ScalarReplayMatchesOriginalBits` compares every derivative, next-state entry,
output and constrained control over 80 fixed-input steps. The GPU tests use 64 independent rollouts,
seven parameter configurations, and six block shapes, including `(32, 2, 1)`, non-power-of-two Y,
multiple X/Z rollouts, and Y workers with no derivative entry. Delay selection/clamping, actuator
time-constant floors, rate-limit boundaries, forward/reverse motion and hold/restart state are covered.
The derivative executable has separate CTest registrations: `test_dynamics_derivative` for the host
case and `test_dynamics_derivative_gpu` for the GPU cases, with `cpu`/`gpu` and `dynamics` labels.
It is separate from `test_costs` because the vendor headers include non-inline risk functions that
produce duplicate definitions when both CUDA translation units are linked into one executable.
Performance collection remains opt-in and absent from CTest. The reference model replaces only
derivative evaluation, retaining the production integration, delay advance, output and synchronization
code.

Run the GPU cases through Compute Sanitizer explicitly. It is available at the path below in the
current environment; adjust it to the target CUDA installation if needed. These commands are for
the developer to execute, and should finish without memory, race or synchronization errors:

```bash
mppi_sanitizer=/usr/local/cuda-12.8/bin/compute-sanitizer
for mppi_check in memcheck racecheck synccheck; do
  "$mppi_sanitizer" --tool "$mppi_check" --error-exitcode 1 \
    build/autoware_mppi_optimizer/test_dynamics_derivative \
    --gtest_filter='GpuDynamicsDerivative.*' || break
done
```

After correctness validation, use the same manually invoked collection modes:

```bash
mppi_followup=src/autoware/universe/planning/autoware_mppi_optimizer/test/run_projection_followup.sh
bash "$mppi_followup" repeat
bash "$mppi_followup" nsys
bash "$mppi_followup" ncu
```

Compare against `projection_repeat_20261105_201641_EMP3VC`,
`projection_nsys_20261105_202220_HH0bQ7`, and `projection_ncu_20261105_202229_UzNsY7`, holding
scene, pruning setting, timing mode, profiles, iterations and hardware conditions fixed. Compare
independent process-level statistics; the paired switch still toggles projection pruning, not the
derivative implementation. New manifests hash the resolved optimizer library as well as the
executable. Source revisions in JSON remain configure-time metadata.

The `projection_*_20261106_025*` captures contain the reverted array binary and provide a closer
baseline for the scalar `projection_*_20261106_012*` captures. These are separate collection periods;
alternating known old/new binaries remains preferable when resolving small latency differences.
Keep clock policy and background GPU load comparable, and preserve the source matching each compiled
library until profiling is finished. The reverted capture embedded the restored scalar source text;
its binary identity was established from machine code and counters, not its source-line display.

Check the production `rolloutDynamicsKernel` source/SASS for elimination of accesses associated
with the old `local_der` array. The old reports contain 1,105,920 executed local-load instructions
and 2,048,000 local-store instructions per sampled launch, attributed to derivative staging. Check
for replacement spills, register use and occupancy, then compare dynamics GPU time and complete-call
median/p95 across all eight scenes. A scalar-looking C++ implementation or a lower instruction count
alone does not establish a speedup. The acceptance target is exact isolated parity and a reproducible
dynamics improvement without meaningful complete-call regressions. The existing adaptive grid
variability remains a separate diagnostic issue.

### Exact obstacle-axis cache validation

The exact device collision loop now reads obstacle cosine/sine values computed with the original
CUDA `cosf`/`sinf`. The cache resides outside the per-cycle upload snapshot, is rebuilt on count/yaw
changes, and retains the 64-register cap. CPU/fractional-step validation and soft-distance/map
trigonometry retain their previous arithmetic. The user-run focused seven-test selection and full
53-test cost suite passed. All four new cache cases also passed memcheck/racecheck/synccheck with
zero reported errors or hazards. The subsequent 128-report repeat collection and nine
profiler-associated benchmark reports are valid, and all stored diagnostics match the pre-cache
baseline. Static-64 complete-call median improved by 6.65% and cost GPU mean by 16.21%; other
obstacle scenes show smaller gains, while obstacle-free medians increased by at most 0.24%.
The agent has not run builds or tests. See the
[implementation and validation notes](../GPU_PERFORMANCE_REVIEW.md#exact-obstacle-axis-cache-implementation-and-correctness-validation).
The [performance analysis](../GPU_PERFORMANCE_REVIEW.md#obstacle-axis-cache-performance-results-2026-11-07)
records all eight scenes, source/counter evidence and the limits of historical comparisons.

After rebuilding in the sourced workspace, run the focused checks, followed by the complete cost
suite before collecting performance results:

```bash
build/autoware_mppi_optimizer/test_costs \
  --gtest_filter='GpuCostEvaluation.CachedObstacleAxes*:GpuCostEvaluation.CombinedAndSplitRollouts*:GpuCostEvaluation.ZeroSoftWeightsStillReportExactSafetyViolations'

build/autoware_mppi_optimizer/test_costs
```

The new checks compare cached trig bits against direct device evaluation, and intersection flags
and first indices against the previous exact collision loop. They cover all 64 slots, every
timestep, partial prediction padding, signed zero, large/nonfinite angles, static/moving cutoffs,
count changes, clear/re-add, zero-length predictions, reuse and batched updates. Nondefault streams,
stream rebinding, repeated setup and teardown/re-setup are included. The production rollout case
rotates an obstacle into and out of collision using combined and both split store variants.

Run the focused cases under the existing sanitizer installation:

```bash
mppi_sanitizer=/usr/local/cuda-12.8/bin/compute-sanitizer
for mppi_check in memcheck racecheck synccheck; do
  "$mppi_sanitizer" --tool "$mppi_check" --error-exitcode 1 \
    build/autoware_mppi_optimizer/test_costs \
    --gtest_filter='GpuCostEvaluation.CachedObstacleAxes*:GpuCostEvaluation.CombinedAndSplitRolloutsUseCachedObstacleAxes' || break
done
```

For performance, retain pruning and cooperative stores in both baseline/candidate measurements.
Use the existing eight-scene matrix in latency-only and phase modes, with repeated processes and
matching clock/load conditions. The cooperative rows from
`projection_stores_20261106_212505_4n792O` supply the current historical baseline; alternating known
old/new builds provides stronger attribution. The existing `repeat` runner can collect the full
matrix, but its switch compares projection pruning, not the obstacle-axis implementation.

The existing `nsys` and `ncu` runner modes remain applicable. Inspect `MPPI/obstacle_axes_build`
and `cacheObstacleAxesKernel` in the timeline and phase JSON: builds should appear on initial
nonempty upload or changed count/yaw, not on unchanged warm calls or position-only updates.
The normal NCU command targets `rolloutCostKernel`, so its source/SASS should show cached axis
loads replacing the obstacle loop's repeated trig. Check registers, replacement spills, local
traffic, cost duration and complete-call timing. A reduction in trig instructions is not by itself
a speedup. Compare numerical diagnostics and eligibility alongside timing. The completed
`projection_repeat_20261107_001334_tVBLp2`, `projection_nsys_20261107_001825_tmYVE2` and
`projection_ncu_20261107_001833_Tr1iAK` collections use the same cached-axis library and provide
the baseline for subsequent work. Cost allocation remains 64 registers, and local/spill counts
are unchanged. Cache builds occur only on initial nonempty upload in these captures; position-only
updates reuse the axes. These stationary scenes do not measure continuously changing yaw or moving
predictions. Stored diagnostic equality complements the isolated parity tests but does not prove
bitwise equality of every complete output trajectory.

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
