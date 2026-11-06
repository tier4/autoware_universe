# GPU performance review of the MPPI optimizer

## Scope and assessment

The original static review covered universe commit `48a3a7223f` and MPPI-Generic vendor commit `32aaff5948091dcc428a9ac5ea877f49f7f6d5d6`, including local vendor test cleanup concerning resource handling, not production kernels. No builds, tests, simulations, benchmarks, or GPU profiling were run as part of that static analysis. This document now also includes a [2026-11-04 measured follow-up](#measured-follow-up-2026-11-04), based on benchmarks and Nsight Compute captures run by the user. The follow-up analysis read saved reports and source code; the assistant did not run builds, tests, or new profiling.

Subsequent local work adds a sorted, device-resident vegetation point list and an exact footprint check after the texture broad phase. This fixes rollout eligibility near the hard-clearance boundary, but adds host sorting, a point-list upload, and conditional device point reads. The measured follow-up does not cover vegetation; its performance impact remains unmeasured here.

The architecture has useful performance foundations: persistent CUDA resources and cuFFT plans, batched scene updates, cached distance textures, device-side weighting and control updates, and one final result download in the normal optimization loop. The remaining opportunities concern the amount and organization of work: exhaustive path projection, dense time-indexed map generation, a single-block percentile/weighting kernel, control-buffer access patterns, and CPU/debug work on the publication path. Static inspection establishes those mechanisms. The measured follow-up now identifies exact path projection as the first optimization target for the sampled RTX 4090 workloads; it does not establish deployment-wide priorities or promise a speedup.

The [requirements inventory](FEATURE_REQUIREMENTS_REVIEW.md) remains the behavior authority. Preserve the primary-candidate limitation, all highest-confidence prediction ties, newly issuable first-command exploration, independent hard safety checks, post-step timing, and transactional execution history. Percentile normalization, cycle-level ESS adaptation, and rejecting unsafe rollouts are the current variant, not defects merely because another MPPI formulation differs. Recommendations below are performance changes unless explicitly marked as policy-dependent.

Line anchors in the original review refer to its source snapshot and may have shifted. The historical benchmark source has since been replaced; its old line references describe the original harness. `I` below means the [CUDA interface](src/first_order_dubins/first_order_dubins_mppi_interface.cu); vendor sources are under `src/vendor/mppi_generic_vendor`.

## Execution path and scale

| Phase                                 | Current work and ordering                                                                                                                                                                                         | Evidence                                                                                                                                                                                                                                 |
| ------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Candidate and activation              | Secondary candidates return immediately. Primary processing performs input checks, scene selection, route queries and optional collision activation on the host before optimization.                              | [plugin, lines 519–684](src/trajectory_mppi_optimizer.cpp#L519)                                                                                                                                                                          |
| Scene and nominal preparation         | Validate selected prediction capacity, prepare the local frame, build nominal controls and limits, sample object hypotheses, stage scene data, then commit the update.                                            | [I, lines 1995–2037](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L1995), [2292–2395](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L2292)                                                                  |
| Upload and map generation             | One deferred runtime-data snapshot upload precedes dirty distance-map and nearest-segment builds on the cost stream.                                                                                              | [cost, lines 377–447](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu#L377)                                                                                                                                        |
| Sampling, every iteration             | cuRAND fills frequency-domain noise; frequency scaling, batched inverse cuFFT, reordering and Gaussian control rewriting follow.                                                                                  | [colored sampler, lines 368–426](../../../../vendor/mppi_generic_vendor/include/mppi/sampling_distributions/colored_noise/colored_noise.cu#L368)                                                                                         |
| Rollouts, every iteration             | A constructor-selected combined kernel, or separate dynamics and cost kernels, evaluates all rollouts.                                                                                                            | [controller, lines 276–300](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L276)                                                                                                                |
| Weighting and update, every iteration | Eligibility, exact percentile normalization, weights and ESS are computed on device; the sampler updates its mean on device.                                                                                      | [controller, lines 315–338](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L315)                                                                                                                |
| Download and host replay              | Per-iteration statistics and the final 160-float control sequence are downloaded. The vendor replays/smooths controls; the interface restores the unsmoothed mean and replays it again.                           | [controller, lines 342–450](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L342), [I, lines 299–329](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L299)                          |
| Validation and application            | Reconstruct terminal state, apply control/profile processing, validate output states, apply engage adjustment, prepare diagnostics/logging, resolve fallback, then commit history only if selected by the plugin. | [I, lines 3169–3257](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L3169), [3337–3466](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L3337), [plugin, lines 837–852](src/trajectory_mppi_optimizer.cpp#L837) |

The fixed workload is **16,384 rollouts × 80 steps × 2 controls**, at 0.1 s. The shipped profile uses **10 iterations**. The historical benchmark used **20**; the current [opt-in benchmark](test/PERFORMANCE_BENCHMARKS.md) loads the shipped profile and uses 10 unless explicitly overridden. This means 1,310,720 state transitions per iteration and 13,107,200 per shipped-profile cycle, excluding initialization, map construction and host replays. These are operation counts, not measured timing. See [I, lines 62–68](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L62), [launch configuration, lines 1163–1195](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L1163), [YAML, line 33](config/mppi_optimizer.param.yaml#L33), and [benchmark constants](test/test_performance_benchmarks.cpp).

Approximate logical buffer payloads for one distribution are:

| Buffer                                | Calculation                                     |   Payload |
| ------------------------------------- | ----------------------------------------------- | --------: |
| Sampled controls                      | 16,384 × 80 × 2 × 4 bytes                       |    10 MiB |
| Frequency noise                       | 32,768 transforms × 81 complex floats × 8 bytes | 20.25 MiB |
| Time-domain noise                     | 32,768 × 160 × 4 bytes                          |    20 MiB |
| Split-rollout outputs                 | 16,384 × 80 × 13 floats                         |    65 MiB |
| Obstacle texture                      | 512 × 512 × 80 floats                           |    80 MiB |
| Static environment texture            | 1,024 × 1,024 × float4                          |    16 MiB |
| Optional preferred-centerline texture | 1,024 × 1,024 floats                            |     4 MiB |
| Nearest-segment index texture         | 256 × 256 × uint8                               |    64 KiB |

These are payload estimates, excluding cuFFT workspace, CUDA-array layout overhead, staging buffers, other controller allocations and driver/context memory. They are not a measured allocation total. The output allocation is made by the base controller even when the combined rollout kernel is selected. Sources: [colored allocation, lines 218–294](../../../../vendor/mppi_generic_vendor/include/mppi/sampling_distributions/colored_noise/colored_noise.cu#L218), [controller allocation, lines 209–225](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/controller.cu#L209), [13-output model, lines 70–86](include/mppi/dynamics/dubins/first_order_dubins_bicycle.cuh#L70), [texture dimensions, lines 17 and 62–71](include/mppi/cost_functions/dubins/distance_map_texture.cuh#L62), and [texture allocations, lines 450–565](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L450).

## Historical benchmark evidence and its limits

The previous authorized build/test run left `/tmp/mppi_build_test_verified.log`. Lines 863–909 record these interface-level measurements on an RTX 4090, CUDA runtime 12080 and driver 13000. They are reproduced here because the temporary log is not a durable repository artifact.

| Synthetic case                                 | Average ms | Minimum ms | Maximum ms |
| ---------------------------------------------- | ---------: | ---------: | ---------: |
| Preferred centerline texture, one segment      |     39.444 |     39.014 |     43.299 |
| Preferred centerline exact, one segment        |     39.114 |     38.962 |     39.912 |
| Preferred centerline moving grid, 256 segments |     39.486 |     39.339 |     40.058 |
| Free space                                     |     38.935 |     38.796 |     39.673 |
| 64 moving obstacles                            |     49.575 |     49.396 |     50.437 |
| Tight corridor, 256 road-border segments       |     48.158 |     47.870 |     48.945 |
| 256-point hairpin                              |     56.242 |     55.072 |     57.417 |

These numbers show workload sensitivity, not which kernel caused it. In particular:

- That historical harness used 20 iterations, one unmeasured warmup, 50 measured calls, interface defaults rather than the product profile, disabled logging/iteration debug, disabled nominal reuse, and `skip_if_invalid=false`. It checked a completed finite optimization, not safe application or tracking quality. See the original snapshot's [benchmark, lines 150–196](test/test_performance_benchmarks.cpp#L150).
- Most cases reuse identical inputs and timestamps after warmup. Unchanged geometry can keep its maps cached, including the heavy-obstacle case: nonzero obstacle velocity changes poses along the prediction horizon, but repeated identical inputs do not advance the next planning epoch. Therefore its extra latency cannot be attributed to rebuilding 80 texture layers on every measured call. Only the explicit moving-grid case changes trajectory and odometry each call. See [lines 222–246](test/test_performance_benchmarks.cpp#L222) and [geometry dirty comparison](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu#L688).
- The moving-grid case concerns a preferred centerline; it is not a moving-object-map benchmark. The one-segment texture/exact comparison also cannot establish the break-even point for 256 segments. See [cases, lines 287–327](test/test_performance_benchmarks.cpp#L287).
- The timed interval includes `optimizeTrajectory` and a final `cudaDeviceSynchronize`, while the preceding drain, assertions and result destruction are outside timing. It excludes the ROS plugin's input selection, route queries, marker construction and publication. It reports mean/min/max, not p95/p99, interference sensitivity or cold-start cost. See [lines 242–273](test/test_performance_benchmarks.cpp#L242).

Do not divide these times by two to predict the shipped 10-iteration latency: scene setup, maps, downloads and host work do not all scale with iteration count. Do not use a 0.1 s model timestep as evidence of an allocated 100 ms scheduling deadline; the deployment latency budget has not been specified.

## Measured follow-up: 2026-11-04

### Evidence, configuration, and validity

The user ran all 13 standalone scenarios successfully in latency-only mode and with optional phase timing. A later instrumented run separated `MPPI/rollout_dynamics` and `MPPI/rollout_cost` inside the inclusive `MPPI/rollout` range. All 13 scenarios passed their workload, outcome, phase-count, and map-rebuild assertions. Dynamics and cost each recorded ten CUDA event pairs per optimizer call, including the first call; startup kernel-selection trials were excluded from those subphase counts. Cached scenes avoided rebuilds, and the geometry/grid update cases rebuilt the expected maps.

The subsequent Nsight Compute captures are in workspace-relative `mppi_benchmark_results/ncu_20261104_174709/`:

- [FreeSpaceCached.txt](../../../../../mppi_benchmark_results/ncu_20261104_174709/FreeSpaceCached.txt) and [saved report](../../../../../mppi_benchmark_results/ncu_20261104_174709/FreeSpaceCached.ncu-rep).
- [Hairpin256Cached.txt](../../../../../mppi_benchmark_results/ncu_20261104_174709/Hairpin256Cached.txt) and [saved report](../../../../../mppi_benchmark_results/ncu_20261104_174709/Hairpin256Cached.ncu-rep).
- [StaticObstacles64Cached.txt](../../../../../mppi_benchmark_results/ncu_20261104_174709/StaticObstacles64Cached.txt) and [saved report](../../../../../mppi_benchmark_results/ncu_20261104_174709/StaticObstacles64Cached.ncu-rep).
- The corresponding configuration/outcome JSON files are in that directory's `benchmark_json/` subdirectory.

These are local measurement artifacts, not guaranteed to be included in a Git checkout. The tables below preserve their principal findings if the artifacts are unavailable elsewhere.

Recorded configuration:

| Item                      | Value                                                                                              |
| ------------------------- | -------------------------------------------------------------------------------------------------- |
| GPU / host                | RTX 4090, compute capability 8.9, 128 SMs / Intel Core i9-14900K                                   |
| Software                  | CUDA toolkit 12.8, runtime 12080, driver API version 13000; Nsight Compute 2025.1.1                |
| Build                     | Release, GNU 11.4.0, `-O3 -DNDEBUG`, CUDA `-lineinfo`, `--maxrregcount=64`                         |
| Configure-time revisions  | Universe `x2/v4.4.0-551-ga26463dca4-dirty`; vendor `cc3104c-dirty`                                 |
| Workload                  | 16,384 rollouts, 80 steps, 0.1 s timestep, 10 iterations, seed 1; split rollout kernel             |
| Profiles                  | Shipped optimizer cost profile and synthetic vehicle fixture; resolved inputs are recorded in JSON |
| Ordinary instrumented run | 10 warmup calls and 100 measured calls per scenario                                                |
| Nsight run                | Phase timing off; 10 warmup calls and one measured optimizer call per scenario                     |

The profiler used kernel replay with the `full` section set, source import, `--clock-control none`, and NVTX filter `MPPI\/rollout_cost/`. It skipped 100 matching launches and collected three cost-kernel launches per scenario. Each capture has the expected `MPPI/rollout` → `MPPI/rollout_cost` stack, 16,384 blocks, and 80 threads per block. All three benchmark JSON reports have `measurements_valid=true`, no unexpected outcomes, and the intended configuration. All nine captures have zero dropped PC-sampling bytes.

The reports warn that GPU clocks were not fixed. Recorded SM frequencies were 2.50–2.52 GHz. The data support identifying large bottlenecks, not small regression claims. The captures cover three successive iterations of one warm optimizer call per scenario, not three independent process-level timing trials. Benchmark wall times produced under Nsight include profiler overhead and must not be used as latency baselines. These fixtures do not establish a deployment deadline, closed-loop quality, or behavior on other GPUs.

### Phase timing before Nsight profiling

These are means per complete ten-iteration optimizer call from the phase-instrumented run, not from Nsight. The reports are [free space](../../../../../mppi_benchmark_results/instrumented/FreeSpaceCached_1793778210159697557_2824689.json), [hairpin](../../../../../mppi_benchmark_results/instrumented/Hairpin256Cached_1793778230976029493_2824689.json), and [64 obstacles](../../../../../mppi_benchmark_results/instrumented/StaticObstacles64Cached_1793778215150399729_2824689.json).

| Scenario                | Dynamics GPU ms | Cost GPU ms | Optimizer wall ms |
| ----------------------- | --------------: | ----------: | ----------------: |
| FreeSpaceCached         |           6.585 |       6.496 |            20.683 |
| Hairpin256Cached        |           6.612 |      15.450 |            29.919 |
| StaticObstacles64Cached |           6.585 |      10.540 |            25.185 |

Dynamics remained approximately 6.58–6.61 ms across the full 13-scenario matrix. Cost evaluation accounted for about 99.7% of the rollout-time increase from free space to the hairpin. The parent rollout time includes both subphases; do not add it to their totals. CUDA event times are stream elapsed times, not utilization. The host's final-result wait overlaps queued GPU work and is also not additive.

### Cost-kernel findings

The following Nsight durations are means across the three captures, **per cost-kernel launch / optimizer iteration**, not per ten-iteration call. Executed instruction counts are warp-level counts, not floating-point operation counts.

| Scenario                | Kernel duration ms | Executed warp instructions | Achieved occupancy | DRAM throughput, % of peak |
| ----------------------- | -----------------: | -------------------------: | -----------------: | -------------------------: |
| FreeSpaceCached         |              0.659 |              474.6 million |         36.4–37.0% |                 12.8–17.6% |
| Hairpin256Cached        |              1.556 |            1,233.9 million |         35.1–35.2% |                       5.4% |
| StaticObstacles64Cached |              1.087 |              707.9 million |         41.0–41.2% |                   7.9–8.0% |

**1. Path projection is the first optimization target.** Offline source attribution from the existing `.ncu-rep` files maps 90.2% of the hairpin's executed warp instructions and 81.8% of its sampled warp states to `path_tracking_geometry.cuh`. The corresponding free-space shares are 74.5% and 53.4%. Almost the entire instruction-count increase between those cases is in this file. The hot lines are the segment-distance calculation and division in `distanceToPolylineSegment`, and the loop and nearest-segment comparison in `projectPointToPolyline`. In these two scenarios there are no obstacles or road-border segments; the reference grows from 80 to 256 points and changes shape. This confirms the projection hotspot, although it is not a controlled experiment varying only point count.

The attribution was obtained by reading each report with the installed `ncu_report` Python API, mapping `inst_executed` and `smsp__pcsamp_sample_count` correlation IDs to source file/line, and aggregating counts over the three captures. Every counted sample/instruction had source attribution. Sample shares are diagnostic evidence, not exact fractions of elapsed kernel time. Source attribution and low DRAM throughput support reducing repeated projection arithmetic/search work before attempting a bandwidth-focused rewrite.

**2. The obstacle workload adds memory-latency and divergent-work costs.** Average long-scoreboard stalls increase from approximately 1.35 to 2.53 cycles per issued instruction, and barrier stalls from approximately 0.46 to 1.45. Average active threads per warp fall from about 20 to 17. Source/SASS attribution points to consumers of obstacle yaw and position loads in `egoIntersectsObstacleAtStep` and `orientedBoxesOverlap`, plus synchronization around cost evaluation. A hot source line containing `cosf` is not evidence that all its attributed samples represent cosine arithmetic: its sampled instruction waits on input data. DRAM throughput near 8% of peak does not indicate external-memory-bandwidth saturation. Repeated obstacle-data access and uneven exact-collision work are a separate, subsequent optimization target; preserve the exact safety checks.

**3. Occupancy and spilling are secondary opportunities.** Every capture uses 64 registers per thread and approximately 11.52 kB of dynamic shared memory per block. The theoretical occupancy ceiling is 50%: shared memory allows eight resident blocks per SM, while the register limit would allow ten. Achieved occupancy is 35–41%. The profiler records about 1.98 million executed register-spilling instructions per launch in all three cases, but those counts alone do not establish their latency contribution. Changing the register cap requires measuring the spill/residency tradeoff. The 80-thread block also leaves half of its third warp inactive before additional divergence; the profiler's generic suggestion to use 128–256 threads cannot be applied directly because the current split-cost launch requires `dimCostBlock.x <= num_timesteps` (80).

### Next implementation: exact path-projection optimization

**Implementation and lifecycle.** The cost projector now reuses the local-frame endpoint differences (`dx`, `dy`) for each reference and corridor segment. The two arrays live in the pinned host runtime snapshot and its device copy; the existing cost-stream runtime upload transfers them with the vertices. Geometry comparison rebuilds only the affected array when its vertices change. The reference array is maintained even while a corridor is active, so corridor removal immediately uses the current reference. Replacing a corridor, including after a local-frame recentering, rebuilds its array. CPU projection reads the host snapshot; GPU projection reads the device snapshot. Neither array is staged in block shared memory, which avoids raising the shared-memory occupancy constraint. At 80 reference points and 256 corridor points the added runtime snapshot storage is 2,688 bytes, included in existing uploads.

Squared segment lengths remain query-side: evaluating `dx * dx + dy * dy` on the querying processor retains the original arithmetic and avoids a host/device rounding or contraction difference. The scan still computes distance with the original division, square root and `<`/`==` tie rule, visits every finite segment, and finalizes with the original endpoint and signed-offset code. The saved Nsight attribution establishes a hotspot, but by itself cannot establish that two cached subtractions and endpoint loads outweigh the extra delta loads and upload bytes. If measured benefit is insufficient, the exact follow-up is a conservative spatial hierarchy or cell structure whose distance lower bounds prune only segments unable to beat the current best (including lower-index ties), followed by the unchanged exact segment calculation for every surviving candidate. This would require separate build/traversal and candidate-count measurements.

There is enough evidence to begin one focused, behavior-preserving projection change. Start by investigating precomputation of invariant segment geometry (`dx`, `dy`, squared lengths and related data) when the reference/corridor geometry changes, and reuse that data across rollouts and iterations. Account for upload/storage costs and invalidation after reference updates, corridor replacement/removal, or local-frame changes. The reference-only fallback and CPU path must remain supported. Measure the complete optimizer call as well as the cost kernel; moving work into every cycle's setup is not automatically a gain.

Keep the current exhaustive implementation as an independent correctness oracle. Preserve globally closest finite-segment selection, lowest-index ties for equal computed distances, raw projection parameter, signed lateral distance, endpoint extrapolation, progress/overshoot and spatial-velocity semantics. Cover empty/singleton/degenerate paths, duplicate vertices, crossings, parallel/nonuniform hairpins, near ties and safety boundaries, all valid/invalid hints, texture on/off and out-of-grid queries. Hints must not restrict global coverage. Reciprocal substitution, squared-distance comparisons, fast math and floating-point reassociation can affect ties or threshold decisions; do not silently change the numerical contract or loosen parity tolerances.

Relevant production code is [path_tracking_geometry.cuh](include/mppi/cost_functions/path_tracking_geometry.cuh), [the cost implementation](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu), [cost data definitions](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh), and [the interface](src/first_order_dubins/first_order_dubins_mppi_interface.cu). Existing independent fixtures and parity coverage are in [projection_test_cases.hpp](test/projection_test_cases.hpp), [test_path_tracking_geometry.cpp](test/test_path_tracking_geometry.cpp), [test_distance_map.cpp](test/test_distance_map.cpp), and [test_costs.cu](test/test_costs.cu). Extend these GTests for the optimized path, cache invalidation and near-tie cases, including both split and combined rollouts and unchanged safety flags. An oracle that calls the newly optimized implementation would not provide independent equivalence evidence.

If invariant precomputation is insufficient, evaluate exact spatial pruning with conservative distance bounds and retain the existing exact segment calculation for candidate evaluation. A bounded local walk or a texture hint alone is not an exact replacement. Keep rollout count, horizon, iterations, cost weights, eligibility policy and final validation unchanged. Keep performance tests opt-in and use the same profiles, seeds, instrumentation modes and scenarios for before/after comparisons. The standing session constraint is **do not run builds, tests, benchmarks or new profiling on the user's behalf**; provide the commands for the user to execute.

### Validation and timing follow-up: 2026-11-04

The user built and ran the supplied commands. All four host geometry tests, three selected GPU projection tests, four selected cost/rollout tests, and all five selected scenarios in each benchmark mode passed. The benchmark JSON reports mark every measurement valid with zero unexpected outcomes, 100 measured calls after ten warmup calls, 16,384 rollouts, 80 steps, ten iterations, seed 1, and the split kernel. The before/after instrumented reports use the same recorded GPU, toolkit, compiler flags, build revision strings, resolved input profiles, and scene definitions. The revision strings end in `-dirty`, so they do not uniquely identify the modified source contents.

The following values are means per complete ten-iteration optimizer call from the [pre-change instrumented reports](../../../../../mppi_benchmark_results/instrumented/) and [post-change instrumented reports](../../../../../mppi_benchmark_results/projection_after_phases/). They are separate benchmark processes on the same RTX 4090; clocks were not fixed. GPU cost is the nested `MPPI/rollout_cost` phase, while wall time covers the whole optimizer call. Neither value is an Nsight duration.

| Scenario                      | Wall before → after (ms) | Cost GPU before → after (ms) | Cost change |
| ----------------------------- | -----------------------: | ---------------------------: | ----------: |
| FreeSpaceCached               |          20.683 → 20.258 |                6.496 → 6.155 |      −5.25% |
| Hairpin256Cached              |          29.919 → 28.809 |              15.450 → 14.407 |      −6.75% |
| StaticObstacles64Cached       |          25.185 → 24.884 |              10.540 → 10.288 |      −2.40% |
| StaticObstacleGeometryUpdates |          22.465 → 22.153 |                7.725 → 7.471 |      −3.29% |
| GridBoundaryCrossings         |          27.009 → 26.796 |              11.813 → 11.656 |      −1.33% |

The hairpin's observed cost reduction is about 1.043 ms per call, and its complete instrumented call is about 1.110 ms lower. Dynamics stays near 6.56–6.61 ms; the hairpin runtime-upload GPU mean is 0.0077 ms before and 0.0075 ms after. This points to reduced cost-kernel work without a visible setup penalty in these runs. The [post-change latency-only reports](../../../../../mppi_benchmark_results/projection_after_latency/) also passed; their medians are 20.096, 28.868, 24.724, 22.003 and 26.653 ms in table order. The available earlier latency-only reports have different recorded package and vendor revisions, so they are not a controlled latency-only before/after comparison for this change.

The selected tests establish projection parity across the listed fixtures and lifecycle transitions, not bitwise equality of complete MPPI output over every cycle. Some per-cycle baseline-cost and adaptive-temperature values diverge after multiple cycles; the grid-crossing run also differs in eligible-rollout count by as many as 80 in 20 samples. The two post-change modes show the same grid-crossing divergence, so it cannot be attributed to the projection change from these reports. All runs still passed their outcome and workload assertions. Repeated independent processes with alternating revisions and controlled GPU conditions would tighten the speedup estimate. No new Nsight capture or deployment-latency measurement was made.

### Review follow-up: 2026-11-05

The review added a direct device-to-device projection regression test. Separate CUDA kernel instantiations run the original exhaustive projector and the prepared projector on identical uploaded vertices, including shared/global access, all valid and invalid hint indices in the tested range, oblique segments, adjacent representable boundary queries, and reference-only fallback. Segment selection, raw projection parameter, and signed lateral distance require exact numerical equality; the production state-cost safety status is also compared against the original device projection's lateral decision. The translated near-tie fixture now uses `std::nextafter` on both sides of 51 m and asserts segment indices 0, 0, and 2. Its name now describes translated vertices rather than actual frame recentering; the [validation commands](test/PERFORMANCE_BENCHMARKS.md#explicit-build-and-execution) additionally select the existing interface test that crosses the recentering threshold. These changes have not been built or run by the assistant and are not covered by the earlier passing results or timing measurements.

The user subsequently ran these tests successfully as part of the grouped-pruning validation below.

### Exact grouped spatial pruning: 2026-11-05

The next implementation groups consecutive segments in sets of eight and stores a conservative
XY box for each group. The deployed 80-point reference and 256-point corridor need 10 and 32 boxes,
respectively: 672 bytes of bounds, plus validity/control flags and any structure alignment. These
are part of the existing pinned runtime snapshot and ordered upload; no shared-memory allocation,
device allocation per query, traversal stack, or reordered segment storage is added. Bounds are
rebuilt when local vertices/counts change, including reference changes while a corridor is active.
Velocity-only updates reuse them. Corridor removal selects the maintained reference bounds.

On the GPU, a valid hint is evaluated first. Groups are visited in original segment order, with
exact candidate evaluation for each retained group and no duplicate hint evaluation. A group is
skipped only when its lower bound is **strictly greater** than the current computed distance.
Equality remains eligible, preserving the lowest-index tie rule. The original candidate distance,
division, square root, projection finalization, and downstream safety/progress/velocity code remain
unchanged. The CPU retains the prepared exhaustive search. This is still linear in the number of
groups; overlapping groups can require all segment evaluations and add overhead.

#### Floating-point bound contract

An ideal endpoint AABB is insufficient: for example, float endpoints `1e8` and `1` have cached delta
`-1e8`, so the computed endpoint `x0 + dx` is `0`, outside the ideal `[1, 1e8]` interval. Each box
therefore encloses `x0` and the rounded `x0 + dx`, expanded outward by one representable float via
`nextafter`; the same construction applies to Y. For any clamped float parameter in `[0,1]`,
monotonic multiplication/addition keeps both the fused and unfused interpolation inside this
interval. Degenerate-segment distances use the starting vertex, also enclosed. The outward step is
a rounding enclosure, not a geometric tolerance or a change to candidate arithmetic.

For a query outside a box, the positive axis gaps are computed with downward-rounded subtraction.
Each is at most the absolute rounded residual to any computed footpoint inside the box. Squaring,
adding, and taking the square root with explicit downward-rounded CUDA intrinsics then gives a
lower bound on the original computed norm, including either FMA contraction choice for its sum of
squares. Comparing this bound directly to `best_dist` avoids introducing squared-distance rounding
ties. The relevant rounding operations are specified in NVIDIA's
[single-precision intrinsic documentation](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-math-api/cuda_math_api/group__CUDA__MATH__INTRINSIC__SINGLE.html).

The supported domain requires finite vertices and queries with each coordinate magnitude at most
`2^30`, with deltas prepared from those same vertices. Residuals, squared lengths, and projection
numerators then stay far below float overflow; a nondegenerate divisor is at least `1e-8`.
Out-of-domain queries, invalid bounds, and paths with at most eight segments use exhaustive search.
The proof assumes standard IEEE arithmetic, gradual underflow, and correctly rounded square roots.
CMake conservatively disables pruning for configured fast-math/FTZ/approximate arithmetic flags;
consumers without its precision definition also fall back. Reconfigure if compiler/environment
flags change, and do not assert `MPPI_PROJECTION_PRECISE_MATH` in an external build without this
contract. NVIDIA documents the relevant switches in its
[NVCC precision options](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-compiler-driver-nvcc/index.html#use-fast-math-use-fast-math).

#### Validation and measurement additions

The GTests now compare original, prepared exhaustive, and grouped device projections with exact
numerical equality, plus production lateral safety status. They check the lower-bound property
against independent original device segment distances, ties crossing group boundaries, adjacent
representable safety queries on a long path, rounded endpoints, seeded fractional/random paths,
degenerate groups, small/large coordinates, all hints, numeric fallback, and geometry lifecycle.
The combined/split rollout fixture now uses a long U-shaped path and runs with pruning off/on.
The existing interface recentering test remains part of the validation commands.

An explicitly disabled GTest reports groups examined/pruned, exact segment evaluations, and
exhaustive fallbacks for synthetic separated/overlapping groups at group sizes 4, 8, and 16. Its
counters are compiled out of the production projector; these are diagnostic workloads, not a trace
of optimizer rollouts. Production uses groups of eight. The opt-in standalone benchmark adds a
pruning switch, active geometry/build metadata, host bounds-preparation phases, and changing-reference,
short-path, and overlapping-group scenes. See the
[commands and comparison limits](test/PERFORMANCE_BENCHMARKS.md#exact-spatial-pruning-validation).

No build, test, benchmark, or profiler was run by the assistant for this change. The user subsequently
ran the validation and benchmark commands; the results below replace the initial pending status.

#### Grouped pruning validation and timing results: 2026-11-05

The user's output shows all 18 selected tests passing: five host geometry tests, seven GPU projection
tests, four cost/safety/rollout tests, one interface recentering test, and the explicitly enabled
work-count diagnostic. This includes the exact original/prepared/pruned device comparisons and
lower-bound checks. The earlier review's direct-parity and translated near-tie additions are also
covered by this run. No test fix was indicated by the supplied output.

All 32 saved benchmark reports are valid, with zero unexpected outcomes and complete measured
workloads: eight scenes, pruning off/on, each in latency-only and phase-timing modes. Each report
contains 100 measured calls after ten warmups, with 16,384 rollouts, 80 steps, ten iterations,
seed 1, and the split kernel on the RTX 4090 / CUDA 12.8. The active projection has 80 points in
the normal straight scenes, 256 in the hairpin/update/overlap scenes, and nine in the short scene.
The recorded profiles, hardware, compiler flags, revision strings, and workload metadata match
within every off/on pair. All reports enable the required precision contract and contain valid
bounds; the pruning switch is correctly recorded. The package revision is
`x2/v4.4.0-552-gafab67af33-dirty`, so that string alone does not identify the modified source snapshot.

The table compares the [exhaustive latency reports](../../../../../mppi_benchmark_results/pruning_0_phase_0/)
with the [pruned latency reports](../../../../../mppi_benchmark_results/pruning_1_phase_0/), and separately
the [exhaustive phase reports](../../../../../mppi_benchmark_results/pruning_0_phase_1/) with the
[pruned phase reports](../../../../../mppi_benchmark_results/pruning_1_phase_1/). All times are per
complete ten-iteration call. Wall columns are latency-only statistics; cost GPU is the mean of
the instrumented `MPPI/rollout_cost` phase, not an Nsight measurement.

| Scene                         | Wall median off → on (ms) | Median reduction | Wall p95 off → on (ms) | Cost GPU mean off → on (ms) |
| ----------------------------- | ------------------------: | ---------------: | ---------------------: | --------------------------: |
| FreeSpaceCached               |           20.244 → 18.271 |             9.7% |        20.946 → 18.473 |               6.160 → 4.098 |
| Hairpin256Cached              |           28.495 → 19.129 |            32.9% |        28.623 → 19.283 |              14.286 → 4.762 |
| StaticObstacles64Cached       |           24.709 → 22.746 |             7.9% |        24.935 → 22.925 |              10.235 → 8.117 |
| StaticObstacleGeometryUpdates |           22.054 → 20.318 |             7.9% |        22.161 → 20.487 |               7.464 → 5.611 |
| GridBoundaryCrossings         |           26.533 → 24.491 |             7.7% |        26.653 → 24.757 |              11.496 → 9.369 |
| ReferenceGeometryUpdates      |           28.529 → 19.151 |            32.9% |        28.856 → 19.396 |              14.281 → 4.722 |
| ShortProjectionPath           |           16.773 → 16.948 |            −1.0% |        16.979 → 17.143 |               3.031 → 3.041 |
| OverlappingProjectionGroups   |           30.281 → 30.248 |             0.1% |        30.531 → 30.390 |             16.194 → 15.979 |

The hairpin saves an observed 9.366 ms in median complete-call latency and 9.524 ms (66.7%) in
mean cost GPU time. The changing-reference scene retains the same approximate benefit, despite
rebuilding both bounds arrays every call. Its instrumented host bounds-preparation means with
pruning enabled are 0.002725 ms for the reference and 0.007586 ms for the corridor, totaling about
10.3 microseconds; runtime-upload GPU mean is 0.008095 ms. Cached scenes have no measured bounds
rebuild ranges. Across scenes, dynamics remains approximately 6.56–6.69 ms. Process CPU means also
fall (hairpin 28.531 → 19.123 ms), but this does not establish a faster CPU projection algorithm:
the CPU projector is unchanged and process CPU measurements include driver/wait work.

The short path uses exhaustive fallback in both modes, so no projection acceleration is expected.
Its 0.175 ms median increase and the overlap scene's 0.033 ms decrease need repeated processes
before being treated as a regression or a gain. The short scene also has small numerical differences
between runs even though no groups are pruned.

The separate synthetic work diagnostic reports 1,060,800 possible segment evaluations across
4,160 query/hint combinations on a 256-point straight path:

| Group size | Groups examined | Groups pruned | Segments evaluated | Segment reduction |
| ---------: | --------------: | ------------: | -----------------: | ----------------: |
|          4 |         266,240 |       197,280 |            277,936 |             73.8% |
|          8 |         133,120 |        95,824 |            300,400 |             71.7% |
|         16 |          66,560 |        45,096 |            345,069 |             67.5% |

All three sizes have zero exhaustive fallbacks in that diagnostic. For fully overlapping segments,
all three prune zero groups and evaluate all 66,300 possible candidates. This confirms both useful
rejection and the expected worst case on these inputs. Four-segment groups evaluate fewer segments
but twice as many boxes as eight-segment groups; counts alone do not establish a faster group size.
Keep eight as the measured production configuration pending kernel-time comparisons.

The tests and successful validation outcomes do not prove bitwise equality of complete optimizer
output across adaptive cycles. Most per-cycle differences are small. The largest exception is the
instrumented grid scene: pruning off/on differs in eligible-rollout count in 93/100 measured calls
(maximum absolute difference 231), and baseline cost in 91 calls (maximum difference 51.925).
However, the two latency-only grid runs match all recorded baseline-cost, eligibility and temperature
values, and the pruned instrumented run matches those values too. The exhaustive instrumented run
therefore also differs from the exhaustive latency-only run. These reports do not isolate pruning
as the cause, and they do not establish the source of the run-to-run variability. Rejection,
validation, output-point count, and iteration count match in every paired scene.

This is one ordered pass (off then on) per timing mode. GPU clocks/power conditions were not recorded,
and independent reverse-order repetitions have not been supplied. The same-binary exhaustive switch
retains the new runtime layout (83,872 bytes) and bounds preparation, so it isolates search savings.
A matched comparison against baseline Universe revision `afab67af33` is still needed to include
their net overhead and compiler effects. The results measure GPU elapsed work, not power or
whole-system GPU utilization.

This first comparison called for reversed-order repetitions to assess the short-path result and grid
numerical variability, followed by fresh profiling of the pruned kernels before choosing another
optimization. In the instrumented hairpin, dynamics is now 6.592 ms versus 4.762 ms
for cost, with random sampling at 3.619 ms: the previous projection-dominated profile no longer
describes the measured phase balance. The repeated runs and fresh profiles are assessed below.

#### Balanced repetitions and pruned-kernel profiles: 2026-11-06

The user ran the manual follow-up runner on November 6 JST (the directory timestamps use November 5
UTC). All three collections completed:

- [Repeated comparison](../../../../../mppi_benchmark_results/projection_repeat_20261105_201641_EMP3VC/):
  128 valid reports, eight scenes × two pruning settings × two timing modes × four process pairs.
- [Nsight Systems](../../../../../mppi_benchmark_results/projection_nsys_20261105_202220_HH0bQ7/):
  three timelines, SQLite exports and valid benchmark reports, one per representative scene.
- [Nsight Compute](../../../../../mppi_benchmark_results/projection_ncu_20261105_202229_UzNsY7/):
  six reports, dynamics/cost separately for each scene, with exactly three matching kernel launches
  and the expected NVTX stack in each. All six companion benchmark reports are valid.

There are no unexpected optimizer outcomes, incomplete workloads, failed cases or skipped cases.
The repeated runs have the expected measured cycle indices 10–109 and ten launch/event pairs per
instrumented iterative phase, including the separately recorded first call. Profiles, hardware,
build metadata, seed, dimensions and split-kernel selection match throughout the repeated matrix.
The only collection warning is Nsight Compute's expected warning about unlocked GPU clocks; its
PM-sampling sections report zero dropped samples. Profiler wall/CPU measurements are excluded from
the latency comparison.

All three manifests record executable SHA-256
`28a5b671995bc30fe2dab755382592e9a911ce3d4c55a7e4e443c6ec2e7b3ccb`, Universe checkout
`df828f42a6`, and vendor `56e4b48`. The benchmark JSON still records the earlier configure-time
Universe revision `x2/v4.4.0-552-gafab67af33-dirty`. This is a matched pruning-switch experiment,
not a verified rebuild-to-checkout comparison; linked library paths are recorded but their contents
are not hashed. Refresh configure-time provenance before a later cross-revision comparison.

Before/after snapshots show a 450 W power limit, temperatures spanning 45–65 °C, and SM clocks
of 210 MHz initially idle and 2520 MHz at the other sampled boundaries. Power-capping counters
remain zero and thermal slowdown is inactive in the snapshots. Desktop graphics processes remain
resident. These are boundary snapshots, not proof of constant conditions during every call or an
energy/utilization measurement. The environment note contains the runner label but no user-supplied
description of GPU conditions.

**Repeated timing results.** Wall values below are the arithmetic mean of the four process-level
medians, not a median pooled across 400 calls. The range is the four paired percentage reductions,
not a confidence interval. Cost values average the four instrumented phase means. All times are
milliseconds per complete ten-iteration call; timing modes remain separate.

| Scene                         | Wall median off → on (ms) | Reduction | Paired reduction range | Cost GPU mean off → on (ms) |
| ----------------------------- | ------------------------: | --------: | ---------------------: | --------------------------: |
| FreeSpaceCached               |           20.078 → 18.085 |     9.92% |        9.03% to 10.58% |               6.170 → 4.097 |
| Hairpin256Cached              |           28.555 → 18.995 |    33.48% |       33.34% to 33.82% |              14.243 → 4.754 |
| StaticObstacles64Cached       |           24.686 → 22.596 |     8.47% |         8.12% to 8.92% |              10.240 → 8.123 |
| StaticObstacleGeometryUpdates |           22.033 → 20.154 |     8.53% |         8.25% to 9.04% |               7.483 → 5.621 |
| GridBoundaryCrossings         |           26.521 → 24.334 |     8.25% |         8.06% to 8.58% |              11.526 → 9.359 |
| ReferenceGeometryUpdates      |           28.556 → 19.002 |    33.46% |       33.15% to 33.87% |              14.242 → 4.720 |
| ShortProjectionPath           |           16.808 → 16.816 |    −0.05% |        −0.76% to 0.56% |               3.049 → 3.042 |
| OverlappingProjectionGroups   |           30.267 → 30.095 |     0.57% |         0.14% to 0.98% |             16.176 → 15.995 |

The main gains survive both run orders. Hairpin median latency falls by 9.560 ms and cost GPU mean
by 9.489 ms (66.6%). The mean of its four process-level p95s falls from 28.725 to 19.176 ms;
process CPU mean falls from 28.595 to 19.033 ms. The changing-reference scene retains the gain,
with approximately 10.35 microseconds of host bounds preparation per call. This still isolates
search savings: both switch settings retain the bounds preparation and expanded runtime upload.

The earlier 1% short-path slowdown is not reproduced as a consistent pruning effect. Its mean
median difference is only +0.008 ms. Pruning-on is slower by 0.76%/0.42% in pairs 1/3, when it runs
first, and faster by 0.41%/0.56% in pairs 2/4, when it runs second. Both settings use exhaustive
fallback, and the direction follows order. Treat this as near-neutral within the observed
variation. The overlapping scene has a small positive median change in all four pairs, but its
p95 change crosses zero; it remains a weak-benefit workload rather than a substantial speedup.

**Numerical reproducibility remains open.** The grid scene has four exact off/on matches and four
nonmatching pairs across the two modes. The largest paired differences are 55.686 in baseline
cost, 228 eligible rollouts, and 0.000532 in temperature. Eleven of the sixteen runs share exactly
the same sequence of baseline cost, eligibility, `lambda_used` and `lambda_next`; the five others
include both pruning settings. Differences also recur with the setting and timing mode unchanged:
for example, exhaustive latency pairs 1 and 2 differ in 29 baseline-cost samples, by up to 55.304,
and in 28 eligibility samples, by up to 124 rollouts. All sixteen first-call values of these four fields match;
some later divergence is already present at the first stored measured cycle, after warmup.

This is stronger evidence that the variability is not exclusive to pruning or phase timing, but
it does not establish its cause or rule out an additional pruning-sensitive effect. Successful
validation and earlier exact projection tests do not prove bitwise complete-optimizer parity.
Retain this as a separate diagnostic issue and locate the first differing intermediate result
under identical inputs before attributing it to floating-point reduction order or a race.

**Steady GPU phase balance.** The saved Systems SQLite data was read without launching a target.
CUDA runtime launches were correlated with their enclosing phase NVTX ranges, excluding the first
100 ranges per phase (ten warmup calls). The remaining 200 ranges cover 20 measured calls. Values
are sums of actual kernel durations per call; inclusive CPU NVTX durations are not GPU durations.

| Scene                   | Dynamics |  Cost | Sampling | Weight statistics | Control reduction |
| ----------------------- | -------: | ----: | -------: | ----------------: | ----------------: |
| FreeSpaceCached         |    6.730 | 4.106 |    3.616 |             1.316 |             0.224 |
| Hairpin256Cached        |    6.626 | 4.763 |    3.602 |             1.293 |             0.366 |
| StaticObstacles64Cached |    6.691 | 8.134 |    3.600 |             1.313 |             0.221 |

From the first sampling kernel through the final control-reduction kernel in each measured call,
the mean span not covered by kernels is only 0.062–0.066 ms (less than 0.4% of that span). This does
not include host preparation/validation outside the iteration burst, but it makes CUDA Graphs or
launch-overhead work a lower priority for this workload. Sampling remains meaningful: approximately
1.62 ms is frequency-noise configuration, 0.75 ms rearrangement, 0.63 ms random generation,
0.44 ms control setup and 0.16 ms inverse FFT per call.

**Dynamics is the next common optimization target.** Compute profiles report 0.689–0.700 ms per
individual dynamics launch across the three scenes. Mean L2 throughput is 87–89% of the profiler's
peak metric, while DRAM throughput is about 9% and achieved occupancy about 16.5%. The launch uses
512 blocks of `(32, 2, 1)`, 61 registers per thread, and 12.40 kB dynamic shared memory per block.
Shared memory limits theoretical occupancy to 29.17%; the grid has only four blocks per SM on
average. This is evidence of memory traffic and limited latency hiding, not a reason to increase
occupancy without measuring the tradeoffs.

Reading the saved `.ncu-rep` files through NVIDIA's report API provides two concrete source targets:

1. All recorded dynamics local-load instructions map to the indexed `local_der[i]` read in
   [computeDynamics](include/mppi/dynamics/dubins/first_order_dubins_bicycle.cu).
   Across three launches per scene, there are 3,317,760 executed local-load instructions and
   6,144,000 local-store instructions. The read line accounts for about 14.6–15.2% of PC samples;
   64% of the local-store instructions initialize zero delay-tap derivatives. Captured SASS contains
   actual `LDL`/`STL` accesses, so this is not an inference from the reported stack size alone.
   The dynamically indexed temporary array is a focused first candidate for eliminating local-memory
   materialization while retaining arithmetic and disjoint writes by Y workers. NVIDIA documents
   nonconstant array indexing as one reason for local-memory allocation in the
   [CUDA 12.8 programming guide](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-c-programming-guide/index.html#device-memory-accesses).
2. The global-access metric reports 16,875,520 excessive sectors per dynamics launch. Source
   correlation assigns 88.35% to the scalar output store in `parallel_utils.cuh:219`, called when
   [rolloutDynamicsKernel](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L412)
   writes `y_d`. Sampled-control reads/writes split the remaining 11.65%. The rollout-major
   output layout separates neighboring rollout threads' destinations by an entire trajectory;
   cooperative stores or a tiled layout are candidates after the smaller derivative change.
   Any layout change must include cost-kernel consumers and be judged on combined dynamics/cost time.

PC-sample shares and Nsight's advisory speedup estimates are not elapsed-time savings predictions.
Keep the existing barriers and control/delay/safety semantics when pursuing either candidate;
the current Y workers exchange state and controls through shared memory. In particular, simply
removing barriers or increasing the register cap is not supported by these findings.

Cost remains larger than dynamics with 64 obstacles. Its sampled source includes repeated obstacle
yaw trigonometry and exact oriented-box safety checks; that is a later scene-specific target.
The hairpin cost profile still attributes 56.2% of executed instructions and 50.2% of PC samples
to `path_tracking_geometry.cuh`, with directed-rounding bound calculations prominent. Further
projection changes remain possible, but the new common dynamics opportunity should be measured
first. Use the latency-only repeats for speedup claims and keep the unresolved numerical diagnostic
separate from any claim of complete-optimizer equivalence.

The assistant analyzed existing JSON, text, SQLite and Compute reports only, and updated this review.
No build, test, benchmark or profiling capture was run, and no commit was made.

#### Scalar derivative storage implementation

The implementation replaces the dynamically indexed 27-element `local_der` array with six
named scalar values. The original expressions, float intermediates, delayed-control selection and
steering clamp feed a shared inline host/device helper. Device workers retain their original
`threadIdx.y`/`blockDim.y` ownership loop; a component selector supplies the six continuous
derivatives and positive zero for the 21 discrete derivatives. State integration, delay advancement,
output handling, all existing barriers, launch dimensions and compiler flags are unchanged.

Three new GTests cover original/scalar host bits, original/scalar device bits and exclusive write
ownership, and 80-step state/derivative/output/control replay. The original arithmetic and temporary
array are retained in a test-only reference model. Fixed controls isolate this change from adaptive
optimizer variability. Tests exercise seven parameter configurations and six block shapes, including
inactive Y workers and independent X/Z rollouts. Their dedicated `test_dynamics_derivative`
executable has separate host/GPU correctness registrations; performance measurement remains opt-in.

Before the user's rebuild, the executable hash still matched the recorded baseline, and the
optimizer library's SHA-256 was read as
`bc1306b9ada06478e7fdb572d0da9e36d2bec1bdb7fcb370fc3ae83bcb773fa1`. This supplements, but does not
retroactively prove, the library identity during the earlier recordings. Future collection manifests
also hash the library resolved by the loader. Reconfigure when rebuilding to refresh source metadata.

The user's first build compiled the new CUDA source but failed while linking `test_costs`: vendor
headers emit non-inline `RiskMeasure::var()` and `cvar()` definitions in both CUDA translation units.
The derivative tests now build as a separate executable, restoring `test_costs` to one CUDA source.
The subsequent user-run build, correctness results and profiling are assessed below. The assistant
ran no build, numerical test, sanitizer, benchmark or profiling capture. See the
[validation and measurement commands](test/PERFORMANCE_BENCHMARKS.md#scalar-dynamics-derivative-validation).

#### Measured scalar derivative follow-up: 2026-11-06

**Conclusion:** the targeted local-memory accesses are eliminated, and the isolated exact-parity
tests pass. Dynamics runs faster in the new captures. Complete-call latency has not improved
consistently, and the collection conditions differ enough that an isolated implementation speedup
cannot yet be assigned. Keep the observed latency regressions visible and perform a controlled
old/new comparison before proceeding to another optimization.

This conclusion describes the first cross-environment comparison. The later
[reverted-build comparison](#reverted-build-comparison-under-similar-conditions-2026-11-06) below
uses a closer baseline and supports retaining the optimization, with modest and scene-dependent
complete-call improvements.

The new collections are under the workspace's `mppi_benchmark_results/`:

- `projection_repeat_20261106_012040_uN9sii`: 128 JSON reports, covering four independent runs per
  scene, pruning setting and timing mode, with 100 measured calls after ten warmup calls.
- `projection_nsys_20261106_012626_K19ygV`: three Systems reports and their SQLite exports.
- `projection_ncu_20261106_012635_cF7Ijs`: six Compute reports, three sampled launches each,
  covering dynamics and cost in free space, hairpin and 64-obstacle scenes.

Comparison uses the previous `projection_repeat_20261105_201641_EMP3VC`,
`projection_nsys_20261105_202220_HH0bQ7` and `projection_ncu_20261105_202229_UzNsY7` collections.
All 137 new companion JSON reports are valid, have zero unexpected outcomes and complete their
requested workloads. Recorded dimensions, seeds, resolved profiles, hardware, compiler flags and
kernel selection match the earlier collection. All three runner logs finish successfully. Compute
records no dropped PM samples or PC-sampling bytes; its unlocked-frequency warning remains relevant.

All three new manifests agree on executable SHA-256
`5280729e6a47db8d3f2b263fffed8fbfc95ecdc606306f572b83e4db5d71a721` and resolved optimizer-library
SHA-256 `4d8bf9c80d07380ca8cc33ea505af9580e8b1d5d6cef77e4d8593b53f530d488`. These also matched the
files in the build directory when that collection was analyzed. Configure-time Universe metadata was
`x2/v4.4.0-554-gdf828f42a6-dirty`; vendor remains `56e4b48`. Imported dynamics source in the new
Compute report matches the current scalar implementation, while the old report contains the array.
The old manifests did not hash the library; their imported source and the previously recorded
pre-rebuild hash supplement that provenance limitation.

**Correctness evidence.** The saved CTest/GTest results from 10:20 JST on November 6 report all
three new tests passing: host exact bits, device exact bits/write ownership, and the 80-step device
replay. They ran without skips. The same collection records 37 host cost tests and 12 GPU cost tests
passing. These are user-run results in `build/autoware_mppi_optimizer/test_results/`; no new tests
were executed during analysis. No saved Compute Sanitizer result was found in the supplied captures.

**The local-memory target is achieved for the sampled production kernel.** Every one of the nine
new dynamics launches has zero executed local loads/stores and zero L1 local-load/store sectors.
The captured dynamics SASS has no `LDL`/`STL` instructions. Before the change, each launch executed
1,105,920 local-load and 2,048,000 local-store instructions. There is no observed replacement spill
traffic. Registers remain 61 per thread, dynamic shared memory remains 12,400 bytes per block,
and the launch remains 512 blocks of `(32, 2, 1)`. Achieved occupancy remains approximately 16–17%.
The reported 1,024-byte stack setting persists; it does not imply executed local-memory traffic.

Compute means below average three individual launches per scene; these are profiled kernel times,
not unprofiled complete-call latency:

| Scene                   | Dynamics before → after (ms/launch) | Observed time reduction | SM elapsed-cycle reduction | L2 sector reduction |
| ----------------------- | ----------------------------------: | ----------------------: | -------------------------: | ------------------: |
| FreeSpaceCached         |                       0.689 → 0.541 |                  21.42% |                     15.55% |              35.13% |
| Hairpin256Cached        |                       0.700 → 0.545 |                  22.19% |                     16.46% |              35.27% |
| StaticObstacles64Cached |                       0.698 → 0.557 |                  20.22% |                     14.35% |              35.13% |

SM frequency is approximately 2.52 GHz before and 2.70–2.71 GHz after. Thus the 20–22% time reduction
cannot all be credited to the refactor; the lower elapsed-cycle count is additional evidence, not
a fully controlled estimate. L2 sectors fall from approximately 30.3 million to 19.6–19.7 million
per launch. Total executed instructions increase by 9.8–10.6%, while the sampled DRAM read/write
byte totals increase. The result is specifically an elimination of local staging and a reduction
in observed L2 traffic, not a reduction in every instruction or memory metric. Cost-kernel local
accesses remain outside this change's scope.

**Complete-call results do not yet meet the performance acceptance target.** The following table
holds projection pruning enabled in both builds. Wall statistics are the arithmetic means of four
process-level medians/p95s from latency-only runs; dynamics values average the four instrumented
phase means. All values are milliseconds per ten-iteration call. A positive wall change is slower.

| Scene                         | Wall median before → after | Wall change | Wall p95 before → after | Dynamics GPU mean before → after |
| ----------------------------- | -------------------------: | ----------: | ----------------------: | -------------------------------: |
| FreeSpaceCached               |            18.085 → 18.065 |      −0.11% |         18.397 → 19.524 |                    6.650 → 5.937 |
| Hairpin256Cached              |            18.995 → 19.374 |      +2.00% |         19.176 → 20.190 |                    6.636 → 6.008 |
| StaticObstacles64Cached       |            22.596 → 23.163 |      +2.51% |         22.797 → 24.126 |                    6.608 → 5.916 |
| StaticObstacleGeometryUpdates |            20.154 → 20.623 |      +2.33% |         20.423 → 21.479 |                    6.609 → 6.019 |
| GridBoundaryCrossings         |            24.334 → 24.852 |      +2.13% |         24.534 → 26.038 |                    6.607 → 5.824 |
| ReferenceGeometryUpdates      |            19.002 → 19.389 |      +2.03% |         19.237 → 20.464 |                    6.651 → 6.028 |
| ShortProjectionPath           |            16.816 → 16.728 |      −0.52% |         16.956 → 18.116 |                    6.644 → 5.854 |
| OverlappingProjectionGroups   |            30.095 → 31.183 |      +3.61% |         30.384 → 32.587 |                    6.633 → 6.020 |

Dynamics phase time decreases by 8.9–11.9%, or 0.59–0.79 ms per call. Six complete-call medians
increase by 2.0–3.6%; p95 increases by 5.2–7.3% in all eight scenes. Process CPU means also increase
by 0.5–3.8%. The pruning-disabled comparisons likewise fail to show a general complete-call gain.
The runner's off/on pairs still toggle projection pruning; both settings use scalar derivatives in
the new build. They are not a contemporaneous old/new derivative comparison.

Other phases consume the dynamics savings. For the hairpin, cost GPU mean increases from 4.754 to
5.021 ms and sampling from 3.626 to 3.960 ms; host reference preparation increases from 0.339 to
0.456 ms. Across all eight pruned scenes, sampling rises by about 3.2–15.5%, and cost rises in seven
scenes. The captured cost-kernel SASS is identical before/after after mapping relocated code
addresses to instruction indices, including its branch targets; its sampled executed-instruction
counts also match. This excludes a change to that captured instruction stream as the explanation,
but does not exclude changed cache behavior or interference during execution.

**The environments differ.** Earlier snapshots show desktop/compositor processes; the new ones
also include VS Code, Slack, FreeTube and Firefox GPU processes. This establishes additional
resident GPU clients, not how much concurrent work each performed. Current-clock snapshots change
from predominantly 2520 to 2700 MHz. Both collections retain a 450 W limit, inactive thermal
slowdown and zero power-capping counters. Mean within-process wall standard deviation across the
32 pruned latency-only reports increases from 0.157 to 0.802 ms. These observations support
investigating interference and clock conditions; they do not prove those explain every regression.

**Systems corroborates a dynamics reduction.** Correlating runtime launches with NVTX ranges and
discarding the first 100 ranges per iterative phase gives these actual kernel-duration sums per
measured call:

| Scene                   | Dynamics before → after | Cost before → after | Sampling before → after | Uncovered burst span before → after |
| ----------------------- | ----------------------: | ------------------: | ----------------------: | ----------------------------------: |
| FreeSpaceCached         |           6.730 → 5.938 |       4.106 → 4.338 |           3.616 → 3.629 |                       0.063 → 0.061 |
| Hairpin256Cached        |           6.626 → 6.079 |       4.763 → 5.198 |           3.602 → 3.770 |                       0.066 → 0.064 |
| StaticObstacles64Cached |           6.691 → 5.732 |       8.134 → 8.169 |           3.600 → 3.884 |                       0.062 → 0.098 |

Mean uncovered time remains below 0.6% of the iteration burst; the 64-obstacle trace has one call
with 0.516 ms uncovered. Launch-overhead optimization remains a lower priority than kernel work
or explaining complete-call regressions. The output-store opportunity is unchanged: 16,875,520
excessive global sectors per dynamics launch, with 88.35% attributed to `parallel_utils.cuh:219`.

**Numerical variability remains a separate issue.** All workload/outcome checks pass, and the new
isolated derivative comparisons pass exactly. Complete optimizer diagnostic sequences still vary
between repeated identical settings in both builds. Across matched scene/mode/pruning/run indices,
the grid scene differs by up to 56.277 in baseline cost, 231 eligible rollouts and 0.000532 in
temperature. These differences neither establish a derivative regression nor prove complete-output
equivalence. Keep the existing reproducibility investigation open.

**Next action:** repeat old/new binaries in alternating order under the same GPU clock policy and
comparable background load, preserving the resolved-library hashes. Use latency-only measurements
for complete-call acceptance and phase timing for attribution. Confirm sanitizer results separately.
If cost/sampling regressions persist under those conditions, investigate cache effects and the first
diverging workload/intermediate before another production optimization. Once end-to-end acceptance
is established, cooperative output stores remain the next common dynamics candidate; cost-kernel
consumers and their combined time must be included in that evaluation.

This follow-up read saved JSON, text, SQLite, GTest and Compute reports. It launched no optimizer,
test, benchmark or profiler target and made no commit.

#### Reverted-build comparison under similar conditions: 2026-11-06

**Conclusion:** this closer comparison supports retaining the scalar derivative implementation.
The dynamics improvement persists at essentially the same measured clocks. With projection pruning
enabled, all eight complete-call means improve, six medians improve, and seven p95s improve. The
complete-call gains are modest in most scenes and do not establish a universal latency improvement.

The user collected a rebuilt array implementation in these workspace result directories:

- `projection_repeat_20261106_025246_ORKWXh`
- `projection_nsys_20261106_025837_0cmWfr`
- `projection_ncu_20261106_025847_c8qw8I`

All 137 companion reports pass the workload/validity checks, with zero unexpected outcomes and
successful completion of the three collection scripts. The comparison below is **reverted array
→ optimized scalar**, using the earlier scalar captures `projection_*_20261106_012*`. This order
describes implementations, not collection chronology. Dimensions, seed, resolved profiles,
hardware and compiler flags agree. There are still four independent processes per scene/setting,
not alternating array/scalar pairs within one collection.

**Binary identity is verified, despite a source-display mismatch.** All three reverted manifests
record executable SHA-256 `3a0bf4a807aa62ce1e158d890e3f2fe5d8b04489629a123c7d0b6578cf758da7` and
resolved optimizer-library SHA-256
`bfb358bbd42266fd34149dbabd8ddab6633ceabed7c29029b2793568fa0d2433`. The library resolves from
`install/autoware_mppi_optimizer/lib/`; the build-directory copy has the same hash. Configure-time
metadata records revert commit `33da5a786d`, whose package content matches the original array
baseline `df828f42a6`. The checkout manifest instead records restored scalar commit `3109148c1f`.

Consequently, the reverted Compute reports embed the scalar `.cu` text even though their machine
code executes the array implementation. After normalizing relocated code addresses, the captured
dynamics instruction stream matches the original array baseline in all three scenes. Its local
loads/stores also return to the original counts. The captured cost instruction stream matches all
three collections. These checks establish which code ran; the reverted report's imported dynamics
source and source-line attribution must not be used as authoritative descriptions of that binary.
For future captures, preserve the source matching the compiled library through source import.

**Conditions are substantially closer.** Both collections' loaded-clock snapshots are predominantly
2700 MHz, and both list the same desktop clients, including VS Code, Slack, Firefox and FreeTube.
The Compute dynamics means differ in measured SM frequency by at most 0.11%, around 2.70–2.71 GHz.
Power limits remain 450 W with no reported thermal slowdown or power-capping activity. Clock locking
was not enabled, and resident-client lists do not measure simultaneous client activity. Mean
within-process wall standard deviation is 0.849 ms for the reverted collection and 0.802 ms for the
scalar collection, so fine distinctions in complete-call latency remain uncertain.

**Dynamics improvement is confirmed at comparable clocks.** Compute averages three launches per
scene:

| Scene                   | Array → scalar dynamics (ms/launch) | Reduction | Array → scalar SM clock (GHz) |
| ----------------------- | ----------------------------------: | --------: | ----------------------------: |
| FreeSpaceCached         |                       0.669 → 0.541 |    19.14% |               2.7067 → 2.7063 |
| Hairpin256Cached        |                       0.653 → 0.545 |    16.62% |               2.7065 → 2.7037 |
| StaticObstacles64Cached |                       0.684 → 0.557 |    18.56% |               2.7060 → 2.7035 |

Elapsed SM cycles decrease by 16.7–19.2%. Every array launch executes 1,105,920 local loads and
2,048,000 local stores; the scalar launches execute zero, without replacement local traffic.
Registers remain 61 per thread, launch shape and shared memory are unchanged, and L2 sectors fall
by 35.3–35.4%. The instruction-count increase of approximately 10% remains; the measured benefit
comes with a different instruction/memory tradeoff. DRAM byte counters still vary and should not
be interpreted as a demonstrated energy or overall bandwidth reduction. Isolated cost-kernel
Compute times differ by less than 0.9%, with identical captured instructions and instruction counts.

**Complete-call improvements are smaller.** Pruning is enabled on both sides. As in the earlier
tables, wall values average four process-level medians/p95s from latency-only runs; dynamics values
average four instrumented phase means. All times are milliseconds per ten-iteration call. A positive
median reduction means the scalar implementation is faster.

| Scene                         | Wall median array → scalar | Median reduction | Wall p95 array → scalar | Dynamics GPU mean array → scalar |
| ----------------------------- | -------------------------: | ---------------: | ----------------------: | -------------------------------: |
| FreeSpaceCached               |            18.400 → 18.065 |            1.82% |         19.497 → 19.524 |                    6.926 → 5.937 |
| Hairpin256Cached              |            19.424 → 19.374 |            0.26% |         20.516 → 20.190 |                    7.131 → 6.008 |
| StaticObstacles64Cached       |            23.415 → 23.163 |            1.08% |         24.684 → 24.126 |                    7.293 → 5.916 |
| StaticObstacleGeometryUpdates |            20.563 → 20.623 |           −0.29% |         22.387 → 21.479 |                    7.138 → 6.019 |
| GridBoundaryCrossings         |            25.500 → 24.852 |            2.54% |         26.989 → 26.038 |                    6.679 → 5.824 |
| ReferenceGeometryUpdates      |            19.290 → 19.389 |           −0.51% |         20.476 → 20.464 |                    7.124 → 6.028 |
| ShortProjectionPath           |            17.831 → 16.728 |            6.18% |         18.900 → 18.116 |                    7.078 → 5.854 |
| OverlappingProjectionGroups   |            31.521 → 31.183 |            1.07% |         33.207 → 32.587 |                    7.123 → 6.020 |

The dynamics phase improves by 12.8–18.9%, saving 0.86–1.38 ms per call. All four scalar dynamics
process means are below all four array process means within every pruned scene. Complete-call
means improve by 0.8–5.0%, and process CPU means by 0.5–4.8%. Most median differences are much
smaller than the spread between processes; the short-path median is lower in all scalar processes
than in any array process. The two slightly worse medians and the free-space p95 increase of
0.14% should remain visible rather than being described as proven improvements.

Pruning-disabled controls remain mixed: five medians improve, while grid crossings, reference
updates and overlapping groups worsen by 1.67%, 5.82% and 1.98%, respectively. The latter two also
have higher p95s. Therefore this comparison supports a common dynamics gain, not a claim that every
configuration has lower complete-call latency.

Other phases still absorb part of the savings. Pruned cost phase means are 4.9–8.3% higher with
scalar derivatives in six scenes, while grid and short-path cost means decrease. Sampling means
also vary. For example, the hairpin saves 1.123 ms in dynamics, while cost increases by 0.235 ms
and sampling by 0.280 ms. The isolated cost kernel's near-equal timings and unchanged instructions
make a regression in its compiled arithmetic unlikely; these observations do not distinguish
inter-kernel cache effects from remaining scheduling/background interference.

Systems independently reports array → scalar dynamics sums of 7.056 → 5.938 ms in free space,
6.997 → 6.079 ms in the hairpin, and 7.305 → 5.732 ms with 64 obstacles. The corresponding
sampling-through-reduction GPU burst spans decrease from 16.436 → 15.614, 17.230 → 16.714 and
20.727 → 19.488 ms. The measured kernel improvement is therefore also present outside Compute
replay. The existing output-store inefficiency remains a candidate for joint dynamics/cost work.

The previously passing isolated scalar parity tests remain the correctness evidence for the
refactor. Full-optimizer numerical variability recurs in the reverted build too: repeated grid
runs with identical pruning/timing settings differ by up to 244 eligible rollouts. Reverting this
optimization does not remove that issue, and these diagnostic fields do not prove full-output
equivalence or identify its cause.

**Recommendation:** retain the scalar derivative implementation as a validated memory-access
optimization with a clear dynamics benefit. Describe its complete-call benefit as modest and
scene-dependent. For the next investigation, examine the dynamics-output handoff and cost-cache
behavior together, retaining separate kernel and complete-call measurements; continue the numerical
reproducibility investigation independently. No production code was changed during this analysis.

This comparison read saved reports and repository history only. No build, test, benchmark or
profiling target was run, and no commit was made.

#### Dynamics-output handoff and cost-cache investigation: 2026-11-06

**Finding:** output-store coalescing is a confirmed remaining dynamics opportunity. The cost
kernel already uses coalesced loads for that buffer. The supplied captures do not establish whether
cache residency explains the higher cost phase after the scalar derivative change.

This investigation reads the same matched-clock array/scalar Compute and Systems captures listed
above. It does not launch the benchmark or either profiler. Source references below are to the
unchanged vendor implementation; the reverted report's imported dynamics-model source remains
unsuitable for attribution, as explained above.

**The handoff is direct and has negligible idle time in these traces.**
[The split launcher](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L1625)
submits dynamics and then cost on the same CUDA stream. Across all 1,200 measured transitions
(200 per scene per implementation), Systems shows adjacent dynamics/cost kernels on stream 7,
with no intervening captured kernel, memcpy or memset. Mean end-to-start gaps are 0.68–0.73 µs;
the largest observed gap is 1.57 µs. Roughly 7 µs of handoff gap per ten-iteration call cannot
explain cost-phase differences of tenths of a millisecond. These traces cover this process, not
all activity from other GPU clients.

**Producer and consumer have different access patterns.** The global buffer is
`[distribution][rollout][time][13 output components]`:

- In [dynamics](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L408),
  X lanes own different rollouts and the two Y workers divide the components. Since 13 is odd,
  `loadArrayParallel<13>` uses scalar copies. Adjacent active warp lanes store the same component
  80 × 13 × 4 = **4,160 bytes apart**. A full warp therefore touches 32 separate 32-byte sectors
  for 128 useful bytes, compared with four sectors for ideal contiguous addresses.
- In [cost](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L262),
  one block cooperatively loads an entire rollout's 80 × 13 = 1,040 floats. The default
  `COALESCE=true` path is active. This whole slice and its offsets satisfy the helper's float4
  conditions even though the per-timestep component count is odd. Captured instructions include
  `LDG.E.128`, and each thread then consumes its timestep from shared memory.

Compute's source-correlated theoretical-sector metrics agree with the address analysis. Per
dynamics launch, the output `STG.E` instructions account for **17,039,360 global sectors**, of
which **14,909,440 are excessive** relative to the metric's ideal access pattern. That is **88.35%**
of all 16,875,520 excessive global sectors in dynamics; the remaining two contributions are the
sampled-control read and write, 983,040 each. These counts are unchanged by scalar derivatives.
The cost buffer load accounts for **2,129,920 sectors with zero excessive sectors**, matching
65 MiB of useful output data. Multiplying the producer's sector requests by 32 gives 520 MiB;
that is request volume, **not 520 MiB of DRAM writes**. Cache hits, repeated sectors and write
combining must not be discarded when interpreting this counter. This is a locality opportunity,
not a prediction of an eightfold kernel speedup.

**Capacity makes cache reuse worth measuring, but does not prove eviction.** The output allocation
is 16,384 × 80 × 13 × 4 = **65 MiB**, and the sampled-control allocation adds **10 MiB**. The
device report records a **72 MiB L2**. Other sampler/cost data also compete for cache, and only
part of each allocation may be resident at once. Removing derivative local accesses changes
traffic and execution timing; its effect on the surviving output cache lines cannot be inferred
from capacity or kernel duration alone.

The existing Compute collections use `--replay-mode kernel --cache-control all`, which flushes
caches before replay. They measure cost after destroying the natural producer-to-consumer cache
state. Kernel replay also cannot restore that state from its memory backup. NVIDIA recommends
application replay when preceding GPU work must recreate cache contents; changing only kernel
replay to `--cache-control none` can instead let the target warm its own cache on later passes.
See [cache control](https://docs.nvidia.com/nsight-compute/ProfilingGuide/index.html#cache-control)
and [application replay](https://docs.nvidia.com/nsight-compute/ProfilingGuide/index.html#application-replay).

Even after an initial flush, cost develops substantial reuse _within_ its own execution. In the
scalar captures, mean whole-kernel L1/TEX and L2 hit rates are respectively 64.61%/81.92% in free
space, 70.34%/80.02% in the hairpin, and 53.13%/92.39% with 64 obstacles. These aggregate rates
mix output, scene, projection, texture and other accesses. They cannot identify output-buffer
residency, and the higher obstacle L2 rate does not make its longer cost kernel faster. Near-equal
array/scalar isolated cost timings therefore neither establish nor exclude a natural-cache effect.

**Next measurement is prepared in the opt-in `handoff` mode of
[run_projection_followup.sh](test/run_projection_followup.sh).** It collects free space, hairpin
and static-64 cost kernels, using application replay for both cache policies: `all, none` followed
by `none, all`, with clock control disabled. Each report selects only iteration zero of the first
measured call after ten warmups. Its dynamics producer and all preceding kernels are unprofiled.
Nine hardware metrics cover duration, SM clock, L1/L2 hit rates, global-load sectors, L2 read
sectors/misses, and DRAM reads/writes. This avoids the full set's software counter instrumentation.
There are twelve Compute reports; the number of companion JSON files depends on replay passes.

The runner checks every emitted JSON's workload/validity and compares configuration plus
first-call/first-measured-call numerical diagnostics within a replay and across cache policies.
It stops if they differ. Strict kernel matching checks launch identity/order, not input bits;
equal JSON diagnostics likewise do not prove identical input buffers. If reproducibility fails,
retain the reports and use a deterministic fixed-input producer/consumer fixture before making a
causal cache claim. Application replay remains intrusive; use Systems and unprofiled timings for
the complete-call assessment.

Compare `none` against `all` separately for each compiled implementation, then compare that
cache-policy effect across array and scalar builds at similar measured clocks. A smaller benefit
from `none` after scalarization, together with more L2 read misses/DRAM reads, would support the
cache hypothesis; stable cache-policy effects would weaken it. Whole-kernel counters still cannot
attribute an effect solely to output data. Preserve matching source during capture and use the
resolved library hashes to identify builds. At investigation time the installed library still
matches the reverted array hash, while the checkout contains scalar source; rebuild the intended
variant before collecting its results.

**Candidate implementation after this measurement:** keep the consumer's global layout and
cooperative load, but have the dynamics block cooperatively store its already-populated shared
output tile across X/Y workers. This changes copy ownership only. Retain the barrier after
`dynamics->step` and the barrier after all output reads, along with all dynamics arithmetic,
constrained controls, and Z-distribution offsets. Avoid a global transpose or extra staging
kernel as the first experiment. Account for 13-component alignment, generic block shapes and
other model instantiations; do not assume every transaction will reach the ideal four sectors.
Require exact full-output parity, synchronization checks, source-counter confirmation and a
joint dynamics-plus-cost/complete-call improvement before keeping the change.

No production kernel was changed in this investigation. The only executable change is the
explicitly invoked collection mode; no build, test, benchmark, profiler capture or commit was run.

#### First cache-policy results: 2026-11-06 06:07 UTC

**Conclusion:** the new scalar-build captures show no cost-kernel improvement from disabling
cache flushing. L2 read misses are essentially unchanged. This does not support a substantial
inherited-cache benefit in these captures, but it does not establish production cache residency
or explain the earlier array/scalar cost-phase difference.

The user supplied `mppi_benchmark_results/projection_handoff_20261106_060707_WcpvOW`.
All twelve Compute reports contain exactly one cost launch with grid `(16384,1,1)` and block
`(80,1,1)`. All nine requested hardware counters fit in **one collection pass**, with zero profiler
warmup passes. Accordingly, one companion JSON per capture is expected; there are no missing
application-replay reports. All twelve JSON reports are valid, with zero unexpected outcomes,
ten warmup calls, three measured calls, ten iterations, split rollout and projection pruning.
Within each scene, configuration and numerical diagnostics match across all four processes for
the first optimizer call and **all three** measured calls. This is stronger than the runner's
first-measured-call gate, but still does not prove equality of complete input/output buffers.
The log contains only the expected uncontrolled-clock/cache warnings, with no collection errors.

**The intended scalar build now ran.** Configure-time metadata records `3109148c1f-dirty`,
with vendor `56e4b48`. The manifest records executable SHA-256
`1ecad70d0c8c3827d6ec93eab86d091fd3d11495a4b7c39a2bd39e01b2a79e49` and resolved optimizer-library
SHA-256 `fea02cf662d03010d6c27c88c2b98454d08cd6c619474470e027d80c5e428f27`. The installed library
still matches that hash at analysis time. Static disassembly of its sm_89 dynamics kernel has
no `LDL` or `STL` instructions, consistent with scalar derivatives. This supersedes the preceding
investigation's observation that the installed library still contained the reverted array code.

Each value below averages the two launches for that policy. `all` flushes caches; `none` disables
the profiler's explicit flush. Positive time change means `none` was slower. Times are per cost
kernel launch, not per optimizer call; MiB means 1,048,576 bytes.

| Scene                   | Cost time all → none (µs) | Time change | L2 read misses all → none (million sectors) | DRAM reads all → none (MiB) |
| ----------------------- | ------------------------: | ----------: | ------------------------------------------: | --------------------------: |
| FreeSpaceCached         |         411.472 → 419.296 |      +1.90% |                         2.498840 → 2.498808 |             77.769 → 78.296 |
| Hairpin256Cached        |         500.592 → 515.696 |      +3.02% |                         2.498480 → 2.498488 |             78.208 → 77.208 |
| StaticObstacles64Cached |         815.216 → 817.856 |      +0.32% |                         2.556445 → 2.556832 |             80.808 → 80.591 |

The cache-none time changes in pair 1 / pair 2 are +3.82% / −0.008% in free space,
+4.57% / +1.45% in the hairpin, and +0.039% / +0.612% with 64 obstacles. Two observations per
policy do not establish a statistically reliable slowdown. Mean measured SM clocks differ by
at most 0.14% between policies, near 2.70 GHz. Snapshot memory clocks vary between 10,251 and
10,501 MHz, so all hardware conditions cannot be treated as fixed.

Global-load sector counts are identical across all four captures within each scene. L2 read
miss changes are at most 0.016%; mean aggregate L2 hit rates change by at most 0.212 percentage
points. DRAM reads change by −1.28% to +0.68%, without a consistent reduction when caches are
preserved. These counters give no evidence of a large reduction in memory reads from inherited
cache contents. They aggregate accesses to multiple allocations and cannot identify which
output lines survived the handoff.

The GPU also drives the desktop: snapshots list Xorg/KDE, VS Code, Slack, Firefox and FreeTube.
Their presence does not measure concurrent activity, but chip-wide DRAM/L2 counters can include
other GPU engines, as described in NVIDIA's
[asynchronous activity guidance](https://docs.nvidia.com/nsight-compute/ProfilingGuide/index.html#asynchronous-gpu-activity).
Profiling also adds preparation overhead before/after the selected launch. The profiled complete
call takes 379–476 ms here; that is not its normal latency, and the report cannot localize all
of that overhead relative to the producer. Consequently, `cache-control none` means no explicit
profiler flush, not a guarantee of the exact unprofiled cache state. The preserved-cache mode
and single hardware pass address replay-induced warming but do not remove every perturbation.

**Next action:** proceed with the producer-only cooperative output-store experiment described
above, using this scalar build as the starting point. The justification remains its confirmed
excess store sectors, not a promised cost-cache speedup. Keep cost code, output layout, arithmetic
and barriers unchanged. Evaluate exact output parity and synchronization first, then dynamics
store counters, dynamics-plus-cost time and complete-call distributions. Retain this collection
as the cache-policy baseline for the same build/settings. A matching array-build `handoff`
collection is needed only to attribute a cache-policy difference specifically to scalarization;
the earlier full-set/kernel-replay profiles cannot serve as that control.

This analysis reads saved reports and disassembles the existing library only. No build, test,
benchmark, profiler capture or commit was run, and no production code was changed.

#### Cooperative output-store implementation prepared: 2026-11-06

The split dynamics kernel now copies odd-sized shared output tiles, including the bicycle's
13-component tile, cooperatively across X/Y workers. Each Z distribution keeps its own shared
tile and global rollout range. Adjacent lanes copy adjacent components within a rollout rather
than the same component of trajectories 4,160 bytes apart. The copy uses scalar loads/stores and
does not assume float2/float4 alignment. Even-sized model outputs retain the previous vectorized
path because their instruction/transaction tradeoff has not been measured.

The existing global `[distribution][rollout][time][component]` layout, cost consumer, model
arithmetic, sampled-control accesses and launch dimensions are unchanged. The output-publish
barrier and the barrier before shared storage reuse remain in place. The helper adds no barrier,
buffer or kernel launch. As before, the launcher requires full X tiles; this change does not add
support for a partially populated final block.

Two new GTests in `test_dynamics_output` cover the copy independently and the real split dynamics
kernel against the old store path. Copy cases cover odd/even output widths, partial warps,
multiple distributions/blocks, idle workers, unaligned base pointers, special floating-point bits,
guard regions and untouched timesteps. Full-rollout cases compare all 80 steps' output bits and
constrained controls using the actual scalar bicycle/colored-noise specialization, fixed samples,
three delay configurations and seven block shapes. The old loop is test-only and does not appear
in the production optimizer or performance benchmark.

Build, correctness, sanitizer and performance commands are in
[cooperative output-store validation](test/PERFORMANCE_BENCHMARKS.md#cooperative-dynamics-output-store-validation).
The source has been reviewed and formatting/whitespace checks performed; **no build, test,
sanitizer or profiler has been run by the implementation agent**. The subsequent user-run
validation and measurements are assessed below. Retain the change only after exact
parity/synchronization checks, reduced output-store sectors without local spills, and a favorable
combined dynamics/cost and complete-call comparison.

#### Cooperative output-store results and constraint race: 2026-11-06

**These pre-fix captures show strong store-performance evidence, but correctness acceptance at
this stage was blocked by a race in the existing bicycle constraint code.** The user supplied the test/sanitizer output
and these completed collections under `mppi_benchmark_results/`:

- `projection_repeat_20261106_080708_3NrJrt`
- `projection_nsys_20261106_081204_tl0RW3`
- `projection_ncu_20261106_081213_BxDldv`
- `projection_handoff_20261106_081618_ZX0VRH`

All four manifests identify benchmark SHA-256
`7c4ebbed63609c5405d5a641a27aab62bd20e7064de3e3c736726a7927e8e5c6` and optimizer-library SHA-256
`a58bf1b27528ced22783eb80f1d0cd246dc7fe68522d04858842be89da21ee02`. The configured revisions are
Universe `3109148c1f-dirty` and vendor `56e4b48-dirty`. The comparison uses the earlier **scalar
derivative with legacy output stores**, not the reverted derivative-array build:
`projection_repeat_20261106_012040_uN9sii`, `projection_nsys_20261106_012626_K19ygV`, and
`projection_ncu_20261106_012635_cF7Ijs` (library `4d8bf9c8…`), plus the rebuilt legacy-store
`projection_handoff_20261106_060707_WcpvOW` (library `fea02cf6…`) for cache-policy comparison.

**Correctness status.** Both `GpuDynamicsOutput` tests passed normally and under memcheck;
memcheck reported zero errors. The independent copy test also completed under racecheck without
reported hazards. During `ProductionRolloutMatchesLegacyStoresBits`, however, racecheck reported
**30 displayed hazards: 12 errors and 18 warnings**, despite both GTests returning PASS. The
shell loop uses `--error-exitcode 1` and `|| break`, so it stopped after racecheck: **synccheck was
not run**. The pasted output does not contain a new derivative-test or combined/split-cost
regression run.

Every displayed race connects the parent range-clamp write in vendor `dynamics.cu:133` with the
bicycle's steering-continuity read/write. In the
[device constraint implementation](include/mppi/dynamics/dubins/first_order_dubins_bicycle.cu#L403),
the parent partitions control components across Y workers, then Y=0 immediately applies steering
continuity without synchronization between those stages. For the deployed `(32,2,1)` block,
Y=1 clamps steering while Y=0 can read or overwrite the same shared steering value. The caller's
barrier after `enforceConstraints` is too late to order operations inside that function. This code
is already present in Universe HEAD and is shared by the legacy-reference and production kernels;
the copy helper does not appear in the reported race sites. The race therefore predates the store
change, and old/new equality through the same model is insufficient to establish race freedom.

**Collection validity.** Independently checked all 149 new benchmark JSON reports: 128 repeat,
3 Nsight Systems, 6 full-set Nsight Compute, and 12 handoff. They report valid measurements,
zero unexpected outcomes, complete ten-iteration work, 16,384 rollouts, an 80-step horizon, seed 1
and the intended split/pruning/timing modes. Every stored call completes successfully. Repeat
runs contain 100 measured calls after 10 warmups, with four processes per scene/mode/pruning
setting. These checks establish the recorded benchmark's workload/outcome contract; they do not
override the sanitizer failure or establish full-trajectory bitwise correctness.

**Complete-call and phase timing.** Values below are milliseconds per ten-iteration optimizer
call, old → new, with projection pruning enabled. Complete-call values are the mean of four
independent process medians in latency-only mode; phase values are the mean of four process GPU
means from the separately instrumented runs. The old and new builds were collected at different
times; the runner's alternating pairs toggle projection pruning, not the store implementation.

| Scene                         | Complete-call median (ms) | Reduction | Dynamics GPU mean (ms) | Cost GPU mean (ms) |
| ----------------------------- | ------------------------: | --------: | ---------------------: | -----------------: |
| FreeSpaceCached               |           18.065 → 14.893 |    17.56% |          5.937 → 3.406 |      4.367 → 4.105 |
| Hairpin256Cached              |           19.374 → 16.161 |    16.58% |          6.008 → 3.735 |      5.021 → 4.742 |
| StaticObstacles64Cached       |           23.163 → 19.487 |    15.87% |          5.916 → 3.402 |      8.567 → 8.129 |
| StaticObstacleGeometryUpdates |           20.623 → 17.046 |    17.34% |          6.019 → 3.406 |      5.941 → 5.617 |
| GridBoundaryCrossings         |           24.852 → 21.299 |    14.29% |          5.824 → 3.451 |      9.716 → 9.353 |
| ReferenceGeometryUpdates      |           19.389 → 16.173 |    16.59% |          6.028 → 3.735 |      5.010 → 4.713 |
| ShortProjectionPath           |           16.728 → 13.943 |    16.65% |          5.854 → 3.705 |      3.002 → 3.067 |
| OverlappingProjectionGroups   |           31.183 → 26.981 |    13.48% |          6.020 → 3.398 |    16.901 → 15.992 |

Dynamics phase time falls 36.7–43.6%; dynamics plus cost falls 15.4–27.1% in all eight scenes.
The short-path cost phase increases 2.15%, so the cost benefit is not universal. Mean process
p95 latency falls 16.0–22.3%, and process CPU time falls 13.5–18.2%; the latter is CPU time over
the complete call, not isolated CPU algorithm work. With pruning disabled, complete-call medians
also fall in all eight scenes, by 12.5–16.6%. In each pruning-enabled scene, all four new process
medians are below all four old process medians.

Saved Nsight Systems timelines independently show dynamics means of 5.938 → 3.392 ms in free
space, 6.079 → 3.718 ms in the hairpin, and 5.732 → 3.384 ms with 64 obstacles. Cost means are
4.338 → 4.106, 5.198 → 4.769, and 8.169 → 8.147 ms respectively. These are sums of kernel
durations over ten iterations, excluding 100 warmup launches, averaged over 20 measured calls.

**The intended memory-access change is confirmed.** All nine new profiled dynamics launches
(three scenes × three launches) show the same output-store sector counts:

| Dynamics metric per launch              | Legacy stores | Cooperative stores |
| --------------------------------------- | ------------: | -----------------: |
| Output-store theoretical global sectors |    17,039,360 |          3,706,880 |
| Output-store excessive sectors          |    14,909,440 |          1,576,960 |
| All global-store sectors                |    18,350,080 |          5,017,600 |
| All excessive global-access sectors     |    16,875,520 |          3,543,040 |
| Local load/store instructions           |         0 / 0 |              0 / 0 |
| Registers per thread                    |            61 |                 61 |
| Dynamic shared memory per block (bytes) |        12,400 |             12,400 |

Output-store excessive sectors fall **89.4%**; output-store sector requests fall **78.2%**.
These are request counts, not equivalent DRAM-byte reductions. The unchanged control reads and
writes still account for 983,040 excessive sectors each. Aggregate dynamics L2 sectors fall
69.5–69.7%, while executed instructions rise 3.8–4.2%; the copy trades some address/instruction
work for substantially fewer memory requests. No static LDL/STL instructions or measured local
load/store traffic appear in the dynamics kernel.

Full-set NCU dynamics duration means fall from 541.301 → 358.923 µs in free space,
544.725 → 372.501 µs in the hairpin, and 556.619 → 358.261 µs with 64 obstacles: reductions
of 31.6–35.6%. Measured SM cycles fall 36.3–40.1%. Cost instructions are identical across builds
after normalizing relocated control-flow addresses, and executed-instruction/local-access counts
match for the selected launches. The cost kernel retains its existing local-memory accesses;
the zero-local-access result applies to dynamics only.

**Conditions differ.** The old NCU runs measured approximately 2.70 GHz SM frequency; the new
runs measured approximately 2.51–2.52 GHz. Accordingly, cold-cache cost durations rise 5.0–5.6%
even though measured cost cycles fall 1.5–2.3%. Desktop snapshots also differ: the old runs list
Code, Slack, Firefox and FreeTube alongside Xorg/KDE; the new snapshots list Xorg/KDE and the
screen locker. This does not measure concurrent load, but the collections are not a controlled
old/new timing experiment. The transaction reduction and lower dynamics cycles support a real
store-efficiency improvement; the entire observed end-to-end gain cannot be assigned to this
change alone.

**Cache-policy follow-up.** Each of the 12 new application-replay reports contains one selected
cost launch, one hardware pass and zero profiler warmup passes. Within each scene, metadata and
all stored numerical call diagnostics agree across all four captures, and also agree with the
earlier handoff collection after excluding build identity and timings. Global-load sectors remain
identical across policies and builds. Two-launch policy means for the new build are:

| Scene                   | Cost all → none (µs) | Time change | L2 read misses all → none (million sectors) | DRAM reads all → none (MiB) |
| ----------------------- | -------------------: | ----------: | ------------------------------------------: | --------------------------: |
| FreeSpaceCached         |    432.704 → 429.616 |      −0.71% |                         2.498334 → 1.379582 |             76.385 → 42.273 |
| Hairpin256Cached        |    527.808 → 528.144 |      +0.06% |                         2.498086 → 1.379534 |             76.379 → 42.288 |
| StaticObstacles64Cached |    866.592 → 865.968 |      −0.07% |                         2.555906 → 1.563592 |             78.157 → 47.896 |

Unlike the earlier legacy-store handoff, the new captures show **38.8–44.8% fewer L2 read misses**
and roughly **30–34 MiB fewer DRAM reads** without an explicit cache flush. This supports improved
cache reuse in the measured setup. It does not identify the allocations supplying those hits or
isolate the store change from the changed environment. DRAM writes increase from about 3–4 MiB
with `all` to 26–28 MiB with `none`, so the read reduction is not the net traffic reduction.
Cost time barely changes; with two observations per policy, this is not evidence of a reliable
cost-kernel latency gain. The earlier profiling-preparation and chip-wide-counter limitations
still apply. Do not use the profiled complete-call timings as normal optimizer latency.

**Numerical repeatability and next action.** Each new scene has one identical sequence of
`baseline_cost`, `eligible_rollouts`, `lambda_used` and `lambda_next` across all 16 repeat
processes (both timing/pruning modes). The old captures have multiple sequences per scene and
some differ from the new sequence. This is better observed repeatability, not proof that the
constraint race disappeared or that it explains every historical divergence.

Fix the constraint ownership before another optimization: let the Y worker that performs the
parent steering clamp also apply steering continuity, and retain acceleration handling with its
component's owner. Both post-clamp operations use immutable state and only their own control
component, so this can preserve arithmetic and the caller's publication barrier without adding
a block barrier. Add focused device-versus-host constraint coverage for multi-Y blocks,
out-of-range/deadband commands, hold/restart behavior and reverse prevention. Then rerun the
output/derivative/combined-split regression tests and all three sanitizer modes. After they pass,
compare legacy/cooperative stores with the same constraint fix and matched clocks/background
conditions to confirm the final end-to-end benefit. Keep this capture as pre-fix evidence.

This analysis only reads saved artifacts and source and updates documentation. No build, test,
benchmark, sanitizer, profiling capture or commit was run by the analysis agent; production code
is unchanged in this follow-up.

#### Constraint ownership fix and validation: 2026-11-06

The device bicycle constraint method now assigns each state-aware operation to the same Y worker
that performs its parent range/deadband clamp: `control_index % blockDim.y`. Steering continuity
therefore runs on Y=1 when multiple Y workers participate, and on Y=0 when `blockDim.y == 1`.
Acceleration/reverse prevention remains on Y=0. Each operation reads immutable state and only its
own control component, so their relative execution order across workers is irrelevant. Within
each component the parent clamp still precedes state-aware filtering. The host path, filtering
formulas, output stores and caller barriers are unchanged; the fix adds no block barrier.

`GpuDynamicsOutput.ControlConstraintsMatchHostAcrossWorkers` independently exercises the device
constraint method against serial host evaluation, rather than comparing two rollouts using the
same parallel constraint path. Its 22 command/state cases cover range/deadband behavior (including
nonzero zero-control values and exact deadband boundaries), hold/release and short-reference
behavior, restart rate/acceleration limits and forward/reverse behavior. Eight configurations
toggle reverse prevention, deadbands and restart blending. All seven existing block shapes run
every case on every X/Z group over multiple blocks with shared-buffer reuse. The deliberately
narrow parent steering range makes clamping before a held-command override observable, and known
range/hold/reverse outcomes anchor the host fixture.

The test permits 2e-6 absolute error against host arithmetic and additionally requires exact
device bits against the Y=1 execution for every other block shape. It checks per-record guards,
unchanged shared state and immutable global inputs. The existing copy-bit/layout and 80-step
legacy-store/production-rollout tests remain in the same executable, now containing three tests;
both full-rollout paths use the corrected constraint method.

The user subsequently supplied successful execution of the updated
[validation commands](test/PERFORMANCE_BENCHMARKS.md#cooperative-dynamics-output-store-validation):

| Validation                          | Result                                                                                                                                      |
| ----------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `test_dynamics_output`              | All 3 tests passed: copy bits/layout, serial-host constraint parity across workers, and 80-step production/legacy-store parity              |
| `test_dynamics_derivative`          | All 3 tests passed: host derivative bits, device derivative bits/write ownership, and replay parity                                         |
| Selected `test_costs` cases         | All 4 tests passed: both combined/split projection/safety/preferred-lane cases, velocity-dependent steering propagation, and delayed replay |
| Memcheck on `test_dynamics_output`  | All 3 tests passed; 0 errors                                                                                                                |
| Racecheck on `test_dynamics_output` | All 3 tests passed; 0 hazards displayed, 0 errors, 0 warnings                                                                               |
| Synccheck on `test_dynamics_output` | All 3 tests passed; 0 errors                                                                                                                |

All ten selected regression tests passed without skips. Unlike the earlier run, all three
sanitizer modes completed; the previously reported shared-control race no longer appears in
these cases. This clears the outstanding correctness/sanitizer gate for the tested configurations,
including the deployed `(32,2,1)` shape. It does not establish correctness for every possible
workload or confirm performance of the rebuilt constraint-fixed implementation.

The preceding performance captures describe the pre-fix build. The next step is fresh performance
collection: use this same constraint fix in both legacy-store and cooperative-store builds under
matched clocks/background conditions, changing only the store implementation. This prevents the
correctness fix from becoming another variable in store attribution. Preserve the new library
hashes and matching source for each capture. **No build, test, sanitizer or profiling capture has
been run by the implementation/analysis agent**; the execution results above are user-provided.

#### Constraint-fixed performance and next experiment: 2026-11-07

The November 6 captures supplied after the successful constraint/sanitizer validation are usable:

- `projection_repeat_20261106_112204_EpT0FH`: 128 benchmark reports.
- `projection_nsys_20261106_112659_KLKvJD`: three timelines and benchmark reports.
- `projection_ncu_20261106_112708_jQfTzB`: six full-set kernel profiles and benchmark reports.
- `projection_handoff_20261106_112753_xjT2oj`: 12 application-replay profiles and benchmark reports.

All **149 JSON reports** mark measurements valid, record zero unexpected outcomes, and complete
the intended workload. All four collections resolve optimizer-library SHA-256
`69559d18e7578275f8ff2278d7c23c14c546b5227996f2bfbb753f02edfdee66`, with executable hash
`7c4ebbed63609c5405d5a641a27aab62bd20e7064de3e3c736726a7927e8e5c6`.
The configure-time revisions still identify dirty universe `3109148` and vendor `56e4b48`;
the changed library hash distinguishes this build from the earlier cooperative implementation
(`a58bf1b27528ced22783eb80f1d0cd246dc7fe68522d04858842be89da21ee02`).

The comparison below uses the previous `080708_3NrJrt` repeat collection, which already used
cooperative output stores but preceded the constraint fix. Projection pruning is enabled in both.
Complete-call values are means of four independent process medians in latency-only mode; GPU
values are means of four process means in the separate instrumented mode. All times are
milliseconds per ten-iteration optimizer call.

| Scene               | Complete call, before → after | Dynamics GPU, before → after | Cost GPU, before → after |
| ------------------- | ----------------------------- | ---------------------------- | ------------------------ |
| Free space          | 14.893 → 14.857               | 3.406 → 3.337                | 4.105 → 4.118            |
| Hairpin 256         | 16.161 → 15.999               | 3.735 → 3.584                | 4.742 → 4.764            |
| 64 static obstacles | 19.487 → 19.361               | 3.402 → 3.334                | 8.129 → 8.144            |
| Obstacle updates    | 17.046 → 16.942               | 3.406 → 3.334                | 5.617 → 5.631            |
| Grid crossings      | 21.299 → 21.172               | 3.451 → 3.370                | 9.353 → 9.357            |
| Reference updates   | 16.173 → 16.044               | 3.735 → 3.584                | 4.713 → 4.739            |
| Short path          | 13.943 → 13.788               | 3.705 → 3.565                | 3.067 → 3.066            |
| Overlapping groups  | 26.981 → 26.847               | 3.398 → 3.330                | 15.992 → 16.013          |

Observed dynamics reductions are **2.00–4.03%**, complete-call median reductions **0.24–1.11%**,
and process CPU mean reductions **0.20–1.22%**. Cost time is essentially unchanged. Tail latency
does not improve uniformly: mean process p95 increases from 16.349 to 16.880 ms for reference
updates and from 15.160 to 15.166 ms for free space. With pruning disabled, complete-call medians
also decrease by 0.20–0.92%. These are separate collection periods, not an interleaved experiment:
GPU snapshots show similar desktop clients and 2,520 MHz active clocks, but clocks were not
locked and snapshots do not describe within-run variation. The small end-to-end differences
should not be treated as precise causal estimates of the ownership fix.

For all eight scenes, each collection's 16 processes produces identical sequences of recorded
baseline cost, eligible-rollout count, and both lambda fields across pruning and timing modes.
Those fields also match at every corresponding cycle across the two builds: **12,800 measured
call pairs**. The handoff reports' first-call and all three measured-call diagnostics also match
across policies and builds. This is stronger reproducibility evidence than the earlier divergent
captures, but these JSON diagnostics are not hashes of the full controls or trajectories.

The fresh Nsight Systems timelines support lower dynamics time: free/hairpin/static-64 fall
from 3.392/3.718/3.384 to 3.326/3.569/3.324 ms per ten iterations. The free-space sampling phase
increases from 3.589 to 3.694 ms in that short trace, more than offsetting its dynamics saving
in the total GPU burst span. This reinforces the need to assess complete calls alongside kernels.

Full-set Nsight Compute dynamics means fall from **358.923/372.501/358.261 µs** to
**348.672/360.096/347.925 µs** at approximately 2.517 GHz in both builds. Dynamics still uses
61 registers and 12,400 bytes of dynamic shared memory; executed local loads/stores and their
local-memory traffic remain zero. Store metrics are unchanged: 5,017,600 global store sectors
per launch, 3,706,880 source-attributed theoretical output-store sectors, and **1,576,960 excessive
output-store sectors**. The earlier 89.4% reduction from the legacy copy therefore persists.
The total excessive count is 3,543,040, including 983,040 each for sampled-control reads and writes.

The cost kernel's 15,098 SASS entries match after normalizing only relocated control-flow targets;
its executed instruction and local-memory counters also match across these builds. It still
uses 64 registers and executes local-memory instructions. The dynamics result does not imply
that the cost kernel is spill-free. Full-set cost durations are nearly unchanged at
424.309/500.064/847.584 µs after the fix, versus 422.741/500.245/849.899 µs before it.

**Cache reuse is variable and does not establish a cost-latency benefit.** The new handoff capture
uses the same strict application-replay policy and hardware-only metric set; every report has
one hardware pass and zero warmup passes. Values below average the two balanced policy pairs.
L2 misses refer specifically to texture-unit read lookup misses, not all L2 requests.

| Scene               | Cost duration, cache `all` → `none` (µs) | L2 read misses (million sectors) | DRAM reads (MiB) |
| ------------------- | ---------------------------------------- | -------------------------------- | ---------------- |
| Free space          | 429.376 → 435.424                        | 2.498 → 1.938                    | 76.386 → 59.329  |
| Hairpin 256         | 527.184 → 540.288                        | 2.498 → 2.498                    | 76.375 → 76.382  |
| 64 static obstacles | 862.944 → 866.432                        | 2.556 → 1.564                    | 78.158 → 47.903  |

Free-space cache-`none` misses differ substantially between pairs: 1,378,645 versus 2,498,281.
The hairpin's earlier reuse benefit does not repeat: both new cache-`none` launches miss about
2.498 million sectors. Static-64 retains a repeatable approximately 39% reduction in misses,
but its cost duration is still slightly higher without flushing. All three policy means are
slower with cache `none` by 0.40–2.49%; this is not sufficient evidence that flushing improves
production performance. The global-load request counts remain identical across policies and
builds. No cache-policy, cost-layout, register-cap, or synchronization change follows from this
experiment alone.

**Implemented next step:** a controlled legacy/cooperative store comparison from one binary,
with the corrected constraints in both variants. The dynamics kernel has two compile-time
specializations; a host-side selection in the split launcher changes only the output copy.
Cooperative stores remain the default. The benchmark reads
`MPPI_BENCHMARK_COOPERATIVE_OUTPUT_STORES=0|1`, explicitly selects split rollouts after startup,
and records the effective `dynamics_output_store`. Production has no new ROS parameter or
environment-variable lookup, and retains its normal kernel selection. Startup selection trials
remain identical for both benchmark variants and are excluded from steady-state summaries.

The new opt-in `stores` runner mode alternates four legacy/cooperative pairs in each timing mode
across all eight scenes, with pruning enabled throughout. It records hashes and GPU snapshots
and produces 128 JSON reports. Its offline comparison script requires matching build/profile
metadata and every stored first/measured-call diagnostic within each scene/timing mode before
reporting process medians/p95, CPU means, dynamics/cost means and paired differences. It rejects
incomplete or divergent collections; matching diagnostics still do not prove full output parity.
The full-rollout correctness test now checks both production specializations against the frozen
reference, and both combined/split cost tests exercise the launcher's legacy and cooperative
branches. Test counts are unchanged, but these expanded cases require a fresh user-run validation.

See [matched store comparison](test/PERFORMANCE_BENCHMARKS.md#matched-store-comparison-from-one-build)
for rebuild, correctness/sanitizer and collection instructions. This closes the outstanding
attribution gap before another layout or cache experiment. Only saved-artifact analysis and
static checks were run by the agent; **no build, test, benchmark, sanitizer, profiler capture or
commit was run** for these additions.

#### Controlled store comparison results: 2026-11-07

The user completed `projection_stores_20261106_212505_4n792O` (November 7 at 06:25 JST;
directory timestamps are UTC). This is the primary performance comparison for the cooperative
output stores: **one build, the corrected constraints in both variants, projection pruning
enabled, and four balanced legacy/cooperative pairs per timing mode across eight scenes**.
Both variants force split rollouts and use the same workload/profile/seed. No profiler is active.
The manifest identifies executable SHA-256
`f31f49f8e7f06b415445a6cddbb90195de70c96d6ba6ceb7bcfeb3b09554c023` and resolved optimizer-library
SHA-256 `4f0daa650ce1e88b349cc0f7b55b62f2ed70f5bc01334f498b63f2178a7ec55c`.

Independent analysis of the saved JSON confirms all **128 reports** are valid, with zero
unexpected outcomes, complete 10-iteration workloads, and matching build/host/GPU/profile
metadata. Each process has 10 warmups and 100 measured calls. The reported medians, p95 values,
CPU means and dynamics/cost GPU means agree with their raw samples; the saved
`store_comparison.json` agrees with the per-process reports. All stored first-call and measured-call
diagnostics match across all 16 processes for each scene, including both timing modes: **12,800
measured calls** in total. These diagnostics remain distinct from full output bit comparisons.

Times below are milliseconds per ten-iteration call. Complete-call values average four process
medians in latency-only mode; dynamics and cost values average four process means in the separate
instrumented mode. They are not pooled-sample medians or independent measurements of every kernel.

| Scene               | Complete call, legacy → cooperative | Reduction | Dynamics GPU, legacy → cooperative | Cost GPU, legacy → cooperative |
| ------------------- | ----------------------------------- | --------- | ---------------------------------- | ------------------------------ |
| Free space          | 16.989 → 14.802                     | 12.87%    | 5.532 → 3.340                      | 4.122 → 4.112                  |
| Hairpin 256         | 17.887 → 15.981                     | 10.65%    | 5.542 → 3.585                      | 4.764 → 4.754                  |
| 64 static obstacles | 21.525 → 19.315                     | 10.26%    | 5.557 → 3.337                      | 8.142 → 8.131                  |
| Obstacle updates    | 19.101 → 16.912                     | 11.46%    | 5.558 → 3.339                      | 5.625 → 5.616                  |
| Grid crossings      | 23.289 → 21.152                     | 9.18%     | 5.546 → 3.370                      | 9.362 → 9.353                  |
| Reference updates   | 17.891 → 15.967                     | 10.76%    | 5.501 → 3.587                      | 4.742 → 4.727                  |
| Short path          | 15.713 → 13.760                     | 12.43%    | 5.500 → 3.565                      | 3.068 → 3.072                  |
| Overlapping groups  | 29.051 → 26.816                     | 7.69%     | 5.539 → 3.335                      | 16.016 → 16.006                |

**The store optimization has a consistent, material benefit on this workload and GPU.**
Dynamics time falls **34.79–39.94%**, saving about **1.91–2.22 ms per call**. Complete-call
medians fall **7.69–12.87%**, about **1.91–2.23 ms**. All 32 latency pairs improve, with individual
median savings of 1.867–2.301 ms; the improvement occurs with either run order. For example,
free-space savings average 2.177 ms when cooperative runs first and 2.198 ms when it runs second.
Both the producer timing and the complete-call result support attribution to the store variant,
unlike the earlier separate-build comparisons with clock/load and constraint-fix differences.

Every paired p95 also improves; the reductions in mean process p95 span **7.77–11.89%**.
The first cooperative free-space process has a noisier p95 (16.925 ms versus 14.870–15.054 ms
in the other three), but still beats its paired legacy process. Process CPU means improve
**7.70–12.60%** in every scene and every pair. Process CPU time includes work spent in the CUDA
runtime/wait path; this does not establish a separate host-algorithm optimization.

**Cost execution is essentially unchanged.** Scene mean differences range from a 0.31% reduction
to a 0.12% increase; only 19 of 32 individual cost pairs improve. Dynamics plus cost decreases
**10.27–22.82%**, without a cost regression that offsets the producer saving. This confirms that
the useful gain is in dynamics output production; it provides no new claim of a cost-cache
speedup. The previously measured sector reduction explains the direction, but this collection
contains no new instruction, register, spill or hardware-transaction counters.

The GPU is an RTX 4090. All 32 boundary snapshots show the same five desktop GPU clients and
a 450 W power limit. Active snapshots report 2,520 MHz SM clocks; the first pre-run snapshot is
idle at 210 MHz. Temperatures range from 44–62 °C, with software thermal slowdown inactive in
every snapshot. The environment note contains only the runner's pair/mode label, and snapshots
do not prove fixed clocks or absence of contention during execution. Four balanced pairs provide
stronger local evidence, not a guarantee across devices, deployment profiles or closed-loop runs.

**Decision:** retain cooperative output stores as the default. The controlled matrix closes the
main performance-attribution gap; another full store comparison is unnecessary unless the code
or target environment changes. This collection contains benchmark output only: it does not
replace the expanded two-specialization correctness tests or sanitizer checks documented above.
The earlier successful correctness/sanitizer evidence predates that test expansion.

For the next optimization investigation, prioritize the **cost kernel** on the representative
obstacle-heavy cases: cost alone is now 8.131 ms for static-64 and 9.353 ms for grid crossings,
while the overlapping-path stress case reaches 16.006 ms. Use source-correlated measurements to
separate remaining exact geometry work from local-memory traffic before choosing a change.
Sampling is the next common alternative at 3.625–3.642 ms across scenes; weighting takes
1.305–1.335 ms and weighted control reduction 0.206–0.381 ms. These are separate GPU phase means,
not fractions obtained by mixing instrumented timings with latency-only wall time. Further cache
policy or output-layout changes are not justified by the present cost results.

The agent read saved reports and updated documentation only; no build, test, benchmark, sanitizer,
profiling capture or commit was run for this analysis.

## Prioritized recommendations

The list below retains the original static-review workstreams. Recommendation 1 led to the prepared
and grouped projection changes assessed above. For the current pruned implementation, the
derivative-storage and cooperative output-store changes have now been implemented and measured;
the [controlled store comparison](#controlled-store-comparison-results-2026-11-07) is the primary
evidence for the latter. The next investigation prioritizes cost evaluation, with sampling as a
common-workload alternative. The remaining workstreams are conditional on their workload and profiling
evidence. Effort estimates assume an engineer familiar with the package and include implementation
and regression work: small is roughly 1–3 days, medium roughly 1–2 weeks, large several weeks.
GPU access and review time are additional dependencies.

### 1. Replace repeated exhaustive corridor projection with an exact acceleration structure

**Confirmed structure and measured hotspot.** [Path projection, lines 193–242](include/mppi/cost_functions/path_tracking_geometry.cuh#L193) explicitly evaluates every segment, including when a valid texture/previous-step hint exists. [The cost caller, lines 935–982](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu#L935) fetches that hint before the scan. The texture is itself built with a per-cell segment scan ([map kernel, lines 123–181](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L123)). The hint changes evaluation order, not asymptotic work or search coverage. The [measured follow-up](#measured-follow-up-2026-11-04) attributes most hairpin cost-kernel work to this projection code.

For one projection per running state, a 256-point corridor implies approximately **3.34 billion segment evaluations per 10-iteration cycle**; even an 80-point corridor implies about 1.04 billion. Terminal and other geometry queries can add work. Compiler optimization and actual branch paths determine runtime, so these counts are not instruction counts. Hard lateral safety remains active with zero lateral soft weights ([cost, lines 1742–1795](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu#L1742)); turning those weights off is not a valid way to remove this work.

**Consequence:** long or folded references can consume substantial rollout time even in otherwise empty scenes. The nearest-segment texture currently adds construction/read work without excluding any segment from the query.

**Direction and tradeoffs:** use the measured baseline to start with invariant segment precomputation and independent numerical/parity checks. Then consider a small BVH or a spatial index with valid distance lower bounds, returning the exact globally closest finite segment. Preserve lowest-index ties, endpoint extrapolation conventions, degenerate segments and hairpin behavior. A coarse cell's nearest index alone is not proof that a nonadjacent branch can be excluded. An exact acceleration structure costs build time, device storage and traversal divergence; very short paths may still favor the current scan. Inverse squared lengths or altered distance comparisons require particular care around floating-point ties.

**Dependencies / effort / risk:** initial kernel timing and source attribution are available; before/after timings, independent projection parity and cache-invalidation coverage are still required. A spatial index additionally benefits from query/candidate counters. Medium to large for exact spatial acceleration, medium/high correctness risk. Relevant rows: REF-08, OBJ-03, OBJ-09, VAL-01. Preserve global projection correctness rather than reverting to a bounded local walk.

### 2. Reduce dense map rebuild work while retaining conservative coverage

**Confirmed scaling; scene-dependent impact.** The obstacle generator visits all **20,971,520 cells** in the 512 × 512 × 80 volume on a nonempty rebuild and loops over active hypotheses in each cell. At 64 active hypotheses that is up to **1.34 billion box-distance evaluations per rebuild**, before any rollout. Static hypotheses repeat the same distance calculations in all layers. A dynamic cutoff skips inactive hypotheses but still writes the corresponding layers. See [obstacle kernel, lines 279–310](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L279) and [launch, lines 698–715](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L698).

Static-map construction likewise scans each road/drivable segment for each of 1,048,576 cells; 256 segments mean roughly 268 million point-segment evaluations per rebuilt channel ([lines 224–258](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L224)). Preferred-centerline construction has the same dense structure ([lines 96–120](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L96)). Both environment grids cover a 153.6 m square, regardless of the smaller area a particular rollout population visits.

Existing caching matters: scene updates are batched; dirty geometry and snapped grid origin control rebuilds; empty obstacles skip the kernel. The ego-local frame is persistent and only recenters after a 32 m coordinate displacement or frame change, not every cycle ([local frame, lines 33–40](include/autoware/mppi_optimizer/detail/local_frame.hpp#L33)). Reference/grid movement and prediction changes can nevertheless invalidate caches ([grid update, lines 355–387](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L355), [refresh, lines 808–838](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L808)). Measure cache hits and invalidation reasons before redesigning them.

**Consequence:** changing scenes can have a much larger setup cost than repeated-input benchmarks reveal. Equal-confidence ties increase hypothesis count and therefore map/exact-collision work even when physical object count stays constant.

**Direction and tradeoffs:** separate static-obstacle distance from genuinely moving layers; share identical layers or represent an inactive dynamic tail without computing every box distance again. Preserve the minimum of all active hypotheses. Evaluate tile-level candidate lists with conservative bounds, a retained larger map with an interior update margin, and lazy allocation for disabled channels. Any cropped/adaptive region needs a conservative exact fallback outside it. Coarser resolution or raster approximations change distance errors and broad-phase behavior and require additional clearance analysis; reducing resolution is not a free optimization. The current exact fallback already protects out-of-map obstacle queries ([cost, lines 1157–1220](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu#L1157)), but its performance must be included.

**Dependencies / effort / risk:** changing-scene and all-ties workloads, cache telemetry, map-query parity near boundaries; medium for static-layer reuse, large for tiled/adaptive maps, medium/high safety risk. Relevant rows: SCN-04, SCN-06, SCN-08, SCN-15–17, OBJ-09, VAL-04. Do not drop tied paths, shorten the approved obstacle horizon, or let zero soft weights disable hard checks.

### 3. Parallelize percentile weighting beyond one CUDA block if profiling confirms its cost

**Confirmed serialization; measured importance unknown.** [The launch, lines 1784–1812](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L1784) uses exactly one block for the entire rollout population. It performs shared reductions, four radix-histogram passes to select an exact float percentile, exponential weighting, another reduction and normalization. Each radix pass scans the rollout array; histogram updates use shared atomics and one thread chooses the selected bin ([lines 1180–1298](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L1180)). Equal eligible costs bypass percentile selection. Concentrated float prefixes can concentrate histogram atomics; that is a profiling hypothesis, not an observed stall measurement.

**Consequence:** each iteration has a stage that exposes only one block of GPU parallelism, and the next control update depends on it. The implementation already avoids a host round trip here; adding one would lose that advantage.

**Direction and tradeoffs:** compare the current fused kernel against multi-block partial statistics/histograms with a small final reduction, and against a device selection primitive plus parallel weighting. Reuse scratch buffers and keep results on device. Preserve exact rank selection, signed costs/zeros, all-equal costs, nonfinite exclusion, encoded crash eligibility, normalization bounds, ESS and failure semantics. Multiple kernels and scratch traffic can lose to the current fusion for smaller populations; an approximate histogram would change the MPPI variant and requires a separate decision.

**Dependencies / effort / risk:** isolated timing and cost-distribution cases, then numerical/status regression; medium effort and medium numerical risk. Relevant rows: OBJ-10, WGT-01–07, SMP-06. Renaming or changing unsafe-rollout policy is not part of this performance change.

### 4. Improve control-buffer locality and fuse noise postprocessing before replacing colored noise

**Confirmed access pattern; bandwidth impact needs counters.** The active sampler uses 32,768 length-160 inverse transforms for an 80-step horizon. The plan and buffers are persistent, not recreated per iteration. Each iteration still generates about 5.31 million normal float values, writes reordered noise into the 10 MiB control buffer and launches a separate Gaussian rewrite of that buffer ([sampler, lines 218–294 and 376–422](../../../../vendor/mppi_generic_vendor/include/mppi/sampling_distributions/colored_noise/colored_noise.cu#L376)).

Controls are stored `[rollout][time][control]`. The weighted reduction uses `sum_strides=32`: at a fixed loop iteration adjacent reduction threads access rollouts 32 apart. With 160 floats per rollout, their control loads are **20,480 bytes apart**. Within one thread, successive rollout controls are 640 bytes apart. See [reduction, lines 1448–1472](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L1448) and [interface configuration, line 1169](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L1169). This is poor contiguous access at the instruction level, although caches and reuse across time blocks may reduce actual DRAM traffic.

**Consequence:** sampling and mean update may consume bandwidth and launch time that does not improve the rollout model or objective. The reduction also reuses the same weights across time blocks.

**Direction and tradeoffs:** first fuse reorder/normalization/offset handling with the nominal-or-pure-noise rewrite, preserving the nominal rollout, 1% pure-noise population, per-iteration decay and first-command variance. Then compare tiled reduction or a transposed scratch view against a wholesale storage change. A new layout may help reduction while hurting rollout consumption or cuFFT output conversion; evaluate the whole iteration. Merely reducing FFT length or replacing colored noise with white/IIR noise changes covariance and can affect control quality, so it belongs to a separately evaluated algorithm experiment.

**Dependencies / effort / risk:** sampler statistical regressions, both control dimensions, stride/delay cases and memory transaction measurements; medium effort, medium risk. Relevant rows: SMP-01–05, REF-04, WGT-03. The existing persistent cuFFT plan should be retained.

### 5. Tune rollout execution using representative scenes and measured resource use

**Confirmed selection limitation; winning kernel unknown.** The constructor chooses combined versus split kernels before the interface has uploaded the current scene. Selection queries device **0**, uses `sharedMemPerBlock`, includes the visualization kernel's shared-memory requirement when judging the combined path, and measures combined then split with synchronized calls ([controller, lines 65–163](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L65)). The interface sets launch dimensions before construction, correctly avoiding the vendor's trivial default dimensions ([I, lines 1189–1195](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L1189)).

The split path writes roughly 65 MiB of model outputs and reads them for cost evaluation each iteration; the combined path avoids that materialization but carries dynamics and cost state together. Split cost uses 80-thread blocks; dynamics uses `(32,2,1)`. The model has 27 state entries, including delay queues, and several state operations execute only in the `threadIdx.y == 0` lane. The device step contains three block barriers; `computeDynamics` calculates a local derivative array per participating thread before partitioned stores ([model, lines 323–350](include/mppi/dynamics/dubins/first_order_dubins_bicycle.cu#L323)). These are tuning candidates, not proof that barriers or the second lane can be deleted safely.

The package also globally caps registers at 64 ([CMake, lines 177–184](CMakeLists.txt#L177)). That may improve residency or introduce spills; source inspection cannot determine which. CUDA-array fetch latency, exhaustive projection, exact near-collision geometry and register pressure may change the preferred kernel by scene.

**Direction and tradeoffs:** expose/report kernel choice and benchmark both paths after representative scene upload, or use an offline device/profile-specific selection. Query the current CUDA device and account for actual supported opt-in shared memory separately from disabled visualization. Sweep a small set of valid block shapes and register caps; inspect compiled register/spill counts, occupancy, barrier stalls and global transactions. Consider a specialized one-thread-per-rollout dynamics implementation only after measuring the current cooperative model. Preserve delay queues, state-dependent constraints and all synchronization dependencies.

**Dependencies / effort / risk:** profiling on deployment GPU(s), split/combined parity including near collisions; medium effort, medium/high risk for model/kernel restructuring. Relevant rows: REF-01, REF-03–07, OBJ-01, OBJ-09. Portable architecture selection is a build concern; compiling fewer GPU architectures shortens builds but does not itself speed the selected device kernel.

### 6. Remove avoidable synchronization outside the iteration loop, then assess graph replay

**Confirmed host synchronization sites; impact depends on outstanding work.** The normal controller loop does not download per-iteration costs or means unless raw iteration diagnostics are enabled. It queues the statistics and final mean before one final synchronization ([controller, lines 304–359](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L304)). This should not be described as an unfixed per-iteration host-copy problem.

Outside that loop, `syncDelayStepsToModel` and `syncKinematicLimitsToModel` call `model.setParams` ([I, lines 1284–1314](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L1284)). The vendor setter invokes `paramsToDevice()` with its default `synchronize=true`, uploading parameters/ranges and synchronizing ([declaration, lines 178–184 and 212](../../../../vendor/mppi_generic_vendor/include/mppi/dynamics/dynamics.cuh#L178), [implementation, lines 4–15](../../../../vendor/mppi_generic_vendor/include/mppi/dynamics/dynamics.cu#L4)). The interface supplies no explicit controller stream, so the default execution stream is used ([I, line 1195](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L1195), [vendor constructor, lines 191–211](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/controller.cuh#L191)). Default-stream coupling with other same-process CUDA users is a deployment hypothesis requiring a trace.

Launch wrappers also call `cudaFuncSetAttribute` for dynamics/combined kernels on each launch despite fixed shapes ([kernel wrappers, lines 1627–1667](../../../../vendor/mppi_generic_vendor/include/mppi/core/mppi_common.cu#L1627)).

**Direction and tradeoffs:** combine model updates, upload only dirty parameters, and stage asynchronous copies in storage with a documented lifetime. Configure kernel attributes once per device/configuration. Consider an owned nonblocking execution stream with explicit dependencies and teardown ordering. After measuring host launch gaps, evaluate graph replay of a steady-state iteration/cycle; cuRAND state, cuFFT capture support, library versions, debug mode, pointer lifetime and runtime reconfiguration all need verification. A graph does not reduce the arithmetic or memory volume of the expensive kernels. Do not add streams without preserving upload → map → rollout → weight → control dependencies.

**Dependencies / effort / risk:** Nsight Systems trace and stream/resource ownership audit; small for dirty-update/attribute cleanup, medium/large for stream/graph changes, medium concurrency risk. Relevant rows: INT-13–14, REF-04, PAR-05.

### 7. Treat rollout visualization and logging as separate performance modes

**Confirmed overhead when enabled; not normal-loop overhead.** Per-iteration debug downloads the entire 16,384-float cost array and synchronizes in the vendor callback path, then copies 128 complete control sequences and synchronizes again in the interface ([controller, lines 304–313](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L304), [I, lines 385–407](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L385)). That is about **144 KiB and two synchronization points per iteration**, plus host reconstruction of up to 128 trajectories per iteration. At 10 iterations, roughly 1,280 debug rollouts are replayed on the CPU ([I, lines 832–855](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L832)). The plugin profile leaves iteration debug disabled.

The separate full visualization path selects up to 256 best and 128 worst rollouts, then calls synchronous `cudaMemcpy` once per selected rollout: up to **384 copies of 640 bytes** ([I, lines 65–68](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L65), [659–677](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L659), [800–828](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L800)). This path must not be confused with the already-contiguous 128-rollout iteration capture. CSV logging additionally opens/writes many per-frame files synchronously ([logger, lines 309–375](include/autoware/mppi_optimizer/mppi_debug_trajectory_logger.hpp#L309)).

**Consequence:** enabling diagnostics can materially change scheduling and mask the behavior being investigated; low transfer byte counts do not imply low synchronization overhead.

**Direction and tradeoffs:** gather selected rollout controls on device into one contiguous staging buffer, copy once, and capture per-iteration snapshots into a bounded device buffer for a final asynchronous download. Decimate captures independently from optimization. Move serialization and marker reconstruction to a bounded worker queue with an explicit drop policy; retain cycle IDs, configuration and scene ownership so delayed diagnostics cannot describe a different cycle. More buffering consumes memory, and diagnostic results arrive later. Keep failure counts/reasons available on the normal path.

**Dependencies / effort / risk:** separately profile each debug mode, immutable snapshots and queue lifetime; small/medium for gathered copies, medium for asynchronous diagnostics, low/medium risk. Relevant rows: DBG-01–07, INT-13. Do not alter control or execution history to accommodate visualization.

### 8. Reduce CPU critical-path work before attributing all cycle time to the GPU

**Confirmed redundant replay and optional host costs.** The vendor smooths and projects the control sequence, then computes its state trajectory; the interface discards that smoothed sequence, restores the already-downloaded raw device mean, computes states, constrains controls and computes states again ([controller, lines 432–450](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L432), [I, lines 316–329](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L316)). Generic postprocessing and velocity-profile application can cause further replay ([I, lines 2398–2450](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L2398)). The interface comment mentioning a vendor “zero-state constraint pass” is stale: the current vendor uses an evolving state. The duplicate/discarded work remains real, but that old correctness explanation should not motivate a new change.

Vegetation is another host boundary: a full 1,024² distance field is allocated/computed by CPU row/column transforms and copied into staging before a 4 MiB H2D upload and channel-write kernel ([map code, lines 53–82](include/mppi/cost_functions/dubins/first_order_dubins_distance_map.cu#L53)). The later exact-check fix also sorts and uploads local point positions when vegetation changes; near-contact GPU rollout checks search that list by x before testing the inflated footprint. Its cost depends on point density and broad-phase hit rate. Vegetation is optional and disabled in the shipped profile; avoid charging it to every cycle. CPU final validation scans geometry analytically and can add up to 79 × 99 = **7,821 intermediate poses** at interpolation setting 100 for an 80-point output ([validator, lines 28–87](include/autoware/mppi_optimizer/detail/trajectory_validator.hpp#L28)).

Selected-output cost reconstruction is also conditional on CSV logging ([I, lines 3354–3356](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L3354)). The interface formats a detailed cost string before its INFO call, and records `total_ms` before that log call, fallback handling and plugin publication ([I, lines 3419–3466](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L3419)). Thus that timing is neither pure GPU time nor complete publication latency.

**Direction and tradeoffs:** add a vendor finalization policy/hook so this interface can request the unsmoothed device result and perform one authoritative constrained replay, while retaining default vendor behavior for other clients. Reuse vegetation scratch memory; consider GPU EDT only if enabled deployments show a host bottleneck. Guard expensive log formatting by the configured log level and measure validation separately. An exact CPU spatial index may help validation before a GPU rewrite; moving small final-validation workloads to the GPU adds transfer/synchronization and common-mode implementation risk.

**Dependencies / effort / risk:** host phase timings and unchanged final controls/states; small/medium for replay/logging cleanup, medium/large for EDT/validation redesign, medium safety risk. Relevant rows: OUT-08–10, VAL-01–04, DBG-03, SCN-12. Interpolation is an accepted optional check; reducing it for speed is a product setting decision, not an invisible optimization.

### 9. Avoid spending the remaining iteration budget after an unrecoverable cycle failure

**Confirmed wasted work on a conditional path.** Any iteration with no eligible rollout or invalid normalization rejects the entire cycle, even if later iterations recover. However, that decision is checked only after all iterations have completed and their statistics are downloaded ([controller, lines 276–359 and 362–377](../../../../vendor/mppi_generic_vendor/include/mppi/controllers/MPPI/mppi_controller.cu#L362)). Under the present policy, later work cannot make the candidate publishable.

**Consequence:** difficult or impossible scenes can incur nearly a full optimization cost before returning the accepted upstream fallback. Such cases can matter more to tail latency than free-space means.

**Direction and tradeoffs:** add a device failure latch so subsequent rollout/weight/update work can safely become a no-op while preserving the first failure and restoring the seed. An ordinary early return must not strand other threads at a barrier. Library cuRAND/cuFFT work cannot be assumed to obey a custom latch; avoiding those launches may require a host check, supported conditional graph execution, or another design. Compare their cost against the benefit before adding per-iteration synchronization. Define what subsequent debug snapshots and RNG advancement should mean. Per-rollout early termination after collision is a separate optimization: it changes raw unsafe-cost diagnostics and split-kernel opportunities and needs its own contract.

**Dependencies / effort / risk:** failure-heavy profiles, DBG diagnostics requirements, WGT-07/VAL-06 preserved; medium/large effort and medium/high control-flow risk. It must not silently turn “any failed iteration rejects” into “last iteration decides,” or change whether unsafe rollouts are eligible.

### 10. Make computational budgets explicit before scaling capacity or changing optimizer quality

**Confirmed fixed dimensions; optional policy-dependent optimization.** The horizon and rollout count are compile-time constants; iterations are configurable. Hypothesis capacity is still 64, and every tied highest-confidence path consumes a slot ([I, lines 62–65](src/first_order_dubins/first_order_dubins_mppi_interface.cu#L62), [prediction selection/count, lines 44–85](include/autoware/mppi_optimizer/predicted_objects_obstacles.hpp#L44)). Increasing that limit increases geometry buffers, per-block map staging, map-construction loops and exact-collision work. It is not simply a harmless host-vector resize.

Near obstacles or map edges, rollout safety calls exact geometry after a conservative distance-field broad phase ([cost, lines 1538–1596](include/mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cu#L1538)). More unsafe samples, more tied paths, changed first-command exploration or out-of-map poses can increase this work; the size of that effect is unknown without counters. Never remove exact checks merely because the texture generally agrees with them.

**Direction and tradeoffs:** define deployment deadline, target GPU, allowable diagnostic load and required hypothesis coverage. If exact implementation optimizations are insufficient, evaluate a small set of rollout-count/iteration profiles and objective-quality gates. Adaptive stopping or smaller populations can reduce cost but change convergence, risk of no eligible solutions, and the quality of first controls. Keep the accepted 80-step/0.1 s time contract unless separately approved. Capacity expansion should follow spatial/map acceleration work and a storage design with explicit overflow behavior. Primary-only processing remains the current scope; allocating a separate controller per secondary candidate is not part of this plan.

**Dependencies / effort / risk:** a quality/latency evaluation plan and decisions on SCN-04; medium to large effort, high behavioral risk if sample budgets change. Relevant rows: INT-02, SCN-04/08, REF-01, SMP-05, WGT-01–07. Exact sampling/geometry optimizations can proceed before those policy decisions.

## Measurement plan and missing performance cases

The standalone harness now provides optional host timings, CUDA events, map-rebuild counts, and separate split dynamics/cost ranges. Events are collected after the final synchronization. The measured next step is the focused projection optimization above; the remaining cases below describe broader coverage needs. Do not insert a synchronization after every measured phase into the production baseline. CPU launch-range duration is not GPU kernel duration.

Record end-to-end time from the primary candidate's processing entry to selected output/publication, together with scene preparation, map build, each sampling/rollout/weight/reduction phase, download wait, host replay, validation and debug serialization. Separate first initialization, parameter-reset cycles, normal warm cycles and failure/fallback cycles. Record active GPU, clock/power state, build flags, vendor/package revisions, kernel choice, profile, iteration count, seed, debug options and concurrent same-process GPU clients. Report distributions and deadline misses; an average alone is insufficient.

The original coverage matrix below is partly addressed by the new harness: stationary-object counts/geometry updates, cached straight/hairpin cases, snapped-grid crossings, and no-eligible-rollout rejection are covered. Moving/tied predictions, deployment profiles, closed-loop history and the other dimensions remain incomplete; see the [current scenario inventory](test/PERFORMANCE_BENCHMARKS.md#scenarios-and-workload-checks).

| Missing or weakly represented case                                                       | What it distinguishes                                                                             |
| ---------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| Real product vehicle and parameter profile, 10 iterations, ROS plugin path               | Synthetic interface latency versus deployment latency and selected-output quality                 |
| Advancing timestamps and changing object poses/predictions each cycle                    | Obstacle map rebuild cost hidden by identical-input caching                                       |
| 1/16/64 hypotheses; all tied paths; 65-hypothesis rejection                              | Map/collision scaling and explicit capacity failure; count paths separately from physical objects |
| Static-only, moving-only, mixed, and dynamic cutoff transitions                          | Potential static-layer reuse and time-tail savings                                                |
| Straight/hairpin/self-near/degenerate corridors, 2/80/256 points                         | Exact projection scaling, tie behavior and acceleration-structure correctness                     |
| Snapped-grid boundary crossing, local-frame recenter, out-of-map rollouts                | Cache invalidation bursts and exact-fallback frequency                                            |
| Dense near-contact scenes, zero soft weights with hard checks enabled                    | Broad-phase hit rate, exact geometry cost and safety coverage                                     |
| All eligible, sparse eligible, no eligible, nonfinite and equal-cost populations         | Single-block weighting behavior and failed-iteration tail latency                                 |
| Debug off; iteration capture; full rollout visualization; CSV; texture viewer separately | Diagnostic synchronization and CPU overhead                                                       |
| Vegetation absent/present/changing; validation substeps 1/4/100                          | CPU EDT/upload and validation scaling                                                             |
| Split/combined kernels, block shapes, register caps, target GPU and co-running workloads | Occupancy, spills, memory transactions, default-stream interference and deployment portability    |

The historical benchmark's `completedOptimization` helper did not require `validation.isValid()` or successful plugin application. The current harness requires validated, unrejected output in successful cases and the expected rejection in its infeasible case, but does not exercise closed-loop/plugin application. Neither harness has a universal latency regression threshold. Preserve functional/parity tests and stable workload counters; use hardware-specific performance baselines. Reducing sample counts or changing geometry semantics requires a separate quality decision.

Use a timeline profiler first to identify GPU/CPU gaps and the largest phases, then kernel counters on the identified kernels. Counter collection changes execution timing; retain an unprofiled baseline. In particular, measure map rebuild counts, projection segment evaluations, exact-collision fallback counts, hypothesis count, eligible fraction, and debug copy counts alongside time. Avoid interpreting the current INFO `total_ms` as either kernel time or the complete scheduling budget.

## Practical sequence and decisions

1. Retain the measured RTX 4090 synthetic baseline and phase/counter instrumentation. Extend to the deployment GPU/product configuration when available; that broader evaluation does not block the identified projection work.
2. Implement one focused exact path-projection optimization, starting with invariant segment precomputation and independent parity/cache-invalidation tests. Have the user compare the full benchmark matrix and the three profiled scenarios before proceeding to more complex exact spatial acceleration.
3. Re-rank the remaining measured costs. Obstacle-data access/exact-collision work, host finalization, map reuse/tiling and weighting are separate changes; measure each independently and preserve behavior before combining them.
4. Tune buffer layout, fused sampling and rollout launch/resource choices against the improved kernels. Their preferred configurations may change after steps 2–3.
5. Evaluate owned streams, graph replay and failure-latch execution once buffers and launch structure are stable. Preserve reset, teardown, debug, failure and history semantics.
6. Only then consider higher capacity or changed rollout/iteration budgets, with explicit quality criteria. Do not promise a latency gain until measured on representative target hardware.

The few decisions that materially change this sequence are: **the deployment latency/GPU budget**, **whether 64 hypotheses must be exceeded while retaining every highest-confidence tie**, **which diagnostics must be available every cycle**, and **whether changes to weighting/eligibility or optimizer sampling budgets are authorized**. Prediction-age policy (SCN-07) affects representative scene preparation; engage/final-output policy (OUT-06, VAL-11) affects the output-quality gate. None of these requires postponing profiling or exact implementation optimizations.

## Claims this review does not carry forward

- No normal per-iteration host weight/mean transfer remains to remove; that optimization already exists.
- cuFFT plans and main CUDA buffers are not allocated afresh every iteration.
- Cost setters do not independently rebuild every map during a batched scene update.
- Empty obstacle scenes do not run the dense obstacle-map kernel.
- The local coordinate origin does not move with every ego movement.
- A nearest-segment texture does not currently eliminate exhaustive corridor scanning.
- Zero soft weights do not authorize disabling hard safety geometry.
- Final validation cannot replace rollout eligibility, and collision-free weighted contributors do not guarantee a collision-free mean trajectory.
- Passing the current synthetic validation gates does not demonstrate deployment deadlines, closed-loop safety, or a measured speedup for an unimplemented proposal.
