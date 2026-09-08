# Cost tests and reports

`test_costs.cu` exercises cost behavior directly, with deterministic inputs. It does not run the
optimizer, sample noise, or measure performance. Production cost and dynamics code are unchanged.

## Running

After building the package, run the entire executable to collect one report directory:

```sh
MPPI_REPORT=1 MPPI_REPORT_DIR=/tmp/mppi_cost_reports \
  build/autoware_mppi_optimizer/test_costs
```

The executable prints the unique `run_*` directory it creates. Use that exact directory:

```sh
python3 src/autoware/universe/planning/autoware_mppi_optimizer/scripts/generate_cost_report.py \
  --input-dir /tmp/mppi_cost_reports/run_REPLACE_WITH_PRINTED_NAME \
  --output-dir /tmp/mppi_cost_report
```

The output is `MPPI_Cost_Report.md` with standalone PNG figures. Plotting requires matplotlib;
`--no-plots` produces validated tables using only the Python standard library. The script exits
nonzero for failed assertions, malformed/incomplete exports, or plot failures. It still writes a
report describing those failures. Legacy `/tmp/mppi_report_*.csv` files are not schema-compatible.

Without `MPPI_REPORT=1`, all assertions still execute; no CSV files are written. `MPPI_REPORT=0`
explicitly disables export. Each process creates a separate run directory; tests running under
CTest therefore produce separate host and GPU runs rather than overwriting or mixing data.

CTest registers `test_costs` for host analytical tests and `test_costs_gpu` for CUDA parity. Both
have the `cost` label; only parity has the `gpu` label. The host tests still require a CUDA-enabled
build and its shared libraries, but never initialize a CUDA device. The GPU fixture skips when
no device is available. To select manually:

```sh
build/autoware_mppi_optimizer/test_costs --gtest_filter='-GpuCostEvaluation.*'
build/autoware_mppi_optimizer/test_costs --gtest_filter='GpuCostEvaluation.*'
python3 src/autoware/universe/planning/autoware_mppi_optimizer/test/test_generate_cost_report.py
```

The parser regressions also run as the CTest entry `test_cost_report_parser`, without CUDA or
matplotlib. No generated reports are committed to the repository.

## Coverage and independent expectations

| Group                          | Cases and assertions                                                                                                                                                                                                             |
| ------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Isolated quadratics            | Zero inputs, positive/negative inputs, disabled and scaled weights for tracking, heading, center tracking, lateral distance/yaw, command magnitudes, physical rates, command rates, initial steering and terminal errors         |
| Reference geometry             | Explicit corridor, interpolated reference speed and spatial progress, missing/degenerate corridor, remaining length and overshoot, heading wraparound, footprint-center rotation, independent terminal target and terminal scale |
| Boundary geometry              | Clearance sweeps across safe margin/contact, disabled weights, empty geometry, obstacle collision margin, rotated boxes, predicted moving-obstacle samples, lateral threshold and crash flag                                     |
| Kinematic limits               | Lower/upper bounds, inside-bound deadband, normalization, coefficient scaling, disabled masks, pointwise speed override and aggregate safety cap                                                                                 |
| Deterministic 80-step rollouts | Straight, circular turn entry, steady turn, lateral offset, stop/start, delayed command step, alternating commands and tight corridor                                                                                            |
| Physical comfort               | Actual acceleration/steering increments, steering-rate limit, actuator saturation, constant-turn inertial jerk, delayed-command response and first-command omission                                                              |
| CPU/GPU parity                 | Component weights, running/terminal totals and crash flags, texture geometry/fallback, moving-obstacle texture slices and full delayed steady-turn replay with a command step                                                    |
| Accounting                     | Sum of all 27 components, direct running-cost agreement, terminal counted once, and `(sum running + terminal)/H`                                                                                                                 |

Each fixture disables every registered coefficient before enabling the terms under test. Numerical
expectations use simple independent geometry/formulas instead of calling the cost function to
produce its own oracle. `cost.<component>` assertions populate the analytical coverage table;
positive expected values are counted separately so zero-only checks cannot imply meaningful coverage.
CPU/GPU agreement is additional evidence, not an independent mathematical oracle. The isolated
sample inputs intentionally need not represent reachable vehicle states.

The circular reference uses the closed-form orbit of the implemented forward-Euler position update,
not a continuous-circle approximation and not a second call to `model.step()`. Both model and cost
use a 2 m wheelbase. Full rollouts apply the same state-aware control constraints before stepping.
The GPU replay uses two Y workers with explicit barriers around shared-buffer reuse.

`SteadyTurn` starts with the actuator already at the constant 0.2 rad command. `Circular` starts
at zero steering and commands 0.2 rad against the same circular reference. It checks the entry
transient against a closed-form response: four rate-limited 0.04 rad increments, followed by
exponential convergence. Its nonzero initial steering penalty, physical steering rate, lateral
jerk and tracking offset are expected; longitudinal jerk and command-change costs remain zero.

These tests validate the **current cost contract**. They do not establish closed-loop controller
stability, prove optimality, or exercise the ROS tracked-object time conversion. The known moving
obstacle arrays test the cost's supplied stage index; wrapper timing retains its separate tests.

## Timing and geometry conventions

- Stage `t` costs the post-step state `x[t+1]` at `(t+1)*dt`, with `dt=0.1 s`. Terminal state
  `x[H]` is a separate row at `H*dt`. The optional initial row adds no cost.
  Control plots place `u[t]` at its issue time `t*dt` and omit the initial row's placeholder
  controls, so a steady turn does not acquire an artificial zero-to-command jump in the figure.
  Command-rate plots use that same issue time. Physical outputs and stage costs retain post-step
  timestamps; total/component cost plots exclude the initial row's placeholder zero cost.
- Command-change costs omit stage zero because previous issued commands may be unknown. The raw
  stage-zero command-rate output is therefore left blank in CSVs and plots. Subsequent rates
  compare consecutive issued commands, independent of the delay queue.
- Initial steering uses its own measured-angle anchor only at stage zero. Steady-turn cases leave
  `PREVIOUS_*` state fields at zero deliberately and verify that this creates no command-change cost.
- Physical longitudinal jerk and steering rate use realized actuator increments over `dt`.
  A newly queued command contributes no physical jerk before its delay elapses. Lateral jerk is
  the vehicle-frame component of inertial jerk, `v²*kappa_dot + 3*v*a*kappa`, evaluated at the
  pre-step state with realized rates.
- Soft obstacle/drivable clearances use four circles covering the ego rectangle. A 4 m by 2 m
  rectangle gives circle radius `sqrt(1.25)`; this differs from rectangular contact geometry.
  Road-border clearance clamps penetration at zero. Drivable-area clearance retains penetration;
  corner-buffer distance is unsigned. These behaviors have separate expected-value sweeps.
- Moving obstacles participate in the soft cost at their supplied predicted sample. Static flags
  describe reusable geometry and do not remove moving objects from the cost.
- Texture parity samples are away from ambiguous contact thresholds. For affine SDF faces, the
  combined geometry check derives its tolerance from the active map resolutions and texture
  interpolation quantization. CPU analytical contact checks use tighter tolerances. This does not
  claim an error bound for arbitrary curved SDFs or all texture-edge configurations.

## CSV schema 2

Every test exports six files with a common name. Floating values use sufficient precision for
round trips. Each reference, geometry and parameter snapshot is keyed by evaluation `row`, so a
parameter sweep cannot silently rewrite the inputs associated with an earlier result.

| File             | Contents                                                                                                                                                                                       |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `.steps.csv`     | Row kind/label, stage/time, physical outputs, issued controls, valid command-rate flag, crash flag, direct running total, breakdown total, all 27 weighted components and their weights        |
| `.checks.csv`    | Row, assertion name, actual value, independent expected value where applicable, tolerance, pass/fail                                                                                           |
| `.reference.csv` | Actual time-indexed reference, spatial corridor and separate terminal target for every row                                                                                                     |
| `.geometry.csv`  | Obstacle pose/half-extents/static flag and road/drivable segments at each evaluated stage                                                                                                      |
| `.params.csv`    | Cost geometry, margins, limits and initial steering; physical replay rows additionally contain model parameters and the complete pre-step state (including delay queues and previous commands) |
| `.meta.csv`      | Schema version, horizon, timestep, case name, GTest status, backend, and GPU/runtime/driver information for CUDA cases                                                                         |

Metadata is written last as a completion marker. The parser rejects missing sidecars, duplicate
keys, missing references, inconsistent sums, nonfinite values and invalid timestamps. The only
blank numeric fields allowed are time for independent samples and command rates when history is
not applicable. A failed or skipped test cannot become PASS through CSV aggregation.

`sample` rows and terminal parameter sweeps are independent evaluations. Their sums are descriptive;
they have no normalized optimizer objective. A complete trajectory must contain exactly `H`
ordered `running` rows and one `terminal` row. The report keeps running and terminal sums separate
and normalizes their sum by `H`, not by the number of exported rows. This accounts for the trajectory
cost only; MPPI sampling/importance corrections are outside this fixture.

Figures distinguish issued controls from actuator states, plot physical jerk separately from
command changes, show the actual reference/corridor and terminal target, and overlay the first and
last geometry snapshots. The complete geometry history and parameter weights remain in the CSVs;
figures are not an animation of every parameter-sweep scene.
