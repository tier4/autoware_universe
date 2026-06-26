# TrajectoryValidatorDiagnostic

`TrajectoryValidatorDiagnostic` reads the `ValidationReport` array produced by the validator node every cycle, decides how severe the situation is, and publishes a set of named `DiagnosticStatus` values that a downstream diagnostic graph can read to choose a safety action.

---

## Concepts

### MetricReport level

Each validator plugin rates every candidate trajectory and writes one `MetricReport` per (candidate, validator) pair.
The `MetricReport.level` field uses three values:

| `MetricReport.level` | Meaning                              |
| -------------------- | ------------------------------------ |
| `OK`                 | trajectory passes this check         |
| `WARN`               | trajectory passes but with a concern |
| `ERROR`              | trajectory fails this check          |

### Action enum

Internally, `TrajectoryValidatorDiagnostic` works with a four-step `Action` enum.
`to_action()` translates a `MetricReport.level` into an `Action`:

| `MetricReport.level` | → `Action` |
| -------------------- | ---------- |
| `OK`                 | `NONE`     |
| `WARN`               | `NONE`     |
| `ERROR`              | `MODERATE` |

**`WARN` maps to `NONE`, the same as `OK`.** A candidate whose worst validator level is `WARN` is treated identically to a fully-clean candidate: it beats any candidate that has an `ERROR`.

`COMFORTABLE` and `EMERGENCY` are reserved for future use — no validator currently produces them.
`to_action()` in `risk_action.hpp` is the **only** place these constants are named, so updating the scale means editing one function.

### DiagnosticStatus published level

Each `Action` maps to a `DiagnosticStatus` level:

| `Action`      | published level |
| ------------- | --------------- |
| `NONE`        | `OK`            |
| `COMFORTABLE` | `WARN`          |
| `MODERATE`    | `ERROR`         |
| `EMERGENCY`   | `ERROR`         |

---

## How a cycle works

The class is **stateless across cycles** — every cycle it recomputes from scratch and re-publishes everything.

```text
ValidationReport[]  (one entry per candidate trajectory)
        │
        ▼
1. For each active validator, find its best (minimum) Action across all candidates
        │
        ▼
2. For each validator whose best Action > NONE:
   fire every binding whose action threshold ≤ the validator's best Action
   (a validator with both MODERATE and EMERGENCY bindings fires both when its best Action
   reaches EMERGENCY, but only the MODERATE binding when its best Action is MODERATE)
        │
        ▼
3. For each shadow-mode validator with a bindings entry,
   mark its bound status names at WARN (unless already marked active in step 2)
        │
        ▼
4. Publish ALL preset statuses every cycle:
   - active ones at their published level (WARN or ERROR)
   - inactive ones at OK
```

**Why publish everything every cycle?** A `DiagnosticStatus` that stops being sent goes stale and the diagnostic graph may interpret the silence as a hardware fault. Sending inactive statuses at `OK` prevents false alarms.

**Why per-validator minimum?** Each validator is evaluated independently. If a validator passes on at least one candidate, it does not fire — there is a safe trajectory available for that check. Only validators that fail on every candidate raise a diagnostic.

**WARN vs ERROR example.** Two candidates — A has one validator at `WARN`, B has one validator at `ERROR`:

| Candidate | validator_x level | Action                     |
| --------- | ----------------- | -------------------------- |
| A         | `WARN`            | `NONE` (WARN maps to NONE) |
| B         | `ERROR`           | `MODERATE`                 |

validator_x best action = `NONE` (minimum of `NONE` and `MODERATE`). No status fires — the validator passes on at least one candidate.

### Pattern reference

**Within one candidate — `std::max` (harshest metric governs)**

| Pattern      | Sequence                                               | Result    |
| ------------ | ------------------------------------------------------ | --------- |
| min only     | NONE                                                   | NONE      |
| max only     | EMERGENCY                                              | EMERGENCY |
| min then max | emplace NONE → `max(NONE, EMERGENCY)` = EMERGENCY      | EMERGENCY |
| max then min | emplace EMERGENCY → `max(EMERGENCY, NONE)` = EMERGENCY | EMERGENCY |

Order does not matter — `std::max` is commutative.

**Across candidates — `std::min` (safest outcome governs)**

| Pattern              | Sequence                                          | Result    |
| -------------------- | ------------------------------------------------- | --------- |
| min only             | NONE                                              | NONE      |
| max only             | EMERGENCY                                         | EMERGENCY |
| min first, max later | emplace NONE → `min(NONE, EMERGENCY)` = NONE      | NONE      |
| max first, min later | emplace EMERGENCY → `min(EMERGENCY, NONE)` = NONE | NONE      |

Order does not matter — `std::min` is commutative.

#### Combined two-stage example

```text
Candidate A: metric at ERROR  →  candidate_action = MODERATE  (max within A)
Candidate B: metric at WARN   →  candidate_action = NONE      (max within B, WARN maps to NONE)

validator_actions after A: MODERATE
validator_actions after B: min(MODERATE, NONE) = NONE
```

No binding fires — there is a safe candidate available (B).

### No candidate trajectories

When the `ValidationReport` array is empty (the generator produced no candidates), the class publishes the `no_candidate_name` status at `ERROR`. This name is fixed as `"trajectory_validator_no_candidate_trajectory"` in `TrajectoryValidatorWrapper`.

### Shadow-mode validators

Validators loaded via `shadow_mode_filter_names` are excluded from the Action aggregation entirely. Their `MetricReport` entries still appear in the published `ValidationReportArray` topic (for debug), but they do not influence which trajectory is chosen as best.

Shadow validators with entries in the `bindings` parameter have all their bound status names published at `WARN` every cycle. This signals to the diagnostic graph that the check is running in monitoring-only mode. An active validator whose action already set a status to `ERROR` is never downgraded — `mark_shadow_statuses` only fills in status names that were not already marked by the active-validator pass.

The wrapper identifies shadow validators by calling `plugin->get_name()` on plugins with `is_shadow_mode() == true` and passing the resulting short names as `active_filter_names` to the class. Any validator name **not** in `active_filter_names` is treated as shadow.

---

## Bindings parameter

The mapping from `(validator_name, Action)` to a status name is configured in:

```text
config/trajectory_validator_diagnostic.param.yaml
```

```yaml
trajectory_validator_diagnostic:
  bindings:
    - uncrossable_boundary_departure_filter:moderate:trajectory_validator_uncrossable_boundary_departure_danger
```

Each entry is a colon-separated triple:

```text
<validator_short_name>:<action>:<diagnostic_status_name>
```

| Field                    | Description                                                                                                  |
| ------------------------ | ------------------------------------------------------------------------------------------------------------ |
| `validator_short_name`   | the short name returned by `plugin->get_name()` — lowercase snake_case                                       |
| `action`                 | one of `none`, `comfortable`, `moderate`, `emergency`                                                        |
| `diagnostic_status_name` | the name published in `DiagnosticStatus.name` (without the node-name prefix added by `DiagnosticsInterface`) |

The class creates one `DiagnosticsInterface` publisher for each distinct `diagnostic_status_name` found in the bindings, plus one for `no_candidate_name`. This preset is built once at startup and never changes at runtime.

### Finding the validator short name

The short name is what `plugin->get_name()` returns, **not** the plugin class name used in `filter_names` / `shadow_mode_filter_names`. The two differ in naming convention:

| Parameter value                                | `get_name()` value                        |
| ---------------------------------------------- | ----------------------------------------- |
| `"safety::UncrossableBoundaryDepartureFilter"` | `"uncrossable_boundary_departure_filter"` |

To find the short name for any plugin, either check its `getName()` / `get_name()` implementation or run the node and look at the `ValidationReportArray` topic — the `validator_name` field in each `MetricReport` is the short name.

---

## How to add a new diagnostic

**Step 1 — identify the validator short name and the action level.**

Run the node (or check the plugin source) to find the short name. Decide which `Action` level should fire the new status. Today only `moderate` is in use.

**Step 2 — choose a diagnostic status name.**

Pick a descriptive name that the diagnostic graph will reference. Convention: `trajectory_validator_<filter>_<tier>`, e.g. `trajectory_validator_collision_check_danger`.

**Step 3 — add a line to `trajectory_validator_diagnostic.param.yaml`.**

```yaml
trajectory_validator_diagnostic:
  bindings:
    - uncrossable_boundary_departure_filter:moderate:trajectory_validator_uncrossable_boundary_departure_danger
    - collision_check_filter:moderate:trajectory_validator_collision_check_danger
```

No code change is needed. The class reads `bindings` at startup and creates the publisher automatically.

**Step 4 — wire the new name in the diagnostic graph.**

The full status name seen on `/diagnostics` is `"<node_name>: <diagnostic_status_name>"`. Add a node in the diagnostic graph that listens to this full name and maps it to the intended safety action.

---

## Data-flow diagram

```text
validator plugins
    │  MetricReport { validator_name, level }
    ▼
ValidationReport[]  ──────────────────────────────────────────────┐
    │                                                              │
    │  TrajectoryValidatorDiagnostic.update_and_publish()          │  (published as
    │                                                              │   ValidationReportArray
    │  1. compute per-candidate worst Action (skip shadow names)   │   topic — unaffected
    │  2. pick best candidate (lowest Action)                      │   by this class)
    │  3. find binding validators on best candidate                │
    │  4. resolve (validator, Action) → status_name via bindings   │
    │  5. publish all preset DiagnosticsInterface, active or OK    │
    │                                                              │
    ▼                                                              │
/diagnostics  ◄────────────────────────────────────────────────────┘
    │
    ▼
diagnostic graph → chooses safety action (comfortable / moderate / emergency stop)
```

The validator report published on the ROS topic is **not modified** by this class — it always reflects what every plugin (including shadow-mode ones) produced. The diagnostic only looks at the report; it does not filter it.
