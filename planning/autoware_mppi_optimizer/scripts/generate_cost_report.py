#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Validate one cost-test run and write a Markdown report with standalone PNG plots."""

import argparse
import csv
from dataclasses import dataclass
from dataclasses import field
import math
from pathlib import Path
import sys

# Schema version 2: keep in sync with test/cost_test_report.hpp::components.
COMPONENTS = (
    "spatial_overspeed track heading terminal_error terminal_heading lateral_distance "
    "lateral_boundary lateral_yaw_error remaining_distance path_overshoot track_center "
    "corner_buffer drivable_area obstacle road_border acceleration_command steering_command "
    "lateral_acceleration lateral_jerk longitudinal_jerk steering_rate initial_steering_rate "
    "acceleration_command_rate steering_command_rate kinematic_velocity_overlimit "
    "kinematic_acceleration_overlimit kinematic_jerk_overlimit"
).split()
STEP_FIELDS = (
    "row kind label stage time_s command_rate_valid x y yaw v speed steering acceleration "
    "steer_cmd accel_cmd physical_steer_rate physical_lateral_jerk physical_longitudinal_jerk "
    "accel_command_rate steer_command_rate crash direct_available direct_total total"
).split() + [prefix + name for name in COMPONENTS for prefix in ("cost_", "weight_")]
HEADERS = {
    "steps": STEP_FIELDS,
    "checks": "row check actual expected tolerance passed".split(),
    "reference": "row kind index x y yaw v".split(),
    "geometry": "row kind index x0 y0 x1 y1 yaw half_length half_width is_static".split(),
    "params": "row parameter value".split(),
    "meta": ["key", "value"],
}


class InvalidReport(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise InvalidReport(message)


def read_table(path, header):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream, strict=True)
        require(reader.fieldnames == header, f"{path.name}: incorrect schema/header")
        rows = list(reader)
    require(
        all(None not in r and None not in r.values() for r in rows),
        f"{path.name}: truncated or extra CSV fields",
    )
    return rows


def finite(value, label):
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise InvalidReport(f"{label}: missing or invalid number {value!r}") from exc
    require(math.isfinite(result), f"{label}: nonfinite value {value!r}")
    return result


def integer(value, label, lower=0):
    result = finite(value, label)
    require(result.is_integer() and result >= lower, f"{label}: invalid integer {value!r}")
    return int(result)


def near(actual, expected):
    # Match the float component accumulation allowance in the C++ fixture.
    return abs(actual - expected) <= 1e-5 + 1e-5 * abs(expected)


@dataclass
class Case:
    name: str
    metadata: dict = field(default_factory=dict)
    rows: list = field(default_factory=list)
    checks: list = field(default_factory=list)
    references: list = field(default_factory=list)
    geometry: list = field(default_factory=list)
    parameters: list = field(default_factory=list)
    errors: list = field(default_factory=list)

    @property
    def status(self):
        if self.errors or any(not c["passed"] for c in self.checks):
            return "FAIL"
        return self.metadata.get("status", "FAIL")

    @property
    def complete_horizon(self):
        running = [r for r in self.rows if r["kind"] == "running"]
        terminal = [r for r in self.rows if r["kind"] == "terminal"]
        return (
            not self.errors
            and not any(r["kind"] == "sample" for r in self.rows)
            and len(running) == int(self.metadata.get("horizon", 0))
            and len(terminal) == 1
        )


def load_case(directory, name):
    """A malformed case stays visible as FAIL; it never disappears from coverage."""
    case = Case(name)
    try:
        meta = read_table(directory / f"{name}.meta.csv", HEADERS["meta"])
        case.metadata = {r["key"]: r["value"] for r in meta}
        require(len(meta) == len(case.metadata), "duplicate metadata key")
        require(case.metadata.get("schema_version") == "2", "expected schema_version=2")
        require(case.metadata.get("name") == name, "metadata name does not match filename")
        require(case.metadata.get("status") in {"PASS", "FAIL", "SKIPPED"}, "invalid test status")
        horizon = integer(case.metadata.get("horizon"), "horizon", 1)
        period = finite(case.metadata.get("dt"), "dt")
        require(period > 0, "dt must be positive")
        tables = {
            key: read_table(directory / f"{name}.{key}.csv", header)
            for key, header in HEADERS.items()
            if key != "meta"
        }
        for i, raw in enumerate(tables["steps"]):
            row = dict(raw)
            require(integer(row["row"], "row") == i, "noncontiguous row IDs")
            kind = row["kind"]
            require(kind in {"initial", "sample", "running", "terminal"}, "invalid row kind")
            row["row"] = i
            row["stage"] = integer(row["stage"], "stage")
            require(row["stage"] < horizon, "stage outside horizon")
            for key in STEP_FIELDS:
                if key not in {
                    "row",
                    "stage",
                    "kind",
                    "label",
                    "time_s",
                    "accel_command_rate",
                    "steer_command_rate",
                }:
                    row[key] = finite(row[key], f"row {i}: {key}")
            valid = kind not in {"initial", "terminal"} and row["stage"] > 0
            require(row["command_rate_valid"] == int(valid), "invalid command history flag")
            require(row["crash"] in (0, 1), "invalid crash flag")
            require(
                row["direct_available"] == int(kind in {"sample", "running"}),
                "invalid direct cost availability",
            )
            for key in ("accel_command_rate", "steer_command_rate"):
                if valid:
                    row[key] = finite(row[key], key)
                else:
                    require(row[key] == "", "unknown/terminal command rate must be blank")
                    row[key] = None
            if kind == "sample":
                require(row["time_s"] == "", "isolated samples must not imply elapsed time")
                row["time_s"] = None
            else:
                row["time_s"] = finite(row["time_s"], "time_s")
                expected_time = 0 if kind == "initial" else (row["stage"] + 1) * period
                require(near(row["time_s"], expected_time), "incorrect post-step timestamp")
            if kind == "terminal":
                require(row["stage"] == horizon - 1, "terminal must describe x[H]")
            costs = [row["cost_" + c] for c in COMPONENTS]
            require(all(c >= 0 for c in costs), "negative cost component")
            require(all(row["weight_" + c] >= 0 for c in COMPONENTS), "negative cost weight")
            require(near(math.fsum(costs), row["total"]), "component total mismatch")
            if row["direct_available"]:
                require(near(row["direct_total"], row["total"]), "direct/breakdown total mismatch")
            if kind == "initial":
                require(row["total"] == 0, "initial state must not add running cost")
            case.rows.append(row)
        count = len(case.rows)
        for raw in tables["checks"]:
            check = dict(raw)
            check["row"] = integer(check["row"], "check row")
            require(check["row"] < count, "check refers to a missing row")
            for key in ("actual", "expected", "tolerance", "passed"):
                check[key] = finite(check[key], "check " + key)
            require(check["tolerance"] >= 0, "negative check tolerance")
            passed = abs(check["actual"] - check["expected"]) <= check["tolerance"]
            require(check["passed"] == int(passed), "check result inconsistent with values")
            case.checks.append(check)
        for key, attribute in (
            ("reference", "references"),
            ("geometry", "geometry"),
            ("params", "parameters"),
        ):
            seen = set()
            for raw in tables[key]:
                item = dict(raw)
                item["row"] = integer(item["row"], key + " row")
                require(item["row"] < count, key + " refers to a missing row")
                for column in HEADERS[key]:
                    if column not in {"row", "kind", "parameter"}:
                        item[column] = finite(item[column], key + ": " + column)
                if "index" in item:
                    item["index"] = integer(item["index"], key + " index")
                identity = (
                    item["row"],
                    item.get("kind", ""),
                    item.get("index", item.get("parameter")),
                )
                require(identity not in seen, "duplicate " + key + " entry")
                seen.add(identity)
                if key == "reference":
                    require(
                        item["kind"] in {"reference", "corridor", "terminal"},
                        "invalid reference kind",
                    )
                elif key == "geometry":
                    require(
                        item["kind"] in {"obstacle", "road_border", "drivable_area"},
                        "invalid geometry kind",
                    )
                    require(
                        item["half_length"] >= 0
                        and item["half_width"] >= 0
                        and item["is_static"] in (0, 1),
                        "invalid obstacle geometry",
                    )
                getattr(case, attribute).append(item)
        for i in range(count):
            refs = [r for r in case.references if r["row"] == i]
            require(
                {r["index"] for r in refs if r["kind"] == "reference"} == set(range(horizon)),
                f"row {i}: missing reference samples",
            )
            require(
                len([r for r in refs if r["kind"] == "terminal"]) == 1,
                f"row {i}: missing terminal reference",
            )
            require(
                any(p["row"] == i and p["parameter"] == "wheel_base" for p in case.parameters),
                f"row {i}: missing cost parameters",
            )
        running = [r for r in case.rows if r["kind"] == "running"]
        if running:
            require(
                [r["stage"] for r in running] == list(range(horizon)),
                "incomplete/duplicate/out-of-order running horizon",
            )
            require(case.complete_horizon, "rollout requires exactly one terminal and no samples")
            kinds = [r["kind"] for r in case.rows]
            expected_kinds = ["running"] * horizon + ["terminal"]
            require(
                kinds == expected_kinds or kinds == ["initial"] + expected_kinds,
                "initial/running/terminal rows are out of order or duplicated",
            )
        if case.metadata["status"] != "SKIPPED":
            require(count > 0 and case.checks, "no recorded evaluations/assertions")
            require(
                {c["row"] for c in case.checks} == set(range(count)),
                "evaluation without assertions",
            )
    except (OSError, csv.Error, InvalidReport, ValueError) as exc:
        case.errors.append(str(exc))
    return case


def cost_totals(case, component="total"):
    """Terminal is separate and normalized only for one complete horizon, never for sweeps."""
    key = component if component == "total" else "cost_" + component
    running = math.fsum(r[key] for r in case.rows if r["kind"] == "running")
    terminal = math.fsum(r[key] for r in case.rows if r["kind"] == "terminal")
    samples = math.fsum(r[key] for r in case.rows if r["kind"] == "sample")
    objective = (
        (running + terminal) / int(case.metadata["horizon"]) if case.complete_horizon else None
    )
    return running, terminal, samples, objective


def plot_case(case, destination):
    # Import only when plotting. CSV validation and its unit tests need only the standard library.
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.patches import Polygon
    import matplotlib.pyplot as plt

    rows = [r for r in case.rows if r["kind"] != "terminal"]
    if not rows:
        rows = case.rows  # A terminal parameter sweep is a collection of isolated samples.
    continuous = case.complete_horizon
    abscissa = [r["time_s"] if continuous else r["row"] for r in rows]
    xlabel = "Time [s] (states post-step)" if continuous else "Evaluation row (independent samples)"
    fig, axes = plt.subplots(3, 3, figsize=(18, 13))
    axes = axes.ravel()
    fig.suptitle(f"{case.name} — {case.status}", fontsize=11)
    path = axes[0]
    path.plot(
        [r["x"] for r in rows], [r["y"] for r in rows], ".-" if continuous else ".", label="Ego"
    )
    first = rows[0]["row"]
    # Actual geometry for the first evaluation; moving/changed geometry is overlaid at the last.
    for kind, style in (("reference", "--"), ("corridor", ":")):
        ref = sorted(
            (r for r in case.references if r["row"] == first and r["kind"] == kind),
            key=lambda r: r["index"],
        )
        if ref:
            path.plot([r["x"] for r in ref], [r["y"] for r in ref], style, label=kind)
    target = next(r for r in case.references if r["row"] == first and r["kind"] == "terminal")
    path.plot(target["x"], target["y"], "x", label="terminal target")
    snapshots = {first, rows[-1]["row"]}
    for g in case.geometry:
        if g["row"] not in snapshots:
            continue
        if g["kind"] == "obstacle":
            c, s = math.cos(g["yaw"]), math.sin(g["yaw"])
            vertices = [
                (g["x0"] + c * x - s * y, g["y0"] + s * x + c * y)
                for x, y in (
                    (-g["half_length"], -g["half_width"]),
                    (g["half_length"], -g["half_width"]),
                    (g["half_length"], g["half_width"]),
                    (-g["half_length"], g["half_width"]),
                )
            ]
            path.add_patch(Polygon(vertices, fill=False, edgecolor="red", alpha=0.6))
        else:
            path.plot(
                [g["x0"], g["x1"]],
                [g["y0"], g["y1"]],
                color="black" if g["kind"] == "road_border" else "purple",
                alpha=0.5,
            )
    path.set(xlabel="x [m]", ylabel="y [m]", title="Scene (first/last geometry snapshots)")
    path.set_aspect("equal", adjustable="datalim")

    def series(ax, field_name, label):
        if continuous and field_name in {"steer_cmd", "accel_cmd"}:
            # u[t] is issued at t*dt; the output in the same CSV row is x[t+1].
            issued = [r for r in rows if r["kind"] == "running"]
            period = float(case.metadata["dt"])
            times = [r["stage"] * period for r in issued] + [issued[-1]["time_s"]]
            values = [r[field_name] for r in issued] + [issued[-1][field_name]]
            ax.step(times, values, where="post", label=label)
        else:
            # The initial state's zero cost is a placeholder, not an evaluated stage cost.
            samples = (
                [r for r in rows if r["kind"] != "initial"]
                if field_name == "total" or field_name.startswith("cost_")
                else rows
            )
            if continuous and field_name in {"steer_command_rate", "accel_command_rate"}:
                # Command increments occur at issue time, like the commands above.
                times = [r["stage"] * float(case.metadata["dt"]) for r in samples]
            else:
                times = [r["time_s"] if continuous else r["row"] for r in samples]
            values = [
                (
                    math.nan
                    if r[field_name] is None
                    or (r["kind"] == "initial" and field_name.startswith("physical_"))
                    else r[field_name]
                )
                for r in samples
            ]
            ax.plot(times, values, label=label)

    groups = [
        (1, "Speed", "m/s", [("v", "longitudinal speed")]),
        (
            2,
            "Steering command and actuator",
            "rad",
            [("steer_cmd", "command"), ("steering", "actuator")],
        ),
        (
            3,
            "Acceleration command and actuator",
            "m/s²",
            [("accel_cmd", "command"), ("acceleration", "actuator")],
        ),
        (
            4,
            "Physical jerk",
            "m/s³",
            [("physical_longitudinal_jerk", "longitudinal"), ("physical_lateral_jerk", "lateral")],
        ),
        (
            5,
            "Physical / command steering rate",
            "rad/s",
            [("physical_steer_rate", "physical"), ("steer_command_rate", "command (t>0)")],
        ),
        (6, "Acceleration command rate", "m/s³", [("accel_command_rate", "command (t>0)")]),
        (7, "Weighted cost per evaluation", "cost", [("total", "total")]),
    ]
    for index, title, unit, fields in groups:
        for key, label in fields:
            series(axes[index], key, label)
        axes[index].set(title=title, ylabel=unit, xlabel=xlabel)
    reference_speed = [
        next(
            r["v"]
            for r in case.references
            if r["row"] == row["row"] and r["kind"] == "reference" and r["index"] == row["stage"]
        )
        for row in rows
    ]
    axes[1].plot(abscissa, reference_speed, "--", label="time-indexed reference")
    dominant = sorted(COMPONENTS, key=lambda c: sum(r["cost_" + c] for r in rows), reverse=True)[:6]
    for component in dominant:
        if any(r["cost_" + component] > 0 for r in rows):
            series(axes[8], "cost_" + component, component)
    axes[8].set(title="Largest component contributions", ylabel="cost", xlabel=xlabel)
    for ax in axes:
        ax.grid(alpha=0.2)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(fontsize=7)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(destination, dpi=140)
    plt.close(fig)


def md(value):
    return str(value).replace("|", "\\|").replace("\n", " ")


def number(value):
    return "—" if value is None else f"{value:.9g}"


def write_report(cases, source, output, plots=True):
    output.mkdir(parents=True, exist_ok=True)
    lines = [
        "# MPPI cost test report",
        "",
        f"Input run: `{source}`",
        "",
        "Cost columns are weighted contributions. Component totals exclude the initial state. "
        "Only complete rollouts have an objective `(sum running + terminal) / H`. "
        "Parameter-sweep sums are descriptive and are not optimizer objectives.",
        "",
        "Stage t evaluates x[t+1] at (t+1)·dt. The first command-rate sample is blank because "
        "pre-horizon command history is unknown; its command-change cost must be zero.",
        "Commands and command rates are plotted at issue time t·dt; physical outputs and "
        "stage costs use post-step time (t+1)·dt. Cost plots omit the initial placeholder row.",
        "",
    ]
    # Plot errors must affect both the summary status and command exit status.
    figures = {}
    for index, case in enumerate(cases):
        if plots and case.rows and not case.errors and case.status != "SKIPPED":
            image = f"case_{index:03d}.png"
            try:
                plot_case(case, output / image)
                figures[case.name] = image
            except (ImportError, OSError, ValueError, RuntimeError) as exc:
                case.errors.append(f"plot generation: {exc}")
    lines += ["| Case | Status | Checks | Failed checks |", "|---|---|---:|---:|"]
    for case in cases:
        lines.append(
            f"| {md(case.name)} | {case.status} | {len(case.checks)} | "
            f"{sum(not c['passed'] for c in case.checks)} |"
        )
    lines += [
        "",
        "## Component coverage",
        "",
        "Analytical checks are named `cost.<component>`. Parity compares implementation paths "
        "and is separate from independent expected-value coverage. Zero-only checks do not "
        "demonstrate a component's response.",
        "",
        "| Component | Analytical checks | Nonzero expected checks | Failed | Enabled rows |",
        "|---|---:|---:|---:|---:|",
    ]
    for component in COMPONENTS:
        checks = [c for case in cases for c in case.checks if c["check"] == "cost." + component]
        enabled = sum(r["weight_" + component] > 0 for case in cases for r in case.rows)
        lines.append(
            f"| {component} | {len(checks)} | {sum(c['expected'] != 0 for c in checks)} | "
            f"{sum(not c['passed'] for c in checks)} | {enabled} |"
        )
    for case in cases:
        lines += ["", f"## {md(case.name)} — {case.status}", ""]
        for key, value in case.metadata.items():
            lines.append(f"- {md(key)}: {md(value)}")
        lines += ["", *[f"**Error:** {md(e)}" for e in case.errors], ""]
        if case.name in figures:
            lines += [f"![Cost and trajectory plots]({figures[case.name]})", ""]
        if case.rows:
            lines += [
                "| Component | Running sum | Terminal evaluations sum | Isolated samples sum | Objective / H |",
                "|---|---:|---:|---:|---:|",
            ]
            for component in [*COMPONENTS, "total"]:
                lines.append(
                    f"| {component} | "
                    + " | ".join(map(number, cost_totals(case, component)))
                    + " |"
                )
            lines += [
                "",
                "<details><summary>Expected versus actual checks (all recorded assertions)</summary>",
                "",
                "| Row | Check | Actual | Expected | Tolerance | Result |",
                "|---:|---|---:|---:|---:|---|",
            ]
            for c in case.checks:
                lines.append(
                    f"| {c['row']} | {md(c['check'])} | {number(c['actual'])} | "
                    f"{number(c['expected'])} | {number(c['tolerance'])} | "
                    f"{'PASS' if c['passed'] else 'FAIL'} |"
                )
            lines += ["", "</details>", ""]
    destination = output / "MPPI_Cost_Report.md"
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        required=True,
        type=Path,
        help="one run_* directory printed by test_costs (never mixes runs)",
    )
    parser.add_argument("--output-dir", type=Path, help="default: INPUT_DIR/report")
    parser.add_argument(
        "--no-plots", action="store_true", help="validate and write tables without matplotlib"
    )
    args = parser.parse_args(argv)
    source = args.input_dir.resolve()
    if not source.is_dir():
        parser.error(f"not a run directory: {source}")
    names = set()
    for suffix in HEADERS:
        names.update(p.name[: -len(f".{suffix}.csv")] for p in source.glob(f"*.{suffix}.csv"))
    if not names:
        parser.error("no schema-2 cost CSVs found; use the run_* directory, not its parent")
    cases = [load_case(source, name) for name in sorted(names)]
    try:
        destination = write_report(
            cases, source, args.output_dir or source / "report", not args.no_plots
        )
    except OSError as exc:
        print(f"Cannot write report: {exc}", file=sys.stderr)
        return 1
    for case in cases:
        print(
            f"{case.status:7} {case.name}" + (": " + "; ".join(case.errors) if case.errors else "")
        )
    print(f"Report: {destination}")
    return int(any(case.status == "FAIL" for case in cases))


if __name__ == "__main__":
    sys.exit(main())
