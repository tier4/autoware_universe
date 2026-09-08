#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Report integrity regressions. No CUDA, pandas, or matplotlib required."""

import csv
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "generate_cost_report.py"
SPEC = importlib.util.spec_from_file_location("generate_cost_report", SCRIPT)
report = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = report
SPEC.loader.exec_module(report)


class ReportIntegrityTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.name = "Fixture.KnownObjective"
        self.tables = {key: [] for key in report.HEADERS}
        self.tables["meta"] = [
            dict(key=key, value=value)
            for key, value in {
                "schema_version": "2",
                "horizon": "2",
                "dt": "0.1",
                "name": self.name,
                "status": "PASS",
            }.items()
        ]
        # Independent objective: running 1+2, terminal 3 -> (1+2+3)/2 = 3.
        for i, (kind, stage, cost) in enumerate(
            (("running", 0, 1), ("running", 1, 2), ("terminal", 1, 3))
        ):
            row = dict.fromkeys(report.STEP_FIELDS, 0)
            valid = kind == "running" and stage > 0
            row.update(
                row=i,
                kind=kind,
                label="known",
                stage=stage,
                time_s=(stage + 1) * 0.1,
                command_rate_valid=int(valid),
                accel_command_rate=0 if valid else "",
                steer_command_rate=0 if valid else "",
                direct_available=int(kind == "running"),
                total=cost,
                direct_total=cost,
                cost_track=cost,
                weight_track=1,
            )
            self.tables["steps"].append(row)
            self.tables["checks"].append(
                dict(
                    row=i, check="cost.track", actual=cost, expected=cost, tolerance=1e-5, passed=1
                )
            )
            for j in range(2):
                self.tables["reference"].append(
                    dict(row=i, kind="reference", index=j, x=j, y=0, yaw=0, v=1)
                )
            self.tables["reference"].append(
                dict(row=i, kind="terminal", index=0, x=1, y=0, yaw=0, v=0)
            )
            self.tables["params"].append(dict(row=i, parameter="wheel_base", value=2))

    def write(self, omit=None):
        for key, rows in self.tables.items():
            if key == omit:
                continue
            with (self.directory / f"{self.name}.{key}.csv").open("w", newline="") as stream:
                writer = csv.DictWriter(stream, report.HEADERS[key])
                writer.writeheader()
                writer.writerows(rows)

    def load(self):
        self.write()
        return report.load_case(self.directory, self.name)

    def test_terminal_counted_once_and_normalized_by_horizon(self):
        case = self.load()
        self.assertEqual(case.status, "PASS", case.errors)
        self.assertEqual(report.cost_totals(case), (3, 3, 0, 3))
        self.assertIsNone(case.rows[0]["steer_command_rate"])
        self.assertEqual(case.rows[1]["steer_command_rate"], 0)

    def test_nonfinite_values_rejected_in_every_numeric_sidecar(self):
        mutations = (
            ("steps", "cost_track"),
            ("checks", "actual"),
            ("reference", "x"),
            ("params", "value"),
        )
        for table, column in mutations:
            original = self.tables[table][0][column]
            for bad in ("nan", "inf", "-inf"):
                with self.subTest(table=table, column=column, value=bad):
                    self.tables[table][0][column] = bad
                    self.assertEqual(self.load().status, "FAIL")
            self.tables[table][0][column] = original

    def test_unknown_first_command_rate_must_be_blank(self):
        self.tables["steps"][0]["steer_command_rate"] = 2.0
        case = self.load()
        self.assertEqual(case.status, "FAIL")
        self.assertIn("must be blank", case.errors[0])

    def test_incorrect_time_is_rejected(self):
        self.tables["steps"][0]["time_s"] = 0
        self.assertEqual(self.load().status, "FAIL")

    def test_totals_cannot_hide_missing_components(self):
        self.tables["steps"][0]["total"] = 9
        case = self.load()
        self.assertEqual(case.status, "FAIL")
        self.assertIn("component total", case.errors[0])

    def test_incomplete_export_without_completion_metadata(self):
        self.write(omit="meta")
        self.assertEqual(report.load_case(self.directory, self.name).status, "FAIL")

    def test_schema_mismatch_is_rejected(self):
        self.tables["meta"][0]["value"] = "1"
        self.assertEqual(self.load().status, "FAIL")

    def test_truncated_csv_is_rejected(self):
        self.write()
        path = self.directory / f"{self.name}.steps.csv"
        with path.open("a") as stream:
            stream.write("3,running,partial\n")
        self.assertEqual(report.load_case(self.directory, self.name).status, "FAIL")

    def test_failed_assertion_cannot_be_hidden_by_passing_metadata(self):
        self.tables["checks"][1].update(actual=7, passed=0)
        case = self.load()
        self.assertEqual(case.status, "FAIL")
        output = self.directory / "report"
        path = report.write_report([case], self.directory, output, plots=False)
        self.assertIn("FAIL", path.read_text())
        self.assertIn("Expected versus actual", path.read_text())

    def test_inconsistent_assertion_flag_is_rejected(self):
        self.tables["checks"][1].update(actual=7, passed=1)
        self.assertEqual(self.load().status, "FAIL")

    def test_missing_terminal_is_rejected(self):
        for key in ("steps", "checks", "params", "reference"):
            self.tables[key] = [r for r in self.tables[key] if r["row"] != 2]
        self.assertEqual(self.load().status, "FAIL")

    def test_parameter_sweeps_have_no_normalized_objective(self):
        for row in self.tables["steps"][:2]:
            row.update(kind="sample", time_s="")
        case = self.load()
        self.assertEqual(case.status, "PASS", case.errors)
        self.assertEqual(report.cost_totals(case), (0, 3, 3, None))

    def test_empty_skipped_gpu_test_does_not_claim_coverage(self):
        for key in self.tables:
            if key != "meta":
                self.tables[key].clear()
        self.tables["meta"][-1]["value"] = "SKIPPED"
        case = self.load()
        self.assertEqual(case.status, "SKIPPED", case.errors)
        self.assertFalse(case.complete_horizon)
        self.assertFalse(case.checks)

    def test_mismatched_case_name_rejects_stale_sidecar(self):
        self.tables["meta"][-2]["value"] = "Different.Run"
        self.assertEqual(self.load().status, "FAIL")


if __name__ == "__main__":
    unittest.main()
