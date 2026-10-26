#!/usr/bin/env python3
"""Tests for compare_eval_ui.

Run from the repository root:
  python3 src/autoware/universe/planning/autoware_mppi_optimizer/scripts/test_compare_eval_ui.py
"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


def _load():
    path = Path(__file__).with_name("compare_eval_ui.py")
    spec = importlib.util.spec_from_file_location("compare_eval_ui", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


ui = _load()


def report(
    scenario_id,
    name,
    status,
    *,
    labels=None,
    message="",
    params=None,
    attempt=1,
    suite="suite-a",
    version=3,
    case_id="case",
):
    return {
        "id": f"{case_id}-{attempt}",
        "attempt_id": attempt,
        "status": status,
        "fail_message": message,
        "failure_cause_labels": list(labels or []),
        "scenario": {"id": scenario_id, "display_name": name, "version_id": version},
        "scenario_parameters": params or {},
        "suite": {"id": "suite", "display_name": suite},
        "job_id": "job",
        "parents": {
            "job_report_id": "report",
            "suite_report_id": "suite-report",
            "spec_report_id": "spec-report",
        },
    }


class CompareTests(unittest.TestCase):
    def test_flips_are_oriented_from_the_base_run(self):
        rows = ui.compare_reports(
            [
                report("s1", "kept", "succeeded"),
                report("s2", "broke", "succeeded"),
                report("s3", "fixed", "failed", labels=["ConditionFailure:Collision"]),
                report("s4", "still bad", "failed", labels=["ConditionFailure:StandStill"]),
            ],
            [
                report("s1", "kept", "succeeded"),
                report("s2", "broke", "failed", labels=["ConditionFailure:Collision"]),
                report("s3", "fixed", "succeeded"),
                report("s4", "still bad", "failed", labels=["ConditionFailure:StandStill"]),
            ],
            project_id="x2_dev",
        )
        by_name = {row["scenario"]: row for row in rows}
        self.assertEqual(by_name["kept"]["outcome"], "both_passed")
        self.assertEqual(by_name["broke"]["outcome"], "regression")
        self.assertEqual(by_name["broke"]["cause"], "Collision")
        self.assertEqual(by_name["fixed"]["outcome"], "improvement")
        self.assertEqual(by_name["fixed"]["cause"], "Collision")
        self.assertEqual(by_name["still bad"]["outcome"], "both_failed")
        self.assertFalse(any(row["backend_failure"] for row in rows))

    def test_simulation_error_is_a_backend_failure_and_simulation_failure_is_not(self):
        rows = ui.compare_reports(
            [report("s1", "backend", "succeeded"), report("s2", "condition", "succeeded")],
            [
                report(
                    "s1",
                    "backend",
                    "failed",
                    labels=["SimulationError:AutowareError"],
                    message="Simulation error [AutowareError]: timed out waiting for engage",
                ),
                report(
                    "s2",
                    "condition",
                    "failed",
                    labels=[
                        "ConditionFailure:StoryboardElementStateCondition:act_start_check",
                        "SimulationFailure",
                    ],
                ),
            ],
            project_id="x2_dev",
        )
        by_name = {row["scenario"]: row for row in rows}
        self.assertTrue(by_name["backend"]["backend_failure"])
        self.assertEqual(by_name["backend"]["outcome"], "regression")
        self.assertEqual(by_name["backend"]["candidate_cause"], "SimulationError:AutowareError")
        self.assertFalse(by_name["condition"]["backend_failure"])
        self.assertFalse(by_name["backend"]["scenario_failure"])
        self.assertTrue(by_name["condition"]["scenario_failure"])
        self.assertEqual(by_name["condition"]["cause"], "act_start_check")

        shown = ui.summarize(rows, hide_backend=True)
        self.assertEqual(shown["regression"], 1)
        self.assertEqual(shown["hidden_backend"], 1)
        raw = ui.summarize(rows, hide_backend=False)
        self.assertEqual(raw["regression"], 2)
        self.assertEqual(raw["hidden_backend"], 0)
        without_scenario = ui.summarize(rows, hide_backend=True, hide_scenario=True)
        self.assertEqual(without_scenario["regression"], 0)
        self.assertEqual(without_scenario["hidden_scenario"], 1)

    def test_backend_failure_on_either_side_hides_the_row(self):
        rows = ui.compare_reports(
            [
                report(
                    "s1",
                    "base backend",
                    "failed",
                    labels=["SimulationError:Crash"],
                    message="Simulation error [Crash]: worker died",
                )
            ],
            [report("s1", "base backend", "succeeded")],
            project_id="x2_dev",
        )
        self.assertEqual(rows[0]["outcome"], "improvement")
        self.assertTrue(rows[0]["backend_failure"])
        self.assertEqual(ui.summarize(rows, hide_backend=True)["improvement"], 0)

    def test_latest_attempt_and_parameters_define_a_case(self):
        rows = ui.compare_reports(
            [
                report("s1", "retry", "failed", labels=["ConditionFailure:Old"], attempt=1),
                report("s1", "retry", "succeeded", attempt=2),
                report("s2", "params", "succeeded", params={"speed": 1.0}),
                report("s2", "params", "failed", params={"speed": 2}, labels=["ConditionFailure:Fast"]),
            ],
            [
                report("s1", "retry", "failed", labels=["ConditionFailure:New"], attempt=1),
                report("s2", "params", "failed", params={"speed": 1}, labels=["ConditionFailure:Slow"]),
                report("s2", "params", "succeeded", params={"speed": 2}),
            ],
            project_id="x2_dev",
        )
        by_params = {(row["scenario"], row["parameters"]): row for row in rows}
        self.assertEqual(by_params[("retry", "")]["outcome"], "regression")
        self.assertEqual(by_params[("retry", "")]["base_attempt"], 2)
        self.assertEqual(by_params[("params", "speed=1")]["outcome"], "regression")
        self.assertEqual(by_params[("params", "speed=2")]["outcome"], "improvement")

    def test_missing_side_and_unfinished_status(self):
        rows = ui.compare_reports(
            [report("s1", "only base", "succeeded"), report("s2", "running", "started")],
            [report("s3", "only candidate", "failed", labels=["TimedOut"]), report("s2", "running", "succeeded")],
            project_id="x2_dev",
        )
        by_name = {row["scenario"]: row for row in rows}
        self.assertEqual(by_name["only base"]["outcome"], "base_only")
        self.assertEqual(by_name["only candidate"]["outcome"], "candidate_only")
        self.assertEqual(by_name["running"]["outcome"], "incomplete")
        self.assertIn(
            "/evaluation/reports/job/tests/suite-report/spec-report/",
            by_name["only candidate"]["candidate_url"],
        )

    def test_report_url_parses_project_and_first_id(self):
        job_id, project_id = ui.parse_run_reference(
            "https://evaluation.ci.tier4.jp/evaluation/reports/"
            "b5a194a5-ef25-5ab3-93fb-775457d9477e/tests/"
            "03546d8d-a01d-5af3-a7ed-16f6ee1b80fe?project_id=x2_dev"
        )
        self.assertEqual(job_id, "b5a194a5-ef25-5ab3-93fb-775457d9477e")
        self.assertEqual(project_id, "x2_dev")
        bare, project = ui.parse_run_reference("4d588d0c-1742-56db-8099-e2cc3c494b15")
        self.assertEqual(bare, "4d588d0c-1742-56db-8099-e2cc3c494b15")
        self.assertIsNone(project)

    def test_html_hides_backend_failures_until_the_filter_is_cleared(self):
        payload = ui.build_payload(
            "x2_dev",
            ui.ResolvedRun("base", "job-b", "report-b", "", "x2_dev"),
            [
                (
                    ui.ResolvedRun("mppi", "job-a", "report-a", "", "x2_dev"),
                    ui.compare_reports(
                        [report("s1", "backend", "succeeded")],
                        [
                            report(
                                "s1",
                                "backend",
                                "failed",
                                labels=["SimulationError:AutowareError"],
                                message="Simulation error [AutowareError]: down",
                            )
                        ],
                        project_id="x2_dev",
                    ),
                )
            ],
            origin=ui.EVAL_ORIGIN,
        )
        html = ui.render_html(payload)
        self.assertIn('id="hide-backend" checked', html)
        self.assertIn("Hide simulation backend failures", html)
        self.assertIn('id="hide-scenario" checked', html)
        self.assertIn("Hide scenario results", html)
        self.assertIn("SimulationError:AutowareError", html)
        self.assertNotIn("</script><script>", html)
        self.assertTrue(html.startswith("<!DOCTYPE html>"))


if __name__ == "__main__":
    unittest.main()
