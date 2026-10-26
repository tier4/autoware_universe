#!/usr/bin/env python3
"""Compare Web.Auto evaluation runs against a base run and write an HTML report.

The report is a single page: regressions, improvements, and a scenario table.
A regression is a scenario the base run passed and the other run failed. An
improvement is a scenario the base run failed and the other run passed.

The page hides two kinds of failure until the matching filter is turned off.
SimulationError means the simulation backend did not finish. SimulationFailure
means the scenario itself exited, which the evaluator shows as a scenario
failure (for example a stand-still condition calling exitFailure).

Examples:
  python3 src/autoware/universe/planning/autoware_mppi_optimizer/scripts/compare_eval_ui.py \\
    --project-id x2_dev \\
    --base-name base \\
    --base 4d588d0c-1742-56db-8099-e2cc3c494b15 \\
    --candidate-name mppi \\
    --candidate b5a194a5-ef25-5ab3-93fb-775457d9477e \\
    --open

  python3 src/autoware/universe/planning/autoware_mppi_optimizer/scripts/compare_eval_ui.py \\
    --base 'https://evaluation.ci.tier4.jp/evaluation/reports/<id>?project_id=x2_dev' \\
    --candidate 'https://evaluation.ci.tier4.jp/evaluation/reports/<id>?project_id=x2_dev'
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import webbrowser
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence
from urllib.parse import parse_qs, urlparse

EVAL_ORIGIN = "https://evaluation.ci.tier4.jp"
PAGE_SIZE = 100
UUID_RE = re.compile(
    r"[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}"
)
MESSAGE_LIMIT = 1500
OUTCOMES = (
    "regression",
    "improvement",
    "both_passed",
    "both_failed",
    "incomplete",
    "base_only",
    "candidate_only",
)


@dataclass(frozen=True)
class RunRef:
    """A job the user asked to compare, before it is resolved against the API."""

    raw: str
    job_or_report_id: str
    project_id: Optional[str]
    name: str


@dataclass(frozen=True)
class ResolvedRun:
    name: str
    job_id: str
    report_id: str
    description: str
    project_id: str

    def report_url(self, origin: str) -> str:
        # The evaluation site addresses a run by job id, the same id as
        # /evaluation/reports/<id> in the browser.
        return f"{origin}/evaluation/reports/{self.job_id}?project_id={self.project_id}"


def parse_run_reference(value: str) -> tuple[str, Optional[str]]:
    """Return (job or report id, project id from the URL if present)."""
    text = value.strip()
    if UUID_RE.fullmatch(text):
        return text, None
    parsed = urlparse(text)
    if parsed.scheme or parsed.path.startswith("/"):
        found = UUID_RE.findall(parsed.path)
        if not found:
            raise ValueError(f"no evaluation id found in {value}")
        project_ids = parse_qs(parsed.query).get("project_id") or []
        return found[0], (project_ids[0] if project_ids else None)
    raise ValueError(
        f"{value!r} is neither an evaluation job id nor an evaluation report URL"
    )


def compare_reports(
    base_reports: Sequence[Mapping[str, Any]],
    candidate_reports: Sequence[Mapping[str, Any]],
    *,
    project_id: str,
    origin: str = EVAL_ORIGIN,
) -> list[dict[str, Any]]:
    """Pair the latest attempt of each scenario and classify the flip."""
    base_index = _index_reports(base_reports)
    candidate_index = _index_reports(candidate_reports)
    rows: list[dict[str, Any]] = []
    for key in sorted(set(base_index) | set(candidate_index)):
        rows.append(
            _compare_pair(
                base_index.get(key),
                candidate_index.get(key),
                project_id=project_id,
                origin=origin,
            )
        )
    rows.sort(key=lambda row: (row["suite"].lower(), row["scenario"].lower(), row["parameters"]))
    return rows


def summarize(
    rows: Sequence[Mapping[str, Any]],
    *,
    hide_backend: bool,
    hide_scenario: bool = False,
) -> dict[str, int]:
    counts = {outcome: 0 for outcome in OUTCOMES}
    hidden_backend = 0
    hidden_scenario = 0
    for row in rows:
        drop_backend = hide_backend and row["backend_failure"]
        drop_scenario = hide_scenario and row["scenario_failure"]
        if drop_backend:
            hidden_backend += 1
        if drop_scenario:
            hidden_scenario += 1
        if drop_backend or drop_scenario:
            continue
        counts[str(row["outcome"])] += 1
    counts["hidden_backend"] = hidden_backend
    counts["hidden_scenario"] = hidden_scenario
    counts["total"] = len(rows)
    return counts


def render_html(payload: Mapping[str, Any]) -> str:
    embedded = json.dumps(payload, ensure_ascii=True, separators=(",", ":")).replace(
        "<", "\\u003c"
    )
    return HTML_TEMPLATE.replace("/*__DATA__*/null", embedded)


def fetch_case_reports(
    job_id: str,
    project_id: str,
    *,
    webauto: str = "webauto",
    profile: Optional[str] = None,
) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    page_token = ""
    seen_tokens: set[str] = set()
    page_number = 1
    while True:
        print(
            f"fetching case reports for {job_id} (page {page_number})",
            file=sys.stderr,
            flush=True,
        )
        command = [
            webauto,
            "ci",
            "evaluation-job-case-report",
            "list",
            "--project-id",
            project_id,
            "--evaluation-job-id",
            job_id,
            "--page-size",
            str(PAGE_SIZE),
            "--output",
            "json",
            "--quiet",
        ]
        if profile:
            command.extend(["--profile", profile])
        if page_token:
            command.extend(["--page-token", page_token])
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            raise RuntimeError(
                f"webauto ci evaluation-job-case-report list failed for {job_id}: {detail}"
            )
        page = json.loads(_json_payload(completed.stdout))
        reports.extend(page.get("reports") or [])
        page_token = page.get("next_page_token") or ""
        if not page_token or page_token in seen_tokens:
            break
        seen_tokens.add(page_token)
        page_number += 1
    if not reports:
        raise RuntimeError(f"no case reports returned for evaluation job {job_id}")
    return reports


def resolve_run(
    ref: RunRef,
    project_id: str,
    *,
    webauto: str = "webauto",
    profile: Optional[str] = None,
) -> ResolvedRun:
    """Resolve a job id or report URL to the job id used by case-report list."""
    described = _describe_report(
        project_id,
        ref.job_or_report_id,
        by="job",
        webauto=webauto,
        profile=profile,
    )
    if described is None:
        described = _describe_report(
            project_id,
            ref.job_or_report_id,
            by="report",
            webauto=webauto,
            profile=profile,
        )
    if described is None:
        raise RuntimeError(
            "could not resolve "
            f"{ref.raw} in project {project_id} as an evaluation job or report"
        )
    description = str(described.get("description") or "").strip()
    name = ref.name or description or ref.job_or_report_id[:8]
    return ResolvedRun(
        name=name,
        job_id=str(described.get("job_id") or ref.job_or_report_id),
        report_id=str(described.get("id") or ref.job_or_report_id),
        description=description,
        project_id=str(described.get("project_id") or project_id),
    )


def load_reports(
    run: ResolvedRun,
    *,
    cache_dir: Optional[Path],
    refresh: bool,
    webauto: str,
    profile: Optional[str],
) -> list[dict[str, Any]]:
    cache_path = None
    if cache_dir is not None:
        cache_dir.mkdir(parents=True, exist_ok=True)
        cache_path = cache_dir / f"{run.project_id}-{run.job_id}.json"
        if cache_path.exists() and not refresh:
            print(f"using cached case reports for {run.name}: {cache_path}", file=sys.stderr)
            cached = json.loads(cache_path.read_text(encoding="utf-8"))
            if not isinstance(cached, list) or not cached:
                raise RuntimeError(f"cache file is empty or invalid: {cache_path}")
            return cached
    reports = fetch_case_reports(
        run.job_id, run.project_id, webauto=webauto, profile=profile
    )
    if cache_path is not None:
        cache_path.write_text(json.dumps(reports), encoding="utf-8")
        print(f"cached {len(reports)} case reports at {cache_path}", file=sys.stderr)
    return reports


def _describe_report(
    project_id: str,
    identifier: str,
    *,
    by: str,
    webauto: str,
    profile: Optional[str],
) -> Optional[dict[str, Any]]:
    flag = "--evaluation-job-id" if by == "job" else "--evaluation-job-report-id"
    command = [
        webauto,
        "ci",
        "evaluation-job-report",
        "describe",
        "--project-id",
        project_id,
        flag,
        identifier,
        "--output",
        "json",
        "--quiet",
    ]
    if profile:
        command.extend(["--profile", profile])
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode != 0:
        return None
    try:
        payload = json.loads(_json_payload(completed.stdout))
    except (RuntimeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict) or not (payload.get("job_id") or payload.get("id")):
        return None
    return payload


def _compare_pair(
    base: Optional[Mapping[str, Any]],
    candidate: Optional[Mapping[str, Any]],
    *,
    project_id: str,
    origin: str,
) -> dict[str, Any]:
    present = base or candidate
    assert present is not None
    suite, scenario_name, parameters = _identity(present)
    base_side = _side(base, project_id=project_id, origin=origin)
    candidate_side = _side(candidate, project_id=project_id, origin=origin)
    outcome = _outcome(base_side["status"], candidate_side["status"])
    if outcome == "regression":
        cause = candidate_side["cause"]
    elif outcome == "improvement":
        cause = base_side["cause"]
    elif outcome == "both_failed" and base_side["cause"] and candidate_side["cause"]:
        if base_side["cause"] == candidate_side["cause"]:
            cause = base_side["cause"]
        else:
            cause = f"{base_side['cause']} → {candidate_side['cause']}"
    else:
        cause = candidate_side["cause"] or base_side["cause"]
    backend = bool(base_side["backend_failure"] or candidate_side["backend_failure"])
    scenario_failure = bool(base_side["scenario_failure"] or candidate_side["scenario_failure"])
    return {
        "suite": suite,
        "scenario": scenario_name,
        "parameters": parameters,
        "outcome": outcome,
        "backend_failure": backend,
        "scenario_failure": scenario_failure,
        "cause": cause,
        "base_status": base_side["status"],
        "base_cause": base_side["cause"],
        "base_labels": base_side["labels"],
        "base_message": base_side["message"],
        "base_url": base_side["url"],
        "base_attempt": base_side["attempt"],
        "base_version": base_side["version"],
        "candidate_status": candidate_side["status"],
        "candidate_cause": candidate_side["cause"],
        "candidate_labels": candidate_side["labels"],
        "candidate_message": candidate_side["message"],
        "candidate_url": candidate_side["url"],
        "candidate_attempt": candidate_side["attempt"],
        "candidate_version": candidate_side["version"],
    }


def _outcome(base_status: str, candidate_status: str) -> str:
    if base_status == "absent":
        return "candidate_only"
    if candidate_status == "absent":
        return "base_only"
    if base_status not in {"succeeded", "failed"} or candidate_status not in {"succeeded", "failed"}:
        return "incomplete"
    if base_status == "succeeded" and candidate_status == "succeeded":
        return "both_passed"
    if base_status == "failed" and candidate_status == "failed":
        return "both_failed"
    if base_status == "succeeded" and candidate_status == "failed":
        return "regression"
    return "improvement"


def _side(
    report: Optional[Mapping[str, Any]],
    *,
    project_id: str,
    origin: str,
) -> dict[str, Any]:
    if report is None:
        return {
            "status": "absent",
            "cause": "",
            "labels": "",
            "message": "",
            "url": "",
            "attempt": 0,
            "version": None,
            "backend_failure": False,
            "scenario_failure": False,
        }
    labels = [str(label) for label in (report.get("failure_cause_labels") or [])]
    message = str(report.get("fail_message") or "").strip()
    scenario = report.get("scenario") or {}
    version = scenario.get("version_id")
    return {
        "status": str(report.get("status") or ""),
        "cause": primary_cause(labels, message),
        "labels": ", ".join(labels),
        "message": _clip(message),
        "url": _case_url(report, project_id=project_id, origin=origin),
        "attempt": int(report.get("attempt_id") or 0),
        "version": int(version) if version is not None else None,
        "backend_failure": is_simulation_backend_failure(labels, message),
        "scenario_failure": is_scenario_failure(labels, message),
    }


def is_scenario_failure(labels: Sequence[str], message: str) -> bool:
    """True when a scenario failure condition fired exitFailure.

    The label is SimulationFailure and the message starts with
    "Simulation failure:". Stand-still, collision, and simulation-time
    checks are in this group. The simulator finished and reported that
    result. SimulationError, where the backend did not finish, is separate.
    """
    for label in labels:
        if label == "SimulationFailure" or label.startswith("SimulationFailure:"):
            return True
    return message.lstrip().startswith("Simulation failure")


def is_simulation_backend_failure(labels: Sequence[str], message: str) -> bool:
    """True when the simulator or Autoware backend failed, not a scenario condition.

    Condition failures are often also tagged SimulationFailure, because the
    scenario calls exitFailure. That tag is a condition result. SimulationError
    is the backend failure.
    """
    for label in labels:
        if label == "SimulationError" or label.startswith("SimulationError:"):
            return True
    return message.lstrip().startswith("Simulation error")


def primary_cause(labels: Sequence[str], message: str) -> str:
    """One cause string per case, so grouped counts sum to the scenario count."""
    conditions: list[str] = []
    others: list[str] = []
    for label in labels:
        if label.startswith("ConditionFailure:") or label.startswith("ConditionFailure::"):
            conditions.append(_short_condition(label))
        elif label == "SimulationFailure":
            continue
        elif label:
            others.append(label)
    if conditions:
        return " + ".join(dict.fromkeys(conditions))
    if others:
        return " + ".join(dict.fromkeys(others))
    text = " ".join(message.split())
    if text.lower().startswith("pass conditions"):
        return "Pass conditions"
    matched = re.match(r"Simulation error \[([^\]]+)\]", text)
    if matched:
        return f"SimulationError:{matched.group(1)}"
    if text.startswith("Simulation error"):
        return "SimulationError"
    if not text:
        return ""
    return text[:120]


def _short_condition(label: str) -> str:
    rest = label.split(":", 1)[1]
    if rest.startswith(":"):
        rest = rest[1:]
    parts = [part for part in rest.split(":") if part]
    if not parts:
        return "ConditionFailure"
    if len(parts) >= 2:
        return parts[-1]
    return parts[0]


def _case_url(report: Mapping[str, Any], *, project_id: str, origin: str) -> str:
    parents = report.get("parents") or {}
    job_id = report.get("job_id") or parents.get("job_report_id")
    suite_id = parents.get("suite_report_id")
    spec_id = parents.get("spec_report_id")
    case_id = report.get("id")
    if not job_id:
        return ""
    if not all((suite_id, spec_id, case_id)):
        return f"{origin}/evaluation/reports/{job_id}?project_id={project_id}"
    return (
        f"{origin}/evaluation/reports/{job_id}/tests/"
        f"{suite_id}/{spec_id}/{case_id}?project_id={project_id}"
    )


def _identity(report: Mapping[str, Any]) -> tuple[str, str, str]:
    suite = report.get("suite") or {}
    scenario = report.get("scenario") or {}
    parameters = _canonicalize(report.get("scenario_parameters") or {})
    rendered = ", ".join(
        f"{key}={_format_value(parameters[key])}" for key in sorted(parameters)
    )
    return (
        str(suite.get("display_name") or suite.get("id") or ""),
        str(scenario.get("display_name") or scenario.get("id") or "<unknown scenario>"),
        rendered,
    )


def _index_reports(reports: Iterable[Mapping[str, Any]]) -> dict[str, Mapping[str, Any]]:
    indexed: dict[str, Mapping[str, Any]] = {}
    attempts: dict[str, int] = {}
    for report in reports:
        key = _scenario_key(report)
        attempt_id = int(report.get("attempt_id") or 0)
        if key not in indexed or attempt_id >= attempts[key]:
            indexed[key] = report
            attempts[key] = attempt_id
    return indexed


def _scenario_key(report: Mapping[str, Any]) -> str:
    scenario = report.get("scenario") or {}
    scenario_id = str(scenario.get("id") or scenario.get("display_name") or "")
    parameters = _canonicalize(report.get("scenario_parameters") or {})
    return json.dumps([scenario_id, parameters], sort_keys=True, separators=(",", ":"))


def _canonicalize(value: Any) -> Any:
    if isinstance(value, float) and value.is_integer():
        return int(value)
    if isinstance(value, dict):
        return {str(key): _canonicalize(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_canonicalize(item) for item in value]
    return value


def _format_value(value: Any) -> str:
    if isinstance(value, str):
        return value
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _clip(text: str, limit: int = MESSAGE_LIMIT) -> str:
    if len(text) <= limit:
        return text
    return text[: limit - 1] + "…"


def _json_payload(stdout: str) -> str:
    start = stdout.find("{")
    if start < 0:
        raise RuntimeError(f"webauto did not return JSON: {stdout.strip()}")
    return stdout[start:]


def _print_summary(name: str, counts: Mapping[str, int]) -> None:
    print(f"{name}")
    print(f"  regressions: {counts['regression']}")
    print(f"  improvements: {counts['improvement']}")
    print(f"  net (improvements - regressions): {counts['improvement'] - counts['regression']}")
    print(f"  both passed: {counts['both_passed']}")
    print(f"  both failed: {counts['both_failed']}")
    print(f"  incomplete: {counts['incomplete']}")
    print(f"  only in one run: {counts['base_only'] + counts['candidate_only']}")
    print(f"  simulation backend failures hidden: {counts['hidden_backend']}")
    print(f"  scenario failures hidden: {counts['hidden_scenario']}")


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compare Web.Auto evaluation runs against a base run and write an HTML report. "
            "The page lists regressions and improvements and can hide simulation backend failures."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "A run can be an evaluation job id or a report URL from "
            "evaluation.ci.tier4.jp. The id in /evaluation/reports/<id> is accepted."
        ),
    )
    parser.add_argument("--project-id", help="Web.Auto project id. Taken from report URLs when omitted.")
    parser.add_argument("--base", required=True, help="base run: job id or report URL")
    parser.add_argument("--base-name", default="", help="label for the base run")
    parser.add_argument(
        "--candidate",
        action="append",
        required=True,
        help="run to compare with the base. Repeat for more than one run.",
    )
    parser.add_argument(
        "--candidate-name",
        action="append",
        default=[],
        help="label for each --candidate, in the same order",
    )
    parser.add_argument("--profile", help="webauto auth profile")
    parser.add_argument("--webauto", default="webauto", help="webauto executable")
    parser.add_argument(
        "--output",
        default="eval-comparison.html",
        help="HTML file to write (default: eval-comparison.html)",
    )
    parser.add_argument(
        "--cache-dir",
        default=str(Path.home() / ".cache" / "webauto-eval-compare"),
        help="directory for cached case reports (default: ~/.cache/webauto-eval-compare)",
    )
    parser.add_argument("--no-cache", action="store_true", help="do not read or write the case-report cache")
    parser.add_argument("--refresh", action="store_true", help="ignore cached case reports and fetch again")
    parser.add_argument("--open", action="store_true", help="open the HTML report in a browser")
    parser.add_argument(
        "--eval-origin",
        default=EVAL_ORIGIN,
        help=f"site used for report links (default: {EVAL_ORIGIN})",
    )
    return parser.parse_args(argv)


def _refs_from_args(args: argparse.Namespace) -> tuple[str, RunRef, list[RunRef]]:
    if len(args.candidate_name) not in (0, len(args.candidate)):
        raise ValueError(
            f"--candidate-name was given {len(args.candidate_name)} times "
            f"for {len(args.candidate)} --candidate values"
        )
    base_id, base_project = parse_run_reference(args.base)
    base = RunRef(args.base, base_id, base_project, args.base_name.strip())
    candidates: list[RunRef] = []
    projects = [project for project in (args.project_id, base_project) if project]
    for index, raw in enumerate(args.candidate):
        candidate_id, candidate_project = parse_run_reference(raw)
        name = args.candidate_name[index].strip() if args.candidate_name else ""
        candidates.append(RunRef(raw, candidate_id, candidate_project, name))
        if candidate_project:
            projects.append(candidate_project)
    unique = set(projects)
    if not unique:
        raise ValueError("pass --project-id or use report URLs that include project_id")
    if len(unique) > 1:
        raise ValueError(f"project ids disagree: {', '.join(sorted(unique))}")
    return next(iter(unique)), base, candidates


def build_payload(
    project_id: str,
    base: ResolvedRun,
    candidate_rows: Sequence[tuple[ResolvedRun, Sequence[Mapping[str, Any]]]],
    *,
    origin: str,
) -> dict[str, Any]:
    return {
        "project_id": project_id,
        "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
        "origin": origin,
        "base": {
            "name": base.name,
            "job_id": base.job_id,
            "report_id": base.report_id,
            "description": base.description,
            "url": base.report_url(origin),
        },
        "candidates": [
            {
                "name": candidate.name,
                "job_id": candidate.job_id,
                "report_id": candidate.report_id,
                "description": candidate.description,
                "url": candidate.report_url(origin),
                "rows": rows,
            }
            for candidate, rows in candidate_rows
        ],
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    project_id, base_ref, candidate_refs = _refs_from_args(args)
    origin = args.eval_origin.rstrip("/")
    cache_dir = None if args.no_cache else Path(args.cache_dir)
    base = resolve_run(base_ref, project_id, webauto=args.webauto, profile=args.profile)
    candidates = [
        resolve_run(ref, project_id, webauto=args.webauto, profile=args.profile)
        for ref in candidate_refs
    ]
    base_reports = load_reports(
        base,
        cache_dir=cache_dir,
        refresh=args.refresh,
        webauto=args.webauto,
        profile=args.profile,
    )
    paired: list[tuple[ResolvedRun, list[dict[str, Any]]]] = []
    for candidate in candidates:
        candidate_reports = load_reports(
            candidate,
            cache_dir=cache_dir,
            refresh=args.refresh,
            webauto=args.webauto,
            profile=args.profile,
        )
        rows = compare_reports(
            base_reports,
            candidate_reports,
            project_id=project_id,
            origin=origin,
        )
        paired.append((candidate, rows))
        print(f"\n{candidate.name} against {base.name}", file=sys.stderr)
        _print_summary(
            "with simulation backend and scenario failures hidden",
            summarize(rows, hide_backend=True, hide_scenario=True),
        )

    payload = build_payload(project_id, base, paired, origin=origin)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(render_html(payload), encoding="utf-8")
    print(f"\nwrote {output.resolve()}")
    if args.open:
        webbrowser.open(output.resolve().as_uri())
    return 0


HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Evaluation comparison</title>
<style>
  :root {
    color-scheme: light;
    --bg: #f4f6f8;
    --card: #ffffff;
    --ink: #1c2430;
    --muted: #5c6b7a;
    --line: #d8e0e8;
    --regression: #b42318;
    --regression-bg: #fdeceb;
    --improvement: #067647;
    --improvement-bg: #e8f6ee;
    --accent: #1d4e89;
  }
  * { box-sizing: border-box; }
  body {
    margin: 0;
    background: var(--bg);
    color: var(--ink);
    font: 14px/1.45 "Segoe UI", system-ui, sans-serif;
  }
  header, main { width: min(1440px, calc(100% - 32px)); margin: 0 auto; }
  header { padding: 24px 0 8px; }
  h1 { margin: 0 0 6px; font-size: 22px; font-weight: 650; }
  h2 { margin: 0 0 8px; font-size: 16px; }
  a { color: var(--accent); }
  .sub { color: var(--muted); margin: 0; }
  .run-links { display: flex; flex-wrap: wrap; gap: 8px 16px; margin-top: 10px; }
  .cards { display: grid; grid-template-columns: repeat(5, minmax(0, 1fr)); gap: 10px; margin: 16px 0; }
  .card { background: var(--card); border: 1px solid var(--line); border-radius: 10px; padding: 12px 14px; }
  .card .k { color: var(--muted); font-size: 12px; }
  .card .v { font-size: 26px; font-weight: 650; margin-top: 2px; }
  .card.regression .v { color: var(--regression); }
  .card.improvement .v { color: var(--improvement); }
  .panel {
    background: var(--card);
    border: 1px solid var(--line);
    border-radius: 10px;
    padding: 14px;
    margin-bottom: 14px;
  }
  .toolbar, .filters { display: flex; flex-wrap: wrap; gap: 10px 16px; align-items: center; }
  .filters { margin-bottom: 10px; }
  label.toggle { display: flex; gap: 8px; align-items: center; font-weight: 600; }
  .help { color: var(--muted); margin: 8px 0 0; max-width: 80ch; }
  input[type="search"], select {
    border: 1px solid var(--line);
    border-radius: 8px;
    padding: 7px 10px;
    background: white;
    color: inherit;
    font: inherit;
  }
  input[type="search"] { min-width: min(420px, 100%); }
  .checks { display: flex; flex-wrap: wrap; gap: 8px 14px; }
  table { width: 100%; border-collapse: collapse; }
  th, td { text-align: left; padding: 7px 8px; border-bottom: 1px solid var(--line); vertical-align: top; }
  th { position: sticky; top: 0; background: #eef3f8; cursor: pointer; white-space: nowrap; }
  tr.outcome-regression td:first-child { box-shadow: inset 3px 0 0 var(--regression); }
  tr.outcome-improvement td:first-child { box-shadow: inset 3px 0 0 var(--improvement); }
  tr[data-cause], tbody tr.outcome-regression, tbody tr.outcome-improvement,
  tbody tr.outcome-both_failed, tbody tr.outcome-both_passed,
  tbody tr.outcome-incomplete, tbody tr.outcome-base_only, tbody tr.outcome-candidate_only { cursor: pointer; }
  tr.selected td { background: #f3f7fb; }
  tr.detail td { background: #f8fafc; color: var(--ink); }
  .status { font-weight: 650; }
  .status-succeeded { color: var(--improvement); }
  .status-failed { color: var(--regression); }
  .status-absent, .status-incomplete { color: var(--muted); }
  .params { color: var(--muted); max-width: 360px; overflow-wrap: anywhere; }
  .cause { overflow-wrap: anywhere; }
  .chip {
    display: inline-flex; gap: 6px; align-items: center;
    background: #e8eef6; border-radius: 999px; padding: 3px 10px; margin-bottom: 8px;
  }
  .chip button { border: 0; background: transparent; cursor: pointer; font: inherit; }
  .message { white-space: pre-wrap; overflow-wrap: anywhere; margin: 4px 0 0; }
  .side-title { font-weight: 650; }
  .empty { color: var(--muted); padding: 12px 0; }
  .num { text-align: right; font-variant-numeric: tabular-nums; }
  .pos { color: var(--improvement); }
  .neg { color: var(--regression); }
  @media (max-width: 900px) {
    .cards { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  }
</style>
</head>
<body>
<header>
  <h1 id="title">Evaluation comparison</h1>
  <p class="sub" id="subtitle"></p>
  <div class="run-links" id="run-links"></div>
</header>
<main>
  <section class="cards" id="stats"></section>
  <section class="panel">
    <div class="toolbar">
      <label class="toggle" for="hide-backend">
        <input type="checkbox" id="hide-backend" checked>
        Hide simulation backend failures
      </label>
      <label class="toggle" for="hide-scenario">
        <input type="checkbox" id="hide-scenario" checked>
        Hide scenario results (stand-still, collision, and other exitFailure checks)
      </label>
      <label>Run
        <select id="candidate"></select>
      </label>
    </div>
    <p class="help" id="hidden-note"></p>
  </section>
  <section class="panel">
    <h2>Failure causes on the scenarios that flipped</h2>
    <p class="help">Each flipped scenario is counted once. A regression is counted under the failure on this run. An improvement is counted under the failure on the base run. Net is improvements minus regressions. Positive means this run failed that cause on fewer flipped scenarios than the base.</p>
    <div id="causes"></div>
  </section>
  <section class="panel">
    <h2>Scenarios</h2>
    <div class="filters">
      <input type="search" id="search" placeholder="Filter by suite, scenario, parameter, or cause">
      <select id="suite"><option value="">All suites</option></select>
      <div class="checks" id="outcomes"></div>
    </div>
    <div id="cause-chip"></div>
    <p class="sub" id="shown"></p>
    <div id="table"></div>
  </section>
</main>
<script>
const DATA = /*__DATA__*/null;
const OUTCOME_LABELS = [
  ["regression", "Regressions"],
  ["improvement", "Improvements"],
  ["both_failed", "Both failed"],
  ["both_passed", "Both passed"],
  ["incomplete", "Incomplete"],
  ["base_only", "Only in base"],
  ["candidate_only", "Only in this run"],
];
const DEFAULT_OUTCOMES = new Set(["regression", "improvement"]);
const state = {
  candidate: 0,
  outcomes: new Set(DEFAULT_OUTCOMES),
  hideBackend: true,
  hideScenario: true,
  query: "",
  suite: "",
  cause: "",
  sortKey: "outcome",
  sortDir: 1,
  openKey: "",
};

function candidate() { return DATA.candidates[state.candidate]; }
function allRows() { return candidate().rows; }

function outcomeRank(outcome) {
  const order = ["regression", "improvement", "both_failed", "both_passed", "incomplete", "base_only", "candidate_only"];
  const index = order.indexOf(outcome);
  return index < 0 ? order.length : index;
}

function matchesQuery(row) {
  if (!state.query) return true;
  const haystack = [row.suite, row.scenario, row.parameters, row.cause, row.base_labels, row.candidate_labels, row.base_message, row.candidate_message].join("\n").toLowerCase();
  return haystack.includes(state.query);
}

function matchesScope(row) {
  return (!state.suite || row.suite === state.suite) && matchesQuery(row);
}

function analysisRows() {
  return allRows().filter((row) => {
    if (state.hideBackend && row.backend_failure) return false;
    if (state.hideScenario && row.scenario_failure) return false;
    return matchesScope(row);
  });
}

function displayedCause(row) {
  if (row.outcome === "regression") return row.candidate_cause || "(no cause label)";
  if (row.outcome === "improvement") return row.base_cause || "(no cause label)";
  return row.cause || "";
}

function tableRows(rows) {
  return rows.filter((row) => state.outcomes.has(row.outcome) && (!state.cause || displayedCause(row) === state.cause));
}

function hiddenCounts() {
  const scoped = allRows().filter(matchesScope);
  return {
    backend: state.hideBackend ? scoped.filter((row) => row.backend_failure).length : 0,
    scenario: state.hideScenario ? scoped.filter((row) => row.scenario_failure).length : 0,
  };
}

function counts(rows) {
  const tally = {regression: 0, improvement: 0, both_passed: 0, both_failed: 0};
  for (const row of rows) {
    if (tally[row.outcome] !== undefined) tally[row.outcome] += 1;
  }
  return tally;
}

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function anchor(href, text) {
  const node = el("a", "", text);
  node.href = href;
  node.target = "_blank";
  node.rel = "noopener";
  return node;
}

function renderHeader() {
  const run = candidate();
  document.title = `${run.name} against ${DATA.base.name}`;
  document.getElementById("title").textContent = `${run.name} against ${DATA.base.name}`;
  document.getElementById("subtitle").textContent =
    `${DATA.project_id} · ${DATA.generated_at} · a regression passed on the base and failed on this run · an improvement failed on the base and passed on this run`;
  const links = document.getElementById("run-links");
  links.replaceChildren();
  links.append(anchor(DATA.base.url, `Base: ${DATA.base.name}`));
  links.append(anchor(run.url, `This run: ${run.name}`));
}

function renderStats(rows) {
  const tally = counts(rows);
  const net = tally.improvement - tally.regression;
  const cards = [
    ["Regressions", String(tally.regression), "regression"],
    ["Improvements", String(tally.improvement), "improvement"],
    ["Net", (net > 0 ? "+" : "") + net, net > 0 ? "improvement" : net < 0 ? "regression" : ""],
    ["Both passed", String(tally.both_passed), ""],
    ["Both failed", String(tally.both_failed), ""],
  ];
  const host = document.getElementById("stats");
  host.replaceChildren();
  for (const [label, value, tone] of cards) {
    const card = el("div", "card" + (tone ? " " + tone : ""));
    card.append(el("div", "k", label), el("div", "v", value));
    host.append(card);
  }
  const hidden = hiddenCounts();
  const note = document.getElementById("hidden-note");
  const backendText = state.hideBackend
    ? `${hidden.backend} simulation backend failure${hidden.backend === 1 ? "" : "s"} hidden (SimulationError: the simulator or Autoware backend did not finish).`
    : "Simulation backend failures are included.";
  const scenarioText = state.hideScenario
    ? `${hidden.scenario} scenario result${hidden.scenario === 1 ? "" : "s"} hidden. SimulationFailure means a scenario condition fired exitFailure. Stand-still is one of those conditions: the ego stopped long enough for the scenario to fail the run. The simulator did finish. Uncheck this filter to list them.`
    : "Scenario results are included.";
  note.textContent = `${backendText} ${scenarioText}`;
}

function causeRows(rows) {
  const buckets = new Map();
  for (const row of rows) {
    let cause = "";
    let field = "";
    if (row.outcome === "regression" || row.outcome === "improvement") {
      cause = displayedCause(row);
      field = row.outcome;
    } else {
      continue;
    }
    if (!buckets.has(cause)) buckets.set(cause, {cause, regression: 0, improvement: 0});
    buckets.get(cause)[field] += 1;
  }
  return [...buckets.values()].sort((a, b) => b.regression - a.regression || b.improvement - a.improvement || a.cause.localeCompare(b.cause));
}

function renderCauses(rows) {
  const host = document.getElementById("causes");
  const buckets = causeRows(rows);
  if (!buckets.length) {
    host.replaceChildren(el("p", "empty", "No pass/fail flips in the current filter."));
    return;
  }
  const table = document.createElement("table");
  const head = document.createElement("thead");
  const headRow = document.createElement("tr");
  for (const label of ["Cause", "Regressions", "Improvements", "Net"]) {
    headRow.append(el("th", label === "Cause" ? "" : "num", label));
  }
  head.append(headRow);
  const body = document.createElement("tbody");
  let regressions = 0;
  let improvements = 0;
  for (const bucket of buckets) {
    regressions += bucket.regression;
    improvements += bucket.improvement;
    const net = bucket.improvement - bucket.regression;
    const tr = document.createElement("tr");
    tr.dataset.cause = bucket.cause;
    if (bucket.cause === state.cause) tr.className = "selected";
    tr.append(el("td", "cause", bucket.cause));
    tr.append(el("td", "num", String(bucket.regression)));
    tr.append(el("td", "num", String(bucket.improvement)));
    const netCell = el("td", "num " + (net > 0 ? "pos" : net < 0 ? "neg" : ""), (net > 0 ? "+" : "") + net);
    tr.append(netCell);
    body.append(tr);
  }
  const totalNet = improvements - regressions;
  const total = document.createElement("tr");
  total.append(el("td", "", "Total"));
  total.append(el("td", "num", String(regressions)));
  total.append(el("td", "num", String(improvements)));
  total.append(el("td", "num " + (totalNet > 0 ? "pos" : totalNet < 0 ? "neg" : ""), (totalNet > 0 ? "+" : "") + totalNet));
  body.append(total);
  table.append(head, body);
  host.replaceChildren(table);
  body.addEventListener("click", (event) => {
    const row = event.target.closest("tr[data-cause]");
    if (!row) return;
    state.cause = state.cause === row.dataset.cause ? "" : row.dataset.cause;
    state.outcomes = new Set(["regression", "improvement"]);
    syncOutcomeChecks();
    render();
  });
}

function statusText(status) {
  if (status === "succeeded") return "passed";
  if (status === "failed") return "failed";
  if (status === "absent") return "absent";
  return status || "unknown";
}

function statusNode(status, url) {
  const wrapper = document.createElement("div");
  wrapper.append(el("span", "status status-" + (status || "incomplete"), statusText(status)));
  if (url) {
    wrapper.append(document.createTextNode(" "));
    wrapper.append(anchor(url, "open"));
  }
  return wrapper;
}

function compareValue(row, key) {
  if (key === "outcome") return outcomeRank(row.outcome);
  if (key === "base_status") return row.base_status;
  if (key === "candidate_status") return row.candidate_status;
  return (row[key] || "").toLowerCase();
}

function renderTable(rows) {
  const visible = tableRows(rows);
  const sorted = visible.slice().sort((a, b) => {
    const left = compareValue(a, state.sortKey);
    const right = compareValue(b, state.sortKey);
    if (left < right) return -1 * state.sortDir;
    if (left > right) return 1 * state.sortDir;
    return a.scenario.localeCompare(b.scenario);
  });
  document.getElementById("shown").textContent = `Showing ${sorted.length} scenarios`;
  const chip = document.getElementById("cause-chip");
  chip.replaceChildren();
  if (state.cause) {
    const node = el("span", "chip");
    node.append(el("span", "", `Cause: ${state.cause}`));
    const button = el("button", "", "clear");
    button.type = "button";
    button.addEventListener("click", () => { state.cause = ""; render(); });
    node.append(button);
    chip.append(node);
  }
  const host = document.getElementById("table");
  if (!sorted.length) {
    host.replaceChildren(el("p", "empty", "No scenarios match the current filters."));
    return;
  }
  const columns = [
    ["suite", "Suite"],
    ["scenario", "Scenario"],
    ["parameters", "Parameters"],
    ["base_status", "Base"],
    ["candidate_status", "This run"],
    ["cause", "Cause"],
  ];
  const table = document.createElement("table");
  const head = document.createElement("thead");
  const headRow = document.createElement("tr");
  for (const [key, label] of columns) {
    const th = el("th", "", state.sortKey === key ? `${label} ${state.sortDir > 0 ? "↑" : "↓"}` : label);
    th.addEventListener("click", () => {
      if (state.sortKey === key) state.sortDir *= -1;
      else { state.sortKey = key; state.sortDir = 1; }
      render();
    });
    headRow.append(th);
  }
  head.append(headRow);
  const body = document.createElement("tbody");
  for (const row of sorted) {
    const key = [row.suite, row.scenario, row.parameters].join("\0");
    const tr = document.createElement("tr");
    tr.className = "outcome-" + row.outcome + (state.openKey === key ? " selected" : "");
    tr.append(el("td", "", row.suite));
    tr.append(el("td", "", row.scenario));
    const params = el("td", "params", row.parameters || "—");
    tr.append(params);
    const baseCell = el("td");
    baseCell.append(statusNode(row.base_status, row.base_url));
    tr.append(baseCell);
    const candidateCell = el("td");
    candidateCell.append(statusNode(row.candidate_status, row.candidate_url));
    tr.append(candidateCell);
    tr.append(el("td", "cause", displayedCause(row) || "—"));
    tr.addEventListener("click", (event) => {
      if (event.target.closest("a")) return;
      state.openKey = state.openKey === key ? "" : key;
      render();
    });
    body.append(tr);
    if (state.openKey === key) body.append(detailRow(row));
  }
  table.append(head, body);
  host.replaceChildren(table);
}

function detailBlock(title, status, labels, message, url, attempt, version) {
  const block = el("div");
  const heading = el("div", "side-title", title);
  block.append(heading);
  block.append(el("div", "", `${statusText(status)} · attempt ${attempt || "—"}${version == null ? "" : ` · scenario v${version}`}`));
  if (labels) block.append(el("div", "", labels));
  if (message) block.append(el("pre", "message", message));
  if (url) block.append(anchor(url, "Open this case"));
  return block;
}

function detailRow(row) {
  const tr = document.createElement("tr");
  tr.className = "detail";
  const cell = document.createElement("td");
  cell.colSpan = 6;
  const grid = document.createElement("div");
  grid.style.display = "grid";
  grid.style.gridTemplateColumns = "1fr 1fr";
  grid.style.gap = "16px";
  grid.append(detailBlock("Base", row.base_status, row.base_labels, row.base_message, row.base_url, row.base_attempt, row.base_version));
  grid.append(detailBlock("This run", row.candidate_status, row.candidate_labels, row.candidate_message, row.candidate_url, row.candidate_attempt, row.candidate_version));
  cell.append(grid);
  tr.append(cell);
  return tr;
}

function fillSuites() {
  const select = document.getElementById("suite");
  const suites = [...new Set(allRows().map((row) => row.suite).filter(Boolean))].sort((a, b) => a.localeCompare(b));
  const current = state.suite;
  select.replaceChildren(new Option("All suites", ""));
  for (const suite of suites) select.add(new Option(suite, suite));
  state.suite = suites.includes(current) ? current : "";
  select.value = state.suite;
}

function syncOutcomeChecks() {
  for (const input of document.querySelectorAll("#outcomes input")) {
    input.checked = state.outcomes.has(input.value);
  }
}

function render() {
  const rows = analysisRows();
  renderHeader();
  renderStats(rows);
  renderCauses(rows);
  renderTable(rows);
}

function init() {
  const select = document.getElementById("candidate");
  DATA.candidates.forEach((run, index) => select.add(new Option(run.name, String(index))));
  if (DATA.candidates.length < 2) select.parentElement.hidden = true;
  select.addEventListener("change", () => {
    state.candidate = Number(select.value);
    state.cause = "";
    state.openKey = "";
    fillSuites();
    render();
  });
  document.getElementById("hide-backend").addEventListener("change", (event) => {
    state.hideBackend = event.target.checked;
    render();
  });
  document.getElementById("hide-scenario").addEventListener("change", (event) => {
    state.hideScenario = event.target.checked;
    render();
  });
  document.getElementById("search").addEventListener("input", (event) => {
    state.query = event.target.value.trim().toLowerCase();
    render();
  });
  document.getElementById("suite").addEventListener("change", (event) => {
    state.suite = event.target.value;
    render();
  });
  const outcomes = document.getElementById("outcomes");
  for (const [value, label] of OUTCOME_LABELS) {
    const wrap = el("label");
    const input = document.createElement("input");
    input.type = "checkbox";
    input.value = value;
    input.checked = DEFAULT_OUTCOMES.has(value);
    input.addEventListener("change", () => {
      if (input.checked) state.outcomes.add(value);
      else state.outcomes.delete(value);
      render();
    });
    wrap.append(input, document.createTextNode(" " + label));
    outcomes.append(wrap);
  }
  fillSuites();
  render();
}

init();
</script>
</body>
</html>
"""


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, json.JSONDecodeError, subprocess.SubprocessError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
