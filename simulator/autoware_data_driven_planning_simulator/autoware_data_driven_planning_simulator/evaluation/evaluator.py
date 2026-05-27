"""Offline one-step and short-horizon rollout evaluation."""

from __future__ import annotations

import csv
from math import atan2, cos, sin, sqrt
from pathlib import Path

import numpy as np
import yaml

from autoware_data_driven_planning_simulator.baseline import Command, State, step_baseline
from autoware_data_driven_planning_simulator.training.linear_model import FEATURES


def _rmse(errors: list[float]) -> float:
    if not errors:
        return 0.0
    return sqrt(sum(e * e for e in errors) / len(errors))


def _mean(values: list[float]) -> float:
    if not values:
        return 0.0
    return sum(values) / len(values)


def _wrap(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def _state_from_row(row: dict, prefix: str = "state") -> State:
    return State(
        x=float(row.get(f"{prefix}_x", 0.0)),
        y=float(row.get(f"{prefix}_y", 0.0)),
        yaw=float(row.get(f"{prefix}_yaw", 0.0)),
        vx=float(row.get(f"{prefix}_vx", 0.0)),
        vy=float(row.get(f"{prefix}_vy", 0.0)),
        wz=float(row.get(f"{prefix}_wz", 0.0)),
        steer=float(row.get(f"{prefix}_steer", 0.0)),
        ax=float(row.get(f"{prefix}_ax", 0.0)),
    )


def _command_from_row(row: dict) -> Command:
    return Command(
        velocity=float(row.get("command_velocity", 0.0)),
        acceleration=float(row.get("command_acceleration", 0.0)),
        steer=float(row.get("command_steer", 0.0)),
        vx=float(row.get("command_vx", 0.0)),
        vy=float(row.get("command_vy", 0.0)),
        wz=float(row.get("command_wz", 0.0)),
    )


def _position_error(predicted: State, truth: State) -> float:
    return sqrt((predicted.x - truth.x) ** 2 + (predicted.y - truth.y) ** 2)


def _yaw_error(predicted: State, truth: State) -> float:
    return _wrap(predicted.yaw - truth.yaw)


def _feature_vector(row: dict) -> np.ndarray:
    return np.array([float(row.get(name, 0.0)) for name in FEATURES] + [1.0], dtype=float)


def _row_with_state(row: dict, state: State) -> dict:
    updated = dict(row)
    updated["state_x"] = str(state.x)
    updated["state_y"] = str(state.y)
    updated["state_yaw"] = str(state.yaw)
    updated["state_vx"] = str(state.vx)
    updated["state_vy"] = str(state.vy)
    updated["state_wz"] = str(state.wz)
    return updated


def _load_artifact(model_artifact: str | None) -> dict | None:
    if model_artifact is None:
        return None
    artifact_path = Path(model_artifact)
    with artifact_path.open() as f:
        artifact = yaml.safe_load(f)
    artifact["_base_dir"] = artifact_path.parent
    return artifact


def _predict_linear(row: dict, weights: np.ndarray) -> State:
    y = _feature_vector(row) @ weights
    return State(x=float(y[0]), y=float(y[1]), yaw=float(y[2]), vx=float(y[3]), vy=float(y[4]), wz=float(y[5]))


def _predict_mlp(row: dict, artifact: dict) -> State:
    data = artifact.get("_weights")
    if data is None:
        data = np.load(Path(artifact["_base_dir"]) / artifact.get("weights_npz", "mlp_weights.npz"))
        artifact["_weights"] = data
    x = np.array([float(row.get(name, 0.0)) for name in FEATURES], dtype=float)
    xn = (x - data["x_mean"]) / data["x_std"]
    h = np.tanh(xn @ data["w1"] + data["b1"])
    y = (h @ data["w2"] + data["b2"]) * data["y_std"] + data["y_mean"]
    if artifact.get("model_type") == "mlp_residual":
        state = _state_from_row(row)
        y = y + np.array([state.x, state.y, state.yaw, state.vx, state.vy, state.wz], dtype=float)
    return State(x=float(y[0]), y=float(y[1]), yaw=float(y[2]), vx=float(y[3]), vy=float(y[4]), wz=float(y[5]))


def _infer_dt(rows: list[dict], default: float) -> float:
    times = [float(row.get("t", 0.0)) for row in rows]
    diffs = [b - a for a, b in zip(times[:-1], times[1:]) if b > a]
    if not diffs:
        return default
    return float(np.median(diffs))


def _row_time(row: dict, fallback: float) -> float:
    return float(row.get("t", fallback))


def _transition_dt(segment_rows: list[dict], index: int, fallback: float) -> float:
    if index + 1 >= len(segment_rows):
        return fallback
    current_t = _row_time(segment_rows[index], float(index) * fallback)
    next_t = _row_time(segment_rows[index + 1], float(index + 1) * fallback)
    dt = next_t - current_t
    return dt if dt > 0.0 else fallback


def _group_segments(rows: list[dict]) -> list[tuple[str, list[dict]]]:
    grouped: dict[str, list[tuple[int, dict]]] = {}
    for index, row in enumerate(rows):
        segment_id = row.get("segment_id") or "__default__"
        grouped.setdefault(segment_id, []).append((index, row))

    segments: list[tuple[str, list[dict]]] = []
    for segment_id in sorted(grouped):
        indexed_rows = grouped[segment_id]
        indexed_rows.sort(key=lambda item: (float(item[1].get("t", item[0])), item[0]))
        segments.append((segment_id, [row for _, row in indexed_rows]))
    return segments


def _metric_block(position_errors: list[float], yaw_errors: list[float]) -> dict:
    return {
        "position_rmse": _rmse(position_errors),
        "yaw_rmse": _rmse(yaw_errors),
    }


def _empty_error_store() -> dict:
    return {"position": [], "yaw": []}


def _predict_one_step(
    model_name: str,
    state: State,
    command_row: dict,
    dt: float,
    baseline_type: str,
    wheelbase: float,
    learned_weights: np.ndarray | None,
    learned_artifact: dict | None,
) -> State:
    if model_name == "no_motion":
        return state
    if model_name == "vehicle_profile_baseline":
        return step_baseline(state, _command_from_row(command_row), dt, baseline_type, wheelbase)
    feature_row = _row_with_state(command_row, state)
    if model_name == "learned" and learned_weights is not None:
        return _predict_linear(feature_row, learned_weights)
    if model_name == "learned" and learned_artifact is not None and learned_artifact.get("backend") == "numpy_mlp_cpu":
        return _predict_mlp(feature_row, learned_artifact)
    raise ValueError(f"unsupported evaluator model: {model_name}")


def _available_models(learned_weights: np.ndarray | None, learned_artifact: dict | None) -> list[str]:
    models = ["no_motion", "vehicle_profile_baseline"]
    if learned_weights is not None or (
        learned_artifact is not None and learned_artifact.get("backend") == "numpy_mlp_cpu"
    ):
        models.append("learned")
    return models


def _horizon_steps(horizons_s: list[float], dt: float) -> dict[str, int]:
    return {str(horizon): max(1, int(round(horizon / max(dt, 1.0e-9)))) for horizon in horizons_s}


def _rollout_windows(segment_rows: list[dict], horizon_s: float, fallback_dt: float) -> list[tuple[int, int, float]]:
    windows = []
    for start in range(len(segment_rows)):
        elapsed = 0.0
        for end in range(start, len(segment_rows)):
            elapsed += _transition_dt(segment_rows, end, fallback_dt)
            if elapsed + 1.0e-9 >= horizon_s:
                windows.append((start, end, elapsed))
                break
    return windows


def _evaluate_one_step(
    segments: list[tuple[str, list[dict]]],
    models: list[str],
    dt: float,
    baseline_type: str,
    wheelbase: float,
    learned_weights: np.ndarray | None,
    learned_artifact: dict | None,
) -> tuple[dict, dict]:
    all_errors = {model: _empty_error_store() for model in models}
    segment_report = {}
    for segment_id, segment_rows in segments:
        segment_errors = {model: _empty_error_store() for model in models}
        for index, row in enumerate(segment_rows):
            row_dt = _transition_dt(segment_rows, index, dt)
            state = _state_from_row(row)
            truth = _state_from_row(row, "next")
            for model in models:
                predicted = _predict_one_step(
                    model, state, row, row_dt, baseline_type, wheelbase, learned_weights, learned_artifact
                )
                position_error = _position_error(predicted, truth)
                yaw_error = _yaw_error(predicted, truth)
                all_errors[model]["position"].append(position_error)
                all_errors[model]["yaw"].append(yaw_error)
                segment_errors[model]["position"].append(position_error)
                segment_errors[model]["yaw"].append(yaw_error)
        segment_report[segment_id] = {
            "sample_count": len(segment_rows),
            "one_step": {
                model: _metric_block(errors["position"], errors["yaw"])
                for model, errors in segment_errors.items()
                if errors["position"]
            },
        }
    one_step = {
        model: _metric_block(errors["position"], errors["yaw"])
        for model, errors in all_errors.items()
        if errors["position"]
    }
    return one_step, segment_report


def _evaluate_rollouts(
    segments: list[tuple[str, list[dict]]],
    models: list[str],
    dt: float,
    baseline_type: str,
    wheelbase: float,
    learned_weights: np.ndarray | None,
    learned_artifact: dict | None,
    horizons_s: list[float],
) -> tuple[dict, dict]:
    steps_by_horizon = _horizon_steps(horizons_s, dt)
    rollout_report = {}
    segment_coverage = {segment_id: {} for segment_id, _ in segments}

    for horizon_s, step_count in steps_by_horizon.items():
        model_errors = {model: _empty_error_store() | {"fde": []} for model in models}
        evaluated_start_count = 0
        skipped_start_count = 0
        elapsed_times = []
        step_counts = []
        for segment_id, segment_rows in segments:
            windows = _rollout_windows(segment_rows, float(horizon_s), dt)
            possible_starts = len(windows)
            skipped = max(len(segment_rows) - possible_starts, 0)
            skipped_start_count += skipped
            segment_coverage[segment_id][horizon_s] = {
                "nominal_step_count": step_count,
                "evaluated_start_count": possible_starts,
                "skipped_start_count": skipped,
            }
            for start, end, elapsed in windows:
                evaluated_start_count += 1
                elapsed_times.append(elapsed)
                step_counts.append(end - start + 1)
                truth = _state_from_row(segment_rows[end], "next")
                for model in models:
                    predicted = _state_from_row(segment_rows[start])
                    for offset in range(start, end + 1):
                        row_dt = _transition_dt(segment_rows, offset, dt)
                        predicted = _predict_one_step(
                            model,
                            predicted,
                            segment_rows[offset],
                            row_dt,
                            baseline_type,
                            wheelbase,
                            learned_weights,
                            learned_artifact,
                        )
                    position_error = _position_error(predicted, truth)
                    model_errors[model]["position"].append(position_error)
                    model_errors[model]["yaw"].append(_yaw_error(predicted, truth))
                    model_errors[model]["fde"].append(position_error)
        rollout_report[horizon_s] = {
            "nominal_step_count": step_count,
            "step_count_min": min(step_counts) if step_counts else 0,
            "step_count_max": max(step_counts) if step_counts else 0,
            "mean_elapsed_s": _mean(elapsed_times),
            "evaluated_start_count": evaluated_start_count,
            "skipped_start_count": skipped_start_count,
            "models": {
                model: {
                    **_metric_block(errors["position"], errors["yaw"]),
                    "final_displacement_error": _mean(errors["fde"]),
                }
                for model, errors in model_errors.items()
                if errors["position"]
            },
        }
    return rollout_report, segment_coverage


def evaluate_dataset(
    dataset_csv: str,
    output_dir: str,
    vehicle_profile: dict | None = None,
    model_artifact: str | None = None,
    sampling_period_s: float = 0.1,
    horizons_s: list[float] | None = None,
) -> dict:
    rows: list[dict] = []
    with open(dataset_csv) as f:
        rows.extend(csv.DictReader(f))
    dt = _infer_dt(rows, sampling_period_s)
    baseline_type = (vehicle_profile or {}).get("baseline", {}).get("type", "ackermann")
    wheelbase = float((vehicle_profile or {}).get("baseline", {}).get("wheelbase", 2.7))
    learned_artifact = _load_artifact(model_artifact)
    learned_weights = None
    if learned_artifact is not None and learned_artifact.get("backend") == "linear_cpu":
        weights_path = Path(learned_artifact["_base_dir"]) / learned_artifact.get("weights_csv", "linear_weights.csv")
        learned_weights = np.loadtxt(weights_path, delimiter=",")

    segments = _group_segments(rows)
    models = _available_models(learned_weights, learned_artifact)
    one_step, segment_report = _evaluate_one_step(
        segments, models, dt, baseline_type, wheelbase, learned_weights, learned_artifact
    )
    if "learned" in one_step and "vehicle_profile_baseline" in one_step:
        base = one_step["vehicle_profile_baseline"]["position_rmse"]
        learned = one_step["learned"]["position_rmse"]
        one_step["learned"]["baseline_improvement_ratio"] = 0.0 if base == 0.0 else 1.0 - learned / base
    requested_horizons = horizons_s or [1.0, 3.0, 5.0, 10.0]
    rollout, segment_coverage = _evaluate_rollouts(
        segments,
        models,
        dt,
        baseline_type,
        wheelbase,
        learned_weights,
        learned_artifact,
        requested_horizons,
    )
    for segment_id, coverage in segment_coverage.items():
        segment_report[segment_id]["rollout_coverage"] = coverage

    report = {
        "sample_count": len(rows),
        "segment_count": len(segments),
        "sampling_period_s": dt,
        "one_step": one_step,
        "rollout_horizons_s": requested_horizons,
        "rollout": rollout,
        "segments": segment_report,
        "complete_bag_drift_is_diagnostic": True,
    }
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "evaluation_report.yaml").open("w") as f:
        yaml.safe_dump(report, f, sort_keys=False)
    return report

