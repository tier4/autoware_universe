import numpy as np
import yaml

from autoware_data_driven_planning_simulator.dataset.filters import (
    split_by_segment,
    unwrap_yaw,
    validate_maneuver_coverage,
)
from autoware_data_driven_planning_simulator.evaluation.cli import parse_horizons
from autoware_data_driven_planning_simulator.evaluation.evaluator import evaluate_dataset


DATASET_HEADER = (
    "segment_id,t,state_x,state_y,state_yaw,state_vx,state_vy,state_wz,"
    "command_velocity,command_acceleration,command_steer,command_vx,command_vy,command_wz,"
    "next_x,next_y,next_yaw,next_vx,next_vy,next_wz\n"
)


def _straight_row(segment_id: str, index: int, dt: float = 0.1) -> str:
    x = index * dt
    next_x = (index + 1) * dt
    return (
        f"{segment_id},{index * dt},{x},0.0,0.0,1.0,0.0,0.0,"
        f"1.0,0.0,0.0,1.0,0.0,0.0,{next_x},0.0,0.0,1.0,0.0,0.0\n"
    )


def _write_constant_velocity_linear_artifact(output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    weights = np.zeros((13, 6))
    weights[0, 0] = 1.0
    weights[-1, 0] = 0.1
    weights[-1, 3] = 1.0
    np.savetxt(output_dir / "linear_weights.csv", weights, delimiter=",")
    artifact = {
        "backend": "linear_cpu",
        "model_type": "direct_next_state",
        "weights_csv": "linear_weights.csv",
    }
    artifact_path = output_dir / "model_artifact.yaml"
    artifact_path.write_text(yaml.safe_dump(artifact, sort_keys=False))
    return artifact_path


def _write_constant_residual_mlp_artifact(output_dir):
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_dir / "mlp_weights.npz",
        w1=np.zeros((12, 1)),
        b1=np.zeros(1),
        w2=np.zeros((1, 6)),
        b2=np.zeros(6),
        x_mean=np.zeros(12),
        x_std=np.ones(12),
        y_mean=np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]),
        y_std=np.ones(6),
    )
    artifact = {
        "backend": "numpy_mlp_cpu",
        "model_type": "mlp_residual",
        "weights_npz": "mlp_weights.npz",
    }
    artifact_path = output_dir / "model_artifact.yaml"
    artifact_path.write_text(yaml.safe_dump(artifact, sort_keys=False))
    return artifact_path


def _synthetic_bag_equivalent_dataset(row_count: int = 20) -> str:
    return DATASET_HEADER + "".join(_straight_row("synthetic_bag", index) for index in range(row_count))


def test_unwrap_yaw_crosses_pi_boundary():
    values = [3.13, -3.13]
    unwrapped = unwrap_yaw(values)
    assert unwrapped[1] > unwrapped[0]
    assert abs((unwrapped[1] - unwrapped[0]) - 0.023185307179586445) < 1.0e-6


def test_validate_maneuver_coverage_fails_missing_required_class():
    samples = [{"maneuver": "straight"} for _ in range(3)]
    counts, failures = validate_maneuver_coverage(
        samples,
        {
            "straight": {"min_samples": 2},
            "left_turn": {"min_samples": 1},
        },
    )
    assert counts["straight"] == 3
    assert failures


def test_split_by_segment_keeps_segments_together():
    samples = [
        {"segment_id": "a", "value": 1},
        {"segment_id": "a", "value": 2},
        {"segment_id": "b", "value": 3},
        {"segment_id": "c", "value": 4},
    ]
    splits = split_by_segment(samples, 0.34, 0.33)
    locations = {}
    for split_name, split_samples in splits.items():
        for sample in split_samples:
            locations.setdefault(sample["segment_id"], split_name)
            assert locations[sample["segment_id"]] == split_name


def test_evaluator_compares_vehicle_profile_baseline(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(
        "t,state_x,state_y,state_yaw,state_vx,state_vy,state_wz,"
        "command_velocity,command_acceleration,command_steer,command_vx,command_vy,command_wz,"
        "next_x,next_y,next_yaw,next_vx,next_vy,next_wz\n"
        "0.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,0.1,0.0,0.0,1.0,0.0,0.0\n"
    )
    report = evaluate_dataset(
        str(dataset),
        str(tmp_path / "eval"),
        vehicle_profile={"baseline": {"type": "ackermann", "wheelbase": 2.7}},
        sampling_period_s=0.1,
    )
    assert report["sample_count"] == 1
    assert "vehicle_profile_baseline" in report["one_step"]
    assert report["one_step"]["vehicle_profile_baseline"]["position_rmse"] < 1.0e-9


def test_evaluator_rollout_matches_straight_ackermann_segment(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(DATASET_HEADER + "".join(_straight_row("a", index) for index in range(5)))
    report = evaluate_dataset(
        str(dataset),
        str(tmp_path / "eval"),
        vehicle_profile={"baseline": {"type": "ackermann", "wheelbase": 2.7}},
        sampling_period_s=0.1,
        horizons_s=[0.3],
    )
    horizon = report["rollout"]["0.3"]
    assert horizon["nominal_step_count"] == 3
    assert horizon["evaluated_start_count"] == 3
    assert horizon["models"]["vehicle_profile_baseline"]["position_rmse"] < 1.0e-9
    assert report["segments"]["a"]["rollout_coverage"]["0.3"]["evaluated_start_count"] == 3


def test_evaluator_reports_skipped_horizon_when_segment_is_too_short(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(DATASET_HEADER + "".join(_straight_row("a", index) for index in range(2)))
    report = evaluate_dataset(str(dataset), str(tmp_path / "eval"), sampling_period_s=0.1, horizons_s=[1.0])
    horizon = report["rollout"]["1.0"]
    assert horizon["nominal_step_count"] == 10
    assert horizon["evaluated_start_count"] == 0
    assert horizon["skipped_start_count"] == 2
    assert horizon["models"] == {}


def test_evaluator_rollout_does_not_cross_segment_boundaries(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(
        DATASET_HEADER
        + "".join(_straight_row("a", index) for index in range(2))
        + "".join(_straight_row("b", index) for index in range(2))
    )
    report = evaluate_dataset(str(dataset), str(tmp_path / "eval"), sampling_period_s=0.1, horizons_s=[0.2])
    horizon = report["rollout"]["0.2"]
    assert horizon["nominal_step_count"] == 2
    assert horizon["evaluated_start_count"] == 2
    assert report["segments"]["a"]["rollout_coverage"]["0.2"]["evaluated_start_count"] == 1
    assert report["segments"]["b"]["rollout_coverage"]["0.2"]["evaluated_start_count"] == 1


def test_evaluator_rolls_out_linear_learned_artifact(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(DATASET_HEADER + "".join(_straight_row("a", index) for index in range(5)))
    artifact_path = _write_constant_velocity_linear_artifact(tmp_path / "artifact")
    report = evaluate_dataset(
        str(dataset),
        str(tmp_path / "eval"),
        model_artifact=str(artifact_path),
        sampling_period_s=0.1,
        horizons_s=[0.3],
    )
    horizon = report["rollout"]["0.3"]
    assert "learned" in horizon["models"]
    assert horizon["models"]["learned"]["position_rmse"] < 1.0e-9


def test_evaluator_rolls_out_mlp_learned_artifact(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(DATASET_HEADER + "".join(_straight_row("a", index) for index in range(5)))
    artifact_path = _write_constant_residual_mlp_artifact(tmp_path / "artifact")
    report = evaluate_dataset(
        str(dataset),
        str(tmp_path / "eval"),
        model_artifact=str(artifact_path),
        sampling_period_s=0.1,
        horizons_s=[0.3],
    )
    horizon = report["rollout"]["0.3"]
    assert "learned" in horizon["models"]
    assert horizon["models"]["learned"]["position_rmse"] < 1.0e-9


def test_evaluator_uses_segment_timestamps_for_irregular_rollouts(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(
        DATASET_HEADER
        + "a,0.0,0.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,0.1,0.0,0.0,1.0,0.0,0.0\n"
        + "a,0.1,0.1,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,0.3,0.0,0.0,1.0,0.0,0.0\n"
        + "a,0.3,0.3,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,1.0,0.0,0.0,0.45,0.0,0.0,1.0,0.0,0.0\n"
    )
    report = evaluate_dataset(
        str(dataset),
        str(tmp_path / "eval"),
        vehicle_profile={"baseline": {"type": "ackermann", "wheelbase": 2.7}},
        sampling_period_s=0.1,
        horizons_s=[0.25],
    )
    horizon = report["rollout"]["0.25"]
    assert horizon["step_count_min"] == 2
    assert horizon["step_count_max"] == 2
    assert horizon["evaluated_start_count"] == 2
    assert horizon["models"]["vehicle_profile_baseline"]["position_rmse"] < 1.0e-9


def test_evaluator_segment_report_contains_acceptance_fields(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(DATASET_HEADER + "".join(_straight_row("segment_a", index) for index in range(4)))
    report = evaluate_dataset(str(dataset), str(tmp_path / "eval"), sampling_period_s=0.1, horizons_s=[0.2])
    segment = report["segments"]["segment_a"]
    assert segment["sample_count"] == 4
    assert "one_step" in segment
    assert "vehicle_profile_baseline" in segment["one_step"]
    coverage = segment["rollout_coverage"]["0.2"]
    assert set(coverage) == {"nominal_step_count", "evaluated_start_count", "skipped_start_count"}
    assert coverage["nominal_step_count"] == 2
    assert coverage["evaluated_start_count"] == 3


def test_evaluator_handles_synthetic_bag_equivalent_dataset(tmp_path):
    dataset = tmp_path / "dataset.csv"
    dataset.write_text(_synthetic_bag_equivalent_dataset())
    report = evaluate_dataset(
        str(dataset),
        str(tmp_path / "eval"),
        vehicle_profile={"baseline": {"type": "ackermann", "wheelbase": 2.7}},
        sampling_period_s=0.1,
        horizons_s=[1.0, 3.0],
    )
    assert report["segment_count"] == 1
    assert report["segments"]["synthetic_bag"]["sample_count"] == 20
    assert report["rollout"]["1.0"]["evaluated_start_count"] > 0
    assert report["rollout"]["3.0"]["evaluated_start_count"] == 0
    assert report["rollout"]["1.0"]["models"]["vehicle_profile_baseline"]["position_rmse"] < 1.0e-9


def test_parse_horizons_rejects_invalid_values():
    assert parse_horizons("0.1, 1.0") == [0.1, 1.0]
    for value in ("", "0", "-1.0", "nan", "abc"):
        try:
            parse_horizons(value)
        except Exception:
            continue
        raise AssertionError(f"expected invalid horizon value: {value}")

