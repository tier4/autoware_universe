"""Deterministic linear residual trainer used as the first CPU-friendly artifact."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import yaml

FEATURES = [
    "state_x",
    "state_y",
    "state_yaw",
    "state_vx",
    "state_vy",
    "state_wz",
    "command_velocity",
    "command_acceleration",
    "command_steer",
    "command_vx",
    "command_vy",
    "command_wz",
]
TARGETS = ["next_x", "next_y", "next_yaw", "next_vx", "next_vy", "next_wz"]


def _load_matrix(dataset_csv: str) -> tuple[np.ndarray, np.ndarray]:
    rows = list(csv.DictReader(open(dataset_csv)))
    x = np.array([[float(row.get(name, 0.0)) for name in FEATURES] for row in rows], dtype=float)
    y = np.array([[float(row.get(name, 0.0)) for name in TARGETS] for row in rows], dtype=float)
    return x, y


def train_linear_model(dataset_csv: str, output_dir: str, ridge: float = 1.0e-6) -> dict:
    x, y = _load_matrix(dataset_csv)
    if x.size == 0:
        raise RuntimeError("empty dataset")
    mean = x.mean(axis=0)
    std = x.std(axis=0)
    std[std < 1.0e-9] = 1.0
    xn = (x - mean) / std
    design = np.hstack([xn, np.ones((xn.shape[0], 1))])
    lhs = design.T @ design + ridge * np.eye(design.shape[1])
    weights = np.linalg.solve(lhs, design.T @ y)
    raw_weights = np.zeros_like(weights)
    raw_weights[:-1, :] = weights[:-1, :] / std[:, None]
    raw_weights[-1, :] = weights[-1, :] - (mean / std) @ weights[:-1, :]
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    np.savetxt(output / "linear_weights.csv", raw_weights, delimiter=",")
    artifact = {
        "backend": "linear_cpu",
        "model_type": "direct_next_state",
        "features": FEATURES,
        "targets": TARGETS,
        "normalization": {
            "mean": mean.tolist(),
            "std": std.tolist(),
        },
        "weights_csv": "linear_weights.csv",
        "determinism": {
            "provider": "numpy_cpu",
            "thread_count": 1,
            "fallback": "baseline",
        },
    }
    with (output / "model_artifact.yaml").open("w") as f:
        yaml.safe_dump(artifact, f, sort_keys=False)
    with (output / "normalization.yaml").open("w") as f:
        yaml.safe_dump(artifact["normalization"], f, sort_keys=False)
    return artifact


def export_metadata(model_dir: str, output_path: str) -> dict:
    with open(Path(model_dir) / "model_artifact.yaml") as f:
        artifact = yaml.safe_load(f)
    artifact["exported_artifact"] = str(output_path)
    with open(output_path, "w") as f:
        yaml.safe_dump(artifact, f, sort_keys=False)
    return artifact

