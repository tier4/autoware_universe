"""Pure dataset filtering helpers.

These functions are intentionally independent from ROS so they can be unit-tested without bags.
"""

from __future__ import annotations

from collections import Counter
from math import atan2, cos, sin
from statistics import median
from typing import Iterable


def wrap_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


def unwrap_yaw(yaws: Iterable[float]) -> list[float]:
    values = list(yaws)
    if not values:
        return []
    unwrapped = [values[0]]
    for prev, cur in zip(values[:-1], values[1:]):
        unwrapped.append(unwrapped[-1] + wrap_angle(cur - prev))
    return unwrapped


def robust_z_scores(values: Iterable[float]) -> list[float]:
    vals = list(values)
    if not vals:
        return []
    med = median(vals)
    deviations = [abs(v - med) for v in vals]
    mad = median(deviations)
    scale = 1.4826 * mad
    if scale <= 1.0e-12:
        return [0.0 for _ in vals]
    return [abs(v - med) / scale for v in vals]


def classify_maneuver(sample: dict, thresholds: dict) -> str:
    vx = abs(float(sample.get("vx", 0.0)))
    vy = abs(float(sample.get("vy", 0.0)))
    wz = float(sample.get("wz", 0.0))
    ax = float(sample.get("ax", 0.0))
    if vx < thresholds.get("stop_speed", 0.1) and vy < thresholds.get("stop_speed", 0.1):
        return "stop"
    if vy > thresholds.get("lateral_speed", 0.1):
        return "lateral_motion"
    if wz > thresholds.get("turn_abs_wz", 0.1):
        return "left_turn"
    if wz < -thresholds.get("turn_abs_wz", 0.1):
        return "right_turn"
    if ax > thresholds.get("accel_abs_ax", 0.3):
        return "acceleration"
    if ax < -thresholds.get("accel_abs_ax", 0.3):
        return "deceleration"
    if abs(wz) < thresholds.get("straight_abs_wz", 0.05):
        return "straight"
    return "high_curvature"


def validate_maneuver_coverage(samples: list[dict], required: dict) -> tuple[dict, list[str]]:
    counts = Counter(sample.get("maneuver", "unknown") for sample in samples)
    failures: list[str] = []
    for name, rule in required.items():
        min_samples = int(rule.get("min_samples", 0))
        if counts.get(name, 0) < min_samples:
            failures.append(
                f"maneuver '{name}' has {counts.get(name, 0)} samples, required {min_samples}"
            )
    return dict(counts), failures


def split_by_segment(samples: list[dict], train_ratio: float, validation_ratio: float) -> dict[str, list[dict]]:
    segments = sorted({sample.get("segment_id", "default") for sample in samples})
    n_segments = max(len(segments), 1)
    train_cut = int(n_segments * train_ratio)
    validation_cut = int(n_segments * (train_ratio + validation_ratio))
    train_segments = set(segments[:train_cut])
    validation_segments = set(segments[train_cut:validation_cut])
    return {
        "train": [s for s in samples if s.get("segment_id", "default") in train_segments],
        "validation": [s for s in samples if s.get("segment_id", "default") in validation_segments],
        "test": [s for s in samples if s.get("segment_id", "default") not in train_segments | validation_segments],
    }

