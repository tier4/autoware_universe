#!/usr/bin/env python3
# Copyright 2026 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Live or offline comparison of diffusion-planner reference vs MPPI-optimized trajectories.

Offline logs (written when enable_debug_trajectory_log:=true):
  <log_dir>/index.csv
  <log_dir>/cost_params.csv
  <log_dir>/vehicle_params.csv
  <log_dir>/000000_reference.csv
  <log_dir>/000000_optimized.csv
  <log_dir>/000000_ego.csv
  ...

Trajectory CSV columns:
  t_from_start_s,x,y,z,yaw,v,a,steer,steer_rate
Ego CSV columns:
  x,y,z,yaw,v,accel,steer

Retune also writes <out_dir>/NNNNNN_costs.csv:
  rollout_index,raw_cost,normalized_weight
(used for cost / weight distribution histograms in --enable-retune mode).

and <out_dir>/NNNNNN_rollouts.csv:
  rollout_index,cost,step,x,y
(top-K weighted sample trajectories overlaid on the XY plot).

Offline retune mode (--enable-retune) overlays a third retuned trajectory and lets you
re-run mppi_offline_retune with editable cost weights.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass
from dataclasses import field
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
from typing import Deque
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import SteeringReport
import matplotlib
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.widgets import Slider
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Bool

# Rolling window for live measured tire-angle / command history.
MEASURED_STEER_HISTORY_S = 16.0
# Rolling window for cycle-to-cycle replan ADE history.
REPLAN_ADE_HISTORY_S = 16.0
# Drop this many leading points from the previous plan before comparing (one
# planning cycle at 10 Hz with MPPI_DT = 0.1 s).
REPLAN_TIME_SHIFT = 1
# Fallback only — same default as FirstOrderDubinsMppiVehicleParams. Prefer ROS /
# vehicle_params.csv (the τ MPPI already uses in FirstOrderDubinsBicycle).
VEHICLE_PARAMS_DEFAULT_STEER_TIME_CONSTANT = 0.27
VEHICLE_PARAMS_DEFAULT_WHEEL_BASE = 4.76
# Must match first_order_dubins_mppi_interface.cu kDt.
MPPI_DT = 0.1

# Must match first_order_dubins_mppi_interface.cu (kNumRollouts / kMaxVizRollouts).
MPPI_NUM_ROLLOUTS = 32 * 1024
MPPI_MAX_VIZ_ROLLOUTS = 200

# Numerical cost params from mppi_optimizer.param.yaml / FirstOrderDubinsMppiCostParams.
# (Excludes bool/string runtime flags: enable_debug_trajectory_log, ignore_*, etc.)
DEFAULT_PARAMS: Dict[str, float] = {
    "lambda": 14000.0,
    "desired_speed": 2.5,
    "speed_coeff": 500.0,
    "track_coeff": 3000.0,
    "track_terminal_scale": 10.0,
    "heading_coeff": 1000.0,
    "lateral_distance_coeff": 0.0,
    "lateral_yaw_error_coeff": 0.0,
    "crash_coeff": 100000.0,
    "boundary_threshold": 0.8,
    "boundary_threshold_left": -1.0,
    "boundary_threshold_right": -1.0,
    "lateral_acceleration_coeff": 500.0,
    "lateral_jerk_coeff": 1000.0,
    "longitudinal_jerk_coeff": 10.0,
    "accel_cmd_coeff": 0.0,
    "steer_cmd_coeff": 10.0,
    "steer_rate_coeff": 0.0,  # cost param; not always present in yaml
    "obstacle_collision_margin": 0.2,
}

# (name, vmin, vmax) — keep in sync with DEFAULT_PARAMS keys.
SLIDER_SPECS: List[Tuple[str, float, float]] = [
    ("lambda", 100.0, 20000.0),
    ("desired_speed", 0.0, 20.0),
    ("track_coeff", 0.0, 10000.0),
    ("track_terminal_scale", 0.0, 50.0),
    ("speed_coeff", 0.0, 5000.0),
    ("heading_coeff", 0.0, 5000.0),
    ("lateral_distance_coeff", 0.0, 10000.0),
    ("lateral_yaw_error_coeff", 0.0, 5000.0),
    ("lateral_acceleration_coeff", 0.0, 5000.0),
    ("lateral_jerk_coeff", 0.0, 10000.0),
    ("longitudinal_jerk_coeff", 0.0, 5000.0),
    ("accel_cmd_coeff", 0.0, 2000.0),
    ("steer_cmd_coeff", 0.0, 5000.0),
    ("steer_rate_coeff", 0.0, 10000.0),
    ("boundary_threshold", 0.1, 5.0),
    ("boundary_threshold_left", -1.0, 5.0),
    ("boundary_threshold_right", -1.0, 5.0),
    ("obstacle_collision_margin", 0.0, 2.0),
    ("crash_coeff", 0.0, 500000.0),
]


def yaw_from_pose(pose) -> float:
    q = pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def trajectory_xy(points) -> Tuple[List[float], List[float]]:
    return [p.pose.position.x for p in points], [p.pose.position.y for p in points]


def trajectory_heading(points) -> List[float]:
    return [yaw_from_pose(p.pose) for p in points]


def trajectory_velocity(points) -> List[float]:
    return [float(p.longitudinal_velocity_mps) for p in points]


def trajectory_acceleration(points) -> List[float]:
    return [float(p.acceleration_mps2) for p in points]


def trajectory_steering(points, wheel_base: float, *, prefer_message: bool = False) -> List[float]:
    """Use front_wheel_angle_rad when set; otherwise derive from discrete curvature."""
    if not points:
        return []

    if prefer_message:
        return [float(p.front_wheel_angle_rad) for p in points]

    headings = trajectory_heading(points)
    steer: List[float] = []
    for i, point in enumerate(points):
        if abs(point.front_wheel_angle_rad) > 1e-6:
            steer.append(float(point.front_wheel_angle_rad))
            continue

        if i + 1 >= len(points):
            steer.append(steer[-1] if steer else 0.0)
            continue

        p0 = points[i].pose.position
        p1 = points[i + 1].pose.position
        ds = math.hypot(p1.x - p0.x, p1.y - p0.y)
        if ds < 1e-6:
            steer.append(steer[-1] if steer else 0.0)
            continue

        dyaw = math.atan2(
            math.sin(headings[i + 1] - headings[i]),
            math.cos(headings[i + 1] - headings[i]),
        )
        curvature = dyaw / ds
        steer.append(math.atan(wheel_base * curvature))

    return steer


def finite_difference_acceleration(velocities: Sequence[float], dt: float) -> List[float]:
    if not velocities:
        return []
    if len(velocities) == 1 or dt <= 0.0:
        return [0.0] * len(velocities)

    accel = [(velocities[i + 1] - velocities[i]) / dt for i in range(len(velocities) - 1)]
    accel.append(0.0)
    return accel


def trajectory_steer_rate(points) -> List[float]:
    """Read cost-consistent steer rate stored in heading_rate_rps by MPPI debug fill."""
    return [float(p.heading_rate_rps) for p in points]


def estimate_dt(points) -> float:
    if len(points) < 2:
        return 0.1
    durations = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
    if durations[-1] > durations[0]:
        return max((durations[-1] - durations[0]) / max(len(points) - 1, 1), 1e-3)
    return 0.1


@dataclass
class LoadedTrajectory:
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    heading: List[float] = field(default_factory=list)
    vel: List[float] = field(default_factory=list)
    accel: List[float] = field(default_factory=list)
    steer: List[float] = field(default_factory=list)
    steer_rate: List[float] = field(default_factory=list)


@dataclass
class MppiDebugFrame:
    reference_xy: Optional[Tuple[List[float], List[float]]] = None
    optimized_xy: Optional[Tuple[List[float], List[float]]] = None
    retuned_xy: Optional[Tuple[List[float], List[float]]] = None
    reference_heading: List[float] = field(default_factory=list)
    optimized_heading: List[float] = field(default_factory=list)
    retuned_heading: List[float] = field(default_factory=list)
    reference_vel: List[float] = field(default_factory=list)
    optimized_vel: List[float] = field(default_factory=list)
    retuned_vel: List[float] = field(default_factory=list)
    reference_accel: List[float] = field(default_factory=list)
    optimized_accel: List[float] = field(default_factory=list)
    retuned_accel: List[float] = field(default_factory=list)
    reference_steer: List[float] = field(default_factory=list)
    optimized_steer: List[float] = field(default_factory=list)
    retuned_steer: List[float] = field(default_factory=list)
    reference_steer_rate: List[float] = field(default_factory=list)
    optimized_steer_rate: List[float] = field(default_factory=list)
    retuned_steer_rate: List[float] = field(default_factory=list)
    measured_steer: Optional[float] = None
    # Live measured δ history: parallel stamp [s] and tire angle [rad] over MEASURED_STEER_HISTORY_S.
    measured_steer_times: List[float] = field(default_factory=list)
    measured_steer_history: List[float] = field(default_factory=list)
    # Live applied command history: δ_cmd[0] of each optimized trajectory (last 16s).
    cmd_steer_times: List[float] = field(default_factory=list)
    cmd_steer_history: List[float] = field(default_factory=list)
    # First-order lag τ from MPPI vehicle params (FirstOrderDubinsBicycle).
    steer_time_constant: float = VEHICLE_PARAMS_DEFAULT_STEER_TIME_CONSTANT
    wheel_base: float = VEHICLE_PARAMS_DEFAULT_WHEEL_BASE
    raw_costs: List[float] = field(default_factory=list)
    normalized_weights: List[float] = field(default_factory=list)
    # Retune top-K rollouts: (cost, xs, ys) from NNNNNN_rollouts.csv.
    rollouts: List[Tuple[float, List[float], List[float]]] = field(default_factory=list)
    stamp_text: str = ""
    metrics_text: str = ""
    # Live: whether diffusion_planner is applying MPPI to the published trajectory.
    # None = unknown / offline; False = disabled or shadow; True = applied.
    mppi_enabled: Optional[bool] = None
    # Cycle-to-cycle replan ADE history (time-aligned mean ‖p_k − p_{k-1}‖).
    # Stamps are absolute seconds in live mode, or frame indices offline.
    replan_ade_diffusion_times: List[float] = field(default_factory=list)
    replan_ade_diffusion: List[float] = field(default_factory=list)
    replan_ade_mppi_times: List[float] = field(default_factory=list)
    replan_ade_mppi: List[float] = field(default_factory=list)


def load_trajectory_csv(path: Path) -> LoadedTrajectory:
    traj = LoadedTrajectory()
    if not path.is_file():
        return traj
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            traj.x.append(float(row["x"]))
            traj.y.append(float(row["y"]))
            traj.heading.append(float(row["yaw"]))
            traj.vel.append(float(row["v"]))
            traj.accel.append(float(row["a"]))
            traj.steer.append(float(row["steer"]))
            traj.steer_rate.append(float(row["steer_rate"]))
    return traj


@dataclass
class LoadedCostDistribution:
    raw_costs: List[float] = field(default_factory=list)
    normalized_weights: List[float] = field(default_factory=list)


def load_costs_csv(path: Path) -> LoadedCostDistribution:
    dist = LoadedCostDistribution()
    if not path.is_file():
        return dist
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                dist.raw_costs.append(float(row["raw_cost"]))
                dist.normalized_weights.append(float(row["normalized_weight"]))
            except (KeyError, ValueError, TypeError):
                continue
    return dist


def load_rollouts_csv(path: Path) -> List[Tuple[float, List[float], List[float]]]:
    """Load top-K rollouts as (cost, xs, ys) ordered by rollout_index."""
    if not path.is_file():
        return []
    by_idx: Dict[int, Tuple[float, List[float], List[float]]] = {}
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                idx = int(row["rollout_index"])
                cost = float(row["cost"])
                step = int(row["step"])
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, ValueError, TypeError):
                continue
            if idx not in by_idx:
                by_idx[idx] = (cost, [], [])
            cost0, xs, ys = by_idx[idx]
            # Grow lists to cover step index (steps are written in order).
            while len(xs) <= step:
                xs.append(float("nan"))
                ys.append(float("nan"))
            xs[step] = x
            ys[step] = y
            by_idx[idx] = (cost0 if abs(cost0) < 1.0e20 else cost, xs, ys)
    rollouts: List[Tuple[float, List[float], List[float]]] = []
    for idx in sorted(by_idx):
        cost, xs, ys = by_idx[idx]
        # Drop incomplete leading NaNs if any; keep contiguous prefix of valid points.
        clean_xs: List[float] = []
        clean_ys: List[float] = []
        for xv, yv in zip(xs, ys):
            if math.isnan(xv) or math.isnan(yv):
                break
            clean_xs.append(xv)
            clean_ys.append(yv)
        if len(clean_xs) >= 2:
            rollouts.append((cost, clean_xs, clean_ys))
    return rollouts


def discover_log_frames(log_dir: Path) -> List[int]:
    index_path = log_dir / "index.csv"
    frame_ids: List[int] = []
    if index_path.is_file():
        with index_path.open(newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                frame_ids.append(int(row["frame_id"]))
        return frame_ids

    for ref in sorted(log_dir.glob("*_reference.csv")):
        stem = ref.name[: -len("_reference.csv")]
        if stem.isdigit():
            frame_ids.append(int(stem))
    return sorted(frame_ids)


def max_pos_err(
    ref_xy: Optional[Tuple[List[float], List[float]]],
    opt_xy: Optional[Tuple[List[float], List[float]]],
) -> float:
    series = indexed_distance_series(opt_xy, ref_xy)
    return max_abs(series)


def max_vel_err(ref_v: Sequence[float], opt_v: Sequence[float]) -> float:
    if not ref_v or not opt_v:
        return float("nan")
    n = min(len(ref_v), len(opt_v))
    return max(abs(opt_v[i] - ref_v[i]) for i in range(n))


def _wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def indexed_distance_series(
    traj_xy: Optional[Tuple[List[float], List[float]]],
    ref_xy: Optional[Tuple[List[float], List[float]]],
) -> List[float]:
    """Index-matched Euclidean distance ‖p[i]−p_ref[i]‖ (matches computeTrackValue)."""
    if not traj_xy or not ref_xy or not traj_xy[0] or not ref_xy[0]:
        return []
    ref_x, ref_y = ref_xy
    n = min(len(traj_xy[0]), len(traj_xy[1]), len(ref_x), len(ref_y))
    return [math.hypot(traj_xy[0][i] - ref_x[i], traj_xy[1][i] - ref_y[i]) for i in range(n)]


def copy_xy(
    xy: Optional[Tuple[List[float], List[float]]],
) -> Optional[Tuple[List[float], List[float]]]:
    if not xy or not xy[0]:
        return None
    return (list(xy[0]), list(xy[1]))


def cycle_to_cycle_ade(
    prev_xy: Optional[Tuple[List[float], List[float]]],
    curr_xy: Optional[Tuple[List[float], List[float]]],
    *,
    shift: int = REPLAN_TIME_SHIFT,
) -> Optional[float]:
    """Mean ‖curr[i] − prev[i+shift]‖ over the overlapping horizon (replan jump ADE)."""
    if not prev_xy or not curr_xy or not prev_xy[0] or not curr_xy[0]:
        return None
    if shift < 0 or len(prev_xy[0]) <= shift or len(prev_xy[1]) <= shift:
        return None
    prev_shifted = (prev_xy[0][shift:], prev_xy[1][shift:])
    series = indexed_distance_series(curr_xy, prev_shifted)
    if not series:
        return None
    return sum(series) / float(len(series))


def indexed_heading_error_series(
    traj_heading: Sequence[float],
    ref_heading: Sequence[float],
) -> List[float]:
    """Time-indexed heading error Δψ[i] = yaw[i] − ref_yaw[i]."""
    return [
        _wrap_pi(traj_heading[i] - ref_heading[i])
        for i in range(min(len(traj_heading), len(ref_heading)))
    ]


def max_abs(series: Sequence[float]) -> float:
    if not series:
        return float("nan")
    return max(abs(v) for v in series)


def steer_rate_from_lag(delta: float, delta_cmd: float, tau: float) -> float:
    """First-order lag rate: δ̇ = -(δ - δ_cmd) / τ = (δ_cmd - δ) / τ."""
    return -(delta - delta_cmd) / max(tau, 1.0e-4)


def steer_rate_along_cmd_sequence(
    cmd_seq: Sequence[float],
    delta0: float,
    tau: float,
    dt: float,
) -> List[float]:
    """Integrate first-order steer lag along a δ_cmd horizon; return δ̇ at each step."""
    rates: List[float] = []
    delta = float(delta0)
    tau_safe = max(tau, 1.0e-4)
    dt_safe = max(dt, 1.0e-6)
    for cmd in cmd_seq:
        rate = steer_rate_from_lag(delta, float(cmd), tau_safe)
        rates.append(rate)
        delta = delta + rate * dt_safe
    return rates


def lateral_jerk_at(
    v: float,
    accel: float,
    steer: float,
    steer_rate: float,
    wheel_base: float,
) -> float:
    """Match comfortTerms(): a_y = v²κ, j_y = v²κ̇ + 2 v a κ."""
    L = max(wheel_base, 1.0e-4)
    cos_s = math.cos(steer)
    sec_sq = 1.0 / max(cos_s * cos_s, 1.0e-6)
    curvature = math.tan(steer) / L
    curvature_dot = sec_sq * steer_rate / L
    return v * v * curvature_dot + 2.0 * v * accel * curvature


def lateral_jerk_along_cmd_sequence(
    cmd_seq: Sequence[float],
    vel_seq: Sequence[float],
    accel_seq: Sequence[float],
    delta0: float,
    tau: float,
    wheel_base: float,
    dt: float,
) -> List[float]:
    """Integrate plant δ along δ_cmd; return lateral jerk at each horizon step."""
    jerks: List[float] = []
    delta = float(delta0)
    tau_safe = max(tau, 1.0e-4)
    dt_safe = max(dt, 1.0e-6)
    for i, cmd in enumerate(cmd_seq):
        rate = steer_rate_from_lag(delta, float(cmd), tau_safe)
        v = float(vel_seq[i]) if i < len(vel_seq) else (float(vel_seq[-1]) if vel_seq else 0.0)
        a = (
            float(accel_seq[i])
            if i < len(accel_seq)
            else (float(accel_seq[-1]) if accel_seq else 0.0)
        )
        jerks.append(lateral_jerk_at(v, a, delta, rate, wheel_base))
        delta = delta + rate * dt_safe
    return jerks


def frame_from_loaded(
    reference: LoadedTrajectory,
    optimized: LoadedTrajectory,
    stamp_text: str,
    retuned: Optional[LoadedTrajectory] = None,
    costs: Optional[LoadedCostDistribution] = None,
    rollouts: Optional[List[Tuple[float, List[float], List[float]]]] = None,
    steer_time_constant: float = VEHICLE_PARAMS_DEFAULT_STEER_TIME_CONSTANT,
    wheel_base: float = VEHICLE_PARAMS_DEFAULT_WHEEL_BASE,
) -> MppiDebugFrame:
    frame = MppiDebugFrame(
        reference_xy=(reference.x, reference.y) if reference.x else None,
        optimized_xy=(optimized.x, optimized.y) if optimized.x else None,
        retuned_xy=(retuned.x, retuned.y) if retuned and retuned.x else None,
        reference_heading=reference.heading,
        optimized_heading=optimized.heading,
        retuned_heading=retuned.heading if retuned else [],
        reference_vel=reference.vel,
        optimized_vel=optimized.vel,
        retuned_vel=retuned.vel if retuned else [],
        reference_accel=reference.accel,
        optimized_accel=optimized.accel,
        retuned_accel=retuned.accel if retuned else [],
        reference_steer=reference.steer,
        optimized_steer=optimized.steer,
        retuned_steer=retuned.steer if retuned else [],
        reference_steer_rate=reference.steer_rate,
        optimized_steer_rate=optimized.steer_rate,
        retuned_steer_rate=retuned.steer_rate if retuned else [],
        steer_time_constant=steer_time_constant,
        wheel_base=wheel_base,
        raw_costs=costs.raw_costs if costs else [],
        normalized_weights=costs.normalized_weights if costs else [],
        rollouts=list(rollouts) if rollouts else [],
        stamp_text=stamp_text,
    )
    orig_pos = max_pos_err(frame.reference_xy, frame.optimized_xy)
    orig_vel = max_vel_err(frame.reference_vel, frame.optimized_vel)
    orig_dpsi = max_abs(
        indexed_heading_error_series(frame.optimized_heading, frame.reference_heading)
    )
    parts = [f"orig max‖Δp‖={orig_pos:.3f}m max|Δψ|={orig_dpsi:.3f}rad max|v|={orig_vel:.3f}m/s"]
    if frame.retuned_xy:
        ret_pos = max_pos_err(frame.reference_xy, frame.retuned_xy)
        ret_vel = max_vel_err(frame.reference_vel, frame.retuned_vel)
        ret_dpsi = max_abs(
            indexed_heading_error_series(frame.retuned_heading, frame.reference_heading)
        )
        vs_logged = max_pos_err(frame.optimized_xy, frame.retuned_xy)
        parts.append(
            f"retune max‖Δp‖={ret_pos:.3f}m max|Δψ|={ret_dpsi:.3f}rad max|v|={ret_vel:.3f}m/s"
        )
        parts.append(f"logged↔retune max‖Δp‖={vs_logged:.3f}m")
    if frame.raw_costs:
        finite_costs = [c for c in frame.raw_costs if abs(c) < 1.0e20]
        if finite_costs:
            parts.append(
                f"cost med={sorted(finite_costs)[len(finite_costs) // 2]:.1f} "
                f"min={min(finite_costs):.1f}"
            )
    if frame.normalized_weights:
        parts.append(f"w_max={max(frame.normalized_weights):.4f}")
    if frame.rollouts:
        parts.append(
            f"showing {len(frame.rollouts)} of {MPPI_NUM_ROLLOUTS} "
            f"top-weighted rollouts (export ≤{MPPI_MAX_VIZ_ROLLOUTS})"
        )
    frame.metrics_text = "  |  ".join(parts)
    return frame


def _limits_close(a, b, *, rtol: float = 1.0e-6, atol: float = 1.0e-9) -> bool:
    return all(math.isclose(x, y, rel_tol=rtol, abs_tol=atol) for x, y in zip(a, b))


def capture_user_view(ax):
    """Return current limits once the user has zoomed/panned, else None.

    Every redraw calls ``Axes.clear()``, which drops back to autoscale. Without this a
    zoom only survives until the next frame. Compare with tolerance so aspect-ratio /
    tick adjustments cannot spuriously freeze a panel on a bad scale.
    """
    if getattr(ax, "_mppi_view_locked", False):
        return (ax.get_xlim(), ax.get_ylim())
    baseline = getattr(ax, "_mppi_view_baseline", None)
    if baseline is None:
        return None
    current = (ax.get_xlim(), ax.get_ylim())
    cur = current[0] + current[1]
    base = baseline[0] + baseline[1]
    if not _limits_close(cur, base):
        ax._mppi_view_locked = True
        return current
    return None


def apply_view_baseline(ax, saved) -> None:
    if saved is not None:
        ax.set_xlim(saved[0])
        ax.set_ylim(saved[1])
    ax._mppi_view_baseline = (ax.get_xlim(), ax.get_ylim())


def reset_view_baselines(axes) -> None:
    """Hand every panel back to autoscale (bound to the 'f' key)."""
    for ax in axes:
        if ax is None:
            continue
        ax._mppi_view_locked = False
        ax._mppi_view_baseline = None
        ax.relim()
        ax.autoscale_view()


def draw_frame(axes, frame: MppiDebugFrame) -> None:
    saved_views = [(ax, capture_user_view(ax)) for ax in axes if ax is not None]

    if len(axes) >= 11:
        (
            ax_xy,
            ax_lat,
            ax_heading_err,
            ax_vel,
            ax_accel,
            ax_steer_cmd,
            ax_steer_meas,
            ax_lat_jerk,
            ax_replan_ade,
            ax_cost,
            ax_weight,
        ) = axes
        ax_heading = None
    elif len(axes) >= 10:
        # Retune layout before replan-ADE panel.
        (
            ax_xy,
            ax_lat,
            ax_heading_err,
            ax_vel,
            ax_accel,
            ax_steer_cmd,
            ax_steer_meas,
            ax_lat_jerk,
            ax_cost,
            ax_weight,
        ) = axes
        ax_replan_ade = None
        ax_heading = None
    elif len(axes) >= 9:
        (
            ax_xy,
            ax_lat,
            ax_heading_err,
            ax_vel,
            ax_accel,
            ax_steer_cmd,
            ax_steer_meas,
            ax_lat_jerk,
            ax_replan_ade,
        ) = axes
        ax_cost = ax_weight = None
        ax_heading = None
    elif len(axes) >= 8:
        (
            ax_xy,
            ax_lat,
            ax_heading_err,
            ax_vel,
            ax_accel,
            ax_steer_cmd,
            ax_steer_meas,
            ax_lat_jerk,
        ) = axes
        ax_replan_ade = None
        ax_cost = ax_weight = None
        ax_heading = None
    elif len(axes) >= 7:
        (
            ax_xy,
            ax_lat,
            ax_heading_err,
            ax_vel,
            ax_accel,
            ax_steer_cmd,
            ax_steer_meas,
        ) = axes
        ax_lat_jerk = ax_replan_ade = None
        ax_cost = ax_weight = None
        ax_heading = None
    else:
        # Pre path-error layout (absolute heading plot).
        ax_xy, ax_heading, ax_vel, ax_accel, ax_steer_cmd, ax_steer_meas = axes
        ax_lat = ax_heading_err = ax_cost = ax_weight = ax_lat_jerk = ax_replan_ade = None

    lengths = [len(frame.reference_vel), len(frame.optimized_vel)]
    if frame.retuned_vel:
        lengths.append(len(frame.retuned_vel))
    if frame.reference_xy:
        lengths.append(len(frame.reference_xy[0]))
    if frame.optimized_xy:
        lengths.append(len(frame.optimized_xy[0]))
    if frame.retuned_xy:
        lengths.append(len(frame.retuned_xy[0]))
    n_compare = min(n for n in lengths if n > 0) if any(lengths) else 0

    ax_xy.clear()
    ax_xy.set_title("Trajectory (diffusion ref vs MPPI)")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.grid(True)
    if frame.rollouts:
        costs = [c for c, _xs, _ys in frame.rollouts]
        min_c = min(costs)
        max_c = max(costs)
        for cost, xs, ys in frame.rollouts:
            if max_c > min_c:
                t = (cost - min_c) / (max_c - min_c)
            else:
                t = 0.5
            # Match mppi_debug_markers.hpp costGradientColor: green (low) -> red (high).
            ax_xy.plot(xs, ys, color=(t, 1.0 - t, 0.0, 0.35), linewidth=0.7, zorder=1)
        ax_xy.plot(
            [],
            [],
            color=(0.5, 0.5, 0.0, 0.8),
            linewidth=0.7,
            label=(
                f"rollouts ({len(frame.rollouts)} of {MPPI_NUM_ROLLOUTS} "
                f"top-weighted, ≤{MPPI_MAX_VIZ_ROLLOUTS} exported)"
            ),
        )

    def _plot_xy_path(ax, xs, ys, *, color, linestyle, linewidth, label, zorder):
        # Legend handle (single stroke); actual path is drawn segment-by-segment colored
        # by index so spatial folds read as progressing time, not one ambiguous polyline.
        ax.plot([], [], color=color, linestyle=linestyle, linewidth=linewidth, label=label)
        if not xs:
            return
        if len(xs) == 1:
            ax.scatter([xs[0]], [ys[0]], s=36, c=color, marker="o", zorder=zorder)
            return
        nseg = len(xs) - 1
        for i in range(nseg):
            t = i / max(nseg - 1, 1)
            if color == "red":
                seg_color = (0.35 + 0.65 * t, 0.05, 0.05)
            elif color == "cyan":
                seg_color = (0.05, 0.45 + 0.40 * t, 0.50 + 0.35 * t)
            else:
                seg_color = color
            ax.plot(
                xs[i : i + 2],
                ys[i : i + 2],
                color=seg_color,
                linestyle=linestyle,
                linewidth=linewidth,
                zorder=zorder,
                solid_capstyle="round",
            )
        ax.scatter(
            [xs[0]],
            [ys[0]],
            s=40,
            c=color,
            marker="o",
            zorder=zorder + 1,
            edgecolors="k",
            linewidths=0.6,
        )
        ax.annotate(
            "idx0",
            (xs[0], ys[0]),
            textcoords="offset points",
            xytext=(5, 5),
            fontsize=8,
            color=color,
            zorder=zorder + 1,
        )

    if frame.reference_xy and len(frame.reference_xy[0]) > 0:
        _plot_xy_path(
            ax_xy,
            frame.reference_xy[0],
            frame.reference_xy[1],
            color="cyan",
            linestyle="--",
            linewidth=2,
            label="diffusion reference",
            zorder=3,
        )
    if frame.optimized_xy and len(frame.optimized_xy[0]) > 0:
        _plot_xy_path(
            ax_xy,
            frame.optimized_xy[0],
            frame.optimized_xy[1],
            color="red",
            linestyle="-",
            linewidth=2,
            label="MPPI optimized (logged)",
            zorder=4,
        )
    if frame.retuned_xy and len(frame.retuned_xy[0]) > 0:
        ax_xy.plot(
            frame.retuned_xy[0],
            frame.retuned_xy[1],
            color="tab:green",
            linewidth=2.2,
            label="MPPI retuned",
            zorder=5,
        )
    overlay = frame.stamp_text
    if frame.metrics_text:
        overlay = f"{overlay}\n{frame.metrics_text}" if overlay else frame.metrics_text
    status_line = ""
    status_color = "black"
    status_background = "white"
    if frame.mppi_enabled is True:
        status_line = "MPPI ENABLED (optimized trajectory applied)"
        status_color = "green"
        status_background = "#e6ffe6"
    elif frame.mppi_enabled is False:
        status_line = "MPPI DISABLED (output is diffusion only)"
        status_color = "tab:red"
        status_background = "#ffe6e6"
    if status_line:
        overlay = f"{status_line}\n{overlay}" if overlay else status_line
    if overlay:
        ax_xy.text(
            0.02,
            0.98,
            overlay,
            transform=ax_xy.transAxes,
            verticalalignment="top",
            fontsize=9,
            color=status_color,
            fontweight=("bold" if frame.mppi_enabled is not None else "normal"),
            bbox={
                "facecolor": status_background,
                "alpha": 0.9,
                "edgecolor": status_color if frame.mppi_enabled is not None else "0.7",
            },
        )
    if frame.mppi_enabled is True:
        ax_xy.set_title("Trajectory (MPPI ENABLED)")
    elif frame.mppi_enabled is False:
        ax_xy.set_title("Trajectory (MPPI DISABLED)")
    if (
        frame.rollouts
        or (frame.reference_xy and len(frame.reference_xy[0]) > 0)
        or (frame.optimized_xy and len(frame.optimized_xy[0]) > 0)
        or (frame.retuned_xy and len(frame.retuned_xy[0]) > 0)
    ):
        ax_xy.relim()
        ax_xy.autoscale_view()
    # "box" (not "datalim") so an explicit zoom keeps the limits it was given.
    ax_xy.set_aspect("equal", adjustable="box")
    ax_xy.legend(loc="best")

    idx = list(range(n_compare)) if n_compare > 0 else []

    if ax_lat is not None:
        ax_lat.clear()
        ax_lat.set_title("Index-matched position error (track cost)")
        ax_lat.set_xlabel("optimized point index")
        ax_lat.set_ylabel("‖p−p_ref‖ [m]")
        ax_lat.grid(True)
        ax_lat.axhline(0.0, color="0.7", linewidth=0.8, linestyle=":")
        dist_opt = indexed_distance_series(frame.optimized_xy, frame.reference_xy)
        dist_ret = indexed_distance_series(frame.retuned_xy, frame.reference_xy)
        if dist_opt:
            ax_lat.plot(
                list(range(len(dist_opt))),
                dist_opt,
                color="tab:red",
                linewidth=2,
                label="MPPI logged",
            )
        if dist_ret:
            ax_lat.plot(
                list(range(len(dist_ret))),
                dist_ret,
                color="tab:green",
                linewidth=2.2,
                label="MPPI retuned",
            )
        if dist_opt or dist_ret:
            ax_lat.legend(loc="best", fontsize=8)

    if ax_heading_err is not None:
        ax_heading_err.clear()
        ax_heading_err.set_title("Time-indexed heading error")
        ax_heading_err.set_xlabel("optimized point index")
        ax_heading_err.set_ylabel("Δψ [rad]")
        ax_heading_err.grid(True)
        ax_heading_err.axhline(0.0, color="0.7", linewidth=0.8, linestyle=":")
        dpsi_opt = indexed_heading_error_series(frame.optimized_heading, frame.reference_heading)
        dpsi_ret = indexed_heading_error_series(frame.retuned_heading, frame.reference_heading)
        if dpsi_opt:
            ax_heading_err.plot(
                list(range(len(dpsi_opt))),
                dpsi_opt,
                color="tab:red",
                linewidth=2,
                label="MPPI logged",
            )
        if dpsi_ret:
            ax_heading_err.plot(
                list(range(len(dpsi_ret))),
                dpsi_ret,
                color="tab:green",
                linewidth=2.2,
                label="MPPI retuned",
            )
        if dpsi_opt or dpsi_ret:
            ax_heading_err.legend(loc="best", fontsize=8)

    if ax_heading is not None:
        ax_heading.clear()
        ax_heading.set_title("Heading")
        ax_heading.set_xlabel("point index")
        ax_heading.set_ylabel("yaw [rad]")
        ax_heading.grid(True)
        if n_compare > 0:
            ax_heading.plot(
                idx, frame.reference_heading[:n_compare], "c--", linewidth=2, label="diffusion"
            )
            ax_heading.plot(
                idx, frame.optimized_heading[:n_compare], "r-", linewidth=2, label="MPPI logged"
            )
            if frame.retuned_heading:
                ax_heading.plot(
                    idx,
                    frame.retuned_heading[:n_compare],
                    color="tab:green",
                    linewidth=2.2,
                    label="MPPI retuned",
                )
            ax_heading.legend(loc="best")

    ax_vel.clear()
    ax_vel.set_title("Longitudinal velocity")
    ax_vel.set_xlabel("point index")
    ax_vel.set_ylabel("v [m/s]")
    ax_vel.grid(True)
    if n_compare > 0:
        ax_vel.plot(idx, frame.reference_vel[:n_compare], "c--", linewidth=2, label="diffusion")
        ax_vel.plot(idx, frame.optimized_vel[:n_compare], "r-", linewidth=2, label="MPPI logged")
        if frame.retuned_vel:
            ax_vel.plot(
                idx,
                frame.retuned_vel[:n_compare],
                color="tab:green",
                linewidth=2.2,
                label="MPPI retuned",
            )
        ax_vel.legend(loc="best")

    ax_accel.clear()
    ax_accel.set_title("Acceleration")
    ax_accel.set_xlabel("point index")
    ax_accel.set_ylabel("a [m/s²]")
    ax_accel.grid(True)
    if n_compare > 0:
        ax_accel.plot(
            idx,
            frame.reference_accel[:n_compare],
            color="tab:blue",
            linestyle="--",
            linewidth=2,
            label="diffusion accel",
        )
        ax_accel.plot(
            idx,
            frame.optimized_accel[:n_compare],
            color="tab:blue",
            linewidth=2,
            label="MPPI logged",
        )
        if frame.retuned_accel:
            ax_accel.plot(
                idx,
                frame.retuned_accel[:n_compare],
                color="tab:green",
                linewidth=2.2,
                label="MPPI retuned",
            )
        ax_accel.legend(loc="best")

    # Drop previous rate twin before clear (twins survive Axes.clear()).
    prev_rate_twin = getattr(ax_steer_cmd, "_steer_rate_twin", None)
    if prev_rate_twin is not None:
        try:
            prev_rate_twin.remove()
        except (AttributeError, ValueError):
            pass
        ax_steer_cmd._steer_rate_twin = None

    ax_steer_cmd.clear()
    ax_steer_cmd.set_title("Current δ_cmd sequence + lag steer rate δ̇")
    ax_steer_cmd.set_xlabel("point index")
    ax_steer_cmd.set_ylabel("δ_cmd [rad]", color="tab:orange")
    ax_steer_cmd.tick_params(axis="y", labelcolor="tab:orange")
    ax_steer_cmd.grid(True)
    cmd_handles = []
    cmd_labels = []
    # Prefer live/current optimized horizon; fall back to retuned offline overlay.
    cmd_seq: List[float] = []
    cmd_label = ""
    if frame.optimized_steer:
        cmd_seq = list(frame.optimized_steer)
        cmd_label = "δ_cmd (current)"
    elif frame.retuned_steer:
        cmd_seq = list(frame.retuned_steer)
        cmd_label = "δ_cmd (retuned)"
    if cmd_seq:
        idx_cmd = list(range(len(cmd_seq)))
        (h_cmd,) = ax_steer_cmd.plot(
            idx_cmd,
            cmd_seq,
            color="tab:orange",
            linewidth=2,
            label=cmd_label,
        )
        cmd_handles.append(h_cmd)
        cmd_labels.append(cmd_label)
        if frame.retuned_steer and frame.optimized_steer:
            (h_ret,) = ax_steer_cmd.plot(
                list(range(len(frame.retuned_steer))),
                frame.retuned_steer,
                color="tab:green",
                linewidth=2.2,
                label="δ_cmd (retuned)",
            )
            cmd_handles.append(h_ret)
            cmd_labels.append("δ_cmd (retuned)")

        # δ̇ along the command sequence from first-order lag, seeded by current δ_meas.
        tau = frame.steer_time_constant
        delta0 = frame.measured_steer if frame.measured_steer is not None else 0.0
        rates = steer_rate_along_cmd_sequence(cmd_seq, delta0, tau, MPPI_DT)
        ax_rate = ax_steer_cmd.twinx()
        ax_steer_cmd._steer_rate_twin = ax_rate
        ax_rate.set_ylabel("δ̇ [rad/s]", color="tab:purple")
        ax_rate.tick_params(axis="y", labelcolor="tab:purple")
        (h_rate,) = ax_rate.plot(
            idx_cmd,
            rates,
            color="tab:purple",
            linewidth=1.8,
            label=f"δ̇=-(δ-δ_cmd)/τ (τ={tau:.2f}s)",
        )
        cmd_handles.append(h_rate)
        cmd_labels.append(f"δ̇=-(δ-δ_cmd)/τ (τ={tau:.2f}s)")
        ax_steer_cmd.legend(cmd_handles, cmd_labels, loc="best", fontsize=8)
    else:
        ax_steer_cmd.text(
            0.5,
            0.5,
            "waiting for current δ_cmd sequence",
            ha="center",
            va="center",
            transform=ax_steer_cmd.transAxes,
        )

    ax_steer_meas.clear()
    ax_steer_meas.set_title(f"δ_meas + δ_cmd[0] history (last {MEASURED_STEER_HISTORY_S:.0f}s)")
    ax_steer_meas.set_xlabel("time [s] (0 = now)")
    ax_steer_meas.set_ylabel("steer [rad]")
    ax_steer_meas.grid(True)
    has_measured_history = len(frame.measured_steer_times) > 0 and len(
        frame.measured_steer_times
    ) == len(frame.measured_steer_history)
    has_cmd_history = len(frame.cmd_steer_times) > 0 and len(frame.cmd_steer_times) == len(
        frame.cmd_steer_history
    )
    hist_handles = []
    hist_labels = []
    t_end_candidates = []
    if has_measured_history:
        t_end_candidates.append(frame.measured_steer_times[-1])
    if has_cmd_history:
        t_end_candidates.append(frame.cmd_steer_times[-1])
    if t_end_candidates:
        t_end = max(t_end_candidates)
        if has_measured_history:
            t_rel_m = [t - t_end for t in frame.measured_steer_times]
            latest_m = frame.measured_steer_history[-1]
            (h_m,) = ax_steer_meas.plot(
                t_rel_m,
                frame.measured_steer_history,
                color="tab:blue",
                linewidth=1.8,
                label=f"δ_meas (now={latest_m:.3f})",
            )
            hist_handles.append(h_m)
            hist_labels.append(f"δ_meas (now={latest_m:.3f})")
        if has_cmd_history:
            t_rel_c = [t - t_end for t in frame.cmd_steer_times]
            latest_c = frame.cmd_steer_history[-1]
            (h_c,) = ax_steer_meas.plot(
                t_rel_c,
                frame.cmd_steer_history,
                color="tab:orange",
                linewidth=1.8,
                label=f"δ_cmd[0] (now={latest_c:.3f})",
            )
            hist_handles.append(h_c)
            hist_labels.append(f"δ_cmd[0] (now={latest_c:.3f})")
        ax_steer_meas.set_xlim(-MEASURED_STEER_HISTORY_S, 0.0)
        ax_steer_meas.legend(hist_handles, hist_labels, loc="best")
    else:
        ax_steer_meas.text(
            0.5,
            0.5,
            "waiting for δ_meas / δ_cmd[0] history",
            ha="center",
            va="center",
            transform=ax_steer_meas.transAxes,
        )

    if ax_lat_jerk is not None:
        ax_lat_jerk.clear()
        ax_lat_jerk.set_title("Lateral jerk along current horizon (comfortTerms)")
        ax_lat_jerk.set_xlabel("point index")
        ax_lat_jerk.set_ylabel("j_y [m/s³]")
        ax_lat_jerk.grid(True)
        jerk_handles = []
        jerk_labels = []
        tau = frame.steer_time_constant
        L = frame.wheel_base
        delta0 = frame.measured_steer if frame.measured_steer is not None else 0.0

        def _plot_jerk(
            cmd: Sequence[float],
            vel: Sequence[float],
            accel: Sequence[float],
            *,
            color: str,
            label: str,
            linewidth: float = 2.0,
        ) -> None:
            if not cmd:
                return
            jerks = lateral_jerk_along_cmd_sequence(cmd, vel, accel, delta0, tau, L, MPPI_DT)
            (h,) = ax_lat_jerk.plot(
                list(range(len(jerks))),
                jerks,
                color=color,
                linewidth=linewidth,
                label=label,
            )
            jerk_handles.append(h)
            jerk_labels.append(label)

        if frame.optimized_steer:
            _plot_jerk(
                frame.optimized_steer,
                frame.optimized_vel,
                frame.optimized_accel,
                color="tab:red",
                label="j_y (current)",
            )
        if frame.retuned_steer:
            _plot_jerk(
                frame.retuned_steer,
                frame.retuned_vel,
                frame.retuned_accel,
                color="tab:green",
                label="j_y (retuned)",
                linewidth=2.2,
            )
        if jerk_handles:
            ax_lat_jerk.axhline(0.0, color="0.5", linewidth=0.8, linestyle="--")
            ax_lat_jerk.legend(jerk_handles, jerk_labels, loc="best", fontsize=8)
        else:
            ax_lat_jerk.text(
                0.5,
                0.5,
                "waiting for δ_cmd / v / a horizon",
                ha="center",
                va="center",
                transform=ax_lat_jerk.transAxes,
            )

    if ax_replan_ade is not None:
        ax_replan_ade.clear()
        ax_replan_ade.set_title(
            f"Cycle-to-cycle ADE (shift={REPLAN_TIME_SHIFT}·dt vs previous plan)"
        )
        ax_replan_ade.set_ylabel("ADE [m]")
        ax_replan_ade.grid(True)
        t_diff = list(frame.replan_ade_diffusion_times)
        ade_diff = list(frame.replan_ade_diffusion)
        t_mppi = list(frame.replan_ade_mppi_times)
        ade_mppi = list(frame.replan_ade_mppi)
        plotted = False
        all_t = t_diff + t_mppi
        live_times = bool(all_t) and max(all_t) > 1.0e6
        if live_times:
            t0 = max(all_t)
            x_diff = [t - t0 for t in t_diff]
            x_mppi = [t - t0 for t in t_mppi]
            ax_replan_ade.set_xlabel(f"time [s] (0 = now, last {REPLAN_ADE_HISTORY_S:.0f}s)")
            ax_replan_ade.set_xlim(-REPLAN_ADE_HISTORY_S, 0.0)
        else:
            x_diff = t_diff
            x_mppi = t_mppi
            ax_replan_ade.set_xlabel("frame index")
        if ade_diff and len(x_diff) == len(ade_diff):
            ax_replan_ade.plot(
                x_diff,
                ade_diff,
                color="cyan",
                linestyle="--",
                linewidth=2,
                label="diffusion",
            )
            plotted = True
        if ade_mppi and len(x_mppi) == len(ade_mppi):
            ax_replan_ade.plot(
                x_mppi,
                ade_mppi,
                color="red",
                linewidth=2,
                label="MPPI",
            )
            plotted = True
        if plotted:
            parts = []
            if ade_diff:
                parts.append(f"diff μ={sum(ade_diff) / len(ade_diff):.3f}m")
            if ade_mppi:
                parts.append(f"MPPI μ={sum(ade_mppi) / len(ade_mppi):.3f}m")
            if parts:
                ax_replan_ade.text(
                    0.02,
                    0.98,
                    "  ".join(parts),
                    transform=ax_replan_ade.transAxes,
                    va="top",
                    fontsize=8,
                    bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.7"},
                )
            ax_replan_ade.legend(loc="best", fontsize=8)
        else:
            ax_replan_ade.text(
                0.5,
                0.5,
                "waiting for ≥2 successive plans",
                ha="center",
                va="center",
                transform=ax_replan_ade.transAxes,
            )

    if ax_cost is not None:
        ax_cost.clear()
        ax_cost.set_title("Retune cost distribution")
        ax_cost.set_xlabel("raw cost")
        ax_cost.set_ylabel("count")
        ax_cost.grid(True, alpha=0.3)
        finite_costs = [c for c in frame.raw_costs if abs(c) < 1.0e20]
        if finite_costs:
            ax_cost.hist(finite_costs, bins=80, color="tab:purple", alpha=0.85)
            ax_cost.axvline(
                min(finite_costs), color="tab:green", linestyle="--", linewidth=1.5, label="min"
            )
            ax_cost.legend(loc="best", fontsize=8)
        else:
            ax_cost.text(
                0.5,
                0.5,
                "Retune to populate",
                ha="center",
                va="center",
                transform=ax_cost.transAxes,
            )

    if ax_weight is not None:
        ax_weight.clear()
        ax_weight.set_title("Retune weight distribution")
        ax_weight.set_xlabel("normalized weight")
        ax_weight.set_ylabel("count")
        ax_weight.grid(True, alpha=0.3)
        if frame.normalized_weights:
            # Log-x helps when mass concentrates near zero.
            positive = [w for w in frame.normalized_weights if w > 0.0]
            if positive:
                ax_weight.hist(positive, bins=80, color="tab:orange", alpha=0.85, log=True)
                ax_weight.axvline(
                    max(positive), color="tab:red", linestyle="--", linewidth=1.5, label="max"
                )
                ax_weight.legend(loc="best", fontsize=8)
            else:
                ax_weight.text(
                    0.5,
                    0.5,
                    "All weights zero",
                    ha="center",
                    va="center",
                    transform=ax_weight.transAxes,
                )
        else:
            ax_weight.text(
                0.5,
                0.5,
                "Retune to populate",
                ha="center",
                va="center",
                transform=ax_weight.transAxes,
            )

    for ax, saved in saved_views:
        apply_view_baseline(ax, saved)


def create_figure(*, with_retune_panel: bool = False):
    trajectory_fig = plt.figure(figsize=(8.5, 7.0))
    ax_xy = trajectory_fig.add_subplot(1, 1, 1)
    trajectory_fig.canvas.manager.set_window_title("MPPI Trajectory")

    if with_retune_panel:
        # Third column stays empty: _build_retune_controls() places the sliders there.
        fig = plt.figure(figsize=(15, 11))
        gs = gridspec.GridSpec(
            5, 3, figure=fig, width_ratios=[1.0, 1.0, 0.85], wspace=0.30, hspace=0.55
        )
        ax_lat = fig.add_subplot(gs[0, 0])
        ax_heading_err = fig.add_subplot(gs[1, 0])
        ax_vel = fig.add_subplot(gs[2, 0])
        ax_accel = fig.add_subplot(gs[3, 0])
        ax_replan_ade = fig.add_subplot(gs[4, 0])
        ax_steer_cmd = fig.add_subplot(gs[0, 1])
        ax_steer_meas = fig.add_subplot(gs[1, 1])
        ax_lat_jerk = fig.add_subplot(gs[2, 1])
        ax_cost = fig.add_subplot(gs[3, 1])
        ax_weight = fig.add_subplot(gs[4, 1])
        fig.canvas.manager.set_window_title("MPPI Diagnostics + Retune")
        fig._mppi_related_figures = (trajectory_fig,)
        return fig, (
            ax_xy,
            ax_lat,
            ax_heading_err,
            ax_vel,
            ax_accel,
            ax_steer_cmd,
            ax_steer_meas,
            ax_lat_jerk,
            ax_replan_ade,
            ax_cost,
            ax_weight,
        )

    fig = plt.figure(figsize=(12, 9))
    gs = gridspec.GridSpec(4, 2, figure=fig, wspace=0.26, hspace=0.55)
    ax_lat = fig.add_subplot(gs[0, 0])
    ax_heading_err = fig.add_subplot(gs[1, 0])
    ax_vel = fig.add_subplot(gs[2, 0])
    ax_replan_ade = fig.add_subplot(gs[3, 0])
    ax_accel = fig.add_subplot(gs[0, 1])
    ax_steer_cmd = fig.add_subplot(gs[1, 1])
    ax_steer_meas = fig.add_subplot(gs[2, 1])
    ax_lat_jerk = fig.add_subplot(gs[3, 1])
    fig.canvas.manager.set_window_title("MPPI Diagnostics")
    fig._mppi_related_figures = (trajectory_fig,)
    return fig, (
        ax_xy,
        ax_lat,
        ax_heading_err,
        ax_vel,
        ax_accel,
        ax_steer_cmd,
        ax_steer_meas,
        ax_lat_jerk,
        ax_replan_ade,
    )


def redraw_figure_group(fig) -> None:
    for current_fig in (fig, *getattr(fig, "_mppi_related_figures", ())):
        current_fig.canvas.draw_idle()
        current_fig.canvas.flush_events()


class AxesNavigator:
    """Scroll to zoom, drag to pan, double-click to re-fit.

    Self-contained so navigation does not depend on the backend toolbar being usable.
    """

    ZOOM_STEP = 1.2

    def __init__(self, fig, axes) -> None:
        self._axes = {ax for ax in axes if ax is not None}
        self._drag = None
        for current_fig in (fig, *getattr(fig, "_mppi_related_figures", ())):
            current_fig.canvas.mpl_connect("scroll_event", self._on_scroll)
            current_fig.canvas.mpl_connect("button_press_event", self._on_press)
            current_fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
            current_fig.canvas.mpl_connect("button_release_event", self._on_release)

    def _on_scroll(self, event) -> None:
        ax = event.inaxes
        if ax not in self._axes or event.xdata is None or event.ydata is None:
            return
        scale = 1.0 / self.ZOOM_STEP if event.button == "up" else self.ZOOM_STEP
        x_lo, x_hi = ax.get_xlim()
        y_lo, y_hi = ax.get_ylim()
        ax.set_xlim(
            event.xdata - (event.xdata - x_lo) * scale,
            event.xdata + (x_hi - event.xdata) * scale,
        )
        ax.set_ylim(
            event.ydata - (event.ydata - y_lo) * scale,
            event.ydata + (y_hi - event.ydata) * scale,
        )
        ax._mppi_view_locked = True
        ax.figure.canvas.draw_idle()

    def _on_press(self, event) -> None:
        ax = event.inaxes
        if event.button != 1 or ax not in self._axes:
            return
        if event.dblclick:
            reset_view_baselines([ax])
            ax.figure.canvas.draw_idle()
            return
        self._drag = (ax, event.xdata, event.ydata)

    def _on_motion(self, event) -> None:
        if self._drag is None:
            return
        ax, anchor_x, anchor_y = self._drag
        if event.inaxes is not ax or event.xdata is None or event.ydata is None:
            return
        dx = anchor_x - event.xdata
        dy = anchor_y - event.ydata
        x_lo, x_hi = ax.get_xlim()
        y_lo, y_hi = ax.get_ylim()
        ax.set_xlim(x_lo + dx, x_hi + dx)
        ax.set_ylim(y_lo + dy, y_hi + dy)
        ax._mppi_view_locked = True
        ax.figure.canvas.draw_idle()

    def _on_release(self, event) -> None:
        self._drag = None


def load_params_yaml(path: Optional[Path]) -> Dict[str, float]:
    params = dict(DEFAULT_PARAMS)
    if path is None or not path.is_file():
        return params
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        value = value.strip().strip('"')
        if key in params:
            try:
                params[key] = float(value)
            except ValueError:
                pass
    return params


def load_key_value_csv(path: Path) -> Dict[str, float]:
    out: Dict[str, float] = {}
    if not path.is_file():
        return out
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            key = (row.get("key") or "").strip()
            if not key:
                continue
            try:
                out[key] = float(row["value"])
            except (KeyError, ValueError, TypeError):
                continue
    return out


def load_params_from_log(log_dir: Path, params_yaml: Optional[Path]) -> Dict[str, float]:
    """Prefer logged cost_params.csv, then yaml, then defaults."""
    params = dict(DEFAULT_PARAMS)
    logged = load_key_value_csv(log_dir / "cost_params.csv")
    for key, value in logged.items():
        if key in params:
            params[key] = value
    if params_yaml is not None:
        yaml_params = load_params_yaml(params_yaml)
        # Yaml only fills keys still at default when log is missing; if log exists, keep log.
        if not logged:
            params.update({k: v for k, v in yaml_params.items() if k in params})
        else:
            # Still allow yaml to supply keys absent from the log file.
            for key, value in yaml_params.items():
                if key in params and key not in logged:
                    params[key] = value
    return params


def load_vehicle_from_log(
    log_dir: Path, wheel_base: float, ego_width: float, ego_length: float
) -> Tuple[float, float, float, float]:
    """Return wheel_base, ego_width, ego_length, steer_time_constant from vehicle_params.csv."""
    logged = load_key_value_csv(log_dir / "vehicle_params.csv")
    return (
        logged.get("wheel_base", wheel_base),
        logged.get("ego_width", ego_width),
        logged.get("ego_length", ego_length),
        logged.get("steer_time_constant", VEHICLE_PARAMS_DEFAULT_STEER_TIME_CONSTANT),
    )


def find_retune_binary(explicit: str = "") -> Path:
    if explicit:
        return Path(explicit)
    env = os.environ.get("MPPI_OFFLINE_RETUNE")
    if env:
        return Path(env)
    which = shutil.which("mppi_offline_retune")
    if which:
        return Path(which)
    try:
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "autoware_mppi_optimizer"], text=True
        ).strip()
        candidate = Path(prefix) / "lib" / "autoware_mppi_optimizer" / "mppi_offline_retune"
        if candidate.is_file():
            return candidate
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass
    raise FileNotFoundError(
        "mppi_offline_retune not found. Build autoware_mppi_optimizer and source install, "
        "or set MPPI_OFFLINE_RETUNE / --retune-bin."
    )


class MppiDebugVisualizer(Node):
    def __init__(
        self,
        *,
        topic_prefix: str,
        update_hz: float,
        wheel_base: float,
        measured_steering_topic: str = "/vehicle/status/steering_status",
        steer_time_constant: Optional[float] = None,
    ) -> None:
        super().__init__("mppi_debug_visualizer")

        update_hz = max(update_hz, 1.0)
        self._wheel_base = wheel_base
        # Same parameter name as simulator_model.param.yaml / MPPI vehicle dynamics.
        self.declare_parameter("steer_time_constant", VEHICLE_PARAMS_DEFAULT_STEER_TIME_CONSTANT)
        if steer_time_constant is not None:
            tau = float(steer_time_constant)
        else:
            tau = float(self.get_parameter("steer_time_constant").value)
        self._steer_time_constant = max(tau, 1.0e-4)
        self._lock = threading.Lock()
        self._frame = MppiDebugFrame()
        self._logged_reference = False
        self._logged_optimized = False
        self._logged_measured_steer = False
        self._logged_cmd_steer = False
        self._logged_mppi_enabled = False
        self._measured_steer_history: Deque[Tuple[float, float]] = deque()
        self._cmd_steer_history: Deque[Tuple[float, float]] = deque()
        self._prev_reference_xy: Optional[Tuple[List[float], List[float]]] = None
        self._prev_optimized_xy: Optional[Tuple[List[float], List[float]]] = None
        self._replan_ade_diffusion: Deque[Tuple[float, float]] = deque()
        self._replan_ade_mppi: Deque[Tuple[float, float]] = deque()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        enabled_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Vehicle status topics are typically SensorDataQoS (BEST_EFFORT).
        measured_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        prefix = topic_prefix.rstrip("/")

        self.create_subscription(
            Trajectory, f"{prefix}/reference_trajectory", self.on_reference_trajectory, qos
        )
        self.create_subscription(
            Trajectory, f"{prefix}/optimized_trajectory", self.on_optimized_trajectory, qos
        )
        self.create_subscription(Bool, f"{prefix}/enabled", self.on_mppi_enabled, enabled_qos)
        self.create_subscription(
            SteeringReport, measured_steering_topic, self.on_measured_steering, measured_qos
        )

        self._fig, self._axes = create_figure()
        for fig in (self._fig, *getattr(self._fig, "_mppi_related_figures", ())):
            fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._navigator = AxesNavigator(self._fig, self._axes)
        self._configure_window_no_focus_steal()
        plt.show(block=False)

        self.get_logger().info("MPPI debug visualizer started (live).")
        self.get_logger().info(f"Reference: {prefix}/reference_trajectory")
        self.get_logger().info(f"Optimized: {prefix}/optimized_trajectory")
        self.get_logger().info(f"Enabled flag: {prefix}/enabled")
        self.get_logger().info(f"Measured steer: {measured_steering_topic}")
        self.get_logger().info(
            f"Horizon δ̇ uses MPPI first-order steer lag τ={self._steer_time_constant:.4f}s "
            "(steer_time_constant from vehicle params / ROS)"
        )
        self.get_logger().info(
            "Subscriptions use RELIABLE QoS (matches diffusion_planner publishers)."
        )
        self.get_logger().info("Ensure use_mppi_optimizer:=true in diffusion_planner params.")
        self.get_logger().info(
            f"Plot navigation (matplotlib backend: {matplotlib.get_backend()}): "
            "scroll = zoom, drag = pan, double-click = re-fit one panel, "
            "escape = re-fit all. Zoom persists across updates."
        )
        self.get_logger().info(
            f"Cycle-to-cycle ADE: mean ‖p_k − p_{{k-1}}[+{REPLAN_TIME_SHIFT}]‖ "
            f"over last {REPLAN_ADE_HISTORY_S:.0f}s (diffusion vs MPPI separately)."
        )

        self.create_timer(1.0 / update_hz, self.on_timer)

    def _on_key(self, event) -> None:
        if event.key == "escape":
            reset_view_baselines(self._axes)

    def _configure_window_no_focus_steal(self) -> None:
        for fig in (self._fig, *getattr(self._fig, "_mppi_related_figures", ())):
            try:
                win = fig.canvas.manager.window
            except (AttributeError, TypeError):
                continue
            if win is None:
                continue
            if hasattr(win, "attributes"):
                try:
                    win.attributes("-topmost", False)
                except Exception:
                    pass
            try:
                from PyQt5 import QtCore  # type: ignore[import-not-found]

                win.setAttribute(QtCore.Qt.WA_ShowWithoutActivating, True)
            except (ImportError, AttributeError):
                pass

    def _process_trajectory(self, msg: Trajectory, *, is_optimized: bool = False) -> MppiDebugFrame:
        points = msg.points
        dt = estimate_dt(points)
        velocities = trajectory_velocity(points)
        if is_optimized:
            accel = trajectory_acceleration(points)
            steer = trajectory_steering(points, self._wheel_base, prefer_message=True)
            steer_rate = trajectory_steer_rate(points)
        else:
            accel = trajectory_acceleration(points)
            if all(abs(a) < 1e-9 for a in accel) and len(velocities) > 1:
                accel = finite_difference_acceleration(velocities, dt)
            steer = trajectory_steering(points, self._wheel_base)
            steer_rate = []
        stamp = f"stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
        return MppiDebugFrame(
            reference_xy=trajectory_xy(points),
            optimized_xy=None,
            reference_heading=trajectory_heading(points),
            optimized_heading=[],
            reference_vel=velocities,
            optimized_vel=[],
            reference_accel=accel,
            optimized_accel=[],
            reference_steer=steer,
            optimized_steer=[],
            reference_steer_rate=steer_rate,
            optimized_steer_rate=[],
            stamp_text=stamp,
        )

    def _append_replan_ade(
        self,
        history: Deque[Tuple[float, float]],
        stamp: float,
        prev_xy: Optional[Tuple[List[float], List[float]]],
        curr_xy: Optional[Tuple[List[float], List[float]]],
    ) -> Optional[Tuple[List[float], List[float]]]:
        """Compare curr vs prev (time-shifted), append ADE, return curr as new prev."""
        ade = cycle_to_cycle_ade(prev_xy, curr_xy)
        if ade is not None:
            history.append((stamp, ade))
            cutoff = stamp - REPLAN_ADE_HISTORY_S
            while history and history[0][0] < cutoff:
                history.popleft()
        return copy_xy(curr_xy)

    def on_reference_trajectory(self, msg: Trajectory) -> None:
        processed = self._process_trajectory(msg)
        stamp = float(msg.header.stamp.sec) + 1.0e-9 * float(msg.header.stamp.nanosec)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1.0e-9
        with self._lock:
            self._prev_reference_xy = self._append_replan_ade(
                self._replan_ade_diffusion,
                stamp,
                self._prev_reference_xy,
                processed.reference_xy,
            )
            self._frame.reference_xy = processed.reference_xy
            self._frame.reference_heading = processed.reference_heading
            self._frame.reference_vel = processed.reference_vel
            self._frame.reference_accel = processed.reference_accel
            self._frame.reference_steer = processed.reference_steer
            self._frame.reference_steer_rate = processed.reference_steer_rate
            self._frame.stamp_text = processed.stamp_text
        if not self._logged_reference and msg.points:
            self._logged_reference = True
            self.get_logger().info(f"Receiving reference_trajectory ({len(msg.points)} points).")

    def on_optimized_trajectory(self, msg: Trajectory) -> None:
        processed = self._process_trajectory(msg, is_optimized=True)
        stamp = float(msg.header.stamp.sec) + 1.0e-9 * float(msg.header.stamp.nanosec)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1.0e-9
        cmd_seq = list(processed.reference_steer)
        cmd0 = cmd_seq[0] if cmd_seq else None
        with self._lock:
            self._prev_optimized_xy = self._append_replan_ade(
                self._replan_ade_mppi,
                stamp,
                self._prev_optimized_xy,
                processed.reference_xy,
            )
            self._frame.optimized_xy = processed.reference_xy
            self._frame.optimized_heading = processed.reference_heading
            self._frame.optimized_vel = processed.reference_vel
            self._frame.optimized_accel = processed.reference_accel
            self._frame.optimized_steer = cmd_seq
            self._frame.optimized_steer_rate = []
            if cmd0 is not None:
                self._cmd_steer_history.append((stamp, float(cmd0)))
                cutoff = stamp - MEASURED_STEER_HISTORY_S
                while self._cmd_steer_history and self._cmd_steer_history[0][0] < cutoff:
                    self._cmd_steer_history.popleft()
            if not self._frame.stamp_text:
                self._frame.stamp_text = processed.stamp_text
        if not self._logged_optimized and msg.points:
            self._logged_optimized = True
            self.get_logger().info(f"Receiving optimized_trajectory ({len(msg.points)} points).")
        if cmd0 is not None and not self._logged_cmd_steer:
            self._logged_cmd_steer = True
            self.get_logger().info(
                f"Recording δ_cmd[0]={cmd0:.4f} rad; "
                f"plotting last {MEASURED_STEER_HISTORY_S:.0f}s on the history panel."
            )

    def on_mppi_enabled(self, msg: Bool) -> None:
        with self._lock:
            self._frame.mppi_enabled = bool(msg.data)
        if not self._logged_mppi_enabled:
            self._logged_mppi_enabled = True
            self.get_logger().info(
                f"Receiving mppi enabled flag ({'ENABLED' if msg.data else 'DISABLED'})."
            )

    def on_measured_steering(self, msg: SteeringReport) -> None:
        steer = float(msg.steering_tire_angle)
        # SteeringReport carries stamp (not header).
        stamp = float(msg.stamp.sec) + 1.0e-9 * float(msg.stamp.nanosec)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1.0e-9
        with self._lock:
            self._frame.measured_steer = steer
            self._measured_steer_history.append((stamp, steer))
            cutoff = stamp - MEASURED_STEER_HISTORY_S
            while self._measured_steer_history and self._measured_steer_history[0][0] < cutoff:
                self._measured_steer_history.popleft()
        if not self._logged_measured_steer:
            self._logged_measured_steer = True
            self.get_logger().info(
                f"Receiving measured steering ({steer:.4f} rad); "
                f"plotting last {MEASURED_STEER_HISTORY_S:.0f}s history."
            )

    def on_timer(self) -> None:
        with self._lock:
            meas_times = [t for t, _ in self._measured_steer_history]
            meas_vals = [v for _, v in self._measured_steer_history]
            cmd_times = [t for t, _ in self._cmd_steer_history]
            cmd_vals = [v for _, v in self._cmd_steer_history]
            ade_diff_t = [t for t, _ in self._replan_ade_diffusion]
            ade_diff = [v for _, v in self._replan_ade_diffusion]
            ade_mppi_t = [t for t, _ in self._replan_ade_mppi]
            ade_mppi = [v for _, v in self._replan_ade_mppi]
            frame = MppiDebugFrame(
                reference_xy=self._frame.reference_xy,
                optimized_xy=self._frame.optimized_xy,
                reference_heading=list(self._frame.reference_heading),
                optimized_heading=list(self._frame.optimized_heading),
                reference_vel=list(self._frame.reference_vel),
                optimized_vel=list(self._frame.optimized_vel),
                reference_accel=list(self._frame.reference_accel),
                optimized_accel=list(self._frame.optimized_accel),
                reference_steer=list(self._frame.reference_steer),
                optimized_steer=list(self._frame.optimized_steer),
                reference_steer_rate=list(self._frame.reference_steer_rate),
                optimized_steer_rate=[],
                measured_steer=self._frame.measured_steer,
                measured_steer_times=meas_times,
                measured_steer_history=meas_vals,
                cmd_steer_times=cmd_times,
                cmd_steer_history=cmd_vals,
                steer_time_constant=self._steer_time_constant,
                wheel_base=self._wheel_base,
                stamp_text=self._frame.stamp_text,
                metrics_text=self._frame.metrics_text,
                mppi_enabled=self._frame.mppi_enabled,
                replan_ade_diffusion_times=ade_diff_t,
                replan_ade_diffusion=ade_diff,
                replan_ade_mppi_times=ade_mppi_t,
                replan_ade_mppi=ade_mppi,
            )
            if frame.reference_xy and frame.optimized_xy:
                frame.metrics_text = (
                    f"max‖Δp‖={max_pos_err(frame.reference_xy, frame.optimized_xy):.3f}m  "
                    f"max|Δψ|={max_abs(indexed_heading_error_series(frame.optimized_heading, frame.reference_heading)):.3f}rad  "
                    f"max|v|={max_vel_err(frame.reference_vel, frame.optimized_vel):.3f}m/s"
                )
            if ade_diff or ade_mppi:
                parts = []
                if ade_diff:
                    parts.append(f"replanADE_diff={ade_diff[-1]:.3f}m")
                if ade_mppi:
                    parts.append(f"replanADE_mppi={ade_mppi[-1]:.3f}m")
                ade_txt = "  ".join(parts)
                frame.metrics_text = (
                    f"{frame.metrics_text}  |  {ade_txt}" if frame.metrics_text else ade_txt
                )
            if frame.measured_steer is not None:
                steer_txt = f"δ_meas={frame.measured_steer:.3f}"
                frame.metrics_text = (
                    f"{frame.metrics_text}  |  {steer_txt}" if frame.metrics_text else steer_txt
                )
            if frame.cmd_steer_history:
                cmd_txt = f"δ_cmd[0]={frame.cmd_steer_history[-1]:.3f}"
                frame.metrics_text = (
                    f"{frame.metrics_text}  |  {cmd_txt}" if frame.metrics_text else cmd_txt
                )
        draw_frame(self._axes, frame)
        redraw_figure_group(self._fig)


class OfflineLogVisualizer:
    def __init__(
        self,
        log_dir: Path,
        *,
        start_frame: int = 0,
        autoplay: bool = False,
        enable_retune: bool = False,
        params_yaml: Optional[Path] = None,
        retune_bin: Optional[Path] = None,
        wheel_base: float = 4.76,
        ego_width: float = 1.9,
        ego_length: float = 5.0,
        steer_time_constant: Optional[float] = None,
    ) -> None:
        self._log_dir = log_dir
        self._frame_ids = discover_log_frames(log_dir)
        if not self._frame_ids:
            raise FileNotFoundError(f"No MPPI debug frames found in {log_dir}")
        self._index = 0
        if start_frame in self._frame_ids:
            self._index = self._frame_ids.index(start_frame)
        self._autoplay = autoplay
        self._enable_retune = enable_retune
        self._params_yaml = params_yaml
        self._params = load_params_from_log(log_dir, params_yaml)
        (
            self._wheel_base,
            self._ego_width,
            self._ego_length,
            logged_steer_tau,
        ) = load_vehicle_from_log(log_dir, wheel_base, ego_width, ego_length)
        self._steer_time_constant = max(
            float(steer_time_constant if steer_time_constant is not None else logged_steer_tau),
            1.0e-4,
        )
        self._status = "Ready"
        if enable_retune and not (log_dir / "000000_ego.csv").is_file():
            # Any frame's ego file; check first available frame.
            first_tag = f"{self._frame_ids[0]:06d}"
            if not (log_dir / f"{first_tag}_ego.csv").is_file():
                self._status = "WARNING: no *_ego.csv — retune uses ref[0] IC; re-log after rebuild"
        self._out_dir = Path(tempfile.mkdtemp(prefix="mppi_retune_")) if enable_retune else None
        self._retune_bin = retune_bin
        self._fig, self._axes = create_figure(with_retune_panel=enable_retune)
        for fig in (self._fig, *getattr(self._fig, "_mppi_related_figures", ())):
            fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._navigator = AxesNavigator(self._fig, self._axes)
        self._sliders: Dict[str, Slider] = {}
        if enable_retune:
            if self._retune_bin is None:
                self._retune_bin = find_retune_binary()
            self._build_retune_controls()
        self._show_current()
        plt.show(block=False)
        keys = "left/right or n/p = step, home/end, a = autoplay, escape = re-fit axes, q = quit"
        if enable_retune:
            keys += ", r = retune"
        print(f"Offline MPPI log: {log_dir} ({len(self._frame_ids)} frames). Keys: {keys}.")
        if enable_retune:
            print(
                f"Retune vehicle: wheel_base={self._wheel_base}, "
                f"ego_length={self._ego_length}, ego_width={self._ego_width}, "
                f"steer_time_constant={self._steer_time_constant}"
            )
            print(f"Retune binary: {self._retune_bin}")
            print("Move sliders, then click Retune (or press r). Sliders alone do nothing.")

    def _build_retune_controls(self) -> None:
        n = max(len(SLIDER_SPECS), 1)
        # Fit all cost-param sliders in the right panel without overlapping buttons.
        slider_h = min(0.022, 0.72 / n - 0.004)
        gap = 0.004
        top = 0.94
        for i, (name, vmin, vmax) in enumerate(SLIDER_SPECS):
            ax = self._fig.add_axes([0.72, top - i * (slider_h + gap), 0.26, slider_h])
            self._sliders[name] = Slider(
                ax, name, vmin, vmax, valinit=self._params.get(name, DEFAULT_PARAMS[name])
            )
        ax_prev = self._fig.add_axes([0.72, 0.06, 0.07, 0.035])
        ax_next = self._fig.add_axes([0.80, 0.06, 0.07, 0.035])
        ax_run = self._fig.add_axes([0.88, 0.06, 0.10, 0.035])
        self._btn_prev = Button(ax_prev, "Prev")
        self._btn_next = Button(ax_next, "Next")
        self._btn_run = Button(ax_run, "Retune")
        self._btn_prev.on_clicked(lambda _e: self._step(-1))
        self._btn_next.on_clicked(lambda _e: self._step(1))
        self._btn_run.on_clicked(lambda _e: self._retune_current())
        self._fig.canvas.manager.set_window_title("MPPI Offline Compare + Retune")

    def _load_frame(self, frame_id: int) -> MppiDebugFrame:
        tag = f"{frame_id:06d}"
        reference = load_trajectory_csv(self._log_dir / f"{tag}_reference.csv")
        optimized = load_trajectory_csv(self._log_dir / f"{tag}_optimized.csv")
        retuned = None
        costs = None
        rollouts = None
        if self._out_dir is not None:
            retuned = load_trajectory_csv(self._out_dir / f"{tag}_optimized.csv")
            if not retuned.x:
                retuned = None
            costs = load_costs_csv(self._out_dir / f"{tag}_costs.csv")
            if not costs.raw_costs:
                costs = None
            rollouts = load_rollouts_csv(self._out_dir / f"{tag}_rollouts.csv")
            if not rollouts:
                rollouts = None
        stamp = f"frame: {frame_id} / {self._frame_ids[-1]}"
        if self._enable_retune:
            stamp = f"{stamp}   |   {self._status}"
        frame = frame_from_loaded(
            reference,
            optimized,
            stamp_text=stamp,
            retuned=retuned,
            costs=costs,
            rollouts=rollouts,
            steer_time_constant=self._steer_time_constant,
            wheel_base=self._wheel_base,
        )
        self._fill_offline_replan_ade(frame, up_to_index=self._index)
        return frame

    def _fill_offline_replan_ade(self, frame: MppiDebugFrame, *, up_to_index: int) -> None:
        """Populate cycle-to-cycle ADE history from log frames [0 .. up_to_index]."""
        end = max(0, min(up_to_index, len(self._frame_ids) - 1))
        start = max(0, end - 200)  # keep the panel readable
        prev_ref: Optional[Tuple[List[float], List[float]]] = None
        prev_opt: Optional[Tuple[List[float], List[float]]] = None
        t_diff: List[float] = []
        ade_diff: List[float] = []
        t_mppi: List[float] = []
        ade_mppi: List[float] = []
        for i in range(start, end + 1):
            fid = self._frame_ids[i]
            tag = f"{fid:06d}"
            ref = load_trajectory_csv(self._log_dir / f"{tag}_reference.csv")
            opt = load_trajectory_csv(self._log_dir / f"{tag}_optimized.csv")
            ref_xy = (ref.x, ref.y) if ref.x else None
            opt_xy = (opt.x, opt.y) if opt.x else None
            ade_r = cycle_to_cycle_ade(prev_ref, ref_xy)
            if ade_r is not None:
                t_diff.append(float(fid))
                ade_diff.append(ade_r)
            ade_o = cycle_to_cycle_ade(prev_opt, opt_xy)
            if ade_o is not None:
                t_mppi.append(float(fid))
                ade_mppi.append(ade_o)
            prev_ref = copy_xy(ref_xy)
            prev_opt = copy_xy(opt_xy)
        frame.replan_ade_diffusion_times = t_diff
        frame.replan_ade_diffusion = ade_diff
        frame.replan_ade_mppi_times = t_mppi
        frame.replan_ade_mppi = ade_mppi

    def _show_current(self) -> None:
        frame_id = self._frame_ids[self._index]
        draw_frame(self._axes, self._load_frame(frame_id))
        redraw_figure_group(self._fig)

    def _step(self, delta: int) -> None:
        self._index = max(0, min(len(self._frame_ids) - 1, self._index + delta))
        self._show_current()

    def _current_params(self) -> Dict[str, float]:
        """All known cost params: log/yaml/defaults, overridden by slider values."""
        params = dict(self._params)
        for name, slider in self._sliders.items():
            params[name] = float(slider.val)
        return params

    def _retune_current(self) -> None:
        if not self._enable_retune or self._retune_bin is None or self._out_dir is None:
            return
        frame_id = self._frame_ids[self._index]
        params = self._current_params()
        cmd = [
            str(self._retune_bin),
            "--log-dir",
            str(self._log_dir),
            "--out-dir",
            str(self._out_dir),
            "--frame",
            str(frame_id),
            "--copy-reference",
            "--wheel-base",
            str(self._wheel_base),
            "--ego-width",
            str(self._ego_width),
            "--ego-length",
            str(self._ego_length),
        ]
        if self._params_yaml is not None:
            cmd.extend(["--params-yaml", str(self._params_yaml)])
        for key, value in params.items():
            cmd.extend(["--set", f"{key}={value}"])

        lam = params.get("lambda", float("nan"))
        track = params.get("track_coeff", float("nan"))
        self._status = (
            f"Retuning frame {frame_id} via {self._retune_bin.name} "
            f"(lambda={lam:.0f}, track={track:.0f})..."
        )
        self._show_current()
        try:
            completed = subprocess.run(cmd, check=True, capture_output=True, text=True)
            lines = [ln for ln in completed.stdout.splitlines() if ln.strip()]
            # Prefer the applied_params line when present; else last status line.
            applied = next((ln for ln in lines if ln.startswith("applied_params ")), "")
            tail = lines[-1] if lines else "OK"
            self._status = f"{applied} | {tail}" if applied else tail
            warn_lines = []
            if completed.stderr:
                warn_lines = [ln for ln in completed.stderr.splitlines() if "WARNING:" in ln]
            # Highlight when retune barely moved vs logged (usually lambda still too high).
            opt = load_trajectory_csv(self._out_dir / f"{frame_id:06d}_optimized.csv")
            logged = load_trajectory_csv(self._log_dir / f"{frame_id:06d}_optimized.csv")
            if opt.x and logged.x:
                vs = max_pos_err((logged.x, logged.y), (opt.x, opt.y))
                self._status = f"{self._status} | logged↔retune Δpos={vs:.3f}m"
                if vs < 0.5 and lam >= 2000.0:
                    self._status = (
                        f"{self._status} || tiny Δ — lower lambda to ~100–500 then Retune again"
                    )
            if warn_lines:
                self._status = f"{self._status}  ||  {warn_lines[-1]}"
        except subprocess.CalledProcessError as exc:
            err = (exc.stderr or exc.stdout or str(exc)).strip()
            self._status = f"Retune failed: {err[-240:]}"
        except FileNotFoundError as exc:
            self._status = f"Retune failed: {exc}"
        self._show_current()

    def _on_key(self, event) -> None:
        if event.key in ("right", "n"):
            self._step(1)
        elif event.key in ("left", "p"):
            self._step(-1)
        elif event.key == "home":
            self._index = 0
            self._show_current()
        elif event.key == "end":
            self._index = len(self._frame_ids) - 1
            self._show_current()
        elif event.key == "a":
            self._autoplay = not self._autoplay
        elif event.key == "escape":
            reset_view_baselines(self._axes)
            self._show_current()
        elif event.key == "r" and self._enable_retune:
            self._retune_current()
        elif event.key == "q":
            for fig in (self._fig, *getattr(self._fig, "_mppi_related_figures", ())):
                plt.close(fig)

    def spin(self) -> None:
        while plt.fignum_exists(self._fig.number):
            if self._autoplay:
                if self._index < len(self._frame_ids) - 1:
                    self._step(1)
                else:
                    self._autoplay = False
            plt.pause(0.05)


def parse_args(argv: List[str]) -> argparse.Namespace:
    default_prefix = (
        "/planning/trajectory_generator/neural_network_based_planner/"
        "diffusion_planner_node/debug/mppi"
    )
    parser = argparse.ArgumentParser(
        description="Plot diffusion-planner reference vs MPPI-optimized trajectories.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--topic-prefix",
        default=default_prefix,
        help="Prefix for ~/debug/mppi/{reference,optimized}_trajectory and enabled topics",
    )
    parser.add_argument(
        "--log-dir",
        type=str,
        default="",
        help="If set, replay CSVs from enable_debug_trajectory_log (offline; no ROS topics)",
    )
    parser.add_argument(
        "--start-frame",
        type=int,
        default=0,
        help="Initial frame id when using --log-dir",
    )
    parser.add_argument(
        "--autoplay",
        action="store_true",
        help="Auto-advance frames when using --log-dir (toggle with 'a')",
    )
    parser.add_argument(
        "--enable-retune",
        action="store_true",
        help="Offline: show cost sliders and re-run MPPI (requires mppi_offline_retune)",
    )
    parser.add_argument(
        "--params-yaml",
        default="",
        help="Baseline mppi_optimizer.param.yaml for --enable-retune",
    )
    parser.add_argument(
        "--retune-bin",
        default="",
        help="Path to mppi_offline_retune (auto-detected if empty)",
    )
    parser.add_argument("--ego-width", type=float, default=1.9)
    parser.add_argument("--ego-length", type=float, default=5.0)
    parser.add_argument(
        "--update-hz",
        type=float,
        default=10.0,
        help="Matplotlib refresh rate (live mode)",
    )
    parser.add_argument(
        "--wheel-base",
        type=float,
        default=4.76,
        help="Wheel base [m] used to derive steering from path curvature when unset (j6_gen2 ~4.76)",
    )
    parser.add_argument(
        "--steer-time-constant",
        type=float,
        default=None,
        help=(
            "Override steer lag τ [s]. Default: live uses ROS param steer_time_constant "
            "(same as MPPI / simulator_model); offline uses vehicle_params.csv."
        ),
    )
    parser.add_argument(
        "--measured-steering-topic",
        default="/vehicle/status/steering_status",
        help="SteeringReport topic for measured tire angle used by MPPI",
    )
    return parser.parse_args(argv)


def main() -> None:
    filtered_argv = remove_ros_args(args=sys.argv)
    cli = parse_args(filtered_argv[1:])

    try:
        matplotlib.rcParams["figure.raise_window"] = False
    except KeyError:
        pass

    if cli.log_dir:
        log_dir = Path(cli.log_dir).expanduser().resolve()
        params_yaml = Path(cli.params_yaml).expanduser().resolve() if cli.params_yaml else None
        retune_bin = Path(cli.retune_bin) if cli.retune_bin else None
        if cli.enable_retune and retune_bin is None:
            retune_bin = find_retune_binary()
        visualizer = OfflineLogVisualizer(
            log_dir,
            start_frame=cli.start_frame,
            autoplay=cli.autoplay,
            enable_retune=cli.enable_retune,
            params_yaml=params_yaml,
            retune_bin=retune_bin,
            wheel_base=cli.wheel_base,
            ego_width=cli.ego_width,
            ego_length=cli.ego_length,
            steer_time_constant=cli.steer_time_constant,
        )
        try:
            visualizer.spin()
        except KeyboardInterrupt:
            pass
        finally:
            plt.close("all")
        return

    rclpy.init(args=filtered_argv)
    node = MppiDebugVisualizer(
        topic_prefix=cli.topic_prefix.rstrip("/"),
        update_hz=cli.update_hz,
        wheel_base=cli.wheel_base,
        measured_steering_topic=cli.measured_steering_topic,
        steer_time_constant=cli.steer_time_constant,
    )

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not plt.fignum_exists(node._fig.number):
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
