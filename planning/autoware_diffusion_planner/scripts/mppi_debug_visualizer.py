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
  <log_dir>/000000_reference.csv
  <log_dir>/000000_optimized.csv
  ...

Trajectory CSV columns:
  t_from_start_s,x,y,z,yaw,v,a,steer,steer_rate

Offline retune mode (--enable-retune) overlays a third retuned trajectory and lets you
re-run mppi_offline_retune with editable cost weights.
"""

from __future__ import annotations

import argparse
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
from typing import Dict
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

from autoware_planning_msgs.msg import Trajectory
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


DEFAULT_PARAMS: Dict[str, float] = {
    "lambda": 3000.0,
    "speed_coeff": 500.0,
    "track_coeff": 1000.0,
    "heading_coeff": 500.0,
    "crash_coeff": 100000.0,
    "boundary_threshold": 0.8,
    "accel_cmd_coeff": 200.0,
    "steer_cmd_coeff": 1000.0,
    "steer_rate_coeff": 3000.0,
    "lateral_acceleration_coeff": 500.0,
    "lateral_jerk_coeff": 3000.0,
    "longitudinal_jerk_coeff": 1000.0,
    "goal_pos_coeff": 1000.0,
    "goal_yaw_coeff": 500.0,
    "road_border_margin": 0.2,
}

SLIDER_SPECS: List[Tuple[str, float, float]] = [
    ("lambda", 100.0, 20000.0),
    ("track_coeff", 0.0, 5000.0),
    ("speed_coeff", 0.0, 5000.0),
    ("heading_coeff", 0.0, 5000.0),
    ("steer_cmd_coeff", 0.0, 5000.0),
    ("steer_rate_coeff", 0.0, 10000.0),
    ("accel_cmd_coeff", 0.0, 2000.0),
    ("lateral_jerk_coeff", 0.0, 10000.0),
    ("longitudinal_jerk_coeff", 0.0, 5000.0),
    ("boundary_threshold", 0.1, 5.0),
    ("road_border_margin", 0.0, 1.0),
    ("crash_coeff", 1000.0, 500000.0),
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
    stamp_text: str = ""
    metrics_text: str = ""


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
    if not ref_xy or not opt_xy or not ref_xy[0] or not opt_xy[0]:
        return float("nan")
    n = min(len(ref_xy[0]), len(opt_xy[0]))
    return max(
        math.hypot(opt_xy[0][i] - ref_xy[0][i], opt_xy[1][i] - ref_xy[1][i]) for i in range(n)
    )


def max_vel_err(ref_v: Sequence[float], opt_v: Sequence[float]) -> float:
    if not ref_v or not opt_v:
        return float("nan")
    n = min(len(ref_v), len(opt_v))
    return max(abs(opt_v[i] - ref_v[i]) for i in range(n))


def frame_from_loaded(
    reference: LoadedTrajectory,
    optimized: LoadedTrajectory,
    stamp_text: str,
    retuned: Optional[LoadedTrajectory] = None,
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
        stamp_text=stamp_text,
    )
    orig_pos = max_pos_err(frame.reference_xy, frame.optimized_xy)
    orig_vel = max_vel_err(frame.reference_vel, frame.optimized_vel)
    parts = [f"orig max|pos|={orig_pos:.3f}m max|v|={orig_vel:.3f}m/s"]
    if frame.retuned_xy:
        ret_pos = max_pos_err(frame.reference_xy, frame.retuned_xy)
        ret_vel = max_vel_err(frame.reference_vel, frame.retuned_vel)
        parts.append(f"retune max|pos|={ret_pos:.3f}m max|v|={ret_vel:.3f}m/s")
    frame.metrics_text = "  |  ".join(parts)
    return frame


def draw_frame(axes, frame: MppiDebugFrame) -> None:
    ax_xy, ax_heading, ax_vel, ax_accel, ax_steer, ax_steer_rate = axes

    lengths = [len(frame.reference_vel), len(frame.optimized_vel)]
    if frame.retuned_vel:
        lengths.append(len(frame.retuned_vel))
    n_compare = min(l for l in lengths if l > 0) if any(lengths) else 0

    ax_xy.clear()
    ax_xy.set_title("Trajectory (diffusion ref vs MPPI)")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.grid(True)
    if frame.reference_xy and len(frame.reference_xy[0]) > 0:
        ax_xy.plot(
            frame.reference_xy[0],
            frame.reference_xy[1],
            color="cyan",
            linestyle="--",
            linewidth=2,
            label="diffusion reference",
        )
    if frame.optimized_xy and len(frame.optimized_xy[0]) > 0:
        ax_xy.plot(
            frame.optimized_xy[0],
            frame.optimized_xy[1],
            color="red",
            linewidth=2,
            label="MPPI optimized (logged)",
        )
    if frame.retuned_xy and len(frame.retuned_xy[0]) > 0:
        ax_xy.plot(
            frame.retuned_xy[0],
            frame.retuned_xy[1],
            color="tab:green",
            linewidth=2.2,
            label="MPPI retuned",
        )
    overlay = frame.stamp_text
    if frame.metrics_text:
        overlay = f"{overlay}\n{frame.metrics_text}" if overlay else frame.metrics_text
    if overlay:
        ax_xy.text(
            0.02,
            0.98,
            overlay,
            transform=ax_xy.transAxes,
            verticalalignment="top",
            fontsize=9,
            bbox={"facecolor": "white", "alpha": 0.85},
        )
    if (
        (frame.reference_xy and len(frame.reference_xy[0]) > 0)
        or (frame.optimized_xy and len(frame.optimized_xy[0]) > 0)
        or (frame.retuned_xy and len(frame.retuned_xy[0]) > 0)
    ):
        ax_xy.relim()
        ax_xy.autoscale_view()
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(loc="best")

    idx = list(range(n_compare)) if n_compare > 0 else []

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

    ax_steer.clear()
    ax_steer.set_title("Steering")
    ax_steer.set_xlabel("point index")
    ax_steer.set_ylabel("δ [rad]")
    ax_steer.grid(True)
    if n_compare > 0:
        ax_steer.plot(
            idx,
            frame.reference_steer[:n_compare],
            color="tab:orange",
            linestyle="--",
            linewidth=2,
            label="diffusion δ",
        )
        ax_steer.plot(
            idx,
            frame.optimized_steer[:n_compare],
            color="tab:orange",
            linewidth=2,
            label="MPPI logged",
        )
        if frame.retuned_steer:
            ax_steer.plot(
                idx,
                frame.retuned_steer[:n_compare],
                color="tab:green",
                linewidth=2.2,
                label="MPPI retuned",
            )
        ax_steer.legend(loc="best")

    ax_steer_rate.clear()
    ax_steer_rate.set_title("Steering rate δ̇ = (δ_cmd − δ) / τ")
    ax_steer_rate.set_xlabel("point index")
    ax_steer_rate.set_ylabel("δ̇ [rad/s]")
    ax_steer_rate.grid(True)
    plotted = False
    if frame.optimized_steer_rate:
        idx_rate = list(range(len(frame.optimized_steer_rate)))
        ax_steer_rate.plot(
            idx_rate,
            frame.optimized_steer_rate,
            color="tab:purple",
            linewidth=2,
            label="MPPI logged δ̇",
        )
        plotted = True
    if frame.retuned_steer_rate:
        idx_rate = list(range(len(frame.retuned_steer_rate)))
        ax_steer_rate.plot(
            idx_rate,
            frame.retuned_steer_rate,
            color="tab:green",
            linewidth=2.2,
            label="MPPI retuned δ̇",
        )
        plotted = True
    if plotted:
        ax_steer_rate.legend(loc="best")


def create_figure(*, with_retune_panel: bool = False):
    if with_retune_panel:
        fig = plt.figure(figsize=(17, 12))
        gs = gridspec.GridSpec(
            5, 3, figure=fig, width_ratios=[1.25, 1.0, 0.78], wspace=0.30, hspace=0.42
        )
        ax_xy = fig.add_subplot(gs[:, 0])
        ax_heading = fig.add_subplot(gs[0, 1])
        ax_vel = fig.add_subplot(gs[1, 1])
        ax_accel = fig.add_subplot(gs[2, 1])
        ax_steer = fig.add_subplot(gs[3, 1])
        ax_steer_rate = fig.add_subplot(gs[4, 1])
    else:
        fig = plt.figure(figsize=(14, 12))
        gs = gridspec.GridSpec(5, 2, figure=fig, width_ratios=[1.2, 1.0], wspace=0.28, hspace=0.42)
        ax_xy = fig.add_subplot(gs[:, 0])
        ax_heading = fig.add_subplot(gs[0, 1])
        ax_vel = fig.add_subplot(gs[1, 1])
        ax_accel = fig.add_subplot(gs[2, 1])
        ax_steer = fig.add_subplot(gs[3, 1])
        ax_steer_rate = fig.add_subplot(gs[4, 1])
    fig.canvas.manager.set_window_title("Diffusion Planner MPPI Debug Visualizer")
    return fig, (ax_xy, ax_heading, ax_vel, ax_accel, ax_steer, ax_steer_rate)


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
    def __init__(self, *, topic_prefix: str, update_hz: float, wheel_base: float) -> None:
        super().__init__("mppi_debug_visualizer")

        update_hz = max(update_hz, 1.0)
        self._wheel_base = wheel_base
        self._lock = threading.Lock()
        self._frame = MppiDebugFrame()
        self._logged_reference = False
        self._logged_optimized = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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

        self._fig, self._axes = create_figure()
        self._configure_window_no_focus_steal()
        plt.show(block=False)

        self.get_logger().info("MPPI debug visualizer started (live).")
        self.get_logger().info(f"Reference: {prefix}/reference_trajectory")
        self.get_logger().info(f"Optimized: {prefix}/optimized_trajectory")
        self.get_logger().info(
            "Subscriptions use RELIABLE QoS (matches diffusion_planner publishers)."
        )
        self.get_logger().info("Ensure use_mppi_optimizer:=true in diffusion_planner params.")

        self.create_timer(1.0 / update_hz, self.on_timer)

    def _configure_window_no_focus_steal(self) -> None:
        try:
            win = self._fig.canvas.manager.window
        except (AttributeError, TypeError):
            return
        if win is None:
            return
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

    def on_reference_trajectory(self, msg: Trajectory) -> None:
        processed = self._process_trajectory(msg)
        with self._lock:
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
        with self._lock:
            self._frame.optimized_xy = processed.reference_xy
            self._frame.optimized_heading = processed.reference_heading
            self._frame.optimized_vel = processed.reference_vel
            self._frame.optimized_accel = processed.reference_accel
            self._frame.optimized_steer = processed.reference_steer
            self._frame.optimized_steer_rate = processed.reference_steer_rate
            if not self._frame.stamp_text:
                self._frame.stamp_text = processed.stamp_text
        if not self._logged_optimized and msg.points:
            self._logged_optimized = True
            self.get_logger().info(f"Receiving optimized_trajectory ({len(msg.points)} points).")

    def on_timer(self) -> None:
        with self._lock:
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
                optimized_steer_rate=list(self._frame.optimized_steer_rate),
                stamp_text=self._frame.stamp_text,
                metrics_text=self._frame.metrics_text,
            )
            if frame.reference_xy and frame.optimized_xy:
                frame.metrics_text = (
                    f"max|pos|={max_pos_err(frame.reference_xy, frame.optimized_xy):.3f}m  "
                    f"max|v|={max_vel_err(frame.reference_vel, frame.optimized_vel):.3f}m/s"
                )
        draw_frame(self._axes, frame)
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()


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
        self._params = load_params_yaml(params_yaml)
        self._wheel_base = wheel_base
        self._ego_width = ego_width
        self._ego_length = ego_length
        self._status = "Ready"
        self._out_dir = Path(tempfile.mkdtemp(prefix="mppi_retune_")) if enable_retune else None
        self._retune_bin = retune_bin
        self._fig, self._axes = create_figure(with_retune_panel=enable_retune)
        self._fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._sliders: Dict[str, Slider] = {}
        if enable_retune:
            if self._retune_bin is None:
                self._retune_bin = find_retune_binary()
            self._build_retune_controls()
        self._show_current()
        plt.show(block=False)
        keys = "left/right or n/p = step, home/end, a = autoplay, q = quit"
        if enable_retune:
            keys += ", r = retune"
        print(f"Offline MPPI log: {log_dir} ({len(self._frame_ids)} frames). Keys: {keys}.")

    def _build_retune_controls(self) -> None:
        slider_h = 0.022
        top = 0.92
        for i, (name, vmin, vmax) in enumerate(SLIDER_SPECS):
            ax = self._fig.add_axes([0.72, top - i * (slider_h + 0.010), 0.26, slider_h])
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
        if self._out_dir is not None:
            retuned = load_trajectory_csv(self._out_dir / f"{tag}_optimized.csv")
            if not retuned.x:
                retuned = None
        stamp = f"frame: {frame_id} / {self._frame_ids[-1]}"
        if self._enable_retune:
            stamp = f"{stamp}   |   {self._status}"
        return frame_from_loaded(reference, optimized, stamp_text=stamp, retuned=retuned)

    def _show_current(self) -> None:
        frame_id = self._frame_ids[self._index]
        draw_frame(self._axes, self._load_frame(frame_id))
        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()

    def _step(self, delta: int) -> None:
        self._index = max(0, min(len(self._frame_ids) - 1, self._index + delta))
        self._show_current()

    def _current_params(self) -> Dict[str, float]:
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

        self._status = f"Retuning frame {frame_id}..."
        self._show_current()
        try:
            completed = subprocess.run(cmd, check=True, capture_output=True, text=True)
            self._status = completed.stdout.strip().splitlines()[-1] if completed.stdout else "OK"
        except subprocess.CalledProcessError as exc:
            err = (exc.stderr or exc.stdout or str(exc)).strip()
            self._status = f"Retune failed: {err[-240:]}"
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
        elif event.key == "r" and self._enable_retune:
            self._retune_current()
        elif event.key == "q":
            plt.close(self._fig)

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
        help="Prefix for ~/debug/mppi/{reference,optimized}_trajectory topics",
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
