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

"""Live comparison of diffusion-planner reference vs MPPI-optimized trajectories."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from dataclasses import field
import math
import sys
import threading
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

from autoware_planning_msgs.msg import Trajectory
import matplotlib
import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.utilities import remove_ros_args


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
    durations = [
        p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points
    ]
    if durations[-1] > durations[0]:
        return max((durations[-1] - durations[0]) / max(len(points) - 1, 1), 1e-3)
    return 0.1


@dataclass
class MppiDebugFrame:
    reference_xy: Optional[Tuple[List[float], List[float]]] = None
    optimized_xy: Optional[Tuple[List[float], List[float]]] = None
    reference_heading: List[float] = field(default_factory=list)
    optimized_heading: List[float] = field(default_factory=list)
    reference_vel: List[float] = field(default_factory=list)
    optimized_vel: List[float] = field(default_factory=list)
    reference_accel: List[float] = field(default_factory=list)
    optimized_accel: List[float] = field(default_factory=list)
    reference_steer: List[float] = field(default_factory=list)
    optimized_steer: List[float] = field(default_factory=list)
    reference_steer_rate: List[float] = field(default_factory=list)
    optimized_steer_rate: List[float] = field(default_factory=list)
    stamp_text: str = ""


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

        self._fig = plt.figure(figsize=(14, 12))
        gs = gridspec.GridSpec(5, 2, figure=self._fig, width_ratios=[1.2, 1.0], wspace=0.28, hspace=0.42)
        self._ax_xy = self._fig.add_subplot(gs[:, 0])
        self._ax_heading = self._fig.add_subplot(gs[0, 1])
        self._ax_vel = self._fig.add_subplot(gs[1, 1])
        self._ax_accel = self._fig.add_subplot(gs[2, 1])
        self._ax_steer = self._fig.add_subplot(gs[3, 1])
        self._ax_steer_rate = self._fig.add_subplot(gs[4, 1])

        self._fig.canvas.manager.set_window_title("Diffusion Planner MPPI Debug Visualizer")
        self._configure_window_no_focus_steal()
        plt.show(block=False)

        self.get_logger().info("MPPI debug visualizer started.")
        self.get_logger().info(f"Reference: {prefix}/reference_trajectory")
        self.get_logger().info(f"Optimized: {prefix}/optimized_trajectory")
        self.get_logger().info("Subscriptions use RELIABLE QoS (matches diffusion_planner publishers).")
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
            # Cost formula: δ̇ = (steer_cmd - steer) / τ, published as heading_rate_rps
            steer_rate = trajectory_steer_rate(points)
        else:
            accel = trajectory_acceleration(points)
            if all(abs(a) < 1e-9 for a in accel) and len(velocities) > 1:
                accel = finite_difference_acceleration(velocities, dt)
            steer = trajectory_steering(points, self._wheel_base)
            # Diffusion has no first-order steer lag state; leave empty (plot MPPI only)
            steer_rate = []
        stamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
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
            )

        n_compare = 0
        if frame.reference_vel and frame.optimized_vel:
            n_compare = min(len(frame.reference_vel), len(frame.optimized_vel))

        # XY path
        self._ax_xy.clear()
        self._ax_xy.set_title("Trajectory (diffusion ref vs MPPI optimized)")
        self._ax_xy.set_xlabel("x [m]")
        self._ax_xy.set_ylabel("y [m]")
        self._ax_xy.grid(True)
        if frame.reference_xy and len(frame.reference_xy[0]) > 0:
            self._ax_xy.plot(
                frame.reference_xy[0],
                frame.reference_xy[1],
                color="cyan",
                linestyle="--",
                linewidth=2,
                label="diffusion reference",
            )
        if frame.optimized_xy and len(frame.optimized_xy[0]) > 0:
            self._ax_xy.plot(
                frame.optimized_xy[0],
                frame.optimized_xy[1],
                color="red",
                linewidth=2,
                label="MPPI optimized",
            )
        if frame.stamp_text:
            self._ax_xy.text(
                0.02,
                0.98,
                f"stamp: {frame.stamp_text}",
                transform=self._ax_xy.transAxes,
                verticalalignment="top",
                fontsize=9,
                bbox={"facecolor": "white", "alpha": 0.8},
            )
        if (
            (frame.reference_xy and len(frame.reference_xy[0]) > 0)
            or (frame.optimized_xy and len(frame.optimized_xy[0]) > 0)
        ):
            self._ax_xy.relim()
            self._ax_xy.autoscale_view()
        self._ax_xy.set_aspect("equal", adjustable="datalim")
        self._ax_xy.legend(loc="best")

        idx = list(range(n_compare)) if n_compare > 0 else []

        # Heading
        self._ax_heading.clear()
        self._ax_heading.set_title("Heading")
        self._ax_heading.set_xlabel("point index")
        self._ax_heading.set_ylabel("yaw [rad]")
        self._ax_heading.grid(True)
        if n_compare > 0:
            self._ax_heading.plot(
                idx, frame.reference_heading[:n_compare], "c--", linewidth=2, label="diffusion"
            )
            self._ax_heading.plot(
                idx, frame.optimized_heading[:n_compare], "r-", linewidth=2, label="MPPI"
            )
            self._ax_heading.legend(loc="best")

        # Velocity
        self._ax_vel.clear()
        self._ax_vel.set_title("Longitudinal velocity")
        self._ax_vel.set_xlabel("point index")
        self._ax_vel.set_ylabel("v [m/s]")
        self._ax_vel.grid(True)
        if n_compare > 0:
            self._ax_vel.plot(
                idx, frame.reference_vel[:n_compare], "c--", linewidth=2, label="diffusion"
            )
            self._ax_vel.plot(
                idx, frame.optimized_vel[:n_compare], "r-", linewidth=2, label="MPPI"
            )
            self._ax_vel.legend(loc="best")

        # Acceleration
        self._ax_accel.clear()
        self._ax_accel.set_title("Acceleration (MPPI: optimal control sequence)")
        self._ax_accel.set_xlabel("point index")
        self._ax_accel.set_ylabel("a [m/s²]")
        self._ax_accel.grid(True)
        if n_compare > 0:
            self._ax_accel.plot(
                idx,
                frame.reference_accel[:n_compare],
                color="tab:blue",
                linestyle="--",
                linewidth=2,
                label="diffusion accel",
            )
            self._ax_accel.plot(
                idx,
                frame.optimized_accel[:n_compare],
                color="tab:blue",
                linewidth=2,
                label="MPPI accel cmd",
            )
            self._ax_accel.legend(loc="best")

        # Steering
        self._ax_steer.clear()
        self._ax_steer.set_title("Steering (MPPI: optimal control sequence)")
        self._ax_steer.set_xlabel("point index")
        self._ax_steer.set_ylabel("δ [rad]")
        self._ax_steer.grid(True)
        if n_compare > 0:
            self._ax_steer.plot(
                idx,
                frame.reference_steer[:n_compare],
                color="tab:orange",
                linestyle="--",
                linewidth=2,
                label="diffusion δ",
            )
            self._ax_steer.plot(
                idx,
                frame.optimized_steer[:n_compare],
                color="tab:orange",
                linewidth=2,
                label="MPPI steer cmd",
            )
            self._ax_steer.legend(loc="best")

        # Steering rate
        self._ax_steer_rate.clear()
        self._ax_steer_rate.set_title("Steering rate δ̇ = (δ_cmd − δ) / τ")
        self._ax_steer_rate.set_xlabel("point index")
        self._ax_steer_rate.set_ylabel("δ̇ [rad/s]")
        self._ax_steer_rate.grid(True)
        if frame.optimized_steer_rate:
            idx_rate = list(range(len(frame.optimized_steer_rate)))
            self._ax_steer_rate.plot(
                idx_rate,
                frame.optimized_steer_rate,
                color="tab:purple",
                linewidth=2,
                label="MPPI δ̇",
            )
            self._ax_steer_rate.legend(loc="best")

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()


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
        "--update-hz",
        type=float,
        default=10.0,
        help="Matplotlib refresh rate",
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
    rclpy.init(args=filtered_argv)

    node = MppiDebugVisualizer(
        topic_prefix=cli.topic_prefix.rstrip("/"),
        update_hz=cli.update_hz,
        wheel_base=cli.wheel_base,
    )

    try:
        matplotlib.rcParams["figure.raise_window"] = False
    except KeyError:
        pass

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
