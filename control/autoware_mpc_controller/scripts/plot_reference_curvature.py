#!/usr/bin/env python3
# Copyright 2025 TIER IV, Inc.
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

"""
Plot curvature of the reference trajectory (same 3-point Menger formula as motion_utils::calcCurvature).
Use to check if noisy or oscillating curvature could explain MPC oscillation.

Run (with planning publishing trajectory):
  python3 scripts/plot_reference_curvature.py
  # or: ros2 run autoware_mpc_controller plot_reference_curvature.py

Subscribes to /planning/trajectory; updates plot when a new trajectory is received.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from autoware_planning_msgs.msg import Trajectory


def calc_distance2d(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def calc_curvature_menger(x1, y1, x2, y2, x3, y3):
    """Same as autoware_utils::calc_curvature (Menger curvature from 3 points)."""
    d12 = calc_distance2d(x1, y1, x2, y2)
    d23 = calc_distance2d(x2, y2, x3, y3)
    d31 = calc_distance2d(x3, y3, x1, y1)
    denom = d12 * d23 * d31
    if abs(denom) < 1e-10:
        return 0.0
    return 2.0 * ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)) / denom


def curvatures_and_arc_lengths(points):
    """Compute curvature (motion_utils style: 3 consecutive points) and arc length per point."""
    n = len(points)
    curvature = [0.0] * n
    arc_length = [0.0] * n
    if n < 3:
        return arc_length, curvature
    for i in range(1, n - 1):
        p1 = points[i - 1].pose.position
        p2 = points[i].pose.position
        p3 = points[i + 1].pose.position
        try:
            curvature[i] = calc_curvature_menger(
                p1.x, p1.y, p2.x, p2.y, p3.x, p3.y
            )
        except Exception:
            curvature[i] = 0.0
    curvature[0] = curvature[1]
    curvature[-1] = curvature[-2]
    for i in range(1, n):
        p0 = points[i - 1].pose.position
        p1 = points[i].pose.position
        arc_length[i] = arc_length[i - 1] + calc_distance2d(p0.x, p0.y, p1.x, p1.y)
    return arc_length, curvature


class CurvaturePlotNode(Node):
    def __init__(self, trajectory_topic: str = "/planning/trajectory"):
        super().__init__("plot_reference_curvature")
        self._traj = None
        self._sub = self.create_subscription(
            Trajectory,
            trajectory_topic,
            self._cb_trajectory,
            1,
        )
        self.get_logger().info(f"Subscribed to {trajectory_topic}. Close the plot window to exit.")

    def _cb_trajectory(self, msg: Trajectory):
        self._traj = msg

    def get_latest_and_clear(self):
        t = self._traj
        self._traj = None
        return t


def run_plot(args=None):
    import matplotlib
    matplotlib.use("QtAgg")
    import matplotlib.pyplot as plt

    rclpy.init(args=args)
    node = CurvaturePlotNode(
        trajectory_topic=args.trajectory_topic if args else "/planning/trajectory"
    )

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax_s, ax_k, ax_v = axes
    ax_s.set_ylabel("arc length (m)")
    ax_s.set_title("Reference path (xy)")
    ax_k.set_ylabel("curvature (1/m)")
    ax_k.set_title("Curvature (same as motion_utils::calcCurvature)")
    ax_v.set_ylabel("v_ref (m/s)")
    ax_v.set_xlabel("arc length (m)")
    ax_v.set_title("Longitudinal velocity reference")

    traj_plotted = None

    def update_plot():
        nonlocal traj_plotted
        traj = node.get_latest_and_clear()
        if traj is not None:
            traj_plotted = traj
        if traj_plotted is None or len(traj_plotted.points) < 2:
            for ax in axes:
                ax.clear()
            ax_s.set_title("Reference path (xy) — waiting for trajectory...")
            ax_k.set_title("Curvature — waiting for trajectory...")
            ax_v.set_title("v_ref — waiting for trajectory...")
            return
        pts = traj_plotted.points
        arc_length, curvature = curvatures_and_arc_lengths(pts)
        xs = [p.pose.position.x for p in pts]
        ys = [p.pose.position.y for p in pts]
        v_ref = [p.longitudinal_velocity_mps for p in pts]

        for ax in axes:
            ax.clear()
        ax_s.plot(xs, ys, "b-")
        ax_s.set_ylabel("y (m)")
        ax_s.set_xlabel("x (m)")
        ax_s.set_title("Reference path (xy)")
        ax_s.axis("equal")
        ax_s.grid(True)

        ax_k.plot(arc_length, curvature, "b-")
        ax_k.set_ylabel("curvature (1/m)")
        ax_k.set_title("Curvature vs arc length (noisy curvature can cause MPC oscillation)")
        ax_k.axhline(0, color="gray", linestyle="--")
        ax_k.grid(True)

        ax_v.plot(arc_length, v_ref, "g-")
        ax_v.set_ylabel("v_ref (m/s)")
        ax_v.set_xlabel("arc length (m)")
        ax_v.set_title("Longitudinal velocity reference")
        ax_v.grid(True)

        plt.tight_layout()

    plt.ion()
    plt.show()
    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            rclpy.spin_once(node, timeout_sec=0.1)
            update_plot()
            plt.pause(0.05)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    import argparse
    parser = argparse.ArgumentParser(description="Plot reference trajectory curvature")
    parser.add_argument(
        "--trajectory-topic",
        type=str,
        default="/planning/trajectory",
        help="Reference trajectory topic",
    )
    parsed = parser.parse_args(args)
    run_plot(parsed)


if __name__ == "__main__":
    main()
