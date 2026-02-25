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
# WITHOUT WARRANTIES OR CONDITIONS UNDER THE KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Visualize MPC controller output: steering, velocity, acceleration, reference and predicted trajectory.

Run (after sourcing the workspace and with control stack running):
  ros2 run autoware_mpc_controller visualize_mpc_control.py

Or:
  python3 scripts/visualize_mpc_control.py

Subscribes to:
  - /control/command/control_cmd (or /control/trajectory_follower/control_cmd): control command
  - /localization/kinematic_state: odometry (actual velocity)
  - /planning/trajectory: reference trajectory
  - /control/trajectory_follower/lateral/predicted_trajectory: MPC predicted path

Requires: matplotlib (pip install matplotlib)
"""

import argparse
from collections import deque
from threading import Event, Lock, Thread

import rclpy
from rclpy.node import Node
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from nav_msgs.msg import Odometry


class ControlVizNode(Node):
    def __init__(self, history_sec: float = 30.0, control_cmd_topic: str = None):
        super().__init__("visualize_mpc_control")
        self.declare_parameter("history_sec", history_sec)
        self.declare_parameter("control_cmd_topic", control_cmd_topic or "")
        history_sec = self.get_parameter("history_sec").value
        control_cmd_topic = self.get_parameter("control_cmd_topic").value or "/control/command/control_cmd"

        self._lock = Lock()
        self._t = deque(maxlen=int(history_sec * 100))  # assume ~10 Hz, keep history_sec
        self._steering = deque(maxlen=self._t.maxlen)
        self._accel_cmd = deque(maxlen=self._t.maxlen)
        self._v_ref = deque(maxlen=self._t.maxlen)
        self._v_actual = deque(maxlen=self._t.maxlen)
        self._ref_traj = None
        self._pred_traj = None
        self._start_time = None
        self._latest_v = 0.0

        self._sub_control = self.create_subscription(
            Control,
            control_cmd_topic,
            self._cb_control,
            10,
        )
        self._sub_odom = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            self._cb_odom,
            10,
        )
        self._sub_ref = self.create_subscription(
            Trajectory,
            "/planning/trajectory",
            self._cb_ref,
            10,
        )
        self._sub_pred = self.create_subscription(
            Trajectory,
            "/control/trajectory_follower/lateral/predicted_trajectory",
            self._cb_pred,
            10,
        )

        self.get_logger().info(
            f"Visualizing control: {control_cmd_topic}, odom, ref trajectory, predicted trajectory"
        )
        self.get_logger().info("Run: python3 -c \"from scripts.visualize_mpc_control import run_viz; run_viz()\" or use the launch helper.")

    def _now_sec(self):
        t = self.get_clock().now()
        return t.nanoseconds * 1e-9

    def _cb_control(self, msg: Control):
        with self._lock:
            if self._start_time is None:
                self._start_time = self._now_sec()
            t = self._now_sec() - self._start_time
            self._t.append(t)
            self._steering.append(msg.lateral.steering_tire_angle)
            self._accel_cmd.append(msg.longitudinal.acceleration)
            self._v_ref.append(msg.longitudinal.velocity)
            self._v_actual.append(self._latest_v)

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            self._latest_v = msg.twist.twist.linear.x

    def _cb_ref(self, msg: Trajectory):
        with self._lock:
            self._ref_traj = msg

    def _cb_pred(self, msg: Trajectory):
        with self._lock:
            self._pred_traj = msg

    def get_snapshot(self):
        with self._lock:
            t = list(self._t)
            steering = list(self._steering)
            accel = list(self._accel_cmd)
            v_ref = list(self._v_ref)
            v_actual = list(self._v_actual)
            ref = self._ref_traj
            pred = self._pred_traj
        return t, steering, accel, v_ref, v_actual, ref, pred


def run_visualizer(viz_opts=None):
    import matplotlib
    matplotlib.use("QtAgg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    rclpy.init()
    node = ControlVizNode(
        history_sec=getattr(viz_opts, "history_sec", 30.0) if viz_opts else 30.0,
        control_cmd_topic=getattr(viz_opts, "control_cmd_topic", None) if viz_opts else None,
    )

    # Run ROS in a background thread so the UI stays responsive
    shutdown_event = Event()

    def ros_spin_thread_fn():
        while rclpy.ok() and not shutdown_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)

    ros_thread = Thread(target=ros_spin_thread_fn, daemon=True)
    ros_thread.start()

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    (ax_steer, ax_vel), (ax_acc, ax_xy) = axes
    plt.tight_layout()

    def animate(_):
        t, steering, accel, v_ref, v_actual, ref, pred = node.get_snapshot()
        for ax in axes.flat:
            ax.clear()

        if not t:
            ax_steer.set_title("Steering (rad)")
            ax_vel.set_title("Velocity (m/s)")
            ax_acc.set_title("Acceleration (m/s²)")
            ax_xy.set_title("Reference vs predicted (xy)")
            ax_xy.text(0.5, 0.5, "Waiting for control_cmd, odom, trajectory...\n"
                       "Try: --control-cmd-topic /control/trajectory_follower/control_cmd",
                       transform=ax_xy.transAxes, ha="center", va="center", fontsize=9)
            return

        ax_steer.plot(t, steering, "b-", label="steering")
        ax_steer.set_ylabel("steering_tire_angle (rad)")
        ax_steer.legend(loc="upper right")
        ax_steer.grid(True)

        ax_vel.plot(t, v_ref, "g--", label="v_ref")
        ax_vel.plot(t, v_actual, "b-", label="v_actual")
        ax_vel.set_ylabel("velocity (m/s)")
        ax_vel.legend(loc="upper right")
        ax_vel.grid(True)

        ax_acc.plot(t, accel, "r-", label="accel_cmd")
        ax_acc.set_ylabel("acceleration (m/s²)")
        ax_acc.legend(loc="upper right")
        ax_acc.grid(True)

        if ref and ref.points:
            xs = [p.pose.position.x for p in ref.points]
            ys = [p.pose.position.y for p in ref.points]
            ax_xy.plot(xs, ys, "g-", alpha=0.7, label="reference", linewidth=2)
        if pred and pred.points:
            xs = [p.pose.position.x for p in pred.points]
            ys = [p.pose.position.y for p in pred.points]
            ax_xy.plot(xs, ys, "b-", alpha=0.8, label="predicted")
        ax_xy.set_xlabel("x (m)")
        ax_xy.set_ylabel("y (m)")
        ax_xy.legend(loc="upper right")
        ax_xy.axis("equal")
        ax_xy.grid(True)

    ani = FuncAnimation(fig, animate, interval=200)
    plt.show()

    shutdown_event.set()
    ros_thread.join(timeout=1.0)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser(description="Visualize MPC control: steering, velocity, accel, ref/pred trajectory")
    parser.add_argument("--history-sec", type=float, default=30.0, help="Time window to show (seconds)")
    parser.add_argument(
        "--control-cmd-topic",
        type=str,
        default="/control/command/control_cmd",
        help="Control command topic (use /control/trajectory_follower/control_cmd if before vehicle_cmd_gate)",
    )
    args = parser.parse_args(args)

    class Wrap:
        pass
    w = Wrap()
    w.history_sec = args.history_sec
    w.control_cmd_topic = args.control_cmd_topic
    run_visualizer(w)


if __name__ == "__main__":
    main()
