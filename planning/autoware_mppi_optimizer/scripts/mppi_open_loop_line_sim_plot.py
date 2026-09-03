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

"""Plot MPPI delay-bicycle straight-line plant logs from mppi_open_loop_line_sim.

Opens an interactive matplotlib window by default (left-click drag to pan, scroll to zoom).
Use --no-show for headless PNG export only.

Example:
  ros2 run autoware_mppi_optimizer mppi_open_loop_line_sim_plot.py -- \
    --csv /tmp/mppi_open_loop/plant.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
from typing import Dict
from typing import List

import matplotlib

if "--no-show" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def hide_navigation_toolbar(fig: plt.Figure) -> None:
    """Remove the default matplotlib toolbar (conflicts with custom pan/zoom)."""
    manager = fig.canvas.manager
    if manager is None:
        return
    toolbar = getattr(manager, "toolbar", None)
    if toolbar is None:
        return
    if hasattr(toolbar, "pack_forget"):
        toolbar.pack_forget()
    if hasattr(toolbar, "hide"):
        toolbar.hide()
    if hasattr(toolbar, "setVisible"):
        toolbar.setVisible(False)


def enable_mouse_navigation(fig: plt.Figure) -> None:
    """Left-click drag pans; mouse wheel zooms toward the cursor."""

    class _PanState:
        active = False
        ax: plt.Axes | None = None
        start_x = 0.0
        start_y = 0.0
        xlim: tuple[float, float] = (0.0, 1.0)
        ylim: tuple[float, float] = (0.0, 1.0)

    pan = _PanState()

    def on_press(event) -> None:
        if event.button != 1 or event.inaxes is None:
            return
        pan.active = True
        pan.ax = event.inaxes
        pan.start_x = event.x
        pan.start_y = event.y
        pan.xlim = pan.ax.get_xlim()
        pan.ylim = pan.ax.get_ylim()

    def on_release(event) -> None:
        if event.button == 1:
            pan.active = False

    def on_motion(event) -> None:
        if not pan.active or pan.ax is None:
            return
        dx_px = event.x - pan.start_x
        dy_px = event.y - pan.start_y
        bbox = pan.ax.get_window_extent()
        if bbox.width <= 0.0 or bbox.height <= 0.0:
            return
        x_width = pan.xlim[1] - pan.xlim[0]
        y_height = pan.ylim[1] - pan.ylim[0]
        dx_data = -dx_px * x_width / bbox.width
        dy_data = -dy_px * y_height / bbox.height
        pan.ax.set_xlim(pan.xlim[0] + dx_data, pan.xlim[1] + dx_data)
        pan.ax.set_ylim(pan.ylim[0] + dy_data, pan.ylim[1] + dy_data)
        fig.canvas.draw_idle()

    def on_scroll(event) -> None:
        if event.inaxes is None or event.xdata is None or event.ydata is None:
            return
        ax = event.inaxes
        scale = 1.0 / 1.2 if event.button == "up" else 1.2
        x_left, x_right = ax.get_xlim()
        y_bottom, y_top = ax.get_ylim()
        x_width = (x_right - x_left) * scale
        y_height = (y_top - y_bottom) * scale
        rel_x = (x_right - event.xdata) / (x_right - x_left)
        rel_y = (y_top - event.ydata) / (y_top - y_bottom)
        ax.set_xlim(event.xdata - x_width * (1.0 - rel_x), event.xdata + x_width * rel_x)
        ax.set_ylim(event.ydata - y_height * (1.0 - rel_y), event.ydata + y_height * rel_y)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_press)
    fig.canvas.mpl_connect("button_release_event", on_release)
    fig.canvas.mpl_connect("motion_notify_event", on_motion)
    fig.canvas.mpl_connect("scroll_event", on_scroll)


def load_csv(path: Path) -> Dict[str, np.ndarray]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"empty csv: {path}")
        columns: Dict[str, List[float]] = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name in reader.fieldnames:
                columns[name].append(float(row[name]))
    return {name: np.asarray(values, dtype=float) for name, values in columns.items()}


def load_horizon_xy(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    if not path.is_file():
        return None, None, None
    data = load_csv(path)
    if "x" not in data or "y" not in data:
        return None, None, None
    t = data["t"] if "t" in data else None
    return data["x"], data["y"], t


def plot(
    plant: Dict[str, np.ndarray],
    horizon_x,
    horizon_y,
    horizon_t,
    out_path: Path,
    show: bool,
) -> None:
    t = plant["t"]
    fig_kwargs = {"figsize": (12.0, 10.5), "layout": "constrained"}
    with matplotlib.rc_context({"toolbar": "none" if show else matplotlib.rcParams["toolbar"]}):
        fig = plt.figure(**fig_kwargs)
    grid = fig.add_gridspec(4, 2, height_ratios=[1.2, 1.0, 1.0, 1.0])

    ax_xy = fig.add_subplot(grid[0, 0])
    x_line = np.linspace(min(float(plant["x"].min()), 0.0), float(plant["x"].max()) + 5.0, 50)
    ax_xy.plot(x_line, np.zeros_like(x_line), color="0.65", linestyle="--", label="reference")
    if horizon_x is not None and horizon_y is not None:
        ax_xy.plot(
            horizon_x,
            horizon_y,
            color="tab:cyan",
            linewidth=1.5,
            label="open-loop optimal @ t=0",
        )
        if horizon_t is not None and len(horizon_t) > 0:
            horizon_end_t = float(horizon_t[-1])
            plant_mask = t <= horizon_end_t + 1.0e-6
            ax_xy.plot(
                plant["x"][plant_mask],
                plant["y"][plant_mask],
                color="tab:red",
                linewidth=1.2,
                linestyle="--",
                alpha=0.75,
                label=f"plant (first {horizon_end_t:.1f}s)",
            )
    ax_xy.plot(plant["x"], plant["y"], color="tab:red", linewidth=2.0, label="plant (closed-loop)")
    ax_xy.scatter(plant["x"][0], plant["y"][0], color="tab:red", zorder=3)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("XY")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best")

    ax_cte = fig.add_subplot(grid[0, 1])
    ax_cte.axhline(0.0, color="0.65", linestyle="--")
    ax_cte.plot(t, plant["cross_track_m"], color="tab:red", linewidth=2.0)
    ax_cte.set_xlabel("t [s]")
    ax_cte.set_ylabel("cross-track [m]")
    ax_cte.set_title("Cross-track error")
    ax_cte.grid(True, alpha=0.3)

    ax_v = fig.add_subplot(grid[1, 0])
    ax_v.plot(t, plant["v"], color="tab:blue", linewidth=1.8)
    ax_v.set_ylabel("v [m/s]")
    ax_v.set_title("Speed")
    ax_v.grid(True, alpha=0.3)

    ax_yaw = fig.add_subplot(grid[1, 1], sharex=ax_v)
    ax_yaw.plot(t, np.rad2deg(plant["yaw"]), color="tab:purple", linewidth=1.8)
    ax_yaw.set_ylabel("yaw [deg]")
    ax_yaw.set_title("Heading")
    ax_yaw.grid(True, alpha=0.3)

    ax_a = fig.add_subplot(grid[2, 0], sharex=ax_v)
    ax_a.plot(t, plant["accel"], color="tab:green", linewidth=1.8)
    ax_a.set_ylabel("a [m/s²]")
    ax_a.set_title("Longitudinal accel (plant)")
    ax_a.grid(True, alpha=0.3)

    ax_steer = fig.add_subplot(grid[2, 1], sharex=ax_v)
    ax_steer.plot(t, np.rad2deg(plant["steer"]), color="tab:orange", linewidth=1.8)
    ax_steer.set_ylabel("steer [deg]")
    ax_steer.set_title("Tire angle (plant)")
    ax_steer.grid(True, alpha=0.3)

    ax_ua = fig.add_subplot(grid[3, 0], sharex=ax_v)
    ax_ua.plot(t, plant["accel_cmd"], color="tab:green", linewidth=1.8)
    ax_ua.set_xlabel("t [s]")
    ax_ua.set_ylabel("accel cmd [m/s²]")
    ax_ua.set_title("Acceleration command")
    ax_ua.grid(True, alpha=0.3)

    ax_ud = fig.add_subplot(grid[3, 1], sharex=ax_v)
    ax_ud.plot(t, np.rad2deg(plant["steer_cmd"]), color="tab:orange", linewidth=1.8)
    ax_ud.set_xlabel("t [s]")
    ax_ud.set_ylabel("steer cmd [deg]")
    ax_ud.set_title("Steer command")
    ax_ud.grid(True, alpha=0.3)

    fig.suptitle("MPPI delay-bicycle open-loop straight-line tracking")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=140)
    print(f"Wrote {out_path}")
    if show:
        enable_mouse_navigation(fig)
        hide_navigation_toolbar(fig)
        if fig.canvas.manager is not None:
            fig.canvas.manager.set_window_title("MPPI open-loop line sim")
        print("Interactive window: left-click drag to pan, mouse wheel to zoom.")
        plt.show(block=True)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, type=Path, help="plant.csv from the sim")
    parser.add_argument(
        "--horizon-csv",
        type=Path,
        default=None,
        help="optional first-horizon trajectory CSV (debug columns)",
    )
    parser.add_argument("--out", type=Path, default=None, help="output PNG (default: next to csv)")
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="save PNG only (no interactive window; for headless/CI)",
    )
    args = parser.parse_args()

    csv_path = args.csv.expanduser().resolve()
    plant = load_csv(csv_path)
    required = {
        "t",
        "x",
        "y",
        "yaw",
        "v",
        "accel",
        "steer",
        "accel_cmd",
        "steer_cmd",
        "cross_track_m",
    }
    missing = required.difference(plant)
    if missing:
        raise SystemExit(f"csv missing columns: {sorted(missing)}")

    horizon_csv = args.horizon_csv
    if horizon_csv is None:
        candidate = csv_path.with_name("horizon0.csv")
        horizon_csv = candidate if candidate.is_file() else None
    horizon_x = horizon_y = horizon_t = None
    if horizon_csv is not None:
        horizon_x, horizon_y, horizon_t = load_horizon_xy(horizon_csv.expanduser().resolve())

    out_path = args.out.expanduser().resolve() if args.out else csv_path.with_suffix(".png")
    plot(plant, horizon_x, horizon_y, horizon_t, out_path, show=not args.no_show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
